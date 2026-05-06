#include "recorder.hpp"

#include <fmt/chrono.h>

#include <cmath>
#include <filesystem>
#include <string>

#include "math_tools.hpp"
#include "tools/logger.hpp"

namespace tools
{
Recorder::Recorder(double fps)
: init_(false),
  stop_thread_(false),
  fps_(fps > 0.0 ? fps : 30.0),
  frame_period_(std::chrono::nanoseconds(static_cast<int64_t>(std::llround(1e9 / fps_)))),
  has_pending_frame_(false),
  pending_frame_emitted_(false),
  queue_(1)
{
  if (fps <= 0.0) tools::logger()->warn("Recorder got invalid fps {}. Fallback to 30.", fps);

  auto folder_path = "records";
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  text_path_ = fmt::format("{}/{}.txt", folder_path, file_name);
  video_path_ = fmt::format("{}/{}.avi", folder_path, file_name);

  std::filesystem::create_directory(folder_path);
}

Recorder::~Recorder()
{
  stop_thread_ = true;
  // 退出时给队列中额外推入一个空帧，避免pop一直等待
  queue_.push({cv::Mat(), Eigen::Quaterniond::Identity(), std::chrono::steady_clock::time_point{}});
  if (saving_thread_.joinable()) saving_thread_.join();  // 等待视频保存线程结束

  if (!init_) return;
  text_writer_.close();
  video_writer_.release();
}

void Recorder::save_to_file()
{
  while (true) {
    if (stop_thread_ && queue_.empty()) {
      flush_pending_frame();
      break;
    }

    FrameData frame;
    if (!queue_.pop(frame)) {
      break;
    }
    if (frame.img.empty()) {
      if (stop_thread_ && queue_.empty()) {
        flush_pending_frame();
        break;
      }
      continue;
    }

    if (!has_pending_frame_) {
      pending_frame_ = frame;
      has_pending_frame_ = true;
      pending_frame_emitted_ = false;
      write_output_frame(pending_frame_, start_time_);
      pending_frame_emitted_ = true;
      next_frame_time_ += frame_period_;
      continue;
    }

    while (next_frame_time_ < frame.timestamp) {
      write_output_frame(pending_frame_, next_frame_time_);
      pending_frame_emitted_ = true;
      next_frame_time_ += frame_period_;
    }

    pending_frame_ = frame;
    pending_frame_emitted_ = false;

    while (next_frame_time_ <= frame.timestamp) {
      write_output_frame(pending_frame_, next_frame_time_);
      pending_frame_emitted_ = true;
      next_frame_time_ += frame_period_;
    }
  }
}

void Recorder::record(
  const cv::Mat & img, const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp)
{
  if (img.empty()) return;
  if (!init_) {
    init(img, timestamp);
    last_time_ = timestamp;
    queue_.push({img, q, timestamp});
    return;
  }

  auto since_last = tools::delta_time(timestamp, last_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = timestamp;
  queue_.push({img, q, timestamp});
}

void Recorder::init(const cv::Mat & img, const std::chrono::steady_clock::time_point & timestamp)
{
  text_writer_.open(text_path_);
  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  video_writer_ = cv::VideoWriter(video_path_, fourcc, fps_, img.size());
  start_time_ = timestamp;
  next_frame_time_ = start_time_;
  saving_thread_ = std::thread(&Recorder::save_to_file, this);  // 启动保存线程
  init_ = true;
}

void Recorder::write_output_frame(
  const FrameData & frame, const std::chrono::steady_clock::time_point & output_time)
{
  video_writer_.write(frame.img);

  // 文本与视频逐帧对齐，时间轴使用输出CFR时间而不是输入采样时间。
  Eigen::Vector4d xyzw = frame.q.coeffs();
  auto since_begin = tools::delta_time(output_time, start_time_);
  text_writer_ << fmt::format(
    "{} {} {} {} {}\n", since_begin, xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
}

void Recorder::flush_pending_frame()
{
  if (!has_pending_frame_ || pending_frame_emitted_) return;

  write_output_frame(pending_frame_, next_frame_time_);
  pending_frame_emitted_ = true;
  next_frame_time_ += frame_period_;
}

}  // namespace tools
