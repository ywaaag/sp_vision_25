#ifndef TOOLS__RECORDER_HPP
#define TOOLS__RECORDER_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "tools/thread_safe_queue.hpp"
namespace tools
{
class Recorder
{
public:
  Recorder(double fps = 30);
  ~Recorder();
  void record(
    const cv::Mat & img, const Eigen::Quaterniond & q,
    const std::chrono::steady_clock::time_point & timestamp);

private:
  struct FrameData
  {
    cv::Mat img;
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };
  bool init_;
  std::atomic<bool> stop_thread_;
  double fps_;
  std::chrono::steady_clock::duration frame_period_;
  std::string text_path_;
  std::string video_path_;
  std::ofstream text_writer_;
  cv::VideoWriter video_writer_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point next_frame_time_;
  std::chrono::steady_clock::time_point last_time_;
  FrameData pending_frame_;
  bool has_pending_frame_;
  bool pending_frame_emitted_;
  tools::ThreadSafeQueue<FrameData> queue_;
  std::thread saving_thread_;  // 负责保存帧数据的线程
  void init(const cv::Mat & img, const std::chrono::steady_clock::time_point & timestamp);
  void save_to_file();
  void write_output_frame(
    const FrameData & frame, const std::chrono::steady_clock::time_point & output_time);
  void flush_pending_frame();
};

}  // namespace tools

#endif  // TOOLS__RECORDER_HPP
