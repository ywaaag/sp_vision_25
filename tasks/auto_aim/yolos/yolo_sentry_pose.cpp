#include "yolo_sentry_pose.hpp"

#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
namespace
{
template <typename T>
T yaml_value_or(const YAML::Node & yaml, const std::string & key, const T & default_value)
{
  return yaml[key] ? yaml[key].as<T>() : default_value;
}

Color parse_color(const std::string & color)
{
  if (color == "red") {
    return Color::red;
  }
  if (color == "blue") {
    return Color::blue;
  }
  if (color == "purple") {
    return Color::purple;
  }
  return Color::extinguish;
}
}  // namespace

YOLOSentryPose::YOLOSentryPose(const std::string & config_path, bool debug) : debug_(debug)
{
  const auto yaml = YAML::LoadFile(config_path);

  model_path_ = yaml["sentry_pose_model_path"].as<std::string>();
  device_ = yaml["device"].as<std::string>();
  score_threshold_ = yaml_value_or<float>(yaml, "sentry_pose_conf", 0.25F);
  keypoint_threshold_ = yaml_value_or<float>(yaml, "sentry_keypoint_conf", 0.25F);
  nms_threshold_ = yaml_value_or<float>(yaml, "sentry_pose_nms", 0.3F);
  detection_color_ = parse_color(yaml_value_or<std::string>(yaml, "enemy_color", "blue"));

  if (!std::filesystem::exists(model_path_)) {
    throw std::runtime_error("Sentry pose OpenVINO model does not exist: " + model_path_);
  }

  const auto model = core_.read_model(model_path_);
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
}

std::list<Armor> YOLOSentryPose::detect(const cv::Mat & bgr_img, int frame_count)
{
  if (bgr_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return {};
  }

  std::vector<float> input_data;
  const auto letterbox = preprocess(bgr_img, input_data);
  ov::Tensor input_tensor(ov::element::f32, {1, 3, INPUT_SIZE, INPUT_SIZE}, input_data.data());

  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  const auto output_tensor = infer_request.get_output_tensor();
  const auto output_shape = output_tensor.get_shape();
  if (output_shape.size() != 3 || output_shape[2] != OUTPUT_COLUMNS) {
    throw std::runtime_error("Unexpected sentry pose model output shape");
  }

  return parse(
    output_tensor.data<const float>(), output_shape[1], bgr_img, letterbox, frame_count);
}

YOLOSentryPose::LetterboxInfo YOLOSentryPose::preprocess(
  const cv::Mat & bgr_img, std::vector<float> & input_data) const
{
  const auto scale = std::min(
    static_cast<double>(INPUT_SIZE) / bgr_img.cols,
    static_cast<double>(INPUT_SIZE) / bgr_img.rows);
  const auto resized_w = static_cast<int>(std::round(bgr_img.cols * scale));
  const auto resized_h = static_cast<int>(std::round(bgr_img.rows * scale));
  const auto pad_x = (INPUT_SIZE - resized_w) / 2;
  const auto pad_y = (INPUT_SIZE - resized_h) / 2;

  cv::Mat letterboxed(INPUT_SIZE, INPUT_SIZE, CV_8UC3, cv::Scalar(114, 114, 114));
  cv::Mat resized;
  cv::resize(bgr_img, resized, cv::Size(resized_w, resized_h));
  resized.copyTo(letterboxed(cv::Rect(pad_x, pad_y, resized_w, resized_h)));
  cv::cvtColor(letterboxed, letterboxed, cv::COLOR_BGR2RGB);

  input_data.assign(3 * INPUT_SIZE * INPUT_SIZE, 0.0F);
  const auto plane_size = INPUT_SIZE * INPUT_SIZE;
  for (int y = 0; y < INPUT_SIZE; ++y) {
    const auto * row = letterboxed.ptr<cv::Vec3b>(y);
    for (int x = 0; x < INPUT_SIZE; ++x) {
      const auto pixel = row[x];
      const auto idx = y * INPUT_SIZE + x;
      input_data[idx] = static_cast<float>(pixel[0]) / 255.0F;
      input_data[plane_size + idx] = static_cast<float>(pixel[1]) / 255.0F;
      input_data[2 * plane_size + idx] = static_cast<float>(pixel[2]) / 255.0F;
    }
  }

  return LetterboxInfo{scale, pad_x, pad_y};
}

std::list<Armor> YOLOSentryPose::parse(
  const float * output_data, std::size_t row_count, const cv::Mat & bgr_img,
  const LetterboxInfo & letterbox, int frame_count) const
{
  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  std::vector<std::vector<cv::Point2f>> keypoints_list;
  std::vector<std::vector<float>> keypoint_confidences_list;

  for (std::size_t row_idx = 0; row_idx < row_count; ++row_idx) {
    const auto * row = output_data + row_idx * OUTPUT_COLUMNS;
    const auto score = row[4];
    const auto class_id = static_cast<int>(std::round(row[5]));
    if (score < score_threshold_ || class_id != 0) {
      continue;
    }

    std::vector<cv::Point2f> keypoints;
    std::vector<float> keypoint_confidences;
    keypoints.reserve(KEYPOINT_COUNT);
    keypoint_confidences.reserve(KEYPOINT_COUNT);
    auto low_confidence = false;
    for (int i = 0; i < KEYPOINT_COUNT; ++i) {
      const auto offset = 6 + i * 3;
      const auto confidence = row[offset + 2];
      if (confidence < keypoint_threshold_) {
        low_confidence = true;
        break;
      }
      keypoints.emplace_back(restore_point(row[offset], row[offset + 1], letterbox));
      keypoint_confidences.emplace_back(confidence);
    }
    if (low_confidence) {
      continue;
    }

    auto top_left = restore_point(row[0], row[1], letterbox);
    auto bottom_right = restore_point(row[2], row[3], letterbox);
    top_left.x = std::clamp(top_left.x, 0.0F, static_cast<float>(bgr_img.cols - 1));
    top_left.y = std::clamp(top_left.y, 0.0F, static_cast<float>(bgr_img.rows - 1));
    bottom_right.x = std::clamp(bottom_right.x, 0.0F, static_cast<float>(bgr_img.cols - 1));
    bottom_right.y = std::clamp(bottom_right.y, 0.0F, static_cast<float>(bgr_img.rows - 1));

    const auto left = static_cast<int>(std::round(std::min(top_left.x, bottom_right.x)));
    const auto top = static_cast<int>(std::round(std::min(top_left.y, bottom_right.y)));
    const auto right = static_cast<int>(std::round(std::max(top_left.x, bottom_right.x)));
    const auto bottom = static_cast<int>(std::round(std::max(top_left.y, bottom_right.y)));
    if (right <= left || bottom <= top) {
      continue;
    }

    boxes.emplace_back(left, top, right - left, bottom - top);
    scores.emplace_back(score);
    keypoints_list.emplace_back(std::move(keypoints));
    keypoint_confidences_list.emplace_back(std::move(keypoint_confidences));
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, scores, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (const auto index : indices) {
    const auto & box = boxes[index];
    const auto left = static_cast<float>(box.x);
    const auto top = static_cast<float>(box.y);
    const auto right = static_cast<float>(box.x + box.width);
    const auto bottom = static_cast<float>(box.y + box.height);
    std::vector<cv::Point2f> box_points{
      {left, top}, {right, top}, {right, bottom}, {left, bottom}};

    Armor armor(0, 0, scores[index], box, box_points);
    armor.color = detection_color_;
    armor.name = ArmorName::sentry;
    armor.type = ArmorType::small;
    armor.priority = ArmorPriority::first;
    armor.sentry_keypoints = keypoints_list[index];
    armor.sentry_keypoint_confidences = keypoint_confidences_list[index];
    armor.center_norm = {armor.center.x / bgr_img.cols, armor.center.y / bgr_img.rows};
    armors.emplace_back(std::move(armor));
  }

  if (debug_) {
    draw_detections(bgr_img, armors, frame_count);
  }
  return armors;
}

cv::Point2f YOLOSentryPose::restore_point(float x, float y, const LetterboxInfo & letterbox) const
{
  return {
    static_cast<float>((x - letterbox.pad_x) / letterbox.scale),
    static_cast<float>((y - letterbox.pad_y) / letterbox.scale)};
}

void YOLOSentryPose::draw_detections(
  const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = img.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    tools::draw_points(detection, armor.sentry_keypoints, {0, 255, 0});
    tools::draw_text(
      detection, fmt::format("{:.2f} sentry_pose", armor.confidence), armor.center, {0, 255, 0});
  }
  cv::resize(detection, detection, {}, 0.5, 0.5);
  cv::imshow("sentry_pose_detection", detection);
}

std::list<Armor> YOLOSentryPose::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  (void)scale;
  const auto row_count = static_cast<std::size_t>(output.rows);
  return parse(
    reinterpret_cast<const float *>(output.data), row_count, bgr_img, LetterboxInfo{}, frame_count);
}

}  // namespace auto_aim
