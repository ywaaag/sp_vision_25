#include "buff_detector.hpp"

#include <algorithm>
#include <numeric>

namespace auto_buff_fyt
{
namespace
{
EnemyColor parse_enemy_color(const std::string & color)
{
  return color == "blue" ? EnemyColor::BLUE : EnemyColor::RED;
}
}  // namespace

Buff_Detector::Buff_Detector(const std::string & config_path) : detector_(config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  detect_color_ = parse_enemy_color(yaml["enemy_color"].as<std::string>());
}

std::optional<PowerRune> Buff_Detector::detect(cv::Mat & bgr_img)
{
  last_objects_.clear();
  last_filtered_objects_.clear();
  last_binary_roi_ = cv::Mat::zeros(1, 1, CV_8UC3);

  if (bgr_img.empty()) return std::nullopt;

  cv::Mat rgb_img;
  cv::cvtColor(bgr_img, rgb_img, cv::COLOR_BGR2RGB);

  last_objects_ = detector_.detect(rgb_img);
  last_filtered_objects_ = last_objects_;
  last_filtered_objects_.erase(
    std::remove_if(
      last_filtered_objects_.begin(), last_filtered_objects_.end(),
      [this](const RuneObject & obj) { return obj.color != detect_color_; }),
    last_filtered_objects_.end());

  if (last_filtered_objects_.empty()) return std::nullopt;

  std::sort(
    last_filtered_objects_.begin(), last_filtered_objects_.end(),
    [](const RuneObject & a, const RuneObject & b) { return a.prob > b.prob; });

  const float prob_sum = std::accumulate(
    last_filtered_objects_.begin(), last_filtered_objects_.end(), 0.0f,
    [](float sum, const RuneObject & obj) { return sum + obj.prob; });
  const cv::Point2f r_prior = std::accumulate(
    last_filtered_objects_.begin(), last_filtered_objects_.end(), cv::Point2f(0.0f, 0.0f),
    [prob_sum](const cv::Point2f & p, const RuneObject & obj) {
      const float weight = prob_sum > 1e-6f ? obj.prob / prob_sum : 1.0f;
      return p + obj.pts.r_center * weight;
    });

  cv::Point2f r_center;
  if (detector_.use_r_tag()) {
    std::tie(r_center, last_binary_roi_) = detector_.detect_r_tag(bgr_img, r_prior);
  } else {
    r_center = r_prior;
  }

  std::for_each(last_filtered_objects_.begin(), last_filtered_objects_.end(), [r_center](RuneObject & obj) {
    obj.pts.r_center = r_center;
  });

  auto result_it = std::find_if(
    last_filtered_objects_.begin(), last_filtered_objects_.end(),
    [](const RuneObject & obj) { return obj.type == RuneType::INACTIVATED; });
  if (result_it == last_filtered_objects_.end()) return std::nullopt;

  std::vector<cv::Point2f> armor_points = {
    result_it->pts.bottom_left,
    result_it->pts.top_left,
    result_it->pts.top_right,
    result_it->pts.bottom_right};
  const cv::Point2f center = std::accumulate(
                               armor_points.begin(), armor_points.end(), cv::Point2f(0.0f, 0.0f)) /
                             static_cast<float>(armor_points.size());

  std::vector<auto_buff::FanBlade> fanblades;
  fanblades.emplace_back(armor_points, center, auto_buff::_light);
  PowerRune power_rune(fanblades, r_center, std::nullopt);
  if (power_rune.is_unsolve()) return std::nullopt;
  return power_rune;
}
}  // namespace auto_buff_fyt
