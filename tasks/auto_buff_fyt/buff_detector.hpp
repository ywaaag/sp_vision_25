#ifndef AUTO_BUFF_FYT__BUFF_DETECTOR_HPP
#define AUTO_BUFF_FYT__BUFF_DETECTOR_HPP

#include <yaml-cpp/yaml.h>

#include <optional>
#include <string>
#include <vector>

#include "buff_type.hpp"
#include "rune_detector.hpp"

namespace auto_buff_fyt
{
class Buff_Detector
{
public:
  explicit Buff_Detector(const std::string & config_path);

  std::optional<PowerRune> detect(cv::Mat & bgr_img);

  const std::vector<RuneObject> & last_objects() const { return last_objects_; }

  const std::vector<RuneObject> & last_filtered_objects() const { return last_filtered_objects_; }

  const cv::Mat & last_binary_roi() const { return last_binary_roi_; }

private:
  RuneDetector detector_;
  EnemyColor detect_color_;
  std::vector<RuneObject> last_objects_;
  std::vector<RuneObject> last_filtered_objects_;
  cv::Mat last_binary_roi_;
};
}  // namespace auto_buff_fyt

#endif  // AUTO_BUFF_FYT__BUFF_DETECTOR_HPP
