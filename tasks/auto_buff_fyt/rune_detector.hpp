#ifndef AUTO_BUFF_FYT__RUNE_DETECTOR_HPP
#define AUTO_BUFF_FYT__RUNE_DETECTOR_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <filesystem>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <tuple>
#include <vector>

#include "types.hpp"

namespace auto_buff_fyt
{
struct GridAndStride
{
  int grid0;
  int grid1;
  int stride;
};

class RuneDetector
{
public:
  explicit RuneDetector(const std::string & config_path);

  std::vector<RuneObject> detect(const cv::Mat & rgb_img);

  std::tuple<cv::Point2f, cv::Mat> detect_r_tag(
    const cv::Mat & bgr_img, const cv::Point2f & prior) const;

  bool use_r_tag() const { return detect_r_tag_; }

private:
  void init();

  std::string model_path_;
  std::string device_name_;
  float conf_threshold_;
  int top_k_;
  float nms_threshold_;
  bool detect_r_tag_;
  int binary_thresh_;
  int r_tag_roi_half_size_;
  float r_tag_max_distance_;
  float r_tag_min_area_;

  std::mutex mtx_;
  std::vector<int> strides_;
  std::vector<GridAndStride> grid_strides_;

  std::unique_ptr<ov::Core> ov_core_;
  std::unique_ptr<ov::CompiledModel> compiled_model_;
};
}  // namespace auto_buff_fyt

#endif  // AUTO_BUFF_FYT__RUNE_DETECTOR_HPP
