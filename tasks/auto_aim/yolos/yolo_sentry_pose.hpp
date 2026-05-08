#ifndef AUTO_AIM__YOLO_SENTRY_POSE_HPP
#define AUTO_AIM__YOLO_SENTRY_POSE_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{
class YOLOSentryPose : public YOLOBase
{
public:
  YOLOSentryPose(const std::string & config_path, bool debug);

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  struct LetterboxInfo
  {
    double scale{1.0};
    int pad_x{0};
    int pad_y{0};
  };

  static constexpr int INPUT_SIZE = 640;
  static constexpr int KEYPOINT_COUNT = 8;
  static constexpr int OUTPUT_COLUMNS = 6 + KEYPOINT_COUNT * 3;

  std::string model_path_;
  std::string device_;
  bool debug_;
  float score_threshold_;
  float nms_threshold_;
  float keypoint_threshold_;
  Color detection_color_;

  ov::Core core_;
  ov::CompiledModel compiled_model_;

  LetterboxInfo preprocess(const cv::Mat & bgr_img, std::vector<float> & input_data) const;

  std::list<Armor> parse(
    const float * output_data, std::size_t row_count, const cv::Mat & bgr_img,
    const LetterboxInfo & letterbox, int frame_count) const;

  cv::Point2f restore_point(float x, float y, const LetterboxInfo & letterbox) const;

  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_SENTRY_POSE_HPP
