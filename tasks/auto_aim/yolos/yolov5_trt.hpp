#ifndef AUTO_AIM__YOLOV5_TRT_HPP
#define AUTO_AIM__YOLOV5_TRT_HPP

#include <list>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{

class YOLOV5TRT : public YOLOBase
{
public:
  struct TimingProfile
  {
    bool valid = false;
    double preprocess_ms = 0.0;
    double input_pack_ms = 0.0;
    double h2d_ms = 0.0;
    double gpu_compute_ms = 0.0;
    double d2h_ms = 0.0;
    double infer_ms = 0.0;
    double postprocess_ms = 0.0;
    double total_ms = 0.0;
  };

  YOLOV5TRT(const std::string & config_path, bool debug);
  ~YOLOV5TRT();

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

  const TimingProfile & last_profile() const;

private:
  struct Impl;

  std::string model_path_;
  std::string save_path_;
  bool debug_{};
  bool use_roi_{};
  bool use_traditional_{};

  const float nms_threshold_ = 0.3F;
  const float score_threshold_ = 0.7F;
  double min_confidence_{};
  double binary_threshold_{};

  cv::Rect roi_;
  cv::Point2f offset_;
  cv::Mat tmp_img_;
  TimingProfile last_profile_;

  Detector detector_;
  std::unique_ptr<Impl> impl_;

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;
  std::list<Armor> parse(double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

  void save(const Armor & armor) const;
  void draw_detections(const cv::Mat & img, const std::list<Armor> & armors, int frame_count) const;
  static double sigmoid(double x);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLOV5_TRT_HPP
