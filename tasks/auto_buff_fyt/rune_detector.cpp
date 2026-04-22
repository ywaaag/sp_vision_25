#include "rune_detector.hpp"

#include <algorithm>
#include <numeric>
#include <unordered_map>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace auto_buff_fyt
{
namespace
{
constexpr int INPUT_W = 480;
constexpr int INPUT_H = 480;
constexpr int NUM_CLASSES = 2;
constexpr int NUM_COLORS = 2;
constexpr int NUM_POINTS = 5;
constexpr int NUM_POINTS_2 = 2 * NUM_POINTS;
constexpr float MERGE_CONF_ERROR = 0.15f;
constexpr float MERGE_MIN_IOU = 0.9f;

const std::unordered_map<int, EnemyColor> DNN_COLOR_TO_ENEMY_COLOR = {
  {0, EnemyColor::BLUE},
  {1, EnemyColor::RED}};

cv::Mat letterbox(
  const cv::Mat & img, Eigen::Matrix3f & transform_matrix,
  const std::vector<int> & new_shape = {INPUT_W, INPUT_H})
{
  const int img_h = img.rows;
  const int img_w = img.cols;

  const float scale =
    std::min(new_shape[1] * 1.0f / img_h, new_shape[0] * 1.0f / img_w);
  const int resize_h = static_cast<int>(std::round(img_h * scale));
  const int resize_w = static_cast<int>(std::round(img_w * scale));

  const int pad_h = new_shape[1] - resize_h;
  const int pad_w = new_shape[0] - resize_w;
  const float half_h = pad_h * 0.5f;
  const float half_w = pad_w * 0.5f;

  const int top = static_cast<int>(std::round(half_h - 0.1f));
  const int bottom = static_cast<int>(std::round(half_h + 0.1f));
  const int left = static_cast<int>(std::round(half_w - 0.1f));
  const int right = static_cast<int>(std::round(half_w + 0.1f));

  transform_matrix << 1.0f / scale, 0.0f, -half_w / scale, 0.0f, 1.0f / scale,
    -half_h / scale, 0.0f, 0.0f, 1.0f;

  cv::Mat resized_img;
  cv::resize(img, resized_img, cv::Size(resize_w, resize_h));
  cv::copyMakeBorder(
    resized_img, resized_img, top, bottom, left, right, cv::BORDER_CONSTANT,
    cv::Scalar(114, 114, 114));
  return resized_img;
}

void generate_grids_and_stride(
  int target_w, int target_h, const std::vector<int> & strides,
  std::vector<GridAndStride> & grid_strides)
{
  for (auto stride : strides) {
    const int num_grid_w = target_w / stride;
    const int num_grid_h = target_h / stride;
    for (int g1 = 0; g1 < num_grid_h; ++g1) {
      for (int g0 = 0; g0 < num_grid_w; ++g0) {
        grid_strides.emplace_back(GridAndStride{g0, g1, stride});
      }
    }
  }
}

void generate_proposals(
  std::vector<RuneObject> & output_objs, const cv::Mat & output_buffer,
  const Eigen::Matrix3f & transform_matrix, float conf_threshold,
  const std::vector<GridAndStride> & grid_strides)
{
  for (size_t anchor_idx = 0; anchor_idx < grid_strides.size(); ++anchor_idx) {
    const float confidence = output_buffer.at<float>(static_cast<int>(anchor_idx), NUM_POINTS_2);
    if (confidence < conf_threshold) continue;

    const int grid0 = grid_strides[anchor_idx].grid0;
    const int grid1 = grid_strides[anchor_idx].grid1;
    const int stride = grid_strides[anchor_idx].stride;

    double color_score = 0.0;
    double class_score = 0.0;
    cv::Point color_id;
    cv::Point class_id;
    const cv::Mat color_scores = output_buffer.row(static_cast<int>(anchor_idx))
                                   .colRange(NUM_POINTS_2 + 1, NUM_POINTS_2 + 1 + NUM_COLORS);
    const cv::Mat class_scores = output_buffer.row(static_cast<int>(anchor_idx))
                                   .colRange(
                                     NUM_POINTS_2 + 1 + NUM_COLORS,
                                     NUM_POINTS_2 + 1 + NUM_COLORS + NUM_CLASSES);
    cv::minMaxLoc(color_scores, nullptr, &color_score, nullptr, &color_id);
    cv::minMaxLoc(class_scores, nullptr, &class_score, nullptr, &class_id);

    const float x1 = (output_buffer.at<float>(static_cast<int>(anchor_idx), 0) + grid0) * stride;
    const float y1 = (output_buffer.at<float>(static_cast<int>(anchor_idx), 1) + grid1) * stride;
    const float x2 = (output_buffer.at<float>(static_cast<int>(anchor_idx), 2) + grid0) * stride;
    const float y2 = (output_buffer.at<float>(static_cast<int>(anchor_idx), 3) + grid1) * stride;
    const float x3 = (output_buffer.at<float>(static_cast<int>(anchor_idx), 4) + grid0) * stride;
    const float y3 = (output_buffer.at<float>(static_cast<int>(anchor_idx), 5) + grid1) * stride;
    const float x4 = (output_buffer.at<float>(static_cast<int>(anchor_idx), 6) + grid0) * stride;
    const float y4 = (output_buffer.at<float>(static_cast<int>(anchor_idx), 7) + grid1) * stride;
    const float x5 = (output_buffer.at<float>(static_cast<int>(anchor_idx), 8) + grid0) * stride;
    const float y5 = (output_buffer.at<float>(static_cast<int>(anchor_idx), 9) + grid1) * stride;

    Eigen::Matrix<float, 3, 5> apex_norm;
    Eigen::Matrix<float, 3, 5> apex_dst;
    apex_norm << x1, x2, x3, x4, x5, y1, y2, y3, y4, y5, 1, 1, 1, 1, 1;
    apex_dst = transform_matrix * apex_norm;

    RuneObject obj;
    obj.pts.r_center = cv::Point2f(apex_dst(0, 0), apex_dst(1, 0));
    obj.pts.bottom_left = cv::Point2f(apex_dst(0, 1), apex_dst(1, 1));
    obj.pts.top_left = cv::Point2f(apex_dst(0, 2), apex_dst(1, 2));
    obj.pts.top_right = cv::Point2f(apex_dst(0, 3), apex_dst(1, 3));
    obj.pts.bottom_right = cv::Point2f(apex_dst(0, 4), apex_dst(1, 4));
    obj.box = cv::boundingRect(obj.pts.to_vector2f());
    obj.color = DNN_COLOR_TO_ENEMY_COLOR.at(color_id.x);
    obj.type = static_cast<RuneType>(class_id.x);
    obj.prob = confidence;
    output_objs.emplace_back(std::move(obj));
  }
}

float intersection_area(const RuneObject & a, const RuneObject & b)
{
  const cv::Rect_<float> inter = a.box & b.box;
  return inter.area();
}

void nms_merge_sorted_bboxes(
  std::vector<RuneObject> & objects, std::vector<int> & picked, float nms_threshold)
{
  picked.clear();
  const int n = static_cast<int>(objects.size());
  std::vector<float> areas(n);
  for (int i = 0; i < n; ++i) areas[i] = objects[i].box.area();

  for (int i = 0; i < n; ++i) {
    auto & a = objects[i];
    bool keep = true;
    for (auto picked_idx : picked) {
      auto & b = objects[picked_idx];
      const float inter_area = intersection_area(a, b);
      const float union_area = areas[i] + areas[picked_idx] - inter_area;
      const float iou = inter_area / union_area;
      if (iou > nms_threshold || std::isnan(iou)) {
        keep = false;
        if (
          a.type == b.type && a.color == b.color && iou > MERGE_MIN_IOU &&
          std::abs(a.prob - b.prob) < MERGE_CONF_ERROR)
        {
          a.pts.children.push_back(b.pts);
        }
      }
    }
    if (keep) picked.push_back(i);
  }
}
}  // namespace

RuneDetector::RuneDetector(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  auto node = yaml["buff_fyt_detector"];

  model_path_ = node && node["model"] ? node["model"].as<std::string>() : "assets/yolox_rune_3.6m.xml";
  device_name_ =
    node && node["device_type"] ? node["device_type"].as<std::string>() : "CPU";
  conf_threshold_ =
    node && node["confidence_threshold"] ? node["confidence_threshold"].as<float>() : 0.7f;
  top_k_ = node && node["top_k"] ? node["top_k"].as<int>() : 128;
  nms_threshold_ =
    node && node["nms_threshold"] ? node["nms_threshold"].as<float>() : 0.3f;
  detect_r_tag_ = node && node["detect_r_tag"] ? node["detect_r_tag"].as<bool>() : true;
  binary_thresh_ = node && node["min_lightness"] ? node["min_lightness"].as<int>() : 100;

  init();
}

void RuneDetector::init()
{
  if (!ov_core_) ov_core_ = std::make_unique<ov::Core>();

  auto model = ov_core_->read_model(model_path_);

  const auto perf_mode =
    device_name_ == "GPU" ?
    ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT) :
    ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY);
  compiled_model_ = std::make_unique<ov::CompiledModel>(
    ov_core_->compile_model(model, device_name_, perf_mode));

  strides_ = {8, 16, 32};
  grid_strides_.clear();
  generate_grids_and_stride(INPUT_W, INPUT_H, strides_, grid_strides_);
}

std::vector<RuneObject> RuneDetector::detect(const cv::Mat & rgb_img)
{
  if (rgb_img.empty()) return {};

  Eigen::Matrix3f transform_matrix;
  cv::Mat resized_img = letterbox(rgb_img, transform_matrix);

  cv::Mat blob = cv::dnn::blobFromImage(
    resized_img, 1.0, cv::Size(INPUT_W, INPUT_H), cv::Scalar(0, 0, 0), true);

  auto input_port = compiled_model_->input();
  ov::Tensor input_tensor(
    input_port.get_element_type(), ov::Shape{1, 3, INPUT_H, INPUT_W}, blob.ptr<float>(0));

  std::lock_guard<std::mutex> lock(mtx_);
  auto infer_request = compiled_model_->create_infer_request();
  infer_request.set_input_tensor(input_tensor);
  infer_request.infer();

  const ov::Tensor output = infer_request.get_output_tensor();
  const auto output_shape = output.get_shape();
  cv::Mat output_buffer(
    static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2]), CV_32F,
    const_cast<float *>(output.data<const float>()));

  std::vector<RuneObject> objects_tmp;
  std::vector<RuneObject> objects_result;
  std::vector<int> picked;

  generate_proposals(
    objects_tmp, output_buffer, transform_matrix, conf_threshold_, grid_strides_);

  std::sort(
    objects_tmp.begin(), objects_tmp.end(),
    [](const RuneObject & a, const RuneObject & b) { return a.prob > b.prob; });
  if (objects_tmp.size() > static_cast<size_t>(top_k_)) {
    objects_tmp.resize(static_cast<size_t>(top_k_));
  }

  nms_merge_sorted_bboxes(objects_tmp, picked, nms_threshold_);
  for (auto idx : picked) {
    objects_result.emplace_back(std::move(objects_tmp[idx]));
    auto & obj = objects_result.back();
    if (!obj.pts.children.empty()) {
      const float n = static_cast<float>(obj.pts.children.size() + 1);
      FeaturePoints merged =
        std::accumulate(obj.pts.children.begin(), obj.pts.children.end(), obj.pts);
      obj.pts = merged / n;
    }
  }

  return objects_result;
}

std::tuple<cv::Point2f, cv::Mat> RuneDetector::detect_r_tag(
  const cv::Mat & bgr_img, const cv::Point2f & prior) const
{
  if (
    prior.x < 0 || prior.x > bgr_img.cols || prior.y < 0 || prior.y > bgr_img.rows ||
    bgr_img.empty())
  {
    return {prior, cv::Mat::zeros(cv::Size(200, 200), CV_8UC3)};
  }

  const cv::Rect roi =
    (cv::Rect(
       static_cast<int>(prior.x) - 100, static_cast<int>(prior.y) - 100, 200, 200) &
    cv::Rect(0, 0, bgr_img.cols, bgr_img.rows));
  const cv::Point2f prior_in_roi = prior - cv::Point2f(roi.tl());

  cv::Mat img_roi = bgr_img(roi);
  cv::Mat gray_img;
  cv::cvtColor(img_roi, gray_img, cv::COLOR_BGR2GRAY);
  cv::Mat binary_img;
  if (binary_thresh_ > 0)
    cv::threshold(gray_img, binary_img, binary_thresh_, 255, cv::THRESH_BINARY);
  else
    cv::threshold(gray_img, binary_img, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::dilate(binary_img, binary_img, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  auto it = std::find_if(
    contours.begin(), contours.end(),
    [p = prior_in_roi](const std::vector<cv::Point> & contour) {
      return cv::boundingRect(contour).contains(p);
    });

  cv::cvtColor(binary_img, binary_img, cv::COLOR_GRAY2BGR);
  if (it == contours.end()) return {prior, binary_img};

  cv::drawContours(
    binary_img, contours, static_cast<int>(std::distance(contours.begin(), it)),
    cv::Scalar(0, 255, 0), 2);

  cv::Point2f center = std::accumulate(it->begin(), it->end(), cv::Point(0, 0));
  center /= static_cast<float>(it->size());
  center += cv::Point2f(roi.tl());
  return {center, binary_img};
}
}  // namespace auto_buff_fyt
