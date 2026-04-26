#ifndef AUTO_BUFF_FYT__TYPES_HPP
#define AUTO_BUFF_FYT__TYPES_HPP

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

namespace auto_buff_fyt
{
enum class EnemyColor { RED = 0, BLUE = 1 };
enum class RuneType { INACTIVATED = 0, ACTIVATED };

inline std::string enemy_color_to_string(EnemyColor color)
{
  switch (color) {
    case EnemyColor::RED:
      return "RED";
    case EnemyColor::BLUE:
      return "BLUE";
    default:
      return "UNKNOWN";
  }
}

struct FeaturePoints
{
  FeaturePoints()
  : r_center(-1.0f, -1.0f),
    bottom_right(-1.0f, -1.0f),
    top_right(-1.0f, -1.0f),
    top_left(-1.0f, -1.0f),
    bottom_left(-1.0f, -1.0f)
  {
  }

  void reset()
  {
    r_center = cv::Point2f(-1.0f, -1.0f);
    bottom_right = cv::Point2f(-1.0f, -1.0f);
    top_right = cv::Point2f(-1.0f, -1.0f);
    top_left = cv::Point2f(-1.0f, -1.0f);
    bottom_left = cv::Point2f(-1.0f, -1.0f);
  }

  FeaturePoints operator+(const FeaturePoints & other) const
  {
    FeaturePoints res;
    res.r_center = r_center + other.r_center;
    res.bottom_right = bottom_right + other.bottom_right;
    res.top_right = top_right + other.top_right;
    res.top_left = top_left + other.top_left;
    res.bottom_left = bottom_left + other.bottom_left;
    return res;
  }

  FeaturePoints operator/(float other) const
  {
    FeaturePoints res;
    res.r_center = r_center / other;
    res.bottom_right = bottom_right / other;
    res.top_right = top_right / other;
    res.top_left = top_left / other;
    res.bottom_left = bottom_left / other;
    return res;
  }

  std::vector<cv::Point2f> to_vector2f() const
  {
    return {r_center, bottom_left, top_left, top_right, bottom_right};
  }

  std::vector<cv::Point> to_vector2i() const
  {
    return {r_center, bottom_left, top_left, top_right, bottom_right};
  }

  cv::Point2f r_center;
  cv::Point2f bottom_right;
  cv::Point2f top_right;
  cv::Point2f top_left;
  cv::Point2f bottom_left;

  std::vector<FeaturePoints> children;
};

struct RuneObject
{
  EnemyColor color = EnemyColor::RED;
  RuneType type = RuneType::ACTIVATED;
  float prob = 0.0f;
  FeaturePoints pts;
  cv::Rect box;
};
}  // namespace auto_buff_fyt

#endif  // AUTO_BUFF_FYT__TYPES_HPP
