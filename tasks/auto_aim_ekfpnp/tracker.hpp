#ifndef AUTO_AIM_EKFPNP__TRACKER_HPP
#define AUTO_AIM_EKFPNP__TRACKER_HPP

#include <chrono>
#include <list>
#include <string>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "target.hpp"

namespace auto_aim_ekfpnp
{
class Tracker
{
public:
  Tracker(const std::string & config_path, auto_aim::Solver & solver);

  std::string state() const;

  std::list<Target> track(
    std::list<auto_aim::Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue,
    std::list<auto_aim::Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

private:
  auto_aim::Solver & solver_;
  auto_aim::Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_;
  std::string pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  auto_aim::ArmorPriority omni_target_priority_;

  void state_machine(bool found);
  bool set_target(std::list<auto_aim::Armor> & armors, std::chrono::steady_clock::time_point t);
  bool update_target(
    std::list<auto_aim::Armor> & armors, std::chrono::steady_clock::time_point t);
};

}  // namespace auto_aim_ekfpnp

#endif  // AUTO_AIM_EKFPNP__TRACKER_HPP
