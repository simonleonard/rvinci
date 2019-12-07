#include "rvinci_dvrk_haptics.h"

#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>

namespace rvinci_dvrk_haptics {
namespace {
constexpr int kQueueSize = 10;
}

DvrkHaptics::DvrkHaptics()
    : left_arm_("Left", nh_, "/dvrk/MTML/"),
      right_arm_("Right", nh_, "/dvrk/MTMR/") {
  operator_present_sub_ =
      nh_.subscribe("/dvrk/footpedals/operatorpresent", kQueueSize,
                    &DvrkHaptics::onOperatorPresentChange, this);

  rvinci_update_sub_ = nh_.subscribe("/rvinci_input_update", kQueueSize,
                                     &DvrkHaptics::inputCallback, this);
}

void DvrkHaptics::inputCallback(
    const rvinci_input_msg::rvinci_input::ConstPtr& r_input) {}

void DvrkHaptics::bringup() {
  left_arm_.startBringup();
  right_arm_.startBringup();
}

void DvrkHaptics::onOperatorPresentChange(const sensor_msgs::Joy& msg) {
  if (msg.buttons.empty()) {
    ROS_ERROR("Ignoring invalid operator present message");
    return;
  }

  bool operator_present = msg.buttons[0];
  left_arm_.setOperatorPresent(operator_present);
  right_arm_.setOperatorPresent(operator_present);
}

} // namespace rvinci_dvrk_haptics