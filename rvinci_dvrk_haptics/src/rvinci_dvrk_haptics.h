#ifndef RVINCI_DVRK_HAPTICS_SRC_RVINCI_DVRK_HAPTICS_H_
#define RVINCI_DVRK_HAPTICS_SRC_RVINCI_DVRK_HAPTICS_H_

// ROS
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

// Messages
#include <rvinci_input_msg/rvinci_input.h>
#include <sensor_msgs/Joy.h>

// Local
#include "rvinci_dvrk_arm_haptics.h"

namespace rvinci_dvrk_haptics {

class DvrkHaptics {
public:
  DvrkHaptics();
  void inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input);

  void bringup();

private:
  ros::NodeHandle nh_;

  ros::Subscriber operator_present_sub_;
  ros::Subscriber rvinci_update_sub_;

  DvrkArmHaptics left_arm_;
  DvrkArmHaptics right_arm_;

  void onOperatorPresentChange(const sensor_msgs::Joy& msg);
};

} // namespace rvinci_dvrk_haptics

#endif // RVINCI_DVRK_HAPTICS_SRC_RVINCI_DVRK_HAPTICS_H_
