#ifndef RVINCI_DVRK_HAPTICS_SRC_RVINCI_DVRK_HAPTICS_H_
#define RVINCI_DVRK_HAPTICS_SRC_RVINCI_DVRK_HAPTICS_H_

// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// Messages
#include <rvinci_input_msg/rvinci_input.h>
#include <sensor_msgs/Joy.h>

// Local
#include "rvinci_dvrk_arm_haptics.h"

namespace rvinci_dvrk_haptics {

class DvrkHaptics {
public:
  DvrkHaptics();
  void inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& rvinci_msg);

  void bringup();

private:
  ros::NodeHandle nh_;

  tf::TransformListener tf_listener_;

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
  ros::Subscriber operator_present_sub_;
  ros::Subscriber rvinci_update_sub_;
#pragma clang diagnostic pop

  DvrkArmHaptics left_arm_;
  DvrkArmHaptics right_arm_;

  bool is_camera_mode_ = false;
  double desired_arm_distance_ = 0.0;

  void onOperatorPresentChange(const sensor_msgs::Joy& msg);
  void enterCameraMode();
  void updateCameraHaptics(const tf::Point& left_pos,
                           const tf::Point& right_pos);
  void updateCameraHapticsForArm(DvrkArmHaptics& main_arm,
                                 const tf::Point& main_arm_pos,
                                 DvrkArmHaptics& other_arm,
                                 const tf::Point& other_arm_pos);
};

} // namespace rvinci_dvrk_haptics

#endif // RVINCI_DVRK_HAPTICS_SRC_RVINCI_DVRK_HAPTICS_H_
