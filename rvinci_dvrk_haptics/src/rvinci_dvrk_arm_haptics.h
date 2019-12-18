#ifndef RVINCI_DVRK_HAPTICS_SRC_RVINCI_DVRK_ARM_HAPTICS_H_
#define RVINCI_DVRK_HAPTICS_SRC_RVINCI_DVRK_ARM_HAPTICS_H_

// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// Messages
#include <cisst_msgs/prmCartesianImpedanceGains.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

namespace rvinci_dvrk_haptics {
class DvrkArmHaptics {
  enum class ArmMode { kNotReady, kCamera, kClutching, kOperating, kIdle };
  enum class DvrkMode { kPosition, kEffort };

public:
  DvrkArmHaptics(std::string arm_name, ros::NodeHandle& nh,
                 tf::TransformListener& listener,
                 const std::string& topic_prefix);

  void startBringup();

  bool isReady() const;
  tf::Point getPositionWorld() const;

  void setIsMovingCamera(bool is_moving_camera);
  void setIsClutching(bool is_clutching);
  void setOperatorPresent(bool operator_present);

  void update();

private:
  const std::string arm_name_;

  tf::TransformListener& tf_listener_;

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
  ros::Subscriber current_state_sub_;
  ros::Subscriber arm_mode_sub_;
  ros::Subscriber position_sub_;
  ros::Subscriber input_cartesian_impedance_sub_;
  ros::Subscriber pose_moving_camera_sub_;
#pragma clang diagnostic pop

  ros::Publisher desired_state_pub_;
  ros::Publisher position_pub_;
  ros::Publisher wrench_pub_;
  ros::Publisher cartesian_impedance_pub_;
  ros::Publisher lock_orientation_pub_;
  ros::Publisher unlock_orientation_pub_;

  ArmMode prev_arm_mode_ = ArmMode::kNotReady;
  geometry_msgs::PoseStamped latest_arm_pose_;
  cisst_msgs::prmCartesianImpedanceGains input_cartesian_impedance_;

  // Factors that control arm mode
  bool arm_status_ready_ = false;
  bool operator_present_ = false;
  bool is_moving_camera_ = false;
  bool is_clutching_ = false;

  // Factors controlled by arm mode
  DvrkMode latest_dvrk_mode_ = DvrkMode::kPosition;
  bool orientation_locked_ = false;

  // ROS Callbacks
  void onCurrentStateChange(const std_msgs::String& msg);
  void onArmModeChange(const std_msgs::String& msg);
  void onCurrentPositionChange(const geometry_msgs::PoseStamped& msg);
  void onSetInputCartesianImpedance(
      const cisst_msgs::prmCartesianImpedanceGains& msg);
  void onSetDesiredPoseMovingCamera(const geometry_msgs::PoseStamped& msg);

  // State machine functions
  ArmMode getArmMode() const;
  void enterCameraMode();
  void enterClutchingMode();
  void enterOperatingMode();
  void enterIdleMode();

  // DVRK Actions
  void activateEffortMode();
  void activatePositionMode();
  void deactivateCartesianImpedance();
  void publishInputCartesianImpedance();
  void unlockOrientation();

};
} // namespace rvinci_dvrk_haptics

#endif // RVINCI_DVRK_HAPTICS_SRC_RVINCI_DVRK_ARM_HAPTICS_H_
