#include "rvinci/rvinci_arm.h"

// RViz/OGRE
#include <rviz/frame_manager.h>
#include <rviz/msg_conversions.h>

// Messages
#include <geometry_msgs/Transform.h>
#include <interaction_cursor_msgs/InteractionCursorUpdate.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

namespace rvinci {
namespace {
constexpr int kQueueSize = 10;
}

void RvinciArm::onInitialize(ros::NodeHandle& nh,
                             const std::string& dvrk_topic_prefix,
                             const std::string& rvinci_topic_prefix) {
  robot_state_pub_ = nh.advertise<std_msgs::String>(
      dvrk_topic_prefix + "set_robot_state", kQueueSize);

  gravity_comp_pub_ = nh.advertise<std_msgs::Bool>(
      dvrk_topic_prefix + "set_gravity_compensation", kQueueSize);

  cursor_update_pub_ =
      nh.advertise<interaction_cursor_msgs::InteractionCursorUpdate>(
          rvinci_topic_prefix + "update", kQueueSize);

  clutch_start_pub_ = nh.advertise<std_msgs::Empty>(
      rvinci_topic_prefix + "clutch_start", kQueueSize);

  clutch_end_pub_ = nh.advertise<geometry_msgs::Transform>(
      rvinci_topic_prefix + "clutch_end", kQueueSize);
}

void RvinciArm::setGravityCompensation(bool gravity_compensation) {
  std_msgs::Bool msg;
  msg.data = gravity_compensation;

  gravity_comp_pub_.publish(msg);
}

void RvinciArm::updateCursor(const std::string& fixed_frame,
                             const rvinci_input_msg::Gripper& gripper_msg,
                             const Ogre::Quaternion& camera_orientation,
                             const Ogre::Vector3& input_scale) {
  // Compute new position
  Ogre::Vector3 arm_position =
      rviz::pointMsgToOgre(gripper_msg.pose.pose.position);
  Ogre::Vector3 cursor_position =
      getCursorPosition(arm_position, camera_orientation, input_scale);

  // Compute new orientation
  Ogre::Quaternion arm_orientation =
      rviz::quaternionMsgToOgre(gripper_msg.pose.pose.orientation);
  Ogre::Quaternion cursor_orientation =
      getCursorOrientation(arm_orientation, camera_orientation);

  uint8_t gripper_action = getGripperAction(gripper_msg.grab);

  publishCursorUpdate(fixed_frame, cursor_position, cursor_orientation,
                      gripper_action);

  prev_arm_position_ = arm_position;
  prev_cursor_position_ = cursor_position;
  prev_grab_ = gripper_msg.grab;
}

void RvinciArm::updateArmOnly(const rvinci_input_msg::Gripper& gripper_msg) {
  rviz::pointMsgToOgre(gripper_msg.pose.pose.position, prev_arm_position_);
  prev_grab_ = gripper_msg.grab;
}

Ogre::Vector3
RvinciArm::getCursorPosition(const Ogre::Vector3& arm_position,
                             const Ogre::Quaternion& camera_orientation,
                             const Ogre::Vector3& input_scale) {
  // These values translate da Vinci to RViz. I have no idea how they were
  // derived.
  //  Ogre::Quaternion pos_shift(0.7071, 0.7071, 0, 0);
  Ogre::Quaternion pos_shift = Ogre::Quaternion::IDENTITY;

  Ogre::Vector3 arm_position_delta = arm_position - prev_arm_position_;

  Ogre::Vector3 cursor_position_delta =
      pos_shift * (input_scale * arm_position_delta);

  return prev_cursor_position_ + camera_orientation * cursor_position_delta;
}

Ogre::Quaternion
RvinciArm::getCursorOrientation(const Ogre::Quaternion& arm_orientation,
                                const Ogre::Quaternion& camera_orientation) {
  // These values translate da Vinci to RViz. I have no idea how they were
  // derived.
  Ogre::Quaternion ori_shift(0, 0, -sqrt(0.5), sqrt(0.5));

  return camera_orientation * (ori_shift * arm_orientation);
}

uint8_t RvinciArm::getGripperAction(bool grab) {
  if (grab && !prev_grab_) {
    return interaction_cursor_msgs::InteractionCursorUpdate::GRAB;
  } else if (!grab && prev_grab_) {
    return interaction_cursor_msgs::InteractionCursorUpdate::RELEASE;
  } else if (grab && prev_grab_) {
    return interaction_cursor_msgs::InteractionCursorUpdate::KEEP_ALIVE;
  }
  return interaction_cursor_msgs::InteractionCursorUpdate::NONE;
}

void RvinciArm::publishCursorUpdate(const std::string& frame,
                                    const Ogre::Vector3& cursor_position,
                                    const Ogre::Quaternion& cursor_orientation,
                                    uint8_t grip_action) {
  interaction_cursor_msgs::InteractionCursorUpdate msg;

  msg.pose.header.frame_id = frame;
  msg.pose.header.stamp = ros::Time::now();
  msg.pose.pose.position = rviz::pointOgreToMsg(cursor_position);
  msg.pose.pose.orientation = rviz::quaternionOgreToMsg(cursor_orientation);
  msg.button_state = grip_action;

  cursor_update_pub_.publish(msg);
}

void RvinciArm::startClutch(const geometry_msgs::Pose& arm_pose) {
  arm_pose_before_clutch_ = arm_pose;
  clutch_start_pub_.publish(std_msgs::Empty());
}

void RvinciArm::stopClutch(const geometry_msgs::Pose& arm_pose) {
  tf::Pose pose_before, pose_after;
  tf::poseMsgToTF(arm_pose_before_clutch_, pose_before);
  tf::poseMsgToTF(arm_pose, pose_after);

  geometry_msgs::Transform transform_msg;
  tf::transformTFToMsg(pose_after * pose_before.inverse(), transform_msg);

  clutch_end_pub_.publish(transform_msg);
}

std::pair<Ogre::Vector3, Ogre::Quaternion>
RvinciArm::updateCamera(const std::string& fixed_frame,
                        const rvinci_input_msg::Gripper& gripper_msg,
                        const Ogre::Quaternion& camera_orientation,
                        const Ogre::Vector3& input_scale) {
  // Compute new position
  Ogre::Vector3 arm_position =
      rviz::pointMsgToOgre(gripper_msg.pose.pose.position);
  Ogre::Vector3 cursor_position =
      getCursorPosition(arm_position, camera_orientation, input_scale);

  // Compute new orientation
  Ogre::Quaternion arm_orientation =
      rviz::quaternionMsgToOgre(gripper_msg.pose.pose.orientation);
  Ogre::Quaternion cursor_orientation =
      getCursorOrientation(arm_orientation, camera_orientation);

  // Publish: old position, new orientation, NONE grip
  publishCursorUpdate(fixed_frame, prev_cursor_position_, cursor_orientation,
                      interaction_cursor_msgs::InteractionCursorUpdate::NONE);

  updateArmOnly(gripper_msg);

  return std::make_pair(cursor_position, cursor_orientation);
}

void RvinciArm::resetCursor(Ogre::Vector3 cursor_position) {
  prev_cursor_position_ = cursor_position;
}

} // namespace rvinci