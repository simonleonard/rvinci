#include "rvinci/rvinci_gui.h"

// ROS
#include <ros/console.h>

// RViz/OGRE
#include <OgreOverlayManager.h>
#include <OgrePanelOverlayElement.h>

// Messages
#include <nasa_interface_msgs/AddWaypoint.h>
#include <nasa_interface_msgs/AddWaypointFromPath.h>
#include <nasa_interface_msgs/DeleteWaypoint.h>

namespace rvinci {
namespace {
constexpr int kQueueSize = 10;
}

RvinciGui::~RvinciGui() {
  if (overlay_) {
    Ogre::OverlayManager::getSingleton().destroy(overlay_);
  }
};

void RvinciGui::initialize(ros::NodeHandle& nh,
                           const Ogre::Vector3& input_scale) {
  if (overlay_) return;

  ROS_INFO("Adding the overlay");

  Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();

  overlay_ = overlay_manager.create("RVinciInterface");
  // This should be below most things, especially the cursor overlays
  overlay_->setZOrder(0);

  overlay_->add2D(waypoint_info_panel_.create());
  overlay_->add2D(waypoints_control_panel_.create());
  overlay_->add2D(preview_panel_.create(input_scale));

  // External publishers
  set_preview_playing_pub_ =
      nh.advertise<std_msgs::Bool>("set_preview_playing", kQueueSize, false);
  set_executing_pub_ =
      nh.advertise<std_msgs::Bool>("plan/execute", kQueueSize, false);
  add_waypoint_here_pub_ =
      nh.advertise<nasa_interface_msgs::AddWaypointFromPath>(
          "waypoints/add_from_path", kQueueSize, false);
  add_waypoint_at_end_pub_ = nh.advertise<nasa_interface_msgs::AddWaypoint>(
      "waypoints/add", kQueueSize, false);
  delete_waypoint_pub_ = nh.advertise<nasa_interface_msgs::DeleteWaypoint>(
      "waypoints/delete", kQueueSize, false);

  // External
  current_waypoint_sub_ =
      nh.subscribe("waypoints/current", kQueueSize,
                   &RvinciGui::onCurrentWaypointChange, this);
  trajectory_duration_sub_ =
      nh.subscribe("trajectory_duration", kQueueSize,
                   &RvinciGui::onTrajectoryDurationChange, this);
  preview_position_sub_ =
      nh.subscribe("preview_position", kQueueSize,
                   &RvinciGui::onPreviewPositionChange, this);
  preview_is_playing_sub_ =
      nh.subscribe("preview_playback_state", kQueueSize,
                   &RvinciGui::onPreviewIsPlayingChange, this);
  is_executable_sub_ = nh.subscribe("plan/is_executable", kQueueSize,
                                    &RvinciGui::onIsExecutableChange, this);
  is_executing_sub_ = nh.subscribe("plan/is_executing", kQueueSize,
                                   &RvinciGui::onIsExecutingChange, this);

  // Internal
  ros::NodeHandle pnh("~");
  play_pause_click_sub_ = pnh.subscribe("rvinci_play_pause", kQueueSize,
                                        &RvinciGui::onPlayPauseClick, this);
  execute_abort_click_sub_ =
      pnh.subscribe("rvinci_execute_abort", kQueueSize,
                    &RvinciGui::onExecuteAbortClick, this);
  add_waypoint_here_click_sub_ =
      pnh.subscribe("rvinci_add_waypoint_here", kQueueSize,
                    &RvinciGui::onAddWaypointHereClick, this);
  add_waypoint_at_end_click_sub_ =
      pnh.subscribe("rvinci_add_waypoint_at_end", kQueueSize,
                    &RvinciGui::onAddWaypointAtEndClick, this);
  delete_waypoint_click_sub_ =
      pnh.subscribe("rvinci_delete_waypoint", kQueueSize,
                    &RvinciGui::onDeleteWaypointClick, this);
}

void RvinciGui::show() {
  if (overlay_) overlay_->show();

  Ogre::Overlay* left_cursor =
      Ogre::OverlayManager::getSingleton().getByName("RVinciCursorLeft");
  if (left_cursor) left_cursor->show();

  Ogre::Overlay* right_cursor =
      Ogre::OverlayManager::getSingleton().getByName("RVinciCursorRight");
  if (right_cursor) right_cursor->show();
}

void RvinciGui::hide() {
  if (overlay_) overlay_->hide();

  Ogre::Overlay* left_cursor =
      Ogre::OverlayManager::getSingleton().getByName("RVinciCursorLeft");
  if (left_cursor) left_cursor->hide();

  Ogre::Overlay* right_cursor =
      Ogre::OverlayManager::getSingleton().getByName("RVinciCursorRight");
  if (right_cursor) right_cursor->hide();
}

void RvinciGui::onPreviewPositionChange(
    const std_msgs::Float64::ConstPtr& msg) {
  position_in_plan_ = msg->data;
  preview_panel_.setScrubberPosition(position_in_plan_);
}

void RvinciGui::onCurrentWaypointChange(
    const nasa_interface_msgs::WaypointStamped::ConstPtr& msg) {
  latest_waypoint_ = msg;
  waypoint_info_panel_.setWaypointName(latest_waypoint_->waypoint.name);

  waypoints_control_panel_.setDeleteEnabled(
      !latest_waypoint_->waypoint.name.empty());
}

void RvinciGui::onPlayPauseClick(const std_msgs::Empty&) {
  std_msgs::Bool msg;
  msg.data = !is_playing_;
  set_preview_playing_pub_.publish(msg);
}

void RvinciGui::onExecuteAbortClick(const std_msgs::Empty&) {
  std_msgs::Bool msg;
  msg.data = !is_executing_;
  set_executing_pub_.publish(msg);
}

void RvinciGui::onPreviewIsPlayingChange(const std_msgs::Bool::ConstPtr& msg) {
  is_playing_ = msg->data;

  preview_panel_.setPreviewButtonPlaying(is_playing_);
}
void RvinciGui::onIsExecutingChange(const std_msgs::Bool::ConstPtr& msg) {
  is_executing_ = msg->data;

  preview_panel_.setExecuteAbortButtonExecuting(is_executing_);
}

void RvinciGui::onAddWaypointHereClick(const std_msgs::Empty&) {
  nasa_interface_msgs::AddWaypointFromPath msg;
  msg.header.stamp = ros::Time::now();
  msg.position_in_path = position_in_plan_;

  add_waypoint_here_pub_.publish(msg);
}

void RvinciGui::onAddWaypointAtEndClick(const std_msgs::Empty&) {
  nasa_interface_msgs::AddWaypoint msg;
  msg.header.stamp = ros::Time::now();
  msg.after_waypoint_id = ""; // Empty string means after last waypoint

  add_waypoint_at_end_pub_.publish(msg);
}

void RvinciGui::onDeleteWaypointClick(const std_msgs::Empty&) {
  nasa_interface_msgs::DeleteWaypoint msg;
  msg.header.stamp = ros::Time::now();
  msg.waypoint_id = latest_waypoint_->waypoint.waypoint_id;

  delete_waypoint_pub_.publish(msg);
}

void RvinciGui::onTrajectoryDurationChange(
    const std_msgs::Float64::ConstPtr& msg) {
  preview_panel_.setPreviewEnabled(msg->data > 0);
}

void RvinciGui::onIsExecutableChange(const std_msgs::Bool::ConstPtr& msg) {
  preview_panel_.setExecuteEnabled(msg->data);
}

void RvinciGui::changeInputScale(Ogre::Vector3 input_scale) {
  preview_panel_.changeInputScale(input_scale);
}

void RvinciGui::setOffset(double offset) {
  if (overlay_) overlay_->setScroll(offset, 0.);

  Ogre::Overlay* left_cursor =
      Ogre::OverlayManager::getSingleton().getByName("RVinciCursorLeft");
  if (left_cursor) left_cursor->setScroll(offset, 0.);

  Ogre::Overlay* right_cursor =
      Ogre::OverlayManager::getSingleton().getByName("RVinciCursorRight");
  if (right_cursor) right_cursor->setScroll(offset, 0.);
}

} // namespace rvinci