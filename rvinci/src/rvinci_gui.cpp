#include "rvinci/rvinci_gui.h"

#include <ros/console.h>

#include <OgreOverlayManager.h>
#include <OgrePanelOverlayElement.h>

namespace rvinci {
namespace {
constexpr int kQueueSize = 10;
}

RvinciGui::~RvinciGui() {
  if (overlay_) {
    Ogre::OverlayManager::getSingleton().destroy(overlay_);
  }
};

void RvinciGui::initialize(ros::NodeHandle& nh) {
  if (overlay_) return;

  ROS_INFO("Adding the overlay");

  Ogre::OverlayManager& overlay_manager = Ogre::OverlayManager::getSingleton();

  overlay_ = overlay_manager.create("RVinciInterface");
  // This should be below most things, especially the cursor overlays
  overlay_->setZOrder(0);

  overlay_->add2D(waypoint_info_panel_.create());
  overlay_->add2D(waypoints_control_panel_.create());
  overlay_->add2D(preview_panel_.create());

  // External publishers
  set_preview_playing_pub_ =
      nh.advertise<std_msgs::Bool>("set_preview_playing", kQueueSize, false);
  set_executing_pub_ =
      nh.advertise<std_msgs::Bool>("plan/execute", kQueueSize, false);

  // External
  current_waypoint_sub_ =
      nh.subscribe("waypoints/current", kQueueSize,
                   &RvinciGui::onCurrentWaypointChange, this);
  preview_position_sub_ =
      nh.subscribe("preview_position", kQueueSize,
                   &RvinciGui::onPreviewPositionChange, this);
  preview_is_playing_sub_ =
      nh.subscribe("preview_playback_state", kQueueSize,
                   &RvinciGui::onPreviewIsPlayingChange, this);
  is_executing_sub_ = nh.subscribe("plan/is_executing", kQueueSize,
                                   &RvinciGui::onIsExecutingChange, this);

  // Internal
  ros::NodeHandle pnh("~");
  play_pause_click_sub_ = pnh.subscribe("rvinci_play_pause", kQueueSize,
                                        &RvinciGui::onPlayPauseClick, this);
  execute_abort_click_sub_ =
      pnh.subscribe("rvinci_execute_abort", kQueueSize,
                    &RvinciGui::onExecuteAbortClick, this);
}

void RvinciGui::show() {
  assert(overlay_);
  overlay_->show();
}

void RvinciGui::hide() {
  assert(overlay_);
  overlay_->hide();
}

void RvinciGui::onPreviewPositionChange(
    const std_msgs::Float64::ConstPtr& msg) {
  preview_panel_.setScrubberPosition(msg->data);
}

void RvinciGui::onCurrentWaypointChange(
    const nasa_interface_msgs::WaypointStamped::ConstPtr& msg) {
  latest_waypoint_ = msg;
  waypoint_info_panel_.setWaypointName(latest_waypoint_->waypoint.name);
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

} // namespace rvinci