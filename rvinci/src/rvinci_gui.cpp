#include "rvinci/rvinci_gui.h"

#include <ros/console.h>

#include <OgreOverlayManager.h>
#include <OgrePanelOverlayElement.h>

namespace rvinci {

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

  overlay_->add2D(bottom_panel_.create());
  overlay_->add2D(top_panel_.create());

  // External
  current_waypoint_sub_ = nh.subscribe(
      "waypoints/current", 1, &RvinciGui::onCurrentWaypointChange, this);
  preview_position_sub_ = nh.subscribe(
      "preview_position", 1, &RvinciGui::onPreviewPositionChange, this);

  // Internal
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
  bottom_panel_.setScrubberPosition(msg->data);
}

void RvinciGui::onCurrentWaypointChange(
    const nasa_interface_msgs::WaypointStamped::ConstPtr& msg) {
  latest_waypoint_ = msg;
  top_panel_.setWaypointName(latest_waypoint_->waypoint.name);
}

} // namespace rvinci