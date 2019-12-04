//
// Created by will on 11/19/19.
//

#ifndef RVINCI_INCLUDE_RVINCI_RVINCI_GUI_H_
#define RVINCI_INCLUDE_RVINCI_RVINCI_GUI_H_

// ROS
#include <ros/ros.h>

// Messages
#include <geometry_msgs/Point.h>
#include <nasa_interface_msgs/WaypointStamped.h>
#include <std_msgs/Float64.h>

// Local
#include <rvinci/gui_elements/bottom_panel.h>
#include <rvinci/gui_elements/top_panel.h>

namespace Ogre {
class Overlay;
}

namespace rvinci {

class RvinciGui {
public:
  ~RvinciGui();

  void initialize(ros::NodeHandle& nh);

  void show();
  void hide();

private:
  Ogre::Overlay* overlay_ = nullptr;

  gui_elements::BottomPanel bottom_panel_{};
  gui_elements::TopPanel top_panel_{};

  // External subscribers
  ros::Subscriber current_waypoint_sub_;
  ros::Subscriber preview_position_sub_;

  nasa_interface_msgs::WaypointStamped::ConstPtr latest_waypoint_;

  // Internal subscribers (from interaction_cursor_rviz)

  // External callbacks
  void onPreviewPositionChange(const std_msgs::Float64::ConstPtr& msg);
  void onCurrentWaypointChange(
      const nasa_interface_msgs::WaypointStamped::ConstPtr& msg);

  // Internal callbacks (from interaction_cursor_rviz)
};

} // namespace rvinci

#endif // RVINCI_INCLUDE_RVINCI_RVINCI_GUI_H_
