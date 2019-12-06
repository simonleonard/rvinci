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
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

// Local
#include <rvinci/gui_elements/waypoint_info_panel.h>
#include <rvinci/gui_elements/waypoints_control_panel.h>
#include <rvinci/gui_elements/preview_panel.h>

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

  gui_elements::WaypointInfoPanel waypoint_info_panel_{};
  gui_elements::WaypointsControlPanel waypoints_control_panel_{};
  gui_elements::PreviewPanel preview_panel_{};

  nasa_interface_msgs::WaypointStamped::ConstPtr latest_waypoint_;
  bool is_playing_ = false;
  bool is_executing_ = false;
  double position_in_plan_ = 0.0;

  // External publishers
  ros::Publisher set_preview_playing_pub_;
  ros::Publisher set_executing_pub_;
  ros::Publisher add_waypoint_here_pub_;
  ros::Publisher add_waypoint_at_end_pub_;

  // External subscribers
  ros::Subscriber current_waypoint_sub_;
  ros::Subscriber preview_position_sub_;
  ros::Subscriber preview_is_playing_sub_;
  ros::Subscriber is_executing_sub_;

  // Internal subscribers (from interaction_cursor_rviz)
  ros::Subscriber play_pause_click_sub_;
  ros::Subscriber execute_abort_click_sub_;
  ros::Subscriber add_waypoint_here_click_sub_;
  ros::Subscriber add_waypoint_at_end_click_sub_;


  // External callbacks
  void onCurrentWaypointChange(
      const nasa_interface_msgs::WaypointStamped::ConstPtr& msg);
  void onPreviewPositionChange(const std_msgs::Float64::ConstPtr& msg);
  void onPreviewIsPlayingChange(const std_msgs::Bool::ConstPtr& msg);
  void onIsExecutingChange(const std_msgs::Bool::ConstPtr& msg);

  // Internal callbacks (from interaction_cursor_rviz)
  void onPlayPauseClick(const std_msgs::Empty&);
  void onExecuteAbortClick(const std_msgs::Empty&);
  void onAddWaypointHereClick(const std_msgs::Empty&);
  void onAddWaypointAtEndClick(const std_msgs::Empty&);
};

} // namespace rvinci

#endif // RVINCI_INCLUDE_RVINCI_RVINCI_GUI_H_
