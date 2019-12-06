#include <ros/ros.h>

#include "rvinci_dvrk_haptics.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rvinci_dvrk_haptics");

  rvinci_dvrk_haptics::dvrk_wrench dvrkw;

  ros::spin();
}
