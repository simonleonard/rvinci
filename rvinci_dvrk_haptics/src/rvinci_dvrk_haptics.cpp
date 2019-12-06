#include "rvinci_dvrk_haptics.h"

#include <geometry_msgs/Wrench.h>

namespace rvinci_dvrk_haptics {
namespace {
constexpr int kQueueSize = 10;
}

dvrk_wrench::dvrk_wrench() {

  rvinci_update_sub_ = nh_.subscribe("/rvinci_input_update", kQueueSize,
                                     &dvrk_wrench::inputCallback, this);
  wrench_pub_left_ = nh_.advertise<geometry_msgs::Wrench>(
      "/dvrk/MTML/set_wrench_body", kQueueSize);
  wrench_pub_right_ = nh_.advertise<geometry_msgs::Wrench>(
      "/dvrk/MTMR/set_wrench_body", kQueueSize);
}

void dvrk_wrench::inputCallback(
    const rvinci_input_msg::rvinci_input::ConstPtr& r_input) {
}

} // namespace rvinci_dvrk_haptics