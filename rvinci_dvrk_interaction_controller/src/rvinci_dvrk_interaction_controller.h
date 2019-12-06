#ifndef DVRK_CAMERA_MODE_SRC_DVRK_CAMERA_MODE_H_
#define DVRK_CAMERA_MODE_SRC_DVRK_CAMERA_MODE_H_

class dvrk_wrench {
public:
  dvrk_wrench();
  void inputCallback(const rvinci_input_msg::rvinci_input::ConstPtr& r_input);
  double vector_magnitude_;
  tf::Vector3 ori_magnitude_[2];
  ros::Rate* r_;

private:
  ros::NodeHandle n;
  ros::Subscriber rvinci_sub;
  ros::Publisher dvrk_pub[2];
  double strength_[2]; // current and previous
  double torqmag_[2];
  double torqmago_[2];
  void publishWrench(tf::Vector3, double, tf::Transform[]);
  tf::Vector3 getVector(rvinci_input_msg::rvinci_input);
};


#endif // DVRK_CAMERA_MODE_SRC_DVRK_CAMERA_MODE_H_
