#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <rvinci_input_msg/rvinci_input.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

class davinci_mtm
{
public:
    davinci_mtm();
  ros::Publisher  rvinci_pub_;
  rvinci_input_msg::rvinci_input rvmsg_;
  void getPose(const geometry_msgs::PoseStamped::ConstPtr&, int);
  void gripCallback(const std_msgs::Bool::ConstPtr& grip, int);
  void getaGrip();
  void cameraCallback(const std_msgs::Bool::ConstPtr& cam);
  void clutchCallback(const std_msgs::Bool::ConstPtr& cltch);

  void cameraTestCallback(const sensor_msgs::Joy::ConstPtr& cam);
  void clutchTestCallback(const sensor_msgs::Joy::ConstPtr& cltch);
private:

  ros::NodeHandle n;
  ros::Subscriber left_pose_sub_;
  ros::Subscriber right_pose_sub_;
  ros::Subscriber left_grip_sub_;
  ros::Subscriber right_grip_sub_;
  ros::Subscriber camera_sub_;
  ros::Subscriber clutch_sub_;

  ros::Subscriber camera_sub_test;
  ros::Subscriber clutch_sub_test;

};

davinci_mtm::davinci_mtm()
{
  rvmsg_.header.frame_id = "/base_link";
  left_grip_sub_ = n.subscribe<std_msgs::Bool>("/dvrk/MTML/gripper_closed_event",10,boost::bind(&davinci_mtm::gripCallback,this,_1,0));
  right_grip_sub_ = n.subscribe<std_msgs::Bool>("/dvrk/MTMR/gripper_closed_event",10,boost::bind(&davinci_mtm::gripCallback,this,_1,1));
  left_pose_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/dvrk/MTML/position_cartesian_current",10,boost::bind(&davinci_mtm::getPose,this,_1,0));
  right_pose_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/dvrk/MTMR/position_cartesian_current",10,boost::bind(&davinci_mtm::getPose,this,_1,1));
  rvinci_pub_ = n.advertise<rvinci_input_msg::rvinci_input>("/rvinci_input_update",10);

  //camera_sub_ = n.subscribe<std_msgs::Bool>("/dvrk/footpedals/camera",10,&davinci_mtm::cameraCallback,this);
  //clutch_sub_ = n.subscribe<std_msgs::Bool>("/dvrk/footpedals/clutch",10,&davinci_mtm::clutchCallback,this);
  
  camera_sub_test = n.subscribe<sensor_msgs::Joy>("/dvrk/footpedals/camera",10,&davinci_mtm::cameraTestCallback,this);
  clutch_sub_ = n.subscribe<sensor_msgs::Joy>("/dvrk/footpedals/clutch",10,&davinci_mtm::clutchTestCallback,this);
}
//constructor creates marker that is used as a visualization for the 3D cursor and establishes
//pubs and subs for controller positon and button states.

void davinci_mtm::getPose(const geometry_msgs::PoseStamped::ConstPtr& pose, int i)
{
  rvmsg_.gripper[i].pose = *pose;
}

void davinci_mtm::gripCallback(const std_msgs::Bool::ConstPtr& grab, int i)
{
 rvmsg_.gripper[i].grab = grab->data;
}
/*
void davinci_mtm::cameraCallback(const std_msgs::Bool::ConstPtr& cam)
{
  rvmsg_.camera = cam->data;
  std::cout << "Camera" << std::endl;
}*/

void davinci_mtm::cameraTestCallback(const sensor_msgs::Joy::ConstPtr& cam)
{
  //rvmsg_.camera = cam->buttons;
  int tmp = cam->buttons[0];
  if(tmp != 0)
    rvmsg_.camera = 1;
  else
    rvmsg_.camera = 0;
}

/*void davinci_mtm::clutchCallback(const std_msgs::Bool::ConstPtr& cltch)
{
  rvmsg_.clutch = cltch->data;
} */

void davinci_mtm::clutchTestCallback(const sensor_msgs::Joy::ConstPtr& cltch)
{
  //rvmsg_.clutch = cltch->buttons;
  int tmp = cltch->buttons[0];
  if(tmp != 0)
    rvmsg_.clutch = true;
  else
    rvmsg_.clutch = false;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "dvrk_to_rvinci");
  davinci_mtm mtmlr;
  ros::Rate r(200);
  
  while(ros::ok())
    {
      ros::spinOnce();
      mtmlr.rvmsg_.header.stamp = ros::Time::now();
      mtmlr.rvinci_pub_.publish(mtmlr.rvmsg_);
      r.sleep();
    }
}

