#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"

std_msgs::Float32MultiArray joints_cmd;
ros::Publisher cmd_pub;
 

void chatterCallback1(const sensor_msgs::JointState::ConstPtr &msg)
{
  ROS_INFO("Callback1[%f,%f,%f,%f,%f,%f]",msg->position[0],msg->position[1],msg->position[2],msg->position[3],msg->position[4],msg->position[5]);
  joints_cmd.data[0] = msg->position[0];
  joints_cmd.data[1] = msg->position[1];
  joints_cmd.data[2] = msg->position[2];
  joints_cmd.data[3] = msg->position[3];
  joints_cmd.data[4] = msg->position[4];
  joints_cmd.data[5] = msg->position[5];
  cmd_pub.publish(joints_cmd);
}



int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "joint_state_sub");

  ros::NodeHandle nh;

  sleep(3.0);

  ros::Subscriber sub1 = nh.subscribe("joint_states", 1000, chatterCallback1);
  cmd_pub = nh.advertise<std_msgs::Float32MultiArray> ("movej_cmd", 1);

 
  joints_cmd.data.resize(6);

  ros::spin();

  return 0;
}
