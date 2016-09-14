#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <moveit/move_group_interface/move_group.h>

std::vector<double> group_variable_values;
moveit::planning_interface::MoveGroup *ptr_group;

void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{ 

  ROS_INFO("[%f,%f,%f,%f,%f,%f]",
           msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]);
  for(int i=0; i<6; i++)
  {
    group_variable_values[i] = msg->data[i];
  }

  ptr_group->setJointValueTarget(group_variable_values);
  ptr_group->asyncMove();

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "sub_move_to_rviz");

  moveit::planning_interface::MoveGroup group("arm_group");
  ptr_group = &group;
  ptr_group->getCurrentState()->copyJointGroupPositions(ptr_group->getCurrentState()->getRobotModel()->getJointModelGroup(ptr_group->getName()), group_variable_values);

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("movej_cmd", 1000, chatterCallback);


  ros::spin(); 

  return 0;
}
