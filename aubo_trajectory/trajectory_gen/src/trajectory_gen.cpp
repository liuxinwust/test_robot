#include <moveit/move_group_interface/move_group.h>
#include <math.h>
#include "aubo_msgs/GoalPoint.h"
#include "aubo_msgs/TraPoint.h"

using namespace std;
double last_road_point[6];

std::vector<double> group_variable_values;
moveit::planning_interface::MoveGroup *ptr_group;

ros::Publisher tra_pub;
aubo_msgs::TraPoint tra_point;

int goal_trajectory(double *road)
{
  int point_num;
  /*Planning to a joint-space goal*/ 
  group_variable_values[0] = road[0];
  group_variable_values[1] = road[1];
  group_variable_values[2] = road[2];
  group_variable_values[3] = road[3];
  group_variable_values[4] = road[4];
  group_variable_values[5] = road[5];
  ptr_group->setJointValueTarget(group_variable_values);

  moveit::planning_interface::MoveGroup::Plan my_plan;

  bool success = ptr_group->plan(my_plan);

  ROS_INFO("Visualizing plan 1 (joint space goal) %s",success?"":"FAILED");

  ptr_group->execute(my_plan);

  point_num = my_plan.trajectory_.joint_trajectory.points.size();


  tra_point.num_of_trapoint = point_num;

  tra_point.trapoints.resize(point_num*6);

  for(int j=0; j< point_num; j++)
  {
     for(int i=0; i<6; i++)
     {
          tra_point.trapoints[j*6+i] = my_plan.trajectory_.joint_trajectory.points[j].positions[i];
     }
  }


  //ptr_group->stop();
  ROS_INFO("way_point num:%d",point_num);

  return point_num;
}

int road_point_compare(double *goal)
{ 
  int ret = 0;
  for(int i=0;i<6;i++)
  {
    if(fabs(goal[i]-last_road_point[i])>=0.000001)
	{
       ret = 1;
	   break;
	}    
  }

  if(ret == 1)
  {
    last_road_point[0]= goal[0];
    last_road_point[1]= goal[1];
    last_road_point[2]= goal[2];
    last_road_point[3]= goal[3];
    last_road_point[4]= goal[4];
    last_road_point[5]= goal[5];
  }
  

  return ret;
}
 

void chatterCallback(const aubo_msgs::GoalPoint::ConstPtr &msg)
{
  double road[6];
  int bus;

  bus = msg->bus;

  road[0] = msg->joint1;
  road[1] = msg->joint2;
  road[2] = msg->joint3;
  road[3] = msg->joint4;
  road[4] = msg->joint5;
  road[5] = msg->joint6;

  if(road_point_compare(road))
  {
      goal_trajectory(road);
      tra_point.bus = bus;
      tra_pub.publish(tra_point);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_gen");
  ros::NodeHandle nh; 

  ros::AsyncSpinner spinner(3);
  spinner.start();
	

  ros::Subscriber sub1 = nh.subscribe("goal_pos", 1000, chatterCallback);
  tra_pub = nh.advertise<aubo_msgs::TraPoint> ("tra_point", 1000);
  

  last_road_point[0]= 0;
  last_road_point[1]= 0;
  last_road_point[2]= 0;
  last_road_point[3]= 0;
  last_road_point[4]= 0;
  last_road_point[5]= 0;

  moveit::planning_interface::MoveGroup group("arm_group");
  ptr_group = &group;
  ptr_group->getCurrentState()->copyJointGroupPositions(ptr_group->getCurrentState()->getRobotModel()->getJointModelGroup(ptr_group->getName()), group_variable_values);
  

  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  ros::waitForShutdown(); 
  return 0;

}
