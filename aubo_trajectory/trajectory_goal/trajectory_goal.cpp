
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "aubo_msgs/GoalPoint.h"
#include "aubo_msgs/TraPoint.h"

aubo_msgs::GoalPoint goal;
ros::Publisher goal_pub;


std_msgs::Float32MultiArray joints_cmd;
ros::Publisher cmd_pub;
ros::Publisher cmd_pub2;

 
aubo_msgs::TraPoint traj_point;

bool recv_flag = false;
ros::Timer timer;

void chatterCallback(const aubo_msgs::TraPoint::ConstPtr &msg)
{
  traj_point.bus = msg->bus;

  traj_point.num_of_trapoint = msg->num_of_trapoint;
  traj_point.trapoints.resize(msg->num_of_trapoint*6);

  for(int i = 0;i<msg->num_of_trapoint;i++)
  {
     for(int j=0;j<6;j++)
     {
       traj_point.trapoints[i*6+j]=msg->trapoints[i*6+j];
     }
  }

  ROS_INFO("point:%d",traj_point.num_of_trapoint);
  recv_flag = true;

  timer.start();
}


void chatterCallback1(const aubo_msgs::GoalPoint::ConstPtr &msg)
{
    //ROS_INFO("goal=[%f,%f,%f,%f,%f,%f]",msg->joint1,msg->joint2,msg->joint3,msg->joint4,msg->joint5,msg->joint6);

    goal.bus = msg->bus;
    goal.joint1 = msg->joint1;
    goal.joint2 = msg->joint2;
    goal.joint3 = msg->joint3;
    goal.joint4 = msg->joint4;
    goal.joint5 = msg->joint5;
    goal.joint6 = msg->joint6;
    goal_pub.publish(goal);
}


void timerCallback(const ros::TimerEvent& e)
{
    static int i=0;

    if(recv_flag == true)
    { 
         if((i>=traj_point.num_of_trapoint)&&(traj_point.num_of_trapoint>0))
         {
            i = 0;
            traj_point.num_of_trapoint = 0;
            recv_flag = false;
            timer.stop();
         }

         
         if(recv_flag == true)
         {
           for(int j=0;j<6;j++)
           {
            joints_cmd.data[j] = traj_point.trapoints[i*6+j];
           }

           if(traj_point.bus == 0)
           {
               cmd_pub.publish(joints_cmd);
               //ROS_INFO("movej:%f,%f,%f,%f,%f,%f",joints_cmd.data[0],joints_cmd.data[1],joints_cmd.data[2],joints_cmd.data[3],joints_cmd.data[4],joints_cmd.data[5]);

           }
           else if(traj_point.bus == 1 )
           {
               cmd_pub2.publish(joints_cmd);
               //ROS_INFO("servoj:%f,%f,%f,%f,%f,%f",joints_cmd.data[0],joints_cmd.data[1],joints_cmd.data[2],joints_cmd.data[3],joints_cmd.data[4],joints_cmd.data[5]);
           }

           i++;
         }  
   }
   else
   {
      timer.stop();
   }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_goal");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(6);
    spinner.start();
 
    goal_pub = nh.advertise<aubo_msgs::GoalPoint> ("goal_pos", 1);

    cmd_pub = nh.advertise<std_msgs::Float32MultiArray> ("pcan_cmd", 1);
    cmd_pub2 = nh.advertise<std_msgs::Float32MultiArray> ("servoj_cmd", 1);

    ros::Subscriber sub = nh.subscribe("tra_point", 1000, chatterCallback);
    ros::Subscriber sub1 = nh.subscribe("send_goal", 1000, chatterCallback1);

    timer = nh.createTimer(ros::Duration(0.01),timerCallback);
    timer.stop();
  
    joints_cmd.data.resize(6);

    sleep(2.0);

    ros::waitForShutdown(); 

    return 0;
}

