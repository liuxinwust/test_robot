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

aubo_msgs::TraPoint interp_one_point(aubo_msgs::TraPoint &recv_points)
{
    aubo_msgs::TraPoint traj_temp;
    traj_temp.bus = recv_points.bus;
    traj_temp.num_of_trapoint = recv_points.num_of_trapoint*2-1;
    traj_temp.trapoints.resize(traj_temp.num_of_trapoint*6 );

    for(int i = 0; i< 6;i++)
    {
       traj_temp.trapoints[i]=recv_points.trapoints[i];
    }

    for(int i = 1;i<traj_temp.num_of_trapoint;i++)
    {
       if((i%2) == 0)
       {
          for(int j = 0; j< 6;j++)
          {
            traj_temp.trapoints[i*6+j]=recv_points.trapoints[(i/2)*6+j];
          }
       }
       else if((i%2) == 1)
       {
          for(int k = 0; k< 6;k++)
          {
             traj_temp.trapoints[i*6+k]= (recv_points.trapoints[(i/2)*6+k] + recv_points.trapoints[(i/2+1)*6+k])/2;
          }
       }
    }

    return traj_temp;
}

void chatterCallback(const aubo_msgs::TraPoint::ConstPtr &msg)
{
  int level = 0;
  aubo_msgs::TraPoint trapoint_recv;
  trapoint_recv.bus = msg->bus;

  trapoint_recv.num_of_trapoint = msg->num_of_trapoint;
  trapoint_recv.trapoints.resize(msg->num_of_trapoint*6);

  for(int i = 0;i<msg->num_of_trapoint;i++)
  {
     for(int j=0;j<6;j++)
     {
       trapoint_recv.trapoints[i*6+j]=msg->trapoints[i*6+j];
     }
  }


  if(msg->num_of_trapoint < 50)
  {
      level = 3;
  }
  else if(msg->num_of_trapoint < 80)
  {
      level = 2;
  }
  else if(msg->num_of_trapoint < 100)
  {
      level = 1;
  }
  else
  {
      level = 0;
  }


  if(level == 0)  traj_point = trapoint_recv;
  else if(level >=1)
  {
     traj_point = interp_one_point(trapoint_recv);

     if(level >= 2)
     {
       trapoint_recv = interp_one_point(traj_point);
       traj_point = trapoint_recv;
     }

     if(level >= 3)
     {
       trapoint_recv = interp_one_point(traj_point);
       traj_point = trapoint_recv;
     }
  }

  ROS_INFO("point:%d",traj_point.num_of_trapoint);
  recv_flag = true;

  timer.start();
}


void chatterCallback1(const aubo_msgs::GoalPoint::ConstPtr &msg)
{
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

    timer = nh.createTimer(ros::Duration(0.008),timerCallback);
    timer.stop();
  
    joints_cmd.data.resize(6);

    sleep(2.0);

    ros::waitForShutdown(); 

    return 0;
}

