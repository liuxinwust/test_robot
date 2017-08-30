#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <aubo_msgs/GoalPoint.h>

using namespace std;
int new_goal = 0;
aubo_msgs::GoalPoint goalPos;
std_msgs::Float32MultiArray curPos;

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;
  
class RobotArm
{
    private:
     // Action client for the joint trajectory action 
     // used to trigger the arm movement action
     TrajClient* traj_client_;
   
   public:
     //! Initialize the action client and wait for action server to come up
     RobotArm() 
     {
       // tell the action client that we want to spin a thread by default
       traj_client_ = new TrajClient("follow_joint_trajectory", true);
   
       // wait for action server to come up
       while(!traj_client_->waitForServer(ros::Duration(5.0))){
         ROS_INFO("Waiting for the follow_joint_trajectory server");
       }
       
     }
   
     //! Clean up the action client
     ~RobotArm()
     {
       delete traj_client_;
     }
   
     //! Sends the command to start a given trajectory
     void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
     {
       traj_client_->sendGoal(goal);
       traj_client_->waitForResult();
     }

     control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory(aubo_msgs::GoalPoint &waypoints)
     {
         //our goal variable
         control_msgs::FollowJointTrajectoryGoal goal;

         // First, the joint names, which apply to all waypoints
         goal.trajectory.joint_names.push_back("shoulder_joint");
         goal.trajectory.joint_names.push_back("upperArm_joint");
         goal.trajectory.joint_names.push_back("foreArm_joint");
         goal.trajectory.joint_names.push_back("wrist1_joint");
         goal.trajectory.joint_names.push_back("wrist2_joint");
         goal.trajectory.joint_names.push_back("wrist3_joint");


         // We will have 2 waypoints in this goal trajectory
         goal.trajectory.points.resize(2);

         // Positions
         goal.trajectory.points[0].positions.resize(6);
         goal.trajectory.points[0].positions[0] = curPos.data[0];
         goal.trajectory.points[0].positions[1] = curPos.data[1];
         goal.trajectory.points[0].positions[2] = curPos.data[2];
         goal.trajectory.points[0].positions[3] = curPos.data[3];
         goal.trajectory.points[0].positions[4] = curPos.data[4];
         goal.trajectory.points[0].positions[5] = curPos.data[5];

         // Velocities
         goal.trajectory.points[0].velocities.resize(6);
         for (size_t j = 0; j < 6; ++j)
         {
           goal.trajectory.points[0].velocities[j] = 0.0;
         }


         // acceleration
         goal.trajectory.points[0].accelerations.resize(6);
         for (size_t j = 0; j < 6; ++j)
         {
           goal.trajectory.points[0].accelerations[j] = 0.0;
         }

         // To be reached 1 second after starting along the trajectory
         goal.trajectory.points[0].time_from_start = ros::Duration(0.0);



         // Second trajectory point
         // Positions
         goal.trajectory.points[1].positions.resize(6);

         goal.trajectory.points[1].positions[0] = waypoints.joint1;
         goal.trajectory.points[1].positions[1] = waypoints.joint2;
         goal.trajectory.points[1].positions[2] = waypoints.joint3;
         goal.trajectory.points[1].positions[3] = waypoints.joint4;
         goal.trajectory.points[1].positions[4] = waypoints.joint5;
         goal.trajectory.points[1].positions[5] = waypoints.joint6;


         // Velocities
         goal.trajectory.points[1].velocities.resize(6);
         for (size_t j = 0; j < 6; ++j)
         {
           goal.trajectory.points[1].velocities[j] = 0.0;
         }

         // acceleration
         goal.trajectory.points[1].accelerations.resize(6);
         for (size_t j = 0; j < 6; ++j)
         {
           goal.trajectory.points[1].accelerations[j] = 0.0;
         }

         // To be reached 2 seconds after starting along the trajectory
         goal.trajectory.points[1].time_from_start = ros::Duration(2.5);


         return goal;

     }

  
    //! Returns the current state of the action
    actionlib::SimpleClientGoalState getState()
    {
      return traj_client_->getState();
    }

  };


  void chatterCallback(const aubo_msgs::GoalPoint::ConstPtr &msg)
  {
    new_goal = 0;

    goalPos.bus =msg->bus;
    goalPos.joint1 = msg->joint1;
    goalPos.joint2 = msg->joint2;
    goalPos.joint3 = msg->joint3;
    goalPos.joint4 = msg->joint4;
    goalPos.joint5 = msg->joint5;
    goalPos.joint6 = msg->joint6;
    new_goal = 1;
    ROS_INFO("Goal");
  }


  void chatterCallback1(const sensor_msgs::JointState::ConstPtr &msg)
  {
    //ROS_INFO("Callback1[%f,%f,%f,%f,%f,%f]",msg->position[0],msg->position[1],msg->position[2],msg->position[3],msg->position[4],msg->position[5]);
    curPos.data[0] = msg->position[0];
    curPos.data[1] = msg->position[1];
    curPos.data[2] = msg->position[2];
    curPos.data[3] = msg->position[3];
    curPos.data[4] = msg->position[4];
    curPos.data[5] = msg->position[5];
  }
  



  int main(int argc, char** argv)
  {
    // Init the ROS node
    ros::init(argc, argv, "move_test");
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);//Hz

    RobotArm arm;

    curPos.data.resize(6);

    ros::Subscriber sub = nh.subscribe("send_goal", 1000, chatterCallback);
    ros::Subscriber sub1 = nh.subscribe("joint_states", 1000, chatterCallback1);

    // Wait for trajectory completion
    while(ros::ok())
    {
        if(new_goal == 1)
        {
            new_goal = 0;
            arm.startTrajectory(arm.armExtensionTrajectory(goalPos));
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
  }
