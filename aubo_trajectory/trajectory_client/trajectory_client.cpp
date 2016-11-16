#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;
int new_goal = 0;
std_msgs::Float32MultiArray goal;
double last_way_point[6];
 
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
       traj_client_ = new TrajClient("arm_controller/follow_joint_trajectory", true);
   
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
       // When to start the trajectory: 1s from now
       goal.trajectory.header.frame_id = "base_Link";  
       goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
       traj_client_->sendGoal(goal);
     }

     control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory(std_msgs::Float32MultiArray &waypoints)
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

         // We will have 1 waypoints in this goal trajectory
         goal.trajectory.points.resize(1);


         // Positions
         goal.trajectory.points[0].positions.resize(6);
         goal.trajectory.points[0].positions[0] = waypoints.data[0];
         goal.trajectory.points[0].positions[1] = waypoints.data[1];
         goal.trajectory.points[0].positions[2] = waypoints.data[2];
         goal.trajectory.points[0].positions[3] = waypoints.data[3];
         goal.trajectory.points[0].positions[4] = waypoints.data[4];
         goal.trajectory.points[0].positions[5] = waypoints.data[5];

         // Velocities
         goal.trajectory.points[0].velocities.resize(6);
         for (size_t j = 0; j < 6; ++j)
         {
           goal.trajectory.points[0].velocities[j] = 0.001;
         }


         // acceleration
         goal.trajectory.points[0].accelerations.resize(6);
         for (size_t j = 0; j < 6; ++j)
         {
           goal.trajectory.points[0].accelerations[j] = 0.0;
         }

         // To be reached 1 second after starting along the trajectory
         goal.trajectory.points[0].time_from_start = ros::Duration(1);

         return goal;

     }

   
     //! Generates a simple trajectory with two waypoints, used as an example
     /*! Note that this trajectory contains two waypoints, joined together
         as a single trajectory. Alternatively, each of these waypoints could
         be in its own trajectory - a trajectory can have one or more waypoints
         depending on the desired application.
     */
     control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
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


       // We will have 1 waypoints in this goal trajectory
       goal.trajectory.points.resize(2);

   
       // First trajectory point
       // Positions
       int ind = 0;
       goal.trajectory.points[ind].positions.resize(6);
       goal.trajectory.points[ind].positions[0] = 0;
       goal.trajectory.points[ind].positions[1] = 0;
       goal.trajectory.points[ind].positions[2] = 0;
       goal.trajectory.points[ind].positions[3] = 0;
       goal.trajectory.points[ind].positions[4] = 0;
       goal.trajectory.points[ind].positions[5] = 0;

       // Velocities
       goal.trajectory.points[ind].velocities.resize(6);
       for (size_t j = 0; j < 6; ++j)
       {
         goal.trajectory.points[ind].velocities[j] = 0.0;
       }


       // acceleration
       goal.trajectory.points[ind].accelerations.resize(6);
       for (size_t j = 0; j < 6; ++j)
       {
         goal.trajectory.points[ind].accelerations[j] = 0.0;
       }


       // To be reached 1 second after starting along the trajectory
       goal.trajectory.points[ind].time_from_start = ros::Duration(1);


       // Second trajectory point
       // Positions
       ind += 1;
       goal.trajectory.points[ind].positions.resize(6);

       goal.trajectory.points[ind].positions[0] = -2.33;
       goal.trajectory.points[ind].positions[1] = 1.38;
       goal.trajectory.points[ind].positions[2] = 0.97;
       goal.trajectory.points[ind].positions[3] = 0.43;
       goal.trajectory.points[ind].positions[4] = -1.01;
       goal.trajectory.points[ind].positions[5] = 0.01;

       last_way_point[0] = -2.33;
       last_way_point[1] = 1.38;
       last_way_point[2] = 0.97;
       last_way_point[3] = 0.43;
       last_way_point[4] = -1.01;
       last_way_point[5] = 0.01;

  
       // Velocities
       goal.trajectory.points[ind].velocities.resize(6);
       for (size_t j = 0; j < 6; ++j)
       {
         goal.trajectory.points[ind].velocities[j] = 0.0;
       }

       // acceleration
       goal.trajectory.points[ind].accelerations.resize(6);
       for (size_t j = 0; j < 6; ++j)
       {
         goal.trajectory.points[ind].accelerations[j] = 0.0;
       }

       // To be reached 2 seconds after starting along the trajectory
       goal.trajectory.points[ind].time_from_start = ros::Duration(2);

       //we are done; return the goal
       return goal;
    }
  
    //! Returns the current state of the action
    actionlib::SimpleClientGoalState getState()
    {
      return traj_client_->getState();
    }
  };

 int road_point_compare(double *goal)
 {
  int ret = 0;
  for(int i=0;i<6;i++)
  {
    if(fabs(goal[i]-last_way_point[i])>=0.000001)
    {
       ret = 1;
       break;
    }
  }

  if(ret == 1)
  {
    last_way_point[0]= goal[0];
    last_way_point[1]= goal[1];
    last_way_point[2]= goal[2];
    last_way_point[3]= goal[3];
    last_way_point[4]= goal[4];
    last_way_point[5]= goal[5];
  }


  return ret;
 }

  void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
  {
    double road[6];
    new_goal = 0;
    road[0] = msg->data[0];
    road[1] = msg->data[1];
    road[2] = msg->data[2];
    road[3] = msg->data[3];
    road[4] = msg->data[4];
    road[5] = msg->data[5];

    if(road_point_compare(road))
    {
        goal.data[0] = road[0];
        goal.data[1] = road[1];
        goal.data[2] = road[2];
        goal.data[3] = road[3];
        goal.data[4] = road[4];
        goal.data[5] = road[5];
        new_goal = 1;
    }
  }
  
  int main(int argc, char** argv)
  {
    // Init the ROS node
    ros::init(argc, argv, "trajectory_client");
    ros::NodeHandle nh;

    ros::Rate loop_rate(20);//Hz

    RobotArm arm;

    goal.data.resize(6);

    ros::Subscriber sub = nh.subscribe("send_goal", 1000, chatterCallback);
    arm.startTrajectory(arm.armExtensionTrajectory());

    // Wait for trajectory completion
    while(ros::ok())
    {
        if(new_goal)
        {
            new_goal = 0;
            if(arm.getState().isDone())
            {
                arm.startTrajectory(arm.armExtensionTrajectory(goal));
            }
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
  }
