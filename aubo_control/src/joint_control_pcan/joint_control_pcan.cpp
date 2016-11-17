#include <ros/ros.h>
#include <aubo_msgs/JointPos.h>
#include <std_msgs/Float32MultiArray.h>
#include "jointcontrolapi.h"

const char* CanDev = "/dev/pcan32";

int can_int_flag = 0;

void chatterCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    ROS_INFO("[%f,%f,%f,%f,%f,%f]",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]);

    double pos[6];
    pos[0] = msg->data[0];
    pos[1] = msg->data[1];
    pos[2] = msg->data[2];
    pos[3] = msg->data[3];
    pos[4] = msg->data[4];
    pos[5] = msg->data[5];

    //use can lib
    if(can_int_flag == 1)
    {
      	set_joint_position(1,pos[0]);
        set_joint_position(2,pos[1]);
        set_joint_position(3,pos[2]);
        set_joint_position(4,pos[3]);
        set_joint_position(5,pos[4]);
        set_joint_position(6,pos[5]);
    }
}

int main(int argc, char **argv)
{
    int ret;
    ros::init(argc, argv, "joint_control_pcan");

    std::string  level;
    int speed_level = 1;

    if (!(ros::param::get("~speed_level", level))) {
        if (argc > 1) {
           level = argv[1];

           if(!level.compare("-S1"))
           {
                ROS_INFO("S1");
                speed_level = 1;
           }
           else if(!level.compare("-S2"))
           {
               ROS_INFO("S2");
                speed_level = 2;
           }
           else if(!level.compare("-S3"))
           {
                ROS_INFO("S3");
                speed_level = 3;
           }
           else
           {
               ROS_WARN("Wrong speed control command line parameter(-S1,-S2 or -S3)");
               exit(1);
           }
        }
    }

    ros::NodeHandle n;
    ros::Rate loop_rate(20);


    if(joint_control_init(CanDev) == 0)
    {
        ROS_INFO("can bus init!");
        can_int_flag = 1;
    }

    if(speed_level == 1)
    {
        set_joint_max_speed(1,1800);
        set_joint_max_speed(2,1800);
        set_joint_max_speed(3,1800);
        set_joint_max_speed(4,1800);
        set_joint_max_speed(5,1800);
        set_joint_max_speed(6,1800);
    }
    else if(speed_level == 2)
    {
        set_joint_max_speed(1,2300);
        set_joint_max_speed(2,2300);
        set_joint_max_speed(3,2300);
        set_joint_max_speed(4,2300);
        set_joint_max_speed(5,2300);
        set_joint_max_speed(6,2300);
    }
    else if(speed_level == 3)
    {
        set_joint_max_speed(1,2800);
        set_joint_max_speed(2,2800);
        set_joint_max_speed(3,2800);
        set_joint_max_speed(4,2800);
        set_joint_max_speed(5,2800);
        set_joint_max_speed(6,2800);
    }

    set_joint_max_acc(1,5000);
    set_joint_max_acc(2,5000);
    set_joint_max_acc(3,5000);
    set_joint_max_acc(4,5000);
    set_joint_max_acc(5,5000);
    set_joint_max_acc(6,5000);

    aubo_msgs::JointPos joint_pos;



    ros::Publisher state_pub = n.advertise<aubo_msgs::JointPos>("current_pos", 1);

    ros::Subscriber cmd_sub = n.subscribe("pcan_cmd", 1000 , chatterCallback);

   
    while(ros::ok())
    {
        if(can_int_flag == 1)
        {
            joint_pos.joint1 = read_joint_position(1);
            joint_pos.joint2 = read_joint_position(2);
            joint_pos.joint3 = read_joint_position(3);
            joint_pos.joint4 = read_joint_position(4);
            joint_pos.joint5 = read_joint_position(5);
            joint_pos.joint6 = read_joint_position(6);

            state_pub.publish(joint_pos);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

