#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <aubo_msgs/JointPos.h>
#include <tf/transform_broadcaster.h>

aubo_msgs::JointPos cur_pos;

void chatterCallback1(const aubo_msgs::JointPos::ConstPtr &msg)
{
   ROS_INFO("[%f,%f,%f,%f,%f,%f]",msg->joint1,msg->joint2,msg->joint3,msg->joint4,msg->joint5,msg->joint6);
   cur_pos.joint1 = msg->joint1;
   cur_pos.joint2 = msg->joint2;
   cur_pos.joint3 = msg->joint3;
   cur_pos.joint4 = msg->joint4;
   cur_pos.joint5 = msg->joint5;
   cur_pos.joint6 = msg->joint6;
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "real_state_rviz");
	ros::NodeHandle n;
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

	ros::Subscriber sub1 = n.subscribe("current_pos", 1000, chatterCallback1);

	tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(10);//

	const double degree = M_PI/180;
	// robot state
    double inc= 0.005, shoulder_inc = 0.003,upper_shoulder_inc = 0.003, fore_upper_inc= 0.003, wrist1_fore_inc= 0.004,wrist2_wrist1_inc= 0.005,wrist3_wrist2_inc= 0.006;
    double angle= 0 ,shoulder = 0, upper_shoulder = 0, fore_upper = 0, wrist1_fore = 0, wrist2_wrist1 = 0, wrist3_wrist2 = 0;

    cur_pos.joint1 = 0;
	cur_pos.joint2 = 0;
	cur_pos.joint3 = 0;
	cur_pos.joint4 = 0;
	cur_pos.joint5 = 0;
	cur_pos.joint6 = 0;
	
	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "base";
	odom_trans.child_frame_id = "base_Link";

	while (ros::ok()) 
    {
		//update joint_state
		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(6);
		joint_state.position.resize(6);
		joint_state.name[0] ="shoulder_joint";
		joint_state.position[0] = cur_pos.joint1;//shoulder;
		joint_state.name[1] ="upperArm_joint";
		joint_state.position[1] = cur_pos.joint2;//upper_shoulder;
		joint_state.name[2] ="foreArm_joint";
		joint_state.position[2] = cur_pos.joint3;//fore_upper;
		joint_state.name[3] ="wrist1_joint";
		joint_state.position[3] = cur_pos.joint4;//wrist1_fore;
		joint_state.name[4] ="wrist2_joint";
		joint_state.position[4] = cur_pos.joint5;//wrist2_wrist1;
		joint_state.name[5] ="wrist3_joint";
		joint_state.position[5] = cur_pos.joint6;//wrist3_wrist2;
		

		// update transform
		// (moving in a circle with radius)
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.transform.translation.x = cos(angle);
		odom_trans.transform.translation.y = sin(angle);
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);
		//send the joint state and transform
        joint_pub.publish(joint_state);
        //broadcaster.sendTransform(odom_trans);

		// Create new robot state
		wrist3_wrist2  += wrist3_wrist2_inc;
		if (wrist3_wrist2 <-3.05 || wrist3_wrist2 >3.05) wrist3_wrist2_inc *= -1;

	
		wrist2_wrist1  += wrist2_wrist1_inc;
		if (wrist2_wrist1 <-1.5 || wrist2_wrist1 >1.5) wrist2_wrist1_inc *= -1;

		wrist1_fore  += wrist1_fore_inc;
		if (wrist1_fore <-1.5 || wrist1_fore >1.5) wrist1_fore_inc *= -1;


		fore_upper  += fore_upper_inc;
		if (fore_upper <-1.5 || fore_upper >1.5) fore_upper_inc *= -1;

		upper_shoulder  += upper_shoulder_inc;
		if (upper_shoulder <-1.5 || upper_shoulder >1.5) upper_shoulder_inc *= -1;

		shoulder  += shoulder_inc;
		if (shoulder <-3.05 || shoulder >3.05) shoulder_inc *= -1;



        //angle += degree/4;


		// This will adjust as needed per iteration
		loop_rate.sleep();
		ros::spinOnce();
	}
    return 0;
}
