#ifndef AUBO_DRIVER_H_
#define AUBO_DRIVER_H_

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "aubo_msgs/IOState.h"
#include "aubo_msgs/JointPos.h"


class AuboDriver {
private:
	bool reverse_connected_;
        
  	ros::NodeHandle nh;
        ros::Publisher  pos_pub;
	ros::Subscriber sub;
	ros::Subscriber sub2;
	ros::Timer timer;
public:
	
	AuboDriver(std::string host,unsigned int reverse_port = 8887);

	void chatterCallback1(const std_msgs::Float32MultiArray::ConstPtr &msg);
	void chatterCallback2(const aubo_msgs::IOState::ConstPtr &msg);
        void timerCallback(const ros::TimerEvent& e);
};

#endif /* AUBO_DRIVER_H_ */
