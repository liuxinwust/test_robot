/*##############################################################################################################################################
Note:aubo_driver is a sample of TCP/IP interface for AUBO-i5.
There ara some main API for application as follow,please see our_control_api.h for more information.
##############################################################################################################################################
void init_move_profile();                                //初始化move的属性
void set_scurve(int val);                                //设置S曲线是否有效
void set_tcp(double *tcp_pose,int count);                //设置TCP参数
void set_relative_offset(double *offset,int count);      //设置MOVE的偏移量
void set_wait_move_finish(int val);                      //设置MOVE的偏移量   设置是否等待到位信号  即  阻塞与非阻塞
void set_feature(const char *feature_name);              //设置坐标系
void add_waypoint(const double *pos, int count);         //用于MOVE 中增加路点

int  movej(double *pos, int count, double acc, double velc);
int  movel(double *pos, int count, double acc, double velc);
int  movel_to(double x, double y, double z, double acc, double velc);
int  movep(double acc, double velc,double blend_radius,int track_mode);

int  set_payload(double weight, double *cog, int count);  //设置运行时负载

int    is_exist_current_io    ( our_contorl_io_type  io_type, our_contorl_io_mode io_mode,int io_index);   //判断对应IO是否存在
int    set_robot_one_io_status( our_contorl_io_type  io_type, our_contorl_io_mode io_mode,int io_index, double io_value);  //设置指定IO 的状态
double get_robot_one_io_status( our_contorl_io_type  io_type, our_contorl_io_mode io_mode,int io_index);   //获取指定IO 的状态
##############################################################################################################################################*/


#ifndef AUBO_DRIVER_H_
#define AUBO_DRIVER_H_

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <aubo_msgs/IOState.h>
#include <aubo_msgs/JointPos.h>


class AuboDriver {
private:
	bool reverse_connected_;
        
  	ros::NodeHandle nh;
        ros::Publisher  pos_pub;
	ros::Subscriber sub1;
	ros::Subscriber sub2;
	ros::Subscriber sub3;
	ros::Timer timer;
public:
	
	AuboDriver(std::string host,unsigned int reverse_port = 8887);

	void chatterCallback1(const std_msgs::Float32MultiArray::ConstPtr &msg);
	void chatterCallback2(const std_msgs::Float32MultiArray::ConstPtr &msg);
	void chatterCallback3(const aubo_msgs::IOState::ConstPtr &msg);
        void timerCallback(const ros::TimerEvent& e);

	
};

#endif /* AUBO_DRIVER_H_ */
