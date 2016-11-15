#ifndef AUBO_NEW_DRIVER_H_
#define AUBO_NEW_DRIVER_H_

#include <mutex>
#include <string>
#include <vector>
#include <math.h>
#include <chrono>

#include "aubo_realtime_communication.h"
#include "do_output.h"




class AuboNewDriver {
private:
	std::vector<std::string> joint_names_;
	std::string ip_addr_;
	bool reverse_connected_;

	int incoming_sockfd_;
	int new_sockfd_;
	const unsigned int REVERSE_PORT_;
	const int MULT_JOINTSTATE_ = 1000000;
	const int MULT_TIME_ = 1000000;

	double servoj_time_;
	bool executing_traj_;

        
public:
	AuboRealtimeCommunication* rt_interface_;
	AuboNewDriver(std::condition_variable& rt_msg_cond,
			std::condition_variable& msg_cond, 
			std::string host,unsigned int reverse_port = 8899,double servoj_time = 0.020);

	bool start();
	void halt();

	bool doTraj(std::vector<double> inp_timestamps,
			std::vector<std::vector<double> > inp_positions,
			std::vector<std::vector<double> > inp_velocities);


	void setSpeed(double q0, double q1, double q2, double q3, double q4,
        double q5, double acc);

	void servoj(std::vector<double> positions, int keepalive = 1);


	void getRobotPos();

	void stopTraj();

	bool openServo();
	void closeServo(std::vector<double> positions);

	std::vector<double> interp_cubic(double t, double T,
			std::vector<double> p0_pos, std::vector<double> p1_pos,
			std::vector<double> p0_vel, std::vector<double> p1_vel);

	std::vector<std::string> getJointNames();
	void setJointNames(std::vector<std::string> jn);

	void setRobotIO(int type, int mode,int index,float state);

	void setServojTime(double t);


	/************************Aubo plan and move API*****************************/
	void initMoveProfile();
	void setBlock(bool flag);
	void setMaxSpeed(double speed);
	void setMaxAcc(double acc);
	void movej(std::vector<double> positions);
	void movel(std::vector<double> positions);
	void movelTo(std::vector<double> positions);
	void addWayPoint(std::vector<double> positions);
	void movep(double blendRadius,int trackMode);
	/************************Aubo plan and move API*****************************/  
};

#endif /* AUBO_NEW_DRIVER_H_ */
