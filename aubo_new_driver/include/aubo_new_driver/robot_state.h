#ifndef ROBOT_STATE_H_
#define ROBOT_STATE_H_

#include <inttypes.h>
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <mutex>
#include <netinet/in.h>
#include <condition_variable>

class RobotState{
private:
	std::vector<double> joint_position_actual_; //Actual joint positions
	std::vector<double> joint_velocity_actual_; //Actual joint velocity
	std::vector<double> tool_position_actual_; //Actual Cartesian coordinates of the tool: (x,y,z)
	std::vector<double> tool_orientation_actual_; //Actual Cartesian orientation of the tool: (w,x,y,z)
	std::vector<double> joint_temperatures_actual_; //Temperature of each joint in degrees celsius
	std::vector<double> joint_current_actual_; //Current of each joint
	std::vector<double> joint_voltage_actual_; //Voltage of each joint
	std::vector<double> tcp_force_actual_; //Actual tcp force

	double end_speed_actual_; //Actual speed of the tool given in Cartesian coordinates

	
	std::mutex val_lock_; // Locks the variables while unpack parses data;

	std::condition_variable* pMsg_cond_; //Signals that new vars are available
	bool data_published_; //to avoid spurious wakes
	bool controller_updated_; //to avoid spurious wakes

public:
	RobotState(std::condition_variable& msg_cond);
	~RobotState();

	std::vector<double> getJonitPosition();
	std::vector<double> getJonitVelocity();
	std::vector<double> getTcpForce();
	std::vector<double> getToolPosition();
	std::vector<double> getToolOrientation();
	std::vector<double> getJointTemperatures();
	std::vector<double> getJointCurrent();
	std::vector<double> getJointVoltage();

	double getEndSpeed();

	void setDataPublished();
	bool getDataPublished();
	bool getControllerUpdated();
	void setControllerUpdated();
	void unpack(uint8_t * buf);
};

#endif /* ROBOT_STATE_H_ */
