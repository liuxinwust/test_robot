#include "aubo_new_driver/robot_state.h"
#include "aubo_new_driver/do_output.h"


RobotState::RobotState(std::condition_variable& msg_cond) {
    joint_position_actual_.assign(6, 0.0);
    joint_velocity_actual_.assign(6, 0.0);
    tool_position_actual_.assign(3, 0.0);
    tool_orientation_actual_.assign(4, 0.0);
    joint_temperatures_actual_.assign(6, 0.0);
    joint_current_actual_.assign(6, 0.0);
    joint_voltage_actual_.assign(6, 0.0);
    tcp_force_actual_.assign(6, 0.0);
    end_speed_actual_= 0.0;
	data_published_ = false;
	controller_updated_ = false;
	pMsg_cond_ = &msg_cond;
}

RobotState::~RobotState() {
	/* Make sure nobody is waiting after this thread is destroyed */
	data_published_ = true;
	controller_updated_ = true;
	pMsg_cond_->notify_all();
}

void RobotState::setDataPublished() {
	data_published_ = false;
}
bool RobotState::getDataPublished() {
	return data_published_;
}

void RobotState::setControllerUpdated() {
	controller_updated_ = false;
}
bool RobotState::getControllerUpdated() {
	return controller_updated_;
}

std::vector<double> RobotState::getJonitPosition() {
	std::vector<double> ret;
	val_lock_.lock();
    ret = joint_position_actual_;
	val_lock_.unlock();
	return ret;
}


std::vector<double> RobotState::getJonitVelocity() {
    std::vector<double> ret;
    val_lock_.lock();
    ret = joint_velocity_actual_;
    val_lock_.unlock();
    return ret;
}

std::vector<double> RobotState::getToolPosition() {
	std::vector<double> ret;
	val_lock_.lock();
    ret = tool_position_actual_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotState::getToolOrientation() {
	std::vector<double> ret;
	val_lock_.lock();
    ret = tool_orientation_actual_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotState::getJointTemperatures() {
	std::vector<double> ret;
	val_lock_.lock();
    ret = joint_temperatures_actual_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotState::getJointCurrent() {
	std::vector<double> ret;
	val_lock_.lock();
    ret = joint_current_actual_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotState::getJointVoltage() {
	std::vector<double> ret;
	val_lock_.lock();
    ret = joint_voltage_actual_;
	val_lock_.unlock();
	return ret;
}


std::vector<double> RobotState::getTcpForce() {
    std::vector<double> ret;
    val_lock_.lock();
    ret = tcp_force_actual_;
    val_lock_.unlock();
    return ret;
}

double RobotState::getEndSpeed() {
	double ret;
	val_lock_.lock();
    ret = end_speed_actual_;
	val_lock_.unlock();
	return ret;
}

void RobotState::unpack(uint8_t * buf) {

	val_lock_.lock();

    char *rec = (char*)buf;

    std::string response =(std::string)rec;

    //print_info(response);

    int pos = response.find("getRobotPos",0);

    if((pos != -1)&&(pos < 4096))
    {
        joint_position_actual_.clear();
        pos = response.find("\"joint1\":",0);
        double data = atof(&rec[pos+9]);
        joint_position_actual_.push_back(data);

        pos = response.find("\"joint2\":",0);
        data = atof(&rec[pos+9]);
        joint_position_actual_.push_back(data);

        pos = response.find("\"joint3\":",0);
        data = atof(&rec[pos+9]);
        joint_position_actual_.push_back(data);

        pos = response.find("\"joint4\":",0);
        data = atof(&rec[pos+9]);
        joint_position_actual_.push_back(data);

        pos = response.find("\"joint5\":",0);
        data = atof(&rec[pos+9]);
        joint_position_actual_.push_back(data);

        pos = response.find("\"joint6\":",0);
        data = atof(&rec[pos+9]);
        joint_position_actual_.push_back(data);


        tool_position_actual_.clear();
        pos = response.find("\"X\":",0);
        data = atof(&rec[pos+4]);
        tool_position_actual_.push_back(data);

        pos = response.find("\"Y\":",0);
        data = atof(&rec[pos+4]);
        tool_position_actual_.push_back(data);

        pos = response.find("\"Z\":",0);
        data = atof(&rec[pos+4]);
        tool_position_actual_.push_back(data);


        tool_orientation_actual_.clear();
        pos = response.find("\"pose_w\":",0);
        data = atof(&rec[pos+9]);
        tool_orientation_actual_.push_back(data);

        pos = response.find("\"pose_x\":",0);
        data = atof(&rec[pos+9]);
        tool_orientation_actual_.push_back(data);

        pos = response.find("\"pose_y\":",0);
        data = atof(&rec[pos+9]);
        tool_orientation_actual_.push_back(data);

        pos = response.find("\"pose_z\":",0);
        data = atof(&rec[pos+9]);
        tool_orientation_actual_.push_back(data);
    }


	val_lock_.unlock();
	controller_updated_ = true;
	data_published_ = true;
	pMsg_cond_->notify_all();
}

