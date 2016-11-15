#include "aubo_new_driver/robot_state.h"
#include "aubo_new_driver/do_output.h"

RobotStateRT::RobotStateRT(std::condition_variable& msg_cond) {
	q_target_.assign(6, 0.0);
	qd_target_.assign(6, 0.0);
	i_target_.assign(6, 0.0);
	m_target_.assign(6, 0.0);
	q_actual_.assign(6, 0.0);
	qd_actual_.assign(6, 0.0);
	i_actual_.assign(6, 0.0);
	tool_vector_actual_.assign(6, 0.0);
	tcp_speed_actual_.assign(6, 0.0);
	tcp_force_.assign(6, 0.0);
	tool_vector_target_.assign(6, 0.0);
	tcp_speed_target_.assign(6, 0.0);
	digital_input_bits_.assign(64, false);
	motor_temperatures_.assign(6, 0.0);
	v_robot_ = 0.0;
	i_robot_ = 0.0;
	v_actual_.assign(6, 0.0);
	data_published_ = false;
	controller_updated_ = false;
	pMsg_cond_ = &msg_cond;
}

RobotStateRT::~RobotStateRT() {
	/* Make sure nobody is waiting after this thread is destroyed */
	data_published_ = true;
	controller_updated_ = true;
	pMsg_cond_->notify_all();
}

void RobotStateRT::setDataPublished() {
	data_published_ = false;
}
bool RobotStateRT::getDataPublished() {
	return data_published_;
}

void RobotStateRT::setControllerUpdated() {
	controller_updated_ = false;
}
bool RobotStateRT::getControllerUpdated() {
	return controller_updated_;
}

double RobotStateRT::ntohd(uint64_t nf) {
	double x;
	nf = be64toh(nf);
	memcpy(&x, &nf, sizeof(x));
	return x;
}

std::vector<double> RobotStateRT::unpackVector(uint8_t * buf, int start_index,
		int nr_of_vals) {
	uint64_t q;
	std::vector<double> ret;
	for (int i = 0; i < nr_of_vals; i++) {
		memcpy(&q, &buf[start_index + i * sizeof(q)], sizeof(q));
		ret.push_back(ntohd(q));
	}
	return ret;
}

std::vector<bool> RobotStateRT::unpackDigitalInputBits(int64_t data) {
	std::vector<bool> ret;
	for (int i = 0; i < 64; i++) {
		ret.push_back((data & (1 << i)) >> i);
	}
	return ret;
}

std::vector<double> RobotStateRT::getQTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = q_target_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getQdTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = qd_target_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getITarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = i_target_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getMTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = m_target_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getQActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = q_actual_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getQdActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = qd_actual_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getIActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = i_actual_;
	val_lock_.unlock();
	return ret;
}

std::vector<double> RobotStateRT::getToolVectorActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tool_vector_actual_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getTcpSpeedActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tcp_speed_actual_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getTcpForce() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tcp_force_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getToolVectorTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tool_vector_target_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getTcpSpeedTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tcp_speed_target_;
	val_lock_.unlock();
	return ret;
}
std::vector<bool> RobotStateRT::getDigitalInputBits() {
	std::vector<bool> ret;
	val_lock_.lock();
	ret = digital_input_bits_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getMotorTemperatures() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = motor_temperatures_;
	val_lock_.unlock();
	return ret;
}


double RobotStateRT::getVRobot() {
	double ret;
	val_lock_.lock();
	ret = v_robot_;
	val_lock_.unlock();
	return ret;
}
double RobotStateRT::getIRobot() {
	double ret;
	val_lock_.lock();
	ret = i_robot_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getVActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = v_actual_;
	val_lock_.unlock();
	return ret;
}
void RobotStateRT::unpack(uint8_t * buf) {
	int64_t digital_input_bits;
	uint64_t unpack_to;
	uint16_t offset = 0;
	val_lock_.lock();
    int len = 0;

    char *rec = (char*)buf;

    std::string response =(std::string)rec;

    int pos = response.find("getRobotPos",0);

    if(pos == 21)
    {
        q_actual_.clear();
        pos = response.find("\"joint1\":",0);
        double data = atof(&rec[pos+9]);
        q_actual_.push_back(data);

        pos = response.find("\"joint2\":",0);
        data = atof(&rec[pos+9]);
        q_actual_.push_back(data);

        pos = response.find("\"joint3\":",0);
        data = atof(&rec[pos+9]);
        q_actual_.push_back(data);

        pos = response.find("\"joint4\":",0);
        data = atof(&rec[pos+9]);
        q_actual_.push_back(data);

        pos = response.find("\"joint5\":",0);
        data = atof(&rec[pos+9]);
        q_actual_.push_back(data);

        pos = response.find("\"joint6\":",0);
        data = atof(&rec[pos+9]);
        q_actual_.push_back(data);
    }

	val_lock_.unlock();
	controller_updated_ = true;
	data_published_ = true;
	pMsg_cond_->notify_all();
}

