#include "aubo_new_driver/aubo_new_driver.h"


AuboNewDriver::AuboNewDriver(std::condition_variable& rt_msg_cond,
		std::condition_variable& msg_cond, std::string host,
        unsigned int reverse_port,double servoj_time):REVERSE_PORT_(reverse_port),servoj_time_(servoj_time)
{

	char buffer[256];
	struct sockaddr_in serv_addr;
	int n, flag;

	reverse_connected_ = false;
	executing_traj_ = false;
    rt_interface_ = new AuboRealtimeCommunication(rt_msg_cond, host);
}


std::vector<double> AuboNewDriver::interp_cubic(double t, double T,
		std::vector<double> p0_pos, std::vector<double> p1_pos,
		std::vector<double> p0_vel, std::vector<double> p1_vel) {
	/*Returns positions of the joints at time 't' */
	std::vector<double> positions;
	for (unsigned int i = 0; i < p0_pos.size(); i++) {
		double a = p0_pos[i];
		double b = p0_vel[i];
		double c = (-3 * p0_pos[i] + 3 * p1_pos[i] - 2 * T * p0_vel[i]
				- T * p1_vel[i]) / pow(T, 2);
		double d = (2 * p0_pos[i] - 2 * p1_pos[i] + T * p0_vel[i]
				+ T * p1_vel[i]) / pow(T, 3);
		positions.push_back(a + b * t + c * pow(t, 2) + d * pow(t, 3));
	}
	return positions;
}

bool AuboNewDriver::doTraj(std::vector<double> inp_timestamps,
		std::vector<std::vector<double> > inp_positions,
		std::vector<std::vector<double> > inp_velocities) {
	std::chrono::high_resolution_clock::time_point t0, t;
	std::vector<double> positions;
	unsigned int j;

    if (!AuboNewDriver::openServo()) {
        return false;
    }
	
	executing_traj_ = true;
	t0 = std::chrono::high_resolution_clock::now();
	t = t0;
	j = 0;
	while ((inp_timestamps[inp_timestamps.size() - 1]
			>= std::chrono::duration_cast<std::chrono::duration<double>>(t - t0).count())
			and executing_traj_) {
		while (inp_timestamps[j]
				<= std::chrono::duration_cast<std::chrono::duration<double>>(
						t - t0).count() && j < inp_timestamps.size() - 1) {
			j += 1;
		}
		positions = AuboNewDriver::interp_cubic(
				std::chrono::duration_cast<std::chrono::duration<double>>(
						t - t0).count() - inp_timestamps[j - 1],
				inp_timestamps[j] - inp_timestamps[j - 1], inp_positions[j - 1],
				inp_positions[j], inp_velocities[j - 1], inp_velocities[j]);

        AuboNewDriver::servoj(positions);

		// oversample with 4 * sample_time
        std::this_thread::sleep_for(std::chrono::milliseconds((int) ((servoj_time_ * 1000) / 4.)));
		t = std::chrono::high_resolution_clock::now();
	}
	executing_traj_ = false;

	//Signal robot to stop driverProg()
	AuboNewDriver::closeServo(positions);
	return true;
}

void AuboNewDriver::servoj(std::vector<double> positions, int keepalive) {
	if (!reverse_connected_) {
		print_error("AuboNewDriver::servoj called without a reverse connection present. Keepalive: "
						+ std::to_string(keepalive));
		return;
	}

    char buf[1024];

    sprintf(buf,"{\"command\":\"servoj\",\"data\":{\"joint1\":%f,\"joint2\":%f,\"joint3\":%f,\"joint4\":%f,\"joint5\":%f,\"joint6\":%f}}\n",positions[0],positions[1],positions[2],positions[3],positions[4],positions[5]);
    print_info((std::string)buf);
    rt_interface_->addCommandToQueue((std::string)buf);
}


void AuboNewDriver::setSpeed(double q0, double q1, double q2, double q3, double q4,
        double q5, double acc) {
    rt_interface_->setSpeed(q0, q1, q2, q3, q4, q5, acc);
}




/************************Aubo plan and move API*****************************/
void AuboNewDriver::initMoveProfile() {
    rt_interface_->addCommandToQueue("{\"command\":\"initMoveProfile\"}\n");
}

void AuboNewDriver::setBlock(bool flag) {
    char buf[128];
    sprintf(buf, "{\"command\":\"enableBlock\",\"data\":{\"value\":%s}}\n",flag ? "true" : "false");
    //print_info((std::string)buf);
    rt_interface_->addCommandToQueue(buf);
}

void AuboNewDriver::setMaxSpeed(double speed){

    char cmd[1024];
    sprintf(cmd,"{\"command\":\"setSpeed\",\"data\":{\"value\":%f}}\n",speed);
    rt_interface_->addCommandToQueue((std::string) (cmd));
}

void AuboNewDriver::setMaxAcc(double acc){

    char cmd[1024];
    sprintf(cmd,"{\"command\":\"setAcce\",\"data\":{\"value\":%f}}\n",acc);
    rt_interface_->addCommandToQueue((std::string) (cmd));
}

void AuboNewDriver::movej(std::vector<double> positions) {

    char buf[1024];
    sprintf(buf,"{\"command\":\"movej\",\"data\":{\"joint1\":%f,\"joint2\":%f,\"joint3\":%f,\"joint4\":%f,\"joint5\":%f,\"joint6\":%f}}\n",positions[0],positions[1],positions[2],positions[3],positions[4],positions[5]);
    //print_info((std::string)buf);
    rt_interface_->addCommandToQueue((std::string)buf);
}

void AuboNewDriver::movel(std::vector<double> positions) {

    char buf[1024];

    sprintf(buf,"{\"command\":\"movel\",\"data\":{\"joint1\":%f,\"joint2\":%f,\"joint3\":%f,\"joint4\":%f,\"joint5\":%f,\"joint6\":%f}}\n",positions[0],positions[1],positions[2],positions[3],positions[4],positions[5]);
    //print_info((std::string)buf);
    rt_interface_->addCommandToQueue((std::string)buf);
}

void AuboNewDriver::movelTo(std::vector<double> positions) {

    char buf[1024];

    sprintf(buf,"{\"command\":\"movelTo\",\"data\":{\"x\":%f,\"y\":%f,\"z\":%f}}\n",positions[0],positions[1],positions[2]);
    //print_info((std::string)buf);
    rt_interface_->addCommandToQueue((std::string)buf);
}

void AuboNewDriver::addWayPoint(std::vector<double> positions) {

    char buf[1024];

    sprintf(buf,"{\"command\":\"addWaypoint\",\"data\":{\"joint1\":%f,\"joint2\":%f,\"joint3\":%f,\"joint4\":%f,\"joint5\":%f,\"joint6\":%f}}\n",positions[0],positions[1],positions[2],positions[3],positions[4],positions[5]);
    //print_info((std::string)buf);
    rt_interface_->addCommandToQueue((std::string)buf);
}

void AuboNewDriver::movep(double blendRadius,int trackMode){
    if (!reverse_connected_) {
        print_error("AuboNewDriver::movep called without a reverse connection present.");
        return;
    }
    char cmd[1024];
    sprintf(cmd,"{\"command\":\"movep\",\"data\":{\"blend_radius\":%f,\"track_mode\":%d}}\n",blendRadius,trackMode);
    rt_interface_->addCommandToQueue((std::string) (cmd));
}

void AuboNewDriver::getRobotPos() {
    rt_interface_->addCommandToQueue("{\"command\":\"getRobotPos\"}\n");
}
/************************Aubo plan and move API end *****************************/



bool AuboNewDriver::openServo() {
    reverse_connected_ = true;
    return true;
}

void AuboNewDriver::closeServo(std::vector<double> positions) {
	if (positions.size() != 6)
        AuboNewDriver::servoj(rt_interface_->robot_state_->getJonitPosition(), 0);
	else
		AuboNewDriver::servoj(positions, 0);

	reverse_connected_ = false;
}

bool AuboNewDriver::start() {
	if (!rt_interface_->start())
		return false;
	ip_addr_ = rt_interface_->getLocalIp();
	print_debug("Listening on " + ip_addr_ + ":" + std::to_string(REVERSE_PORT_)
					+ "\n");
	return true;

}

void AuboNewDriver::halt() {
	if (executing_traj_) {
		AuboNewDriver::stopTraj();
	}
	rt_interface_->halt();
}

void AuboNewDriver::stopTraj() {
	executing_traj_ = false;
    rt_interface_->addCommandToQueue("{\"command\":\"robotStop\"}\n");
}

std::vector<std::string> AuboNewDriver::getJointNames() {
	return joint_names_;
}

void AuboNewDriver::setJointNames(std::vector<std::string> jn) {
	joint_names_ = jn;
}


void AuboNewDriver::setRobotIO(int type, int mode,int index,float state) {
    char buf[256];
    sprintf(buf, "set_digital_out(%d, %d, %d, %f)\n", type, mode, index, state);

    //not implement yet
    //rt_interface_->addCommandToQueue(buf);
    //print_debug(buf);
}


void AuboNewDriver::setServojTime(double t) {
    if (t > 0.020) {
        servoj_time_ = t;
    } else {
        servoj_time_ = 0.020;
    }
}



