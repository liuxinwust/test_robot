
#ifndef AUBO_REALTIME_COMMUNICATION_H_
#define AUBO_REALTIME_COMMUNICATION_H_

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>

#include "robot_state.h"

class AuboRealtimeCommunication {
private:
	int sockfd_;
	struct sockaddr_in serv_addr_;
	struct hostent *server_;
	std::string local_ip_;
	bool keepalive_;
	std::thread comThread_;
	int flag_;
	std::recursive_mutex command_string_lock_;
	std::string command_;
	void run();


public:
	bool connected_;
	RobotState* robot_state_;

	AuboRealtimeCommunication(std::condition_variable& msg_cond, std::string host);
	bool start();
	void halt();
        
	void setSpeed(double q0, double q1, double q2,double q3, double q4, double q5, double acc);
	void addCommandToQueue(std::string inp);
	std::string getLocalIp();

	void setMessagePush(bool flag);
	void getRobotPosition();
	void getRobotJointStatus();
	void getRobotSystemStatus();
	void getRobotEndSpeed();

};

#endif /* Aubo_REALTIME_COMMUNICATION_H_ */
