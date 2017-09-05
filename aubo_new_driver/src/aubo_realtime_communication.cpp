#include "aubo_new_driver/aubo_realtime_communication.h"
#include "aubo_new_driver/do_output.h"

AuboRealtimeCommunication::AuboRealtimeCommunication(
        std::condition_variable& msg_cond, std::string host) {
    robot_state_ = new RobotState(msg_cond);
	bzero((char *) &serv_addr_, sizeof(serv_addr_));
	sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd_ < 0) {
		print_fatal("ERROR opening socket");
	}
	server_ = gethostbyname(host.c_str());
	if (server_ == NULL) {
		print_fatal("ERROR, no such host");
	}
	serv_addr_.sin_family = AF_INET;
	bcopy((char *) server_->h_addr, (char *)&serv_addr_.sin_addr.s_addr, server_->h_length);

    	serv_addr_.sin_port = htons(8899);
	flag_ = 1;
	setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_, sizeof(int));
	setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag_, sizeof(int));
	setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *) &flag_, sizeof(int));
	fcntl(sockfd_, F_SETFL, O_NONBLOCK);
	connected_ = false;
	keepalive_ = false;
}

bool AuboRealtimeCommunication::start() {
	fd_set writefds;
	struct timeval timeout;

	keepalive_ = true;
	print_debug("Realtime port: Connecting...");

	connect(sockfd_, (struct sockaddr *) &serv_addr_, sizeof(serv_addr_));

	FD_ZERO(&writefds);
	FD_SET(sockfd_, &writefds);
	timeout.tv_sec = 10;
	timeout.tv_usec = 0;
	select(sockfd_ + 1, NULL, &writefds, NULL, &timeout);
	unsigned int flag_len;
	getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &flag_, &flag_len);
	if (flag_ < 0) {
        print_fatal("Error connecting to RT port 8899");
		return false;
	}
	sockaddr_in name;
	socklen_t namelen = sizeof(name);
	int err = getsockname(sockfd_, (sockaddr*) &name, &namelen);
	if (err < 0) {
		print_fatal("Could not get local IP");
		close(sockfd_);
		return false;
	}
	char str[18];
	inet_ntop(AF_INET, &name.sin_addr, str, 18);
	local_ip_ = str;
	comThread_ = std::thread(&AuboRealtimeCommunication::run, this);

	return true;
}

void AuboRealtimeCommunication::halt() {
	keepalive_ = false;
	comThread_.join();
}

void AuboRealtimeCommunication::addCommandToQueue(std::string inp) {
	int bytes_written;
	if (inp.back() != '\n') {
		inp.append("\n");
	}
	if (connected_)
    {
		bytes_written = write(sockfd_, inp.c_str(), inp.length());
    }
	else
		print_error("Could not send command \"" +inp + "\". The robot is not connected! Command is discarded" );
}


void AuboRealtimeCommunication::setSpeed(double q0, double q1, double q2,
        double q3, double q4, double q5, double acc) {
    char cmd[1024];

    sprintf(cmd,
            "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f)\n",
            q0, q1, q2, q3, q4, q5, acc);

    print_info((std::string)cmd);

    //not implement yet
    //addCommandToQueue((std::string) (cmd));

}

void AuboRealtimeCommunication::setMessagePush(bool flag) {
    char cmd[128];
    sprintf(cmd, "{\"command\":\"enableMessagePush\",\"data\":{\"value\":%s}}\n",flag ? "true" : "false");
    //print_info((std::string)buf);
    addCommandToQueue((std::string) (cmd));
}

void AuboRealtimeCommunication::getRobotPosition() {
    char cmd[128];
    sprintf(cmd,"{\"command\":\"getRobotPos\"}\n");
    addCommandToQueue((std::string) (cmd));
}

void AuboRealtimeCommunication::getRobotJointStatus() {
    char cmd[128];
    sprintf(cmd,"{\"command\":\"getRobotJointStatus\"}\n");
    addCommandToQueue((std::string) (cmd));
}

void AuboRealtimeCommunication::getRobotSystemStatus() {
    char cmd[128];
    sprintf(cmd,"{\"command\":\"getRobotSystemStatus\"}\n");
    addCommandToQueue((std::string) (cmd));
}

void AuboRealtimeCommunication::getRobotEndSpeed() {
    char cmd[128];
    sprintf(cmd,"{\"command\":\"getRobotEndSpeed\"}\n");
    addCommandToQueue((std::string) (cmd));
}


void AuboRealtimeCommunication::run() {
	uint8_t buf[2048];
	int bytes_read;
	bzero(buf, 2048);
	struct timeval timeout;
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(sockfd_, &readfds);
	print_debug("Realtime port: Got connection");
	connected_ = true;
    int time = 0;

    setMessagePush(true);

	while (keepalive_) {
		while (connected_ && keepalive_) {

            timeout.tv_sec = 0; //do this each loop as selects modifies timeout
            timeout.tv_usec = 50000; // timeout of 0.05 sec
            select(sockfd_ + 1, &readfds, NULL, NULL, &timeout);

            bzero(buf, 2048);

			bytes_read = read(sockfd_, buf, 2048);


			if (bytes_read > 0) {
				setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag_, 
						sizeof(int));
                time = 0;
				robot_state_->unpack(buf);
            }
            else       
            {
               time = time + 1;

               if(time > 100)
               {
                   connected_ = false;
                   close(sockfd_);
                   time  = 0;
               }
            }

            if(connected_ == true)
            {
                //getRobotPosition();
            }
		}

		if (keepalive_) {
			//reconnect
			print_warning("Realtime port: No connection. Is controller crashed? Will try to reconnect in 10 seconds...");
			sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
			if (sockfd_ < 0) {
				print_fatal("ERROR opening socket");
			}
			flag_ = 1;
			setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_,
					sizeof(int));
			setsockopt(sockfd_, IPPROTO_TCP, TCP_QUICKACK, (char *) &flag_, 
					sizeof(int));
	
			setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *) &flag_,
					sizeof(int));
			fcntl(sockfd_, F_SETFL, O_NONBLOCK);

			while (keepalive_ && !connected_) {
				std::this_thread::sleep_for(std::chrono::seconds(10));
				fd_set writefds;

				connect(sockfd_, (struct sockaddr *) &serv_addr_,
						sizeof(serv_addr_));
				FD_ZERO(&writefds);
				FD_SET(sockfd_, &writefds);
				select(sockfd_ + 1, NULL, &writefds, NULL, NULL);
				unsigned int flag_len;
				getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &flag_, &flag_len);
				if (flag_ < 0) {
                    print_error("Error re-connecting to RT port 8899. Is controller started? Will try to reconnect in 10 seconds...");
				} else {
					connected_ = true;
					print_info("Realtime port: Reconnected");
				}
			}
		}
	}
    setSpeed(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,100);
	close(sockfd_);
}


std::string AuboRealtimeCommunication::getLocalIp() {
	return local_ip_;
}
