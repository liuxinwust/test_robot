#include "aubo_driver/aubo_driver.h"
#include "aubo_control_api.h"

double last_road_point[6];

int road_point_compare(double *goal)
{
  int ret = 0;
  for(int i=0;i<6;i++)
  {
    if(fabs(goal[i]-last_road_point[i])>0.001)
    {
       ret = 1;
       break;
    }
  }

  if(ret == 1)
  {
    last_road_point[0]= goal[0];
    last_road_point[1]= goal[1];
    last_road_point[2]= goal[2];
    last_road_point[3]= goal[3];
    last_road_point[4]= goal[4];
    last_road_point[5]= goal[5];
  }

  return ret;
}

AuboDriver::AuboDriver(std::string host,unsigned int reverse_port)
{

    int ret = -1;
	
    ret= connect_server(host.c_str(), reverse_port);

    if(ret == 0)
    {
        ROS_INFO("Connect robot server %s:%d success!",host.c_str(),reverse_port);
        reverse_connected_ = true;
        get_current_position(last_road_point);
    }
    else
    {
        ROS_WARN("Connect robot server %s:%d failure!",host.c_str(),reverse_port);
        reverse_connected_ = false;
    }


    timer = nh.createTimer(ros::Duration(0.100),&AuboDriver::timerCallback,this);
    timer.stop();

    pos_pub = nh.advertise<aubo_msgs::JointPos>("current_pos", 1);
    sub = nh.subscribe("movej_cmd", 1000, &AuboDriver::chatterCallback1,this);
    sub2 = nh.subscribe("io_state", 1, &AuboDriver::chatterCallback2,this);

    timer.start();
}


void AuboDriver::chatterCallback1(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    ROS_INFO("goal=[%f,%f,%f,%f,%f,%f]",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]);

    double pos[6];   
    pos[0] = msg->data[0];
    pos[1] = msg->data[1];
    pos[2] = msg->data[2];
    pos[3] = msg->data[3];
    pos[4] = msg->data[4];
    pos[5] = msg->data[5];


    if(road_point_compare(pos))
    {
        pos[0] = msg->data[0]*180.0/M_PI;
        pos[1] = msg->data[1]*180.0/M_PI;
        pos[2] = msg->data[2]*180.0/M_PI;
        pos[3] = msg->data[3]*180.0/M_PI;
        pos[4] = msg->data[4]*180.0/M_PI;
        pos[5] = msg->data[5]*180.0/M_PI;

        if(reverse_connected_)
        {
           movej(pos,2000,2000);
        }
    }

}

void AuboDriver::chatterCallback2(const aubo_msgs::IOState::ConstPtr &msg)
{
    ROS_INFO("[%d,%d,%f]", msg->type,msg->index,msg->state);

    if(reverse_connected_)
    {
        int io_type = msg->type ;
        int io_index = msg->index;
        double io_state = msg->state;
        set_robot_io(io_type, io_index,io_state);
    }

}


void AuboDriver::timerCallback(const ros::TimerEvent& e)
{
    aubo_msgs::JointPos cur_pos;
    double pos[6];
    if(reverse_connected_ == true)
    {
        get_current_position(pos);
        cur_pos.joint1 = pos[0];
        cur_pos.joint2 = pos[1];
        cur_pos.joint3 = pos[2];
        cur_pos.joint4 = pos[3];
        cur_pos.joint5 = pos[4];
        cur_pos.joint6 = pos[5];
        pos_pub.publish(cur_pos);
    }
}


int main(int argc, char **argv) {
	std::string host;
	int reverse_port;

	ros::init(argc, argv, "aubo_driver");

	if (!(ros::param::get("~robot_ip", host))) {
		if (argc > 1) {
			host = argv[1];
		} else 
            {
            ROS_WARN("Could not get robot ip. Please supply it as command line parameter or on the parameter server as robot_ip");
			exit(1);
		}

	}

	if (!(ros::param::get("~reverse_port", reverse_port))) {
		if (argc > 2) {
			reverse_port = atoi(argv[2]);
		}
        else
		{
            reverse_port = 8877;
		}

		if((reverse_port <= 0) or (reverse_port >= 65535)) {
            ROS_WARN("Reverse port value is not valid (Use number between 1 and 65534. Using default value of 8887");
            reverse_port = 8877;
		}
	} 

	AuboDriver auboDriver(host, reverse_port);

    ros::AsyncSpinner spinner(3);
	spinner.start();

	ros::waitForShutdown();

	return(0);
}
