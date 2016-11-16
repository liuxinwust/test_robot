#include "aubo_driver/aubo_driver.h"
#include "our_control_api.h"


double last_road_point[6];

int road_point_compare(double *goal)
{
  int ret = 0;
  for(int i=0;i<6;i++)
  {
    if(fabs(goal[i]-last_road_point[i])>=0.000001)
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


int connect_server(const char * server_ip, int server_port)
{
    int ret = -1;
	
    ret= login(server_ip, server_port,"123","123");
    if(ret == 0)
    {
       init_move_profile_for_script();
       set_enable_block(false);
    }
    return ret;
}

int disconnect_server()
{
    int ret = -1;
    ret =logout_system();
    return ret;
}


int get_current_position(double *pos)
{
    int ret = -1;
    our_robot_road_point road_point;
    ret = get_robot_position(&road_point);
    if(ret == 0)
    {
    	pos[0] = road_point.joint_pos[0];
        pos[1] = road_point.joint_pos[1];
        pos[2] = road_point.joint_pos[2];
        pos[3] = road_point.joint_pos[3];
        pos[4] = road_point.joint_pos[4];
        pos[5] = road_point.joint_pos[5];
    }

    return ret;
}

/*see enum type define: io_type,io_mode in ourcontrol.h*/
int  set_robot_io(int io_type,int io_mode, int io_index, double io_value)
{
   int ret = -1;
   ret = set_single_io_status((our_contorl_io_type)io_type,(our_contorl_io_mode)io_mode, io_index, io_value);
   return ret;
}

/*see enum type define: io_type,io_mode in ourcontrol.h*/
double get_robot_io(int  io_type,int io_mode, int io_index)
{
   double io_value;
   get_single_io_status((our_contorl_io_type)io_type,(our_contorl_io_mode)io_mode, io_index,&io_value);
   return io_value;    
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
    sub1 = nh.subscribe("movej_cmd", 1000, &AuboDriver::chatterCallback1,this);
    sub2 = nh.subscribe("servoj_cmd", 1, &AuboDriver::chatterCallback2,this);
    sub3 = nh.subscribe("io_state", 1, &AuboDriver::chatterCallback3,this);

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
        pos[0] = msg->data[0];
        pos[1] = msg->data[1];
        pos[2] = msg->data[2];
        pos[3] = msg->data[3];
        pos[4] = msg->data[4];
        pos[5] = msg->data[5];

        if(reverse_connected_)
        {
           robot_moveJ_for_script(pos,6,1000,1000);
        }
    }

}


void AuboDriver::chatterCallback2(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    //ROS_INFO("goal=[%f,%f,%f,%f,%f,%f]",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]);

    double pos[6];
    pos[0] = msg->data[0];
    pos[1] = msg->data[1];
    pos[2] = msg->data[2];
    pos[3] = msg->data[3];
    pos[4] = msg->data[4];
    pos[5] = msg->data[5];


    if(road_point_compare(pos))
    {
        pos[0] = msg->data[0];
        pos[1] = msg->data[1];
        pos[2] = msg->data[2];
        pos[3] = msg->data[3];
        pos[4] = msg->data[4];
        pos[5] = msg->data[5];

        if(reverse_connected_)
        {
           robot_servoj(pos,6);
        }
    }
}

void AuboDriver::chatterCallback3(const aubo_msgs::IOState::ConstPtr &msg)
{
    ROS_INFO("IO[%d,%d,%d,%f]", msg->type,msg->mode,msg->index,msg->state);

    if(reverse_connected_)
    {
        int io_type = msg->type ;
        int io_mode = msg->mode;
        int io_index = msg->index;
        double io_state = msg->state;
        set_robot_io((our_contorl_io_type)io_type, (our_contorl_io_mode)io_mode ,io_index,io_state);
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
            reverse_port = 8887;
		}

		if((reverse_port <= 0) or (reverse_port >= 65535)) {
            ROS_WARN("Reverse port value is not valid (Use number between 1 and 65534. Using default value of 8887");
            reverse_port = 8887;
		}
	} 

	AuboDriver auboDriver(host, reverse_port);

    ros::AsyncSpinner spinner(6);
	spinner.start();

	ros::waitForShutdown();

	return(0);
}
