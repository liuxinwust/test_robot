#include "mainwindow.h"
#include <QApplication>
#include <pthread.h>


pthread_t tid;
extern std_msgs::Float32MultiArray joints;
extern aubo_msgs::IOState iostate;
extern int Control_Mode;
extern int Send_Goal;

struct arg_holder
{
    int argc;
    char **argv;
};

arg_holder *arg_struct;


void *thread_caller(void *arg)
{
    ros::init(arg_struct->argc, arg_struct->argv, "control_panel");
    ros::NodeHandle nh;
    ros::Publisher command_pub = nh.advertise<std_msgs::Float32MultiArray> ("movej_cmd", 1000);
    ros::Publisher goal = nh.advertise<std_msgs::Float32MultiArray> ("send_goal", 1);
    ros::Publisher io_cmd = nh.advertise<aubo_msgs::IOState> ("io_state",1);

    ros::Rate loop_rate(100);//Hz

    while(ros::ok())
    {
      if(Control_Mode == 0)
      {
        command_pub.publish(joints);
      }
      else if(Control_Mode == 1)
      {
          if(Send_Goal == 1)
          {
            goal.publish(joints);
          }
          Send_Goal = 0;
      }
      else if(Control_Mode == 2)
      {
          if(Send_Goal == 1)
          {
            command_pub.publish(joints);
          }
          else if(Send_Goal == 2)
          {
            io_cmd.publish(iostate);
          }

          Send_Goal = 0;
      }

      Send_Goal = 0;

      loop_rate.sleep();
    }

    return 0;
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    arg_struct = (arg_holder *)malloc(sizeof(struct arg_holder));
    arg_struct->argc = argc;
    arg_struct->argv = argv;

    int err = pthread_create(&tid, NULL, thread_caller, NULL);

    return a.exec();
}
