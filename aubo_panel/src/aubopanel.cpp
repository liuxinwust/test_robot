#include "aubopanel.h"
#include "ui_aubopanel.h"
#include <math.h>


namespace aubo_panel
{

AuboPanel::AuboPanel(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::AuboPanel),
    m_step(0.16),
    m_speed(50.0),
    m_jointStateSync(0),
    m_controMode(3),
    m_busInterface(1)

{
    ui->setupUi(this);

    ui->rbx_pcan->setChecked(false);
    ui->rbx_tcp->setChecked(true);


    ui->rbx_continuous->setChecked(false);
    ui->rbx_goal->setChecked(false);
    ui->rbx_moveit->setChecked(false);
    ui->rbx_sync->setChecked(true);


    ui->pbn_setIO->setEnabled(false);
    ui->pbn_sendGoal->setEnabled(false);

    ui->cbx_ioType->setMaxCount(4);
    ui->cbx_ioType->addItem("PLC_IO");
    ui->cbx_ioType->addItem("TOOL_IO");
    ui->cbx_ioType->addItem("Board_IO");
    ui->cbx_ioType->addItem("Modbus_IO");

    ui->cbx_ioMode->setMaxCount(4);
    ui->cbx_ioMode->addItem("DO");
    ui->cbx_ioMode->addItem("DI");
    ui->cbx_ioMode->addItem("AO");
    ui->cbx_ioMode->addItem("AI");

    ui->cbx_ioIndex->setMaxCount(8);
    ui->cbx_ioIndex->addItem("1");
    ui->cbx_ioIndex->addItem("2");
    ui->cbx_ioIndex->addItem("3");
    ui->cbx_ioIndex->addItem("4");
    ui->cbx_ioIndex->addItem("5");
    ui->cbx_ioIndex->addItem("6");
    ui->cbx_ioIndex->addItem("7");
    ui->cbx_ioIndex->addItem("8");


    m_timer = new QTimer(this);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(sendCommand()));

    initROS();

    m_timer->start(10);     // Start the timer.
}

AuboPanel::~AuboPanel()
{
    delete ui;
}

void AuboPanel::initROS()
{
    joints.data.resize(6);
    cmd1_publisher_ = nh_.advertise<std_msgs::Float32MultiArray>("pcan_cmd", 1000);
    cmd2_publisher_ = nh_.advertise<std_msgs::Float32MultiArray>("movej_cmd", 1000);
    cmd3_publisher_ = nh_.advertise<std_msgs::Float32MultiArray>("servoj_cmd", 1000);
    goal_publisher_ = nh_.advertise<aubo_msgs::GoalPoint>("send_goal", 1);
    io_publisher_ = nh_.advertise<aubo_msgs::IOState> ("io_state",1);
    subJointState_ = nh_.subscribe("joint_states", 1000, &AuboPanel::jointStateCallback,this);
}


int AuboPanel::pointCompare(void)
{
  int ret = 0;
  for(int i=0;i<6;i++)
  {
    if(fabs(joints.data[i]-lastJointPosition[i])>=0.000001)
    {
       ret = 1;
       break;
    }
  }
  return ret;
}

void AuboPanel::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    lastJointPosition[0] = msg->position[0];
    lastJointPosition[1] = msg->position[1];
    lastJointPosition[2] = msg->position[2];
    lastJointPosition[3] = msg->position[3];
    lastJointPosition[4] = msg->position[4];
    lastJointPosition[5] = msg->position[5];

    if(m_controMode == 3)//sync with real robot
    {
        if(pointCompare())
        {
            joints.data[0] = lastJointPosition[0];
            joints.data[1] = lastJointPosition[1];
            joints.data[2] = lastJointPosition[2];
            joints.data[3] = lastJointPosition[3];
            joints.data[4] = lastJointPosition[4];
            joints.data[5] = lastJointPosition[5];

            ui->lb_joint1->setText(QString::number((joints.data[0]*180/M_PI),'f',6));
            ui->lb_joint2->setText(QString::number((joints.data[1]*180/M_PI),'f',6 ));
            ui->lb_joint3->setText(QString::number((joints.data[2]*180/M_PI),'f',6 ));
            ui->lb_joint4->setText(QString::number((joints.data[3]*180/M_PI),'f',6 ));
            ui->lb_joint5->setText(QString::number((joints.data[4]*180/M_PI),'f',6 ));
            ui->lb_joint6->setText(QString::number((joints.data[5]*180/M_PI),'f',6 ));
        }
    }
}

void AuboPanel::on_pbn_joint1Left_pressed()
{
    if((m_controMode == 0)&&(m_busInterface == 0)) m_step = 0.16;
    else m_step = 0.035;

    joints.data[0] = joints.data[0] - m_step * m_speed/100 < -M_PI ?
                     joints.data[0] :
                     joints.data[0] - m_step  * m_speed/100;
    if(joints.data[0]< -3.05) joints.data[0] = -3.05;

    ui->lb_joint1->setText(QString::number(joints.data[0]*180.0/M_PI, 'f', 6));
}


void AuboPanel::on_pbn_joint1Right_pressed()
{
    if((m_controMode == 0)&&(m_busInterface == 0)) m_step = 0.16;
    else m_step = 0.035;

    joints.data[0] = joints.data[0] + m_step * m_speed/100 > M_PI ?
                     joints.data[0] :
                     joints.data[0] + m_step * m_speed/100;
    if(joints.data[0]> 3.05) joints.data[0] = 3.05;

    ui->lb_joint1->setText(QString::number(joints.data[0]*180.0/M_PI, 'f', 6));
}

void AuboPanel::on_pbn_joint2Left_pressed()
{
    if((m_controMode == 0)&&(m_busInterface == 0)) m_step = 0.16;
    else m_step = 0.035;

    joints.data[1] = joints.data[1] - m_step * m_speed/100 < -M_PI ?
                     joints.data[1] :
                     joints.data[1] - m_step * m_speed/100;
    if(joints.data[1]< -3.05) joints.data[1] = -3.05;

    ui->lb_joint2->setText(QString::number(joints.data[1]*180.0/M_PI, 'f', 6));
}

void AuboPanel::on_pbn_joint2Right_pressed()
{    
    if((m_controMode == 0)&&(m_busInterface == 0)) m_step = 0.16;
    else m_step = 0.035;

    joints.data[1] = joints.data[1] + m_step * m_speed/100 > M_PI ?
                     joints.data[1] :
                     joints.data[1] + m_step * m_speed/100;
    if(joints.data[1]> 3.05) joints.data[1] = 3.05;

    ui->lb_joint2->setText(QString::number(joints.data[1]*180.0/M_PI, 'f', 6));
}

void AuboPanel::on_pbn_joint3Left_pressed()
{
    if((m_controMode == 0)&&(m_busInterface == 0)) m_step = 0.16;
    else m_step = 0.035;

    joints.data[2] = joints.data[2] - m_step * m_speed/100 < -M_PI ?
                     joints.data[2] :
                     joints.data[2] - m_step * m_speed/100;

    if(joints.data[2]< -3.05) joints.data[2] = -3.05;
    ui->lb_joint3->setText(QString::number(joints.data[2]*180.0/M_PI, 'f', 6));
}

void AuboPanel::on_pbn_joint3Right_pressed()
{
    if((m_controMode == 0)&&(m_busInterface == 0)) m_step = 0.16;
    else m_step = 0.035;

    joints.data[2] = joints.data[2] + m_step * m_speed/100 > M_PI ?
                     joints.data[2] :
                     joints.data[2] + m_step * m_speed/100;
    if(joints.data[2]> 3.05) joints.data[2] = 3.05;

    ui->lb_joint3->setText(QString::number(joints.data[2]*180.0/M_PI, 'f', 6));
}

void AuboPanel::on_pbn_joint4Left_pressed()
{
    if((m_controMode == 0)&&(m_busInterface == 0)) m_step = 0.16;
    else m_step = 0.035;

    joints.data[3] = joints.data[3] - m_step * m_speed/100 < -M_PI ?
                     joints.data[3] :
                     joints.data[3] - m_step * m_speed/100;
    if(joints.data[3]< -3.05) joints.data[3] = -3.05;

    ui->lb_joint4->setText(QString::number(joints.data[3]*180.0/M_PI, 'f', 6));
}

void AuboPanel::on_pbn_joint4Right_pressed()
{
    if((m_controMode == 0)&&(m_busInterface == 0)) m_step = 0.16;
    else m_step = 0.035;

    joints.data[3] = joints.data[3] + m_step * m_speed/100 > M_PI ?
                     joints.data[3] :
                     joints.data[3] + m_step * m_speed/100;
    if(joints.data[3]> 3.05) joints.data[3] = 3.05;

    ui->lb_joint4->setText(QString::number(joints.data[3]*180.0/M_PI, 'f', 6));
}

void AuboPanel::on_pbn_joint5Left_pressed()
{
    if((m_controMode == 0)&&(m_busInterface == 0)) m_step = 0.16;
    else m_step = 0.035;

    joints.data[4] = joints.data[4] - m_step * m_speed/100 < -M_PI ?
                     joints.data[4] :
                     joints.data[4] - m_step * m_speed/100;
    if(joints.data[4]< -3.05) joints.data[4] = -3.05;

    ui->lb_joint5->setText(QString::number(joints.data[4]*180.0/M_PI, 'f', 6));
}

void AuboPanel::on_pbn_joint5Right_pressed()
{
    if((m_controMode == 0)&&(m_busInterface == 0)) m_step = 0.16;
    else m_step = 0.035;

    joints.data[4] = joints.data[4] + m_step * m_speed/100 > M_PI ?
                     joints.data[4] :
                     joints.data[4] + m_step * m_speed/100;
    if(joints.data[4]> 3.05) joints.data[4] = 3.05;

    ui->lb_joint5->setText(QString::number(joints.data[4]*180.0/M_PI, 'f', 6));
}

void AuboPanel::on_pbn_joint6Left_pressed()
{
    if((m_controMode == 0)&&(m_busInterface == 0)) m_step = 0.16;
    else m_step = 0.035;

    joints.data[5] = joints.data[5] - m_step * m_speed/100 < -M_PI ?
                     joints.data[5] :
                     joints.data[5] - m_step * m_speed/100;
    if(joints.data[5]< -3.05) joints.data[5] = -3.05;

    ui->lb_joint6->setText(QString::number(joints.data[5]*180.0/M_PI, 'f', 6));
}

void AuboPanel::on_pbn_joint6Right_pressed()
{
    if((m_controMode == 0)&&(m_busInterface == 0)) m_step = 0.16;
    else m_step = 0.035;

    joints.data[5] = joints.data[5] + m_step * m_speed/100 > M_PI ?
                     joints.data[5] :
                     joints.data[5] + m_step * m_speed/100;
    if(joints.data[5]> 3.05) joints.data[5] = 3.05;

    ui->lb_joint6->setText(QString::number(joints.data[5]*180.0/M_PI, 'f', 6));
}


void AuboPanel::on_pbn_zero_clicked()
{
    joints.data[0] = 0;
    joints.data[1] = 0;
    joints.data[2] = 0;
    joints.data[3] = 0;
    joints.data[4] = 0;
    joints.data[5] = 0;
    ui->lb_joint1->setText(QString::number(0.0, 'f', 6));
    ui->lb_joint2->setText(QString::number(0.0, 'f', 6));
    ui->lb_joint3->setText(QString::number(0.0, 'f', 6));
    ui->lb_joint4->setText(QString::number(0.0, 'f', 6));
    ui->lb_joint5->setText(QString::number(0.0, 'f', 6));
    ui->lb_joint6->setText(QString::number(0.0, 'f', 6));
}

void AuboPanel::on_pbn_classicPos1_clicked()
{
    if((m_controMode == 1)||(m_controMode == 2))
    {
        joints.data[0] = 91*M_PI/180;
        joints.data[1] = 52.3*M_PI/180;
        joints.data[2] = -95*M_PI/180;
        joints.data[3] = 40*M_PI/180;
        joints.data[4] = 92.5*M_PI/180;
        joints.data[5] = 123.8*M_PI/180;
        ui->lb_joint1->setText(QString::number(91.0, 'f', 6));
        ui->lb_joint2->setText(QString::number(52.3, 'f', 6));
        ui->lb_joint3->setText(QString::number(-95.0, 'f', 6));
        ui->lb_joint4->setText(QString::number(40.0, 'f', 6));
        ui->lb_joint5->setText(QString::number(92.5, 'f', 6));
        ui->lb_joint6->setText(QString::number(123.8, 'f', 6));
    }
}

void AuboPanel::on_pbn_classicPos2_clicked()
{
    if((m_controMode == 1)||(m_controMode == 2))
    {
        joints.data[0] = -92*M_PI/180;
        joints.data[1] = -61.7*M_PI/180;
        joints.data[2] = 102*M_PI/180;
        joints.data[3] = -24*M_PI/180;
        joints.data[4] = -83.6*M_PI/180;
        joints.data[5] = -101.1*M_PI/180;
        ui->lb_joint1->setText(QString::number(-92.0, 'f', 6));
        ui->lb_joint2->setText(QString::number(-61.7, 'f', 6));
        ui->lb_joint3->setText(QString::number(102.0, 'f', 6));
        ui->lb_joint4->setText(QString::number(-24.0, 'f', 6));
        ui->lb_joint5->setText(QString::number(-83.6, 'f', 6));
        ui->lb_joint6->setText(QString::number(-101.1, 'f', 6));
    }
}

void AuboPanel::on_pbn_sendGoal_clicked()
{   
    if(m_controMode == 1)
    {
        cmd2_publisher_.publish(joints);
    }
    else if(m_controMode == 2)
    {
        goalPoint.bus = m_busInterface;
        goalPoint.joint1 = joints.data[0];
        goalPoint.joint2 = joints.data[1];
        goalPoint.joint3 = joints.data[2];
        goalPoint.joint4 = joints.data[3];
        goalPoint.joint5 = joints.data[4];
        goalPoint.joint6 = joints.data[5];
        goal_publisher_.publish(goalPoint);
    }
}

void AuboPanel::on_rbx_pcan_clicked()
{
    ui->rbx_pcan->setChecked(true);
    ui->rbx_tcp->setChecked(false);
    ui->pbn_setIO->setEnabled(false);
    m_busInterface = 0;
}


void AuboPanel::on_rbx_tcp_clicked()
{
    ui->rbx_pcan->setChecked(false);
    ui->rbx_tcp->setChecked(true);
    ui->pbn_setIO->setEnabled(true);
    m_busInterface = 1;
}

void AuboPanel::on_rbx_continuous_clicked()
{
    ui->rbx_continuous->setChecked(true);
    ui->rbx_goal->setChecked(false);
    ui->rbx_moveit->setChecked(false);
    ui->rbx_sync->setChecked(false);

    ui->pbn_sendGoal->setEnabled(false);
    m_controMode = 0;
}

void AuboPanel::on_rbx_goal_clicked()
{
    ui->rbx_continuous->setChecked(false);
    ui->rbx_goal->setChecked(true);
    ui->rbx_moveit->setChecked(false);
    ui->rbx_sync->setChecked(false);

    ui->pbn_sendGoal->setEnabled(true);
    m_controMode = 1;

}

void AuboPanel::on_rbx_moveit_clicked()
{

    ui->rbx_continuous->setChecked(false);
    ui->rbx_goal->setChecked(false);
    ui->rbx_moveit->setChecked(true);
    ui->rbx_sync->setChecked(false);

    ui->pbn_sendGoal->setEnabled(true);
    m_controMode = 2;
}

void AuboPanel::on_rbx_sync_clicked()
{

    ui->rbx_continuous->setChecked(false);
    ui->rbx_goal->setChecked(false);
    ui->rbx_moveit->setChecked(false);
    ui->rbx_sync->setChecked(true);

    ui->pbn_sendGoal->setEnabled(false);
    m_controMode = 3;
}



void AuboPanel::on_pbn_setIO_clicked()
{
    iostate.type = ui->cbx_ioType->currentIndex()+1;
    iostate.mode = ui->cbx_ioMode->currentIndex()+1;
    iostate.index = ui->cbx_ioMode->currentIndex()+1;

    iostate.state = ui->le_ioState->text().toDouble();

    if((m_controMode == 1)&&(m_busInterface == 1))
    {
        io_publisher_.publish(iostate);
    }
}


void AuboPanel::sendCommand()
{
    if( ros::ok()&&(cmd1_publisher_||cmd3_publisher_))
    {
        if(m_controMode == 0)
        {
            ROS_WARN("Send");
            if(m_busInterface == 0)
            {
                cmd1_publisher_.publish(joints);
            }
            else if(m_busInterface == 1)
            {
                cmd3_publisher_.publish(joints);
            }
        }
    }
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aubo_panel::AuboPanel,rviz::Panel)





