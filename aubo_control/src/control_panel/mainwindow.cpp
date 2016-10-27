#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <math.h>


std_msgs::Float32MultiArray joints;
aubo_msgs::IOState iostate;

int Control_Mode = 1;
int Send_Goal = 0;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_step(0.16),
    m_speed(50.0)
{
    ui->setupUi(this);
    joints.data.resize(6);
    Control_Mode = 1;
    Send_Goal = 0;
    ui->rbx_pcan->setChecked(false);
    ui->rbx_moveit->setChecked(true);
    ui->rbx_tcp->setChecked(false);
    ui->pbn_setIO->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pbn_joint1Left_pressed()
{
    if(Control_Mode == 0) m_step = 0.16;
    else m_step = 0.035;

    joints.data[0] = joints.data[0] - m_step * m_speed/100 < -M_PI ?
                     joints.data[0] :
                     joints.data[0] - m_step  * m_speed/100;
    if(joints.data[0]< -3.05) joints.data[0] = -3.05;

    ui->lb_joint1->setText(QString::number(joints.data[0]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint1Right_pressed()
{
    if(Control_Mode == 0) m_step = 0.16;
    else m_step = 0.035;

    joints.data[0] = joints.data[0] + m_step * m_speed/100 > M_PI ?
                     joints.data[0] :
                     joints.data[0] + m_step * m_speed/100;
    if(joints.data[0]> 3.05) joints.data[0] = 3.05;

    ui->lb_joint1->setText(QString::number(joints.data[0]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint2Left_pressed()
{
    if(Control_Mode == 0) m_step = 0.16;
    else m_step = 0.035;

    joints.data[1] = joints.data[1] - m_step * m_speed/100 < -M_PI ?
                     joints.data[1] :
                     joints.data[1] - m_step * m_speed/100;
    if(joints.data[1]< -3.05) joints.data[1] = -3.05;

    ui->lb_joint2->setText(QString::number(joints.data[1]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint2Right_pressed()
{    
    if(Control_Mode == 0) m_step = 0.16;
    else m_step = 0.035;

    joints.data[1] = joints.data[1] + m_step * m_speed/100 > M_PI ?
                     joints.data[1] :
                     joints.data[1] + m_step * m_speed/100;
    if(joints.data[1]> 3.05) joints.data[1] = 3.05;

    ui->lb_joint2->setText(QString::number(joints.data[1]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint3Left_pressed()
{
    if(Control_Mode == 0) m_step = 0.16;
    else m_step = 0.035;

    joints.data[2] = joints.data[2] - m_step * m_speed/100 < -M_PI ?
                     joints.data[2] :
                     joints.data[2] - m_step * m_speed/100;

    if(joints.data[2]< -3.05) joints.data[2] = -3.05;
    ui->lb_joint3->setText(QString::number(joints.data[2]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint3Right_pressed()
{
    if(Control_Mode == 0) m_step = 0.16;
    else m_step = 0.035;

    joints.data[2] = joints.data[2] + m_step * m_speed/100 > M_PI ?
                     joints.data[2] :
                     joints.data[2] + m_step * m_speed/100;
    if(joints.data[2]> 3.05) joints.data[2] = 3.05;

    ui->lb_joint3->setText(QString::number(joints.data[2]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint4Left_pressed()
{
    if(Control_Mode == 0) m_step = 0.16;
    else m_step = 0.035;

    joints.data[3] = joints.data[3] - m_step * m_speed/100 < -M_PI ?
                     joints.data[3] :
                     joints.data[3] - m_step * m_speed/100;
    if(joints.data[3]< -3.05) joints.data[3] = -3.05;

    ui->lb_joint4->setText(QString::number(joints.data[3]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint4Right_pressed()
{
    if(Control_Mode == 0) m_step = 0.16;
    else m_step = 0.035;

    joints.data[3] = joints.data[3] + m_step * m_speed/100 > M_PI ?
                     joints.data[3] :
                     joints.data[3] + m_step * m_speed/100;
    if(joints.data[3]> 3.05) joints.data[3] = 3.05;

    ui->lb_joint4->setText(QString::number(joints.data[3]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint5Left_pressed()
{
    if(Control_Mode == 0) m_step = 0.16;
    else m_step = 0.035;

    joints.data[4] = joints.data[4] - m_step * m_speed/100 < -M_PI ?
                     joints.data[4] :
                     joints.data[4] - m_step * m_speed/100;
    if(joints.data[4]< -3.05) joints.data[4] = -3.05;

    ui->lb_joint5->setText(QString::number(joints.data[4]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint5Right_pressed()
{
    if(Control_Mode == 0) m_step = 0.16;
    else m_step = 0.035;

    joints.data[4] = joints.data[4] + m_step * m_speed/100 > M_PI ?
                     joints.data[4] :
                     joints.data[4] + m_step * m_speed/100;
    if(joints.data[4]> 3.05) joints.data[4] = 3.05;

    ui->lb_joint5->setText(QString::number(joints.data[4]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint6Left_pressed()
{
    if(Control_Mode == 0) m_step = 0.16;
    else m_step = 0.035;

    joints.data[5] = joints.data[5] - m_step * m_speed/100 < -M_PI ?
                     joints.data[5] :
                     joints.data[5] - m_step * m_speed/100;
    if(joints.data[5]< -3.05) joints.data[5] = -3.05;

    ui->lb_joint6->setText(QString::number(joints.data[5]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_joint6Right_pressed()
{
    if(Control_Mode == 0) m_step = 0.16;
    else m_step = 0.035;

    joints.data[5] = joints.data[5] + m_step * m_speed/100 > M_PI ?
                     joints.data[5] :
                     joints.data[5] + m_step * m_speed/100;
    if(joints.data[5]> 3.05) joints.data[5] = 3.05;

    ui->lb_joint6->setText(QString::number(joints.data[5]*180.0/M_PI, 'f', 6));
}

void MainWindow::on_pbn_zero_clicked()
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

void MainWindow::on_pbn_classicPos1_clicked()
{
    if(Control_Mode != 0)
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

void MainWindow::on_pbn_classicPos2_clicked()
{
    if(Control_Mode != 0)
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

void MainWindow::on_pbn_sendGoal_clicked()
{
    if(Control_Mode != 0)
    {
        Send_Goal = 1;
    }
}

void MainWindow::on_rbx_pcan_clicked()
{
    ui->rbx_pcan->setChecked(true);
    ui->rbx_moveit->setChecked(false);
    ui->rbx_tcp->setChecked(false);
    ui->pbn_setIO->setEnabled(false);
    Control_Mode = 0;
}

void MainWindow::on_rbx_moveit_clicked()
{

    ui->rbx_pcan->setChecked(false);
    ui->rbx_moveit->setChecked(true);
    ui->rbx_tcp->setChecked(false);
    ui->pbn_setIO->setEnabled(false);
    Control_Mode = 1;

}

void MainWindow::on_rbx_tcp_clicked()
{
    ui->rbx_pcan->setChecked(false);
    ui->rbx_moveit->setChecked(false);
    ui->rbx_tcp->setChecked(true);
    ui->pbn_setIO->setEnabled(true);
    Control_Mode = 2;
}

void MainWindow::on_pbn_setIO_clicked()
{
    iostate.type = ui->le_ioType->text().toInt();
    iostate.mode = ui->le_ioMode->text().toInt();
    iostate.index = ui->le_ioIndex->text().toInt();
    iostate.state = ui->le_ioValue->text().toDouble();
    Control_Mode = 2;
    Send_Goal = 2;
}




