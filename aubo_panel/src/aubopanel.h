#ifndef AUBO_PANEL_H
#define AUBO_PANEL_H

#include  <ros/ros.h>
#include  <rviz/panel.h>
#include  <sensor_msgs/JointState.h>
#include  <std_msgs/Float32MultiArray.h>
#include  <aubo_msgs/IOState.h>
#include  <aubo_msgs/GoalPoint.h>
#include  <QWidget>
#include  <QTimer>


namespace Ui {
    class AuboPanel;
}


namespace aubo_panel
{
    class AuboPanel : public rviz::Panel
    {
        Q_OBJECT

    public:
        explicit AuboPanel(QWidget *parent = 0);
        ~AuboPanel();

    public:
        void initROS();

    private Q_SLOTS:
        void on_pbn_joint1Left_pressed();
        void on_pbn_joint1Right_pressed();
        void on_pbn_joint2Left_pressed();
        void on_pbn_joint2Right_pressed();
        void on_pbn_joint3Left_pressed();
        void on_pbn_joint3Right_pressed();
        void on_pbn_joint4Left_pressed();
        void on_pbn_joint4Right_pressed();
        void on_pbn_joint5Left_pressed();
        void on_pbn_joint5Right_pressed();
        void on_pbn_joint6Left_pressed();
        void on_pbn_joint6Right_pressed();

        void on_pbn_zero_clicked();
        void on_pbn_classicPos1_clicked();
        void on_pbn_classicPos2_clicked();
        void on_pbn_sendGoal_clicked();

        void on_rbx_pcan_clicked();
        void on_rbx_tcp_clicked();

        void on_rbx_continuous_clicked();
        void on_rbx_goal_clicked();
        void on_rbx_moveit_clicked();
        void on_rbx_sync_clicked();

        void on_pbn_setIO_clicked();

    protected Q_SLOTS:
        void sendCommand();

    protected:
        QTimer* m_timer;

        // The ROS publisher/subscriber
        ros::Publisher cmd1_publisher_;
        ros::Publisher cmd2_publisher_;
        ros::Publisher cmd3_publisher_;
        ros::Publisher goal_publisher_;
        ros::Publisher io_publisher_;

        sensor_msgs::JointState jointMsg;
        std_msgs::Float32MultiArray joints;
        aubo_msgs::IOState iostate;
        aubo_msgs::GoalPoint goalPoint;

        ros::Subscriber subJointState_;
        void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
        int pointCompare(void);

        // The ROS node handle.
        ros::NodeHandle nh_;

        float jointVelocities[6];
        float jointPosition[6];
        float lastJointPosition[6];

    private:
        Ui::AuboPanel *ui;
        float m_step;
        float m_speed;

        int m_jointStateSync;
        int m_controMode;
        int m_busInterface;
    };
}//end of aubo_rviz_plugin

#endif // AUBO_PANEL_H
