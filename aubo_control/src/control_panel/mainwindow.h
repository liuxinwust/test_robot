#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include  <QMainWindow>
#include  <ros/ros.h>
#include  <std_msgs/Float32MultiArray.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
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
    void on_rbx_moveit_clicked();

private:
    Ui::MainWindow *ui;
    float m_step;
    float m_speed;
};

#endif // MAINWINDOW_H
