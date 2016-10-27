/********************************************************************************************************
  Copyright (C), 2016, Aubo Tech. Co., Ltd.
  FileName: our_control_api.h
  Author:                  Date:2016-05-24
  Description:
        本文件是一个.h文件，文件包含了通过API 接口控制机械臂的自定义数据类型还有接口声明，通过本接口用户可以
    实现远程实现对机械臂的控制，本接口提供了的功能有：使用该接口必要的初始化功能;登录;获取机械臂当前的运动状
    态;获取机械臂当前的所有状态;获取机械臂当前的位置信息;获取示教器内部的所有工程列表;加载指定示教器工程;
    对示教器当前加载的工程进行控制;获取当前所加载的工程的状态；针对机械臂的关节运动控制;直线运动控制；关于事
    件推送的使能和失能;设置事件处理的回调函数有几类:通知类事件回调函数，工程状态类事件，实时位置信息类事件;
    (每类事件可能包含多种事件类型，详细分类见下面事件的定义)

    使用该接口函数的顺序和依赖性：
        使用接口的必要条件:必须先进行接口初始化，初始化完成后进行登录。只要登录成功后才能正确调用其他接口。
        关于工程控制的接口依赖性：只有加载成功接口才能进行成功控制。
        关于事件处理的依赖性，只要设置了对应事件的回调函数，并且使能事件推送才能实现事件的推送。

  Version:         V1.1.0
  Function List:   // 主要函数及其功能
    1. initProxy();                               //接口的初始化
    2. int login();                               //登录
    3. our_robot_status get_robot_status();       //获取机械臂当前的运动状态;
    4. int get_robot_all_status();                //获取机械臂所有状态;
    5. int get_robot_current_position();          //获取机械臂当前的位置信息;
    6. int get_robot_project_list();              //获取示教器内部的所有工程列表
    7. int load_robot_project();                  //加载指定示教器工程;
    8. int project_control();                     //对示教器当前加载的工程进行控制
    9. int get_project_status();                  //获取当前所加载的工程的状态；
   10. int movej();                               //关节运动控制
   11. int movep();                               //直线运动控制；
   12. void setCurrentPosEventCallback();         //设置实时位置类事件的回调函数
   13. void setNoticeEventCallback();             //设置通知类事件的回调函数
   14. void set_project_status_event_callback();  //设置获取工程状态类的回调函数
   15. int enable_position_event();               //使能事件推送
   16. int disenable_position_event();            //失能事件推送
History:
      <author>    <time>      <version >     <desc>
                2016-05-24      V1.1.0       creat
********************************************************************************************************/
#ifndef OUR_CONTROL_API_H
#define OUR_CONTROL_API_H

#include <stdio.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>

#include "ourcontrol.h"
#include "ourcontrolerrorinfo.h"

#define PROJECT_NAME_MAX_BYTE 64          //工程名的最大长度
#define PROJECT_DESCRIPTION_MAX_BYTE 64   //工程描述的最大长度
#define FEATURE_NAME_MAX_BYTE 32

#define OUR_CONTROL_ERROR_MSG_BYTE 128
#define ERROR_MSG_BYTE 64

#define PROJECT_MAX_COUNT 50     //工程列表允许的最大数目
#define FEATURE_MAX_COUNT 50     //


#define MOVEP_POINT_MAX_COUNT    100

typedef int8_t              int8;
typedef int16_t             int16;
typedef int32_t             int32;
typedef uint8_t             uint8;
typedef uint16_t            uint16;
typedef uint32_t            uint32;
typedef int64_t             int64;
typedef uint64_t            uint64;
typedef float               float32;
typedef double              float64;

//工程定义
typedef struct
{
    char project_name[PROJECT_NAME_MAX_BYTE];
    char project_discription[PROJECT_DESCRIPTION_MAX_BYTE];
}project_info;


//工程加载时的属性定义
typedef struct
{
    char project_name[PROJECT_NAME_MAX_BYTE];
    char feature_name[FEATURE_NAME_MAX_BYTE];
    float relative_offset_x;
    float relative_offset_y;
    float relative_offset_z;
}control_project_load;

typedef struct
{
    char feature_name[FEATURE_NAME_MAX_BYTE];
    our_robot_road_point plane[3];
}control_feature;

typedef struct{
    double max_velc_jc;
    double max_acc_jc;
    bool scurve;
    our_move_mode_type move_mode;
    our_move_track_type move_track;
    bool tool_track;
    bool relative_move;
    our_feature_type user_coord;
    float relative_position[3];
    float tcp[3];
    double blend_radius;
    control_feature feature;
}move_profile;


typedef struct
{
    our_control_command response_command;
    char error_msg[ERROR_MSG_BYTE];
    uint8_t error_code;
}cmd_response;

typedef enum
{
    connect_server_fail=10001,
    disconnected_server,
    login_fail,
    call_fail,
    read_error,
    project_Load_fail,
    project_num_too_much,
    feature_num_too_much,
}error_code;


//用于描述脚本调用中MOVE的属性
typedef struct
{
    bool move_finish_flag;                     //到位信号标志位
    bool is_wait_move_finish;                  //是否等待到位信号  即本move是阻塞还是非阻塞  默认阻塞
    robot_move_profile move_profile;           //关于move的参数
    int current_movep_point_index;             //配合下面的数组使用   moveP 中使用
    our_robot_road_point movep_point_vector[MOVEP_POINT_MAX_COUNT];    //moveP  要用的一系列路点

}robot_move_api_profile;


typedef struct PACKED
{
    //硬件版本 (u16)(100)
    uint16 hw_version;
    //固件版本 (u16)((15<<9)|(5<<5)|(13)) 表示：2015年5月13日
    uint16 sw_version;
}robot_joint_version;


typedef struct PACKED
{
    //设备型号、芯片型号：上位机主站：0x01  接口板0x02
    uint8 type;
    //设备版本号，V1.0
    char revision[16];
    //厂家ID，"OUR "的ASCII码0x4F 55 52 00
    char manu_id[32];
    //机械臂类型
    char joint_type[16];
    //机械臂关节及工具端信息
    robot_joint_version joint_ver[7];
    //设备描述字符串以0x00结束
    char desc[64];

}robot_device_info;




//工程状态事件的 函数指针类型
typedef void (*projectStatusEventCallbackType)(our_project_status);

//通知类事件的 函数指针类型
typedef void (*noticeEventCallbackType)(our_control_event,bool);

//实时位置事件的 函数指针类型
typedef void (*currentPosEventCallbackType)(our_control_event,our_robot_road_point *road_point,float end_speed);

//异常事件  函数指针类型
typedef void (*ExceptionEventCallbackType)(our_control_event,bool);


//获取错误号
int get_current_error_code();

void set_current_error_code(int errorCode);

//根据错误号从错误列表中获取错误信息
char *get_error_msg_by_error_code(int error_code);

//获取时当前错误信息
int get_current_detail_error_msg(char *buf, unsigned int len);

void set_current_detail_error_msg(const char *errorMsg);



//void initProxy(char * server_ip, int server_port);

//设置实时位置事件的回调函数
void setCurrentPosEventCallback(currentPosEventCallbackType eventCallback);

//设置通知类事件的回调函数
void setNoticeEventCallback(noticeEventCallbackType eventCallback);

//设置获取工程状态的回调函数
void set_project_status_event_callback(projectStatusEventCallbackType eventCallback);

//设置异常事件的回调函数
void set_exception_event_callback(ExceptionEventCallbackType eventCallback);


void set_debug_messaage_display(int status);

/**************************************************************************
 * Function:login()
 * Description:连接并登录到ORPE服务器
 * Input:
 *      server_ip   服务器IP
 *      server_port 服务器端口号
 *      user_name:用户名
 *      password: 密码
 *      timeOutMsec:登录超时时间
 * Output:
 *      result:服务器返回的登录结果(结构体类型,在头文件中有定义)
 * Return: 0 表示成功 , -1  表示失败
 * Other:
 **************************************************************************/
int login(const char * server_ip, int server_port,const char *user_name, const char *password, int timeOutMsec);

//退出登录
int logout();

/**************************************************************************
 * Function:get_robot_status()
 * Description:获取当前的状态
 * Input:
 * Output:
 * Return: our_robot_status:机械臂当前状态(结构体类型,在头文件中有定义)
 * Other:
 **************************************************************************/
int get_robot_status(our_robot_status *project_status);


/**************************************************************************
 * Function:get_robot_all_status()
 * Description:获取机械臂所有状态(1.当前运动状态 2.基于 基坐标系的位置，姿态，关节角度 3.
 *                             3.基于 工具坐标系的位置，姿态，关节角度
 *                             4.基于 用户坐标系的位置，姿态，关节角度)
 * Input:
 * Output: robot_all_status   用于存放机械臂所有状态的结构体指针
 * Return: 0 表示命令发送成功 , -1  表示命令发送失败
 * Other:
 **************************************************************************/
int get_robot_all_status(our_control_robot_status *robot_all_status);


/**************************************************************************
 * Function:get_robot_current_position()
 * Description:获取机械臂当前位置信息(基于 XX坐标系的位置，姿态，关节角度)
 * Input:
 * Output: our_robot_road_point   用于存放机械臂当前位置信息的结构体指针
 * Return: 1 表示命令发送成功 , 0  表示命令发送失败
 * Other:
 **************************************************************************/
int get_robot_current_position(our_robot_road_point *pos);


/**************************************************************************
 * Function:get_robot_project_list()
 * Description:获取示教器内部的所有工程
 * Input:
 * Output: project_count    存放工程个数信息的内存指针
 *         project_list     存放工程信息指针的内存地址（双重指针） 存放工程信息指针的内存地址（双重指针）
 *                          注：使用完后必须将此地址空间使用delete()函数释放
 * Return: 0 表示命令发送成功 , -1  表示命令发送失败
 * Other:  project_list 使用完后必须将此地址空间使用delete()函数释放
 **************************************************************************/
int get_robot_project_list(int *project_count, project_info **project_list);


/**************************************************************************
 * Function:load_robot_project()
 * Description:加载指定的示教器工程
 * Input:
 *      project_info  指定的工程信息(结构体类型,在头文件中有定义)
 * Output:
 *      result:服务器返回针对命令是否发送成功的结果(结构体类型,在头文件中有定义)
 * Return: 0 表示命令发送成功 , -1  表示命令发送失败
 * Other:
 **************************************************************************/
int load_robot_project(const control_project_load * const project_info);


/**************************************************************************
 * Function:project_control()
 * Description: 控制工程的启动  暂停 停止
 * Input:
 *      project_control_cmd  控制命令 (枚举类型,在头文件中有定义)
 * Output:
 *      result:服务器返回针对命令是否发送成功的结果(结构体类型,在头文件中有定义)
 * Return: 0 表示命令发送成功 , -1  表示命令发送失败
 * Other:
 **************************************************************************/
int project_control(our_project_control project_control_cmd);


/**************************************************************************
 * Function:get_project_status()
 * Description: 获取当前加载工程的状态
 * Input:
 * Output:
 *      project_status:存放工程状态的结构内存地址
 * Return: 0 表示命令发送成功 , -1  表示命令发送失败
 * Other:
 **************************************************************************/
int get_project_status(our_project_status *project_status);


/**************************************************************************
 * Function:get_robot_feature_list()
 * Description:
 * Input:
 * Output:
 * Return: 1 表示命令发送成功 , 0  表示命令发送失败
 * Other:
 **************************************************************************/
int get_robot_feature_list(int *feature_count, control_feature **features);


/**************************************************************************
 * Function:movel()
 * Description:关节运动 直线运动  轨迹运动
 * Input:
 *
 * Output:
 *      move_profile      运动属性
 *      road_point        moveJ,moveL  的目标点
 *      point_track       轨迹运动的若干点的指针
 *      count             轨迹运动的路点的个数
 *
 * Return: 0 表示命令发送成功 , -1  表示命令发送失败
 * Other:
 **************************************************************************/
int movej_Api(move_profile *movej_profile, const our_robot_road_point * const road_point);

int movel_Api(move_profile *movel_profile, our_robot_road_point * road_point);

int movel_to_Api(move_profile *movel_profile, double X,double Y,double Z);

int movep_Api(move_profile *movep_profile, int count, our_robot_road_point *point_track);


int  robot_joint_move_teach(bool dir, int jointId, int speed);             //试教关节运动
int  robot_joint_step_move_teach(bool dir, int jointId, int speed, double jointStep);  //试教关节运动
int  robot_point_move_teach(robot_coord_type coordinage, robot_moveway move_code, int speed);             //试教点运动
int  robot_point_step_move_teach(robot_coord_type coordinage, robot_moveway move_code, int speed, double step);  //试教点运动

/**************************************************************************
 * Function:enable_position_event()
 * Description:允许和禁止实时位置信息推送事件
 * Input:
 * Output:
 * Return: 0 表示命令发送成功 , -1  表示命令发送失败
 * Other:
 **************************************************************************/
int enable_position_event();       //允许ORPE持续返回当前位置（5毫秒一次）
int disenable_position_event();    //禁止ORPE持续返回当前位置

int coord_convert(our_control_coord_convert *convert_info);    //坐标转换

int get_robot_control_box_status(our_control_robot_system_status *robot_system_status);   //读取控制柜状态

int get_robot_control_system_status(our_control_robot_system_status *robot_system_status);   //读取系统状态


int get_robot_joint_status(int *joint_count, our_control_joint_status **joint_status);    //读取机械臂关节状态

int get_robot_joint_status(our_control_joint_status *joint_status,int len);    //读取机械臂关节状态

int get_robot_joint_status_api(our_control_joint_status *joint_status,int len);


int set_robot_power(int power_status);                    //设置机械臂48V电源状态
int set_robot_brake(int Brake_status);                    //设置机械臂刹车状态
int set_collision_class(int val);
int set_robot_enalbe_read_pose(int status);               //设置接口板是否允许读取自身位姿


int get_robot_io_config_api(int *io_count, our_contorl_io_config **io_list);   //获取机械臂IO的配置
int get_robot_io_status_api(int  io_count,  our_control_io_status *io_list);   //获取机械臂IO的状态
int set_robot_io_status_api(int  io_count,  our_control_io_status *io_list);   //设置机械臂IO的状态


int  set_tcp_center(our_control_tcp_center_param *robot_tcp_center);      //设置TCP中心
int  get_tcp_center(our_control_tcp_center_param *robot_tcp_center);      //获取TCP中心


int get_robot_init_status(our_control_init_status *status);
int robot_control_api(our_control_robot_command robot_control_command);
int set_robot_mode(our_control_robot_mode mode);
int get_robot_mode(our_control_robot_mode *mode);


int get_robot_device_info(robot_device_info *device_info);

int robot_ik(double targetX,double targetY,double targetZ,double *currentJointPos,our_robot_road_point *road_point);
int robot_fk(double *jointPos,our_robot_road_point *road_point);

/**************************************************************************
 * Function:
 * Description:
 * Other:   下面这些函数是有关脚本的接口
 **************************************************************************/
void init_move_profile();                                //初始化move的属性
void set_scurve(int val);                                //设置S曲线是否有效
void set_tcp(double *tcp_pose,int count);                //设置TCP参数
void set_relative_offset(double *offset,int count);      //设置MOVE的偏移量
void set_wait_move_finish(int val);                      //设置MOVE的偏移量   设置是否等待到位信号  即  阻塞与非阻塞
void set_feature(const char *feature_name);              //设置坐标系
void add_waypoint(const double *pos, int count);         //用于MOVE 中增加路点

int  movej(double *pos, int count, double acc, double velc);
int  movel(double *pos, int count, double acc, double velc);
int  movel_to(double x, double y, double z, double acc, double velc);
int  movep(double acc, double velc,double blend_radius,int track_mode);

int  set_payload(double weight, double *cog, int count);  //设置运行时负载

int    is_exist_current_io    ( our_contorl_io_type  io_type, our_contorl_io_mode io_mode,int io_index);   //判断对应IO是否存在
int    set_robot_one_io_status( our_contorl_io_type  io_type, our_contorl_io_mode io_mode,int io_index, double io_value);  //设置指定IO 的状态
double get_robot_one_io_status( our_contorl_io_type  io_type, our_contorl_io_mode io_mode,int io_index);   //获取指定IO 的状态


void our_control_robot_device_info_To_robot_device_info(our_control_robot_device_info &src, robot_device_info &target);

#endif // OUR_CONTROL_API_H
