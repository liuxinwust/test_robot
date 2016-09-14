/********************************************************************************************************
  Copyright (C), 2016, Aubo Tech. Co., Ltd.
  FileName: jointcontrolapi.h
  Author:                  
  Date:2016-08-17
  Description: can driver for aubo robot arm
       
  Version:         V1.1.0
  Function List:  
	int   joint_control_init(const char *device);                
	int   joint_control_uninit(void);
	bool  set_joint_position(int joint_id, float angle);
	float read_joint_position(int joint_id);
	int   read_joint_brake_status(int joint_id);
  History:
      <author>    <time>      <version >     <desc>
                2016-08-17     V1.1.0        creat
********************************************************************************************************/

#ifndef JOINTCONTROLAPI_H
#define JOINTCONTROLAPI_H

#define JOINT_BRAKE_RELEASE 0
#define JOINT_BRAKE_CLOSED  1



/**************************************************************************
 * Function:joint_control_init
 * Description: init CAN bus
 * Input: 
 * 	 device： can descriptor,example：/dev/pcan32
 * Return: 0-success
 * Other:
 **************************************************************************/
int   joint_control_init(const char *device);
int   joint_control_uninit(void);

/**************************************************************************
 * Function:set_joint_position
 * Description:set joint target angle 
 * Input: 
 * 		joint_id    1～6
 * 		angle       rad
 * Return: 0-success
 **************************************************************************/
bool  set_joint_position(int joint_id, float angle);

/**************************************************************************
 * Function:read_joint_position
 * Description:read joint current position  
 * Input: 
 * 		joint_id   	1～6
 * Return: joint angle  
 * Other: 
 **************************************************************************/
float read_joint_position(int joint_id);


/**************************************************************************
 * Function:set_joint_max_speed
 * Description:set joint max speed
 * Input:
 * 		joint_id   	1～6
 * 		speed       rpm
 * Return: 0-success
 **************************************************************************/
bool  set_joint_max_speed(int joint_id, float speed);



/**************************************************************************
 * Function:set_joint_max_acc
 * Description:set joint max acc
 * Input:
 * 		joint_id   	1～6
 * 		acc         rpm/s
 * Return: 0-success
 **************************************************************************/
bool  set_joint_max_acc(int joint_id, float acc);



/**************************************************************************
 * Function:read_joint_brake_status
 * Description:read joint break status
 * Input: 
 * 		joint_id   	1～6
 * Return: 0-release , 1  break
 * Other: 
 **************************************************************************/
int   read_joint_brake_status(int joint_id);


#endif // JOINTCONTROLAPI_H
