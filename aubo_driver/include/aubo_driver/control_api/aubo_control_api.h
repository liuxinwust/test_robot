/********************************************************************************************************
  Copyright (C), 2016, Aubo Tech. Co., Ltd.
  FileName: aubo_control_api.h
  Author:                  
  Date:2016-09-4
  Description: tcp/ip control driver for aubo robot arm
       
  Version:         V1.0.0
********************************************************************************************************/

#ifndef AUBO_CONTROL_H
#define AUBO_CONTROL_H


/**************************************************************************
 * Function:connect_server
 * Description: connect tcp/ip server.
 * Input: 
 * 	 server_ip : ip address
 *       server_port　: port,default 8877
 * Return: 0-success
 * Other:
 **************************************************************************/
int    connect_server(const char * server_ip, int server_port);


/**************************************************************************
 * Function:disconnect_server
 * Description: disconnect tcp/ip server.
 * Input: 
 * 	 none
 * Return: 0-success
 * Other:
 **************************************************************************/
int    disconnect_server();


/**************************************************************************
 * Function:get_current_position
 * Description: get robot current position.
 * Input: 
 * 	 *pos : pointer of data array
 * Return: 0-success
 * Other:
 **************************************************************************/
int    get_current_position(double *pos);


/**************************************************************************
 * Function:movej
 * Description: control joint move 
 * Input: 
 * 	 *pos: target position data array pointer
 *       acc　: joint Acceleration
 *       velc: joint velocity
 * Return: 0-success
 * Other:
 **************************************************************************/
int    movej(double *pos, double acc, double velc);


/**************************************************************************
 * Function:set_robot_io
 * Description: set robot io status
 * Input: 
 * 	 io_type: IO type
 *       io_index　: IO index
 *       io_value: value
 * Return: 0-success
 * Other:
 **************************************************************************/
int    set_robot_io(int  io_type, int io_index, double io_value);


/**************************************************************************
 * Function:get_robot_io
 * Description: get robot io status
 * Input: 
 * 	 io_type: target position data array pointer
 *       io_index　: joint Acceleration
 * Return: the value of IO
 * Other:
 **************************************************************************/
double get_robot_io(int  io_type, int io_index);

#endif // AUBO_CONTROL_H
