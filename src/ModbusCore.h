//+*********************************************************************
//
// File:        ModbusCore.cpp
//
// Project:     Device Servers in C++
//
// Description: public include file containing definitions and declarations
//		for implementing the Modbus protocol class in C++
//              independently of TACO or TANGO.
//
// Author(s);   Andy Gotz
//
// Original:    August 2001
//
// $Log: not supported by cvs2svn $
// Revision 1.5  2008/07/18 10:40:26  taurel
// - Add a new TCPTimeout property used during communication between
// the device and the Modbus equipment
//
// Revision 1.4  2008/07/03 09:16:26  jensmeyer
// Added a SocketConnectionSleep property to define the wait time between
// a socket closing and the reopening of a new socket to connect to the hardware.
//
// Revision 1.3  2008/03/17 14:37:35  taurel
// - Add a data cache for the ReadHoldingRegisters, ReadMultipleCoilsStatus, ReadInputRegisters and ReadInputStatus commands
// - Fix some bugs related to closing the sockets in TCP mode
// - The Address property is used for the modbus ID sent at the frame
// beginning
//
// Revision 1.2  2008/02/11 14:29:29  taurel
// - Ported to Windows VC8
//
// Revision 1.1  2005/01/14 15:36:55  jlpons
// Initial import
//
// Revision 2.0  2004/12/02 14:24:06  perez
// Split Modbus.cpp
//
//
//
//
//
// Copyright (c) 2001 by European Synchrotron Radiation Facility,
//                       Grenoble, France
//
//
//
//-*********************************************************************

#ifndef _ModbusCore_H
#define _ModbusCore_H

#ifdef WIN32
#include <winsock2.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#endif
#include <ModbusCoreSL.h>

//+=====================================================================
// Global definitions
//-=====================================================================
#define MBUS_RTU	1
#define MBUS_ASCII	2
#define MBUS_TCP	3                                     

#define OK		0
#define NOTOK		(-1)

#define READ_COIL_STATUS                        1
#define READ_INPUT_STATUS                       2
#define READ_HOLDING_REGISTERS                  3
#define READ_INPUT_REGISTERS                    4
#define FORCE_SINGLE_COIL                       5
#define PRESET_SINGLE_REGISTER                  6
#define READ_EXCEPTION_STATUS                   7
#define FETCH_COMM_EVENT_CTR                    11
#define FETCH_COMM_EVENT_LOG                    12
#define FORCE_MULTIPLE_COILS                    15
#define PRESET_MULTIPLE_REGISTERS               16
#define REPORT_SLAVE_ID                         17
#define READ_GENERAL_REFERENCE                  20
#define WRITE_GENERAL_REFERENCE                 21
#define MASK_WRITE_REGISTER                     22
#define READ_WRITE_REGISTERS                    23
#define READ_FIFO_QUEUE                         24

#define MODBUS_ERR_SendTCPFrame                 -2
#define MODBUS_ERR_GetRTUResponse_1             -3
#define MODBUS_ERR_GetRTUResponse_2             -4
#define MODBUS_ERR_GetRTUResponse_3             -5
#define MODBUS_ERR_GetRTUResponse_4             -6
#define MODBUS_ERR_GetRTUResponse_5             -7
#define MODBUS_ERR_GetRTUResponse_6             -8
#define MODBUS_ERR_GetRTUResponse_8             -9
#define MODBUS_ERR_GetRTUResponse_9             -10
#define MODBUS_ERR_GetRTUResponse_CRC           -11
#define MODBUS_ERR_GetTCPResponse_1             -33
#define MODBUS_ERR_GetTCPResponse_2             -34
#define MODBUS_ERR_GetTCPResponse_3             -35
#define MODBUS_ERR_GetTCPResponse_4             -36
#define MODBUS_ERR_GetTCPResponse_5             -37
#define MODBUS_ERR_GetTCPResponse_6             -38
#define MODBUS_ERR_GetTCPResponse_8             -39
#define MODBUS_ERR_GetTCPResponse_9             -40
#define MODBUS_ERR_GetTCPResponse_Resp          -41
#define MODBUS_ERR_GetTCPResponse_TO            -42
#define MODBUS_ERR_GetTCPResponse_Select        -43
#define MODBUS_ERR_GetTCPResponse_Recv          -44
#define MODBUS_ERR_Serial_Read                  -100
#define MODBUS_ERR_Serial_Write                 -101

//+=====================================================================
// Class definition
//-=====================================================================

class ModbusCore {


//
// public members
//
public:

   short                protocol;            /* Modbus protocol RTU/ASCII */
   bool                 ip_connection;       /* true if sockect is connected*/
   short                address;             /* modbus node address */
   char                 *serialline_name;    /* device file */
   char                 *ip_host;            /* ip host name for tcp/ip */
   int					ip_timeout;			 /* Timeout used for TCP commmunication */

   ModbusCore (
        char  *serialline_name,
        short protocol,
        short address,
        char  *ip_host,
		long  socketConnectionSleep,
 		int   tcp_to,
		long  *error);
   ~ModbusCore ();

   char *Status();
   long SendGet (
	unsigned char *frame, 
	short frame_length, 
	unsigned char *response, 
	short response_length, 
	long *error);
	
   long Send (
	unsigned char *frame, 
	short frame_length,
	long *error);

   char *GetErrorMessage(long code);

//
// protected members
//

protected:
   int                  ip_socket;           /* open socket for tcp/ip */
   struct sockaddr_in   ip_address;          /* tcp/ip address */
   int                  ip_status;           /* status of last tcp/ip call */
	long 						connection_sleep;     /* sleep time in ms between two socket connections */
   omni_mutex			modb_access;

   ModbusCoreSL         *sl;



   long TCPOpenSocket (void);
   long CalculateCRC (
	unsigned char *frame, 
	short frame_length, 
	unsigned char *crc);
   long SendFrame (unsigned char *frame, short frame_length, long *error);
   long SendRTUFrame (unsigned char *frame, short frame_length, long *error);
   long SendTCPFrame (unsigned char *frame, short frame_length, long *error);
   long GetResponse (unsigned char *frame, short frame_length, long *error);
   long GetRTUResponse (unsigned char *frame, short frame_length, long *error);
   long GetTCPResponse (unsigned char *frame, short frame_length, long *error);
   
};

#endif /* _ModbusCore_H */
