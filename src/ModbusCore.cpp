static char RcsId[] = "@(#) $Header: /users/chaize/newsvn/cvsroot/Communication/Modbus/src/ModbusCore.cpp,v 1.4 2012-11-07 08:56:13 pascal_verdier Exp $ ";

//+*********************************************************************
//
// File:	ModbusCore.cpp
//
// Project:	Device Servers in C++
//
// Description:	Code for implementing the Modbus protocol class in C++
//		independently of TACO or TANGO.
//
// Author(s);	Andy Gotz
//
// Original:	August 2001
//
// $Log: not supported by cvs2svn $
// Revision 1.3  2011/05/19 14:59:53  jensmeyer
// Added mutex to protect the serial line access in case of serveral modbus devices in the same server, accessing the same serial line.
// Example : RS485 with serveral nodes
//
// Revision 1.2  2010/03/11 11:45:59  buteau
// - change include files order to avoid conflict reagarding WINVER symbol (due to omnithread 4.1.4)
//
// Revision 1.1  2009/02/25 13:58:41  buteau
// - files moved to src subdirectory
//
// Revision 1.9  2008/12/01 08:02:44  taurel
// - Fix a bug in the WriteCoil command
// - Fix warnings generated by gcc 4.3
//
// Revision 1.8  2008/07/18 10:40:26  taurel
// - Add a new TCPTimeout property used during communication between
// the device and the Modbus equipment
//
// Revision 1.7  2008/07/03 09:16:26  jensmeyer
// Added a SocketConnectionSleep property to define the wait time between
// a socket closing and the reopening of a new socket to connect to the hardware.
//
// Revision 1.6  2008/06/06 13:51:13  taurel
// - Better management of error when using the cache thread
// - Fix open file leak in case of TCP protocol and reconnection to the
// modbus device
//
// Revision 1.5  2008/03/17 14:37:35  taurel
// - Add a data cache for the ReadHoldingRegisters, ReadMultipleCoilsStatus, ReadInputRegisters and ReadInputStatus commands
// - Fix some bugs related to closing the sockets in TCP mode
// - The Address property is used for the modbus ID sent at the frame
// beginning
//
// Revision 1.4  2008/02/11 14:29:29  taurel
// - Ported to Windows VC8
//
// Revision 1.3  2005/03/31 15:07:05  jlpons
// Changed namespace name
//
// Revision 1.2  2005/03/01 17:53:35  jlpons
// Few updates.
//
// Revision 1.1  2005/01/14 15:36:55  jlpons
// Initial import
//
// Revision 2.0  2004/12/02 14:23:59  perez
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

#include <Modbus.h>

#include <stdio.h>
#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>

#ifdef WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif




using namespace std;

	
	// mutex to protect the serial line access in case
	// of serveral modbus devices in the same server 
	// accessing the same serial line.
	// Example : RS485 with serveral nodes
	
	omni_mutex		serialAccess;
    


//+======================================================================
// Function:    ModbusCore::ModbusCore()
//
// Description:	create a ModbusCore object
//
// Arg(s) In:	
//
// Arg(s) Out:	
//-=====================================================================
ModbusCore::ModbusCore (
	char  *serialline_name,
	short protocol,
	short address,
	char  *ip_host,
	long  socketConnectionSleep,
	int tcp_to,
	long  *error)
{

	this->serialline_name = serialline_name;
	this->address         = address;
	this->protocol        = protocol;
  	this->ip_host         = ip_host;
	this->connection_sleep = socketConnectionSleep;
	this->ip_timeout 	  = tcp_to;

	if (protocol == MBUS_RTU || protocol == MBUS_ASCII)
	{
         sl = new ModbusCoreSL(
		serialline_name,
		error);
         if ((sl == NULL) || (*error != 0))
	 {
	  cout << "ModbusCore::ModbusCore(): ModbusCoreSL() failed";
    	  return;
 	 }
	}
	else
	{
	 if (protocol == MBUS_TCP)
	 {
//
// establish a tcp/ip connection to the node using the ASA standard port 502
//
	  TCPOpenSocket();
	 }
	 else
	 {
	  cout << "ModbusCore::ModbusCore(): protocol not recognised, ";
	  cout << "must be one of rtu, tcp or ascii" << endl;
    	  return;
	 }
	}

}


//+=====================================================================
// Function:    ModbusCore::~ModbusCore()
//
// Description: destructor to destroy an object of the Modbus class
//
// input:       none
//
// Output:      none
//
//-=====================================================================
ModbusCore::~ModbusCore()
{
	if (protocol == MBUS_TCP) 
	{
		if (ip_connection == true)
		{
#ifdef WIN32
			closesocket(ip_socket);
#else
			close(ip_socket);
#endif
		}
	}
}


//+=====================================================================
// Function:    ModbusCore::Status()
//
// Description: return a static status string
//
// input:       none
//
// Output:      none
//
//-=====================================================================
char *ModbusCore::Status()
{
	static const char *protocol_type[] = {"None", "RTU", "ASCII", "TCP"};
	static char mess[2048];

	//
	// request parameters
	//
	sprintf(mess,"Modbus node address %d protocol %s ",
                address, protocol_type[protocol]);
	if (protocol == MBUS_TCP)
	{
		if (ip_host != NULL)
		sprintf(mess+strlen(mess)," iphost %s\n", ip_host);
	}
	else
	{
		sprintf(mess+strlen(mess)," iphost UNDEFINED !\n");
	}
	if (protocol == MBUS_RTU && protocol == MBUS_ASCII)
	{
		if (serialline_name != NULL)
		{
			sprintf(mess+strlen(mess)," serialline %s\n", serialline_name);
		}
		else
		{
			sprintf(mess+strlen(mess)," serialline UNDEFINED !\n");
		}
	}

        return mess;
}




//+=====================================================================
// Function:    ModbusCore::CalculateCRC()
//
// Description:	Calculate the 16 bit CRC for a Modbus RTU frame.
//
//		This routine is taken from the example in Modicon Modbus
//		Protocol Reference Guide (PI-MBUS-300 Rev E.) page 114.
//		It uses lookup tables to generate the CRC and is should
//		therefore be faster than calculating the CRC.
//
//
// Arg(s) In:	unsigned char *frame - frame
//		short length - frame length
//
// Arg(s) Out:	none
//-=====================================================================

long ModbusCore::CalculateCRC (unsigned char *frame, short frame_length, unsigned char *crc)
{
/* Table of CRC values for high-order byte */

static unsigned char auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
} ;

/* Table of CRC values for low-order byte */

static unsigned char auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;
        unsigned char uchCRCHi = 0xFF ; /* high CRC byte initialized */
        unsigned char uchCRCLo = 0xFF ; /* low CRC byte initialized  */
        unsigned uIndex ;               /* will index into CRC lookup*/
                                                                                        /* table */

        while (frame_length--)             /* pass through message buffer */
        {
                uIndex = uchCRCHi ^ *frame++ ;        /* calculate the CRC */
                uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
                uchCRCLo = auchCRCLo[uIndex] ;
        }

	crc[0] = uchCRCHi;
	crc[1] = uchCRCLo;

        return (OK) ;
}




//+=====================================================================
// Function:    ModbusCore::TCPOpenSocket()
//
// Description:	Open a TCP/IP socket connection
//
// Arg(s) In:	none
//
// Arg(s) Out:	none
//-=====================================================================

long ModbusCore::TCPOpenSocket(void)
{
	struct timeval timeout;
	fd_set fds_write;
	int socket_err;
#ifdef WIN32
	int serrlen = sizeof socket_err;
#else
	socklen_t serrlen = sizeof socket_err;
#endif


	/* create a socket */
	ip_socket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

	/* Set it to non-blocking */
#ifdef WIN32
	unsigned long iMode = 0;
	if (ioctlsocket(ip_socket,FIONBIO,&iMode) != 0)
	{
#else
	if (fcntl(ip_socket, F_SETFL, O_NONBLOCK) == -1) 
	{
#endif
	 ip_connection = false;
#ifdef WIN32
	 closesocket(ip_socket);
#else
	 close(ip_socket);
#endif
	 cout << "ModbusCore::TCPOpenSocket(): cannot set non-blocking socket";
	 cout << " (error=" << errno << ")" << endl;
  	 return(-1);
	}


	/* Attempt to initiate a connection */
	ip_address.sin_family = AF_INET;
	ip_address.sin_port = htons(502);
	ip_address.sin_addr.s_addr = inet_addr(ip_host);
	ip_status = connect(ip_socket, (struct sockaddr *)&ip_address, sizeof(struct sockaddr));
	if (ip_status < 0)
	{
	 ip_connection = false;
	 cout << "ModbusCore::TCPOpenSocket(): cannot connect to ip node ";
	 cout << ip_host << " (error=" << errno << ")" << endl;
	}


	/* Wait for the connect() to finish, or timeout seconds */
	timeout.tv_usec = 0;
	timeout.tv_sec  = 2;
	cout << "ModbusCore::TCPOpenSocket(): wait with timeout: ";
	cout << timeout.tv_sec << endl;
	FD_ZERO(&fds_write);
	FD_SET(ip_socket, &fds_write);
	if (select(ip_socket + 1, NULL, &fds_write, NULL, &timeout) == -1) 
	{
	 perror("Select: ");
	 ip_connection = false;
#ifdef WIN32
	 closesocket(ip_socket);
#else
	 close(ip_socket);
#endif
	 cout << "ModbusCore::TCPOpenSocket(): select() failed";
	 cout << " (error=" << errno << ")" << endl;
  	 return(-1);
	}

	/* Check if the connect() has finished */
	if (FD_ISSET(ip_socket, &fds_write)) 
	{
#ifdef WIN32
		if (getsockopt(ip_socket, SOL_SOCKET, SO_ERROR, (char *)&socket_err, &serrlen) != 0)
#else
		if (getsockopt(ip_socket, SOL_SOCKET, SO_ERROR, &socket_err, &serrlen) == -1)
#endif
		{
			ip_connection = false;
#ifdef WIN32
	 		closesocket(ip_socket);
#else
	 		close(ip_socket);
#endif
			cout << "ModbusCore::TCPOpenSocket(): getsockopt() failed";
			cout << " (error=" << errno << ")" << endl;
  			return(-1);
		}
		if (socket_err == 0) 
		{
			ip_connection = true;
			cout << "ModbusCore::TCPOpenSocket(): connect() works successfully"<<endl;

  	  /* Set it to blocking (no known way to do it without closing and
	     reopening it) */
			cout << "ModbusCore::TCPOpenSocket(): set back blocking socket"<<endl;

#ifdef WIN32
			closesocket(ip_socket);
#else
			close(ip_socket);
			
			// convert into seconds and ns
			struct timespec ts;
			div_t timing;
			
			timing = div ((int)connection_sleep, 1000);
			//cout << "quot = " << timing.quot << " rem = " << timing.rem << endl;
			 
			ts.tv_sec  = timing.quot;
			ts.tv_nsec = timing.rem * 1000000;
			nanosleep(&ts,NULL);
			
#endif
			ip_socket = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
			ip_status = connect(ip_socket, (struct sockaddr *)&ip_address, sizeof(struct sockaddr));
		} 
		else 
		{
			errno = socket_err;
			ip_connection = false;
#ifdef WIN32
	 		closesocket(ip_socket);
#else
	 		close(ip_socket);
#endif
			cout << "ModbusCore::TCPOpenSocket(): connect() failed";
			cout << " (error=" << errno << ")" << endl;
			perror("Socket error");
  			return(-1);
		}
	} 
	else 
	{     
	 ip_connection = false;
#ifdef WIN32
	 closesocket(ip_socket);
#else
	 close(ip_socket);
#endif
	 cout << "ModbusCore::TCPOpenSocket(): connect() timed out";
	 cout << " (error=" << errno << ")" << endl;
  	 return(-1);
	}

	return(ip_status);
}




//+=====================================================================
// Function:    ModbusCore::SendFrame()
//
// Description:	Send a Modbus frame to a node via serial line (RTU/ASCII)
//		or TCP/IP
//
// Arg(s) In:	unsigned char function - function code
//		short * data - data to send
//		short length - number of data words to send
//
// Arg(s) Out:	none
//-=====================================================================

long ModbusCore::SendFrame (unsigned char *query, short query_length, long *error)
{

	if (protocol == MBUS_RTU) 
	{
		return(SendRTUFrame(query, query_length, error));
	}

	if (protocol == MBUS_TCP) 
	{
		return(SendTCPFrame(query, query_length, error));
	}

	return(NOTOK);
}

//+=====================================================================
// Function:    ModbusCore::SendRTUFrame()
//
// Description:	Send a Modbus frame to a node via serial line using the
//		RTU (binary) protocol
//
// Arg(s) In:	unsigned char function - function code
//		short * data - data to send
//		short length - number of data words to send
//
// Arg(s) Out:	none
//-=====================================================================

long ModbusCore::SendRTUFrame (unsigned char *query, short query_length, long *error)
{
	unsigned char frame[1024], crc[2];
	long status, iframe, i;


	iframe=0;

	frame[iframe++] = address;
	for (i=0; i<query_length; i++)
	{
		frame[iframe++] = query[i];
	}
	CalculateCRC(frame, query_length+1, crc);
	frame[iframe++] = crc[0];
	frame[iframe++] = crc[1];

	status = sl->write(frame, iframe, error);
	if (status != OK) {
	  *error = MODBUS_ERR_Serial_Write;
	  return(NOTOK);
	}

	return(OK);
}

//+=====================================================================
// Function:    ModbusCore::SendTCPFrame()
//
// Description:	Send a Modbus frame to a node using the TCP protocol
//
// Arg(s) In:	unsigned char function - function code
//		short * data - data to send
//		short length - number of data words to send
//
// Arg(s) Out:	none
//-=====================================================================

long ModbusCore::SendTCPFrame (unsigned char *query, short query_length, long *error)
{
	unsigned char frame[1024];
	long iframe, i;

	iframe=0;
	frame[iframe++] = address;

	for (i=0; i<4; i++)
	{
		frame[iframe++] = 0;
	}
	frame[iframe++] = query_length+1; /* no. of bytes */
	frame[iframe++] = address;
	for (i=0; i<query_length; i++)
	{
		frame[iframe++] = query[i];
	}

	if (ip_connection == true)
	{
#ifdef WIN32
		const char *tmp_ptr = (const char *)&(frame[0]);
		ip_status = send(ip_socket, tmp_ptr, query_length+7, 0);
#else
		ip_status = send(ip_socket, frame, query_length+7, 0);
#endif
	}
/*
 * try to reopen and resend the frame if it fails the first time
 * maybe there is a problem with the socket being closed on the
 * node side e.g. for the Beckhoff modules with watchdog enabled
 *
 * - andy 28/11/01
 */
	if (ip_status != query_length+7 ||
		 ip_connection == false)
	{
		if (ip_connection == true)
		{
#ifdef WIN32
			closesocket(ip_socket);
#else
			close(ip_socket);
#endif
		}

		if (TCPOpenSocket() == 0)
		{
			if (ip_connection == true)
			{
#ifdef WIN32
				const char *tmp_ptr = (const char *)&(frame[0]);
				ip_status = send(ip_socket, tmp_ptr, query_length+7, 0);
#else
				ip_status = send(ip_socket, frame, query_length+7, 0);
#endif
			}
		}
	}

	if (ip_status != query_length+7)
	{
		*error = MODBUS_ERR_SendTCPFrame;
		return(NOTOK);
	}

	return(OK);
}
//+=====================================================================
// Function:    ModbusCore::GetResponse()
//
// Description:	Get response from Modbus node via serial line (RTU/ASCII)
//		or TCP/IP
//
// Arg(s) In:	short * data - data to send
//		short length - number of data words to send
//
// Arg(s) Out:	none
//-=====================================================================

long ModbusCore::GetResponse (unsigned char *response, short response_length, long *error)
{
	if (protocol == MBUS_RTU) 
	{
		return(GetRTUResponse(response, response_length, error));
	}

	if (protocol == MBUS_TCP) 
	{
		return(GetTCPResponse(response, response_length, error));
	}

	return(NOTOK);
}

//+=====================================================================
// Function:    ModbusCore::GetRTUResponse()
//
// Description:	Get response from Modbus node via serial line using the
//		RTU (binary) protocol
//
// Arg(s) In:	short * data - data to send
//		short length - number of data words to send
//
// Arg(s) Out:	none
//-=====================================================================

long ModbusCore::GetRTUResponse (unsigned char *response, short response_length, long *error)
{
	unsigned char frame[1024], crc[2];
	long status, ncharexp, nchar, i;

    	ncharexp = 2;
	status = sl->read(frame, ncharexp, &nchar, error);
	if (status != OK) {
	  *error = MODBUS_ERR_Serial_Read;
	  return(NOTOK);
	}

	if (frame[1] & 0x80)
	{
		ncharexp = 3;
		status = sl->read(frame, ncharexp, &nchar, error);
		if (status != OK)
			return(NOTOK);

		switch (frame[0])
		{
			case (1) : 
				*error = MODBUS_ERR_GetRTUResponse_1;
				break;

			case (2) : 
				*error = MODBUS_ERR_GetRTUResponse_2;
				break;

			case (3) : 
				*error = MODBUS_ERR_GetRTUResponse_3;
				break;

			case (4) : 
				*error = MODBUS_ERR_GetRTUResponse_4;
				break;

			case (5) : 
				*error = MODBUS_ERR_GetRTUResponse_5;
				break;

			case (6) : 
				*error = MODBUS_ERR_GetRTUResponse_6;
				break;

			case (8) : 
				*error = MODBUS_ERR_GetRTUResponse_8;
				break;

			default : 
				*error = MODBUS_ERR_GetRTUResponse_9;
				break;
		}
		return(NOTOK);

	}

	response[0] = frame[1];
	/* function code echoed correctly, read rest of response */        

	ncharexp = (response_length+1);
	status = sl->read((frame+2), ncharexp, &nchar, error);
	if (status != OK)
		return(NOTOK);


#if DEBUG
	cout << "GetRTUResponse(): nchar " << nchar << endl;
#endif /* DEBUG */
	CalculateCRC(frame, nchar, crc);
#if DEBUG
	cout << "GetRTUResponse(): crc " << crc[0] << " " << crc[1] << endl;
#endif /* DEBUG */

	if ((crc[0] != frame[nchar]) && (crc[1] != frame[nchar+1]))
	{
		*error = MODBUS_ERR_GetRTUResponse_CRC;
		return(NOTOK);
	}

	for (i=1; i<response_length; i++)
	{
		response[i] = frame[i+1];
	}

	return(OK);
}

//+=====================================================================
// Function:    ModbusCore::GetTCPResponse()
//
// Description:	Get response from Modbus node via ethernet using the
//		TCP protocol
//
// Arg(s) In:	short * data - data to send
//		short length - number of data words to send
//
// Arg(s) Out:	none
//-=====================================================================

long ModbusCore::GetTCPResponse (unsigned char *response, short response_length, long *error)
{
	unsigned char frame[1024];
	long status, i;
	struct timeval timeout = {this->ip_timeout,0};
	fd_set fds;

	FD_ZERO(&fds);
	FD_SET(ip_socket, &fds);
	status = select(ip_socket+1, &fds, NULL, NULL, &timeout);

	if (status < 0)
	{
		*error = MODBUS_ERR_GetTCPResponse_Resp;
		return(NOTOK);
	}

	if (status == 0)
	{
		*error = MODBUS_ERR_GetTCPResponse_TO;
		return(NOTOK);
	}

	if (!FD_ISSET(ip_socket, &fds))
	{
		*error = MODBUS_ERR_GetTCPResponse_Select;
		return(NOTOK);
	}

#ifdef WIN32
	char *tmp_ptr = (char *)&(frame[0]);
	status = recv(ip_socket, tmp_ptr, 1024, 0);
#else
	status = recv(ip_socket, frame, 1024, 0);
#endif
	if (status == 0)
	{
		*error = MODBUS_ERR_GetTCPResponse_Recv;
		return(NOTOK);
	}
	
	if (frame[7] & 0x80)
	{
		switch (frame[8])
		{
			case (1) : 
				*error = MODBUS_ERR_GetTCPResponse_1;
				break;

			case (2) : 
				*error = MODBUS_ERR_GetTCPResponse_2;
				break;

			case (3) : 
				*error = MODBUS_ERR_GetTCPResponse_3;
				break;

			case (4) : 
				*error = MODBUS_ERR_GetTCPResponse_4;
				break;

			case (5) : 
				*error = MODBUS_ERR_GetTCPResponse_5;
				break;

			case (6) : 
				*error = MODBUS_ERR_GetTCPResponse_6;
				break;

			case (8) : 
				*error = MODBUS_ERR_GetTCPResponse_8;
				break;

			default : 
				*error = MODBUS_ERR_GetTCPResponse_9;
				break;
		}
		return(NOTOK);

	}

	for (i=0; i<response_length+1; i++)
	{
		response[i] = frame[i+7];
	}

	return(OK);
}


//+=====================================================================
// Function:    ModbusCore::SendGet()
//
// Description:	Send a Modbus frame and get its answer
//
// Arg(s) In:	short * data - data to send
//		short length - number of data words to send
//
// Arg(s) Out:	none
//-=====================================================================

long ModbusCore::SendGet (
	unsigned char *query, 
	short         query_length, 
	unsigned char *response, 
	short         response_length,
	long          *error)
{
	long status;
	
	{
		// For the serial lne protocol, protect against
		// unsynchronized writing and reading
		
		if (protocol != MBUS_TCP)
		{
			serialAccess.lock();
		}
		
		
		omni_mutex_lock oml(modb_access);
	
		status = SendFrame(query, query_length, error);
		if (status != OK)
		{
			if (protocol == MBUS_TCP) 
			{
#ifdef WIN32
				closesocket(ip_socket);
#else
				close(ip_socket);
#endif
				TCPOpenSocket();
			}
			else
			{
				serialAccess.unlock();	
			}
			return(NOTOK);
		}
		
		status = GetResponse(response, response_length, error);
		if (status != OK)
		{
			if ( protocol == MBUS_TCP ) 
			{
				if ( ip_connection == true )
				{
#ifdef WIN32
					closesocket(ip_socket);
#else
					close(ip_socket);
#endif
					TCPOpenSocket();
				}
			}
			else
			{
				serialAccess.unlock();	
			}
			return(NOTOK);
		}
	}
	
	if ( protocol != MBUS_TCP ) 
	{
		serialAccess.unlock();
	}
	
	return(OK);
}

//+=====================================================================
// Function:    ModbusCore::Send()
//
// Description:	Send a Modbus frame, does not wait for the answer
//
// Arg(s) In:	short * data - data to send
//		short length - number of data words to send
//
// Arg(s) Out:	none
//-=====================================================================

long ModbusCore::Send (
	unsigned char *query, 
	short         query_length,
	long          *error)
{
	long status;
	
	{
		// For the serial lne protocol, protect against
		// unsynchronized writing and reading
		
		if (protocol != MBUS_TCP)
		{
			serialAccess.lock();
		}
		
		
		omni_mutex_lock oml(modb_access);
	
		status = SendFrame(query, query_length, error);
		if (status != OK)
		{
			if (protocol == MBUS_TCP) 
			{
#ifdef WIN32
				closesocket(ip_socket);
#else
				close(ip_socket);
#endif
				TCPOpenSocket();
			}
			else
			{
				serialAccess.unlock();	
			}
			return(NOTOK);
		}
		
	}
	
	if ( protocol != MBUS_TCP ) 
	{
		serialAccess.unlock();
	}
	
	return(OK);
}

//+=====================================================================
// Function:    ModbusCore::GetErrorMessage()
//
// Description:	Returns an error string for the given error code.
//              Note: Returns a handle to a static reference so the
//              returned string must not be freed.
//
// Arg(s) In:	long code - Error code
//
// Arg(s) Out:	none
//-=====================================================================

char *ModbusCore::GetErrorMessage(long code) {

 static char ret_str[1024];
 sprintf(ret_str,"Unknown error code:%ld",code);

 switch(code)
 {
 
  case MODBUS_ERR_GetRTUResponse_1:
  case MODBUS_ERR_GetRTUResponse_2:
  case MODBUS_ERR_GetRTUResponse_3:
  case MODBUS_ERR_GetRTUResponse_4:
  case MODBUS_ERR_GetRTUResponse_5:
  case MODBUS_ERR_GetRTUResponse_6:
  case MODBUS_ERR_GetRTUResponse_8:
  case MODBUS_ERR_GetRTUResponse_9:
  	strcpy(ret_str,"ModbusCore::GetRTUResponse(): ");
	break;
  case MODBUS_ERR_GetTCPResponse_1:
  case MODBUS_ERR_GetTCPResponse_2:
  case MODBUS_ERR_GetTCPResponse_3:
  case MODBUS_ERR_GetTCPResponse_4:
  case MODBUS_ERR_GetTCPResponse_5:
  case MODBUS_ERR_GetTCPResponse_6:
  case MODBUS_ERR_GetTCPResponse_8:
  case MODBUS_ERR_GetTCPResponse_9:
  	strcpy(ret_str,"ModbusCore::GetTCPResponse(): ");	
	break;
	
  case MODBUS_ERR_Serial_Read:
	strcpy(ret_str,"Error reading from serial line");  
	break;

  case MODBUS_ERR_Serial_Write:
	strcpy(ret_str,"Error writing to serial line");  
	break;
  
 }

 switch(code)
 {
  case MODBUS_ERR_SendTCPFrame:
	strcpy(ret_str,"ModbusCore::SendTCPFrame(): failed to send frame to ");
	strcat(ret_str,"node using TCP protocol ");
	strcat(ret_str,"(hint: check the network cable)");
	break;


  case MODBUS_ERR_GetTCPResponse_1:
  case MODBUS_ERR_GetRTUResponse_1:
	strcat(ret_str,"error getting response, ");
	strcat(ret_str,"illegal function (hint: contact the programmer)!");
	break;

  case MODBUS_ERR_GetTCPResponse_2:
  case MODBUS_ERR_GetRTUResponse_2:
	strcat(ret_str,"error getting response, ");
	strcat(ret_str,"illegal data address (hint: check the module address)");
	break;

  case MODBUS_ERR_GetTCPResponse_3:
  case MODBUS_ERR_GetRTUResponse_3:
	strcat(ret_str,"error getting response, ");
	strcat(ret_str,"illegal data value (hint: check the module data)");
	break;

  case MODBUS_ERR_GetTCPResponse_4:
  case MODBUS_ERR_GetRTUResponse_4:
	strcat(ret_str,"error getting response, ");
	strcat(ret_str,"slave device failure (hint:check the module hardware)");
	break;

  case MODBUS_ERR_GetTCPResponse_5:
  case MODBUS_ERR_GetRTUResponse_5:
	strcat(ret_str,"error getting response, ");
	strcat(ret_str,"answer will take some time");
        strcat(ret_str,"(hint: poll the node to get the answer)");
	break;

  case MODBUS_ERR_GetTCPResponse_6:
  case MODBUS_ERR_GetRTUResponse_6:
	strcat(ret_str,"slave is busy, ");
	strcat(ret_str,"cannot process request ");
	strcat(ret_str,"(hint: wait and then retransmit request)");
	break;

  case MODBUS_ERR_GetTCPResponse_8:
  case MODBUS_ERR_GetRTUResponse_8:
	strcat(ret_str,"memory parity error while reading ");
	strcat(ret_str,"extended memory (hint: check the node hardware)");
	break;

  case MODBUS_ERR_GetTCPResponse_9:
  case MODBUS_ERR_GetRTUResponse_9:
	strcat(ret_str,"unknown exception code ");
	strcat(ret_str,"(hint: check the node hardware)");
	break;

  case MODBUS_ERR_GetRTUResponse_CRC:
	strcpy(ret_str,"ModbusCore::GetRTUResponse(): failed to get response ");
	strcat(ret_str,"from node, error in CRC");
	break;

  case MODBUS_ERR_GetTCPResponse_Resp:
	strcpy(ret_str,"ModbusCore::GetTCPResponse(): failed to get TCP ");
	strcat(ret_str,"response from node ");
	break;

  case MODBUS_ERR_GetTCPResponse_TO:
	strcpy(ret_str,"ModbusCore::GetTCPResponse(): failed to get TCP ");
	strcat(ret_str,"response from node (timeout > ");
	sprintf(&(ret_str[strlen(ret_str)]),"%d s)",ip_timeout);
	break;

  case MODBUS_ERR_GetTCPResponse_Select:
	strcpy(ret_str,"ModbusCore::GetTCPResponse(): failed to get TCP ");
	strcat(ret_str,"response from node (select() returned not for us)");
	break;

  case MODBUS_ERR_GetTCPResponse_Recv:
	strcpy(ret_str,"ModbusCore::GetTCPResponse(): failed to get TCP ");
        strcat(ret_str,"response from node, unexpected close of connection ");
        strcat(ret_str,"at remote end");
	break;
 
 }
 
 return ret_str;
 
}
