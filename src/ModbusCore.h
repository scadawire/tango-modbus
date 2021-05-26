//+*********************************************************************
//
// File:        ModbusCore.h
//
// Project:     Modbus
//
// Description: Code for implementing the Modbus protocol class in C++
//
// This file is part of Tango device class.
// 
// Tango is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// Tango is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with Tango.  If not, see <http://www.gnu.org/licenses/>.
// 
// $Author:  $
//
// $Revision:  $
// $Date:  $
//
// $log:  $
//
//-*********************************************************************

#ifndef _ModbusCore_H
#define _ModbusCore_H

#include <tango.h>

// Global definitions

// A modbus response frame is limited to 250 bytes.
// Limiting the number of registers to be read at 120 per call seems OK.
#define MAX_NB_REG 120
#define MAX_FRAME_SIZE 512

// MODBUS command code

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

// -----------------------------------------------------------------
// Abstract Modbus class
// -----------------------------------------------------------------

class ModbusCore {

public:
    virtual ~ModbusCore() {}

   // Return state
   virtual Tango::DevState State() = 0;

   // Return status
   virtual string Status() = 0;

   // Send a query and wait for the answer
   virtual void SendGet (unsigned char *query, 
	         short query_length, 
	         unsigned char *response, 
	         short response_length) = 0;

   // Send a query and ignore answer	
   virtual void Send ( unsigned char *query, 
  	               short query_length) = 0;

};

// -----------------------------------------------------------------
// Modbus RTU class
// -----------------------------------------------------------------

class ModbusRTU: public ModbusCore {

public:

   // Construct a ModbusCore RTU object
   ModbusRTU(std::string serialDevice,short node,std::string logFile);
   ~ModbusRTU();

   // Return state
   Tango::DevState State();

   // Return status
   string Status();

   // Send a query and wait for the answer
   void SendGet (unsigned char *query, 
	         short query_length, 
	         unsigned char *response, 
	         short response_length);

   // Send a query and ignore answer	
   void Send ( unsigned char *query, 
  	       short query_length);

private:
   
  Tango::DeviceProxy *serialDS;
  std::string logFileName;
  short node;
  Tango::DevState state;
  std::string lastError;

  void SendGetInternal (unsigned char *query, 
	         short query_length, 
	         unsigned char *response, 
	         short response_length);

  // Log error   
  void LogError(const char *msg,unsigned char *inFrame,short inFrameLgth,unsigned char *outFrame,short outFrameLgth);

  // Calculate CRC
  void CalculateCRC(unsigned char *frame, 
                    short frame_length, 
	            unsigned char *crc);
   
};

// -----------------------------------------------------------------
// Modbus TCP class
// -----------------------------------------------------------------

class ModbusTCP: public ModbusCore {

public:

   // Construct a ModbusCore TCP object
   ModbusTCP(std::string ipHost, short port, short node, double tcpTimeout, double connectTimeout, bool tcpNoDelay, bool tcpQuickAck, bool tcpKeepAlive);
   ~ModbusTCP();

   // Return state
   Tango::DevState State();

   // Return status
   string Status();

   // Send a query and wait for the answer
   void SendGet (unsigned char *query, 
	         short query_length, 
	         unsigned char *response, 
	         short response_length);

   // Send a query and ignore answer	
   void Send ( unsigned char *query, 
  	       short query_length);

private:
   
  short node;
  std::string ipHost;
  int tcpTimeout;
  int connectTimeout;
  bool tcpNoDelay;
  bool tcpQuickAck;
  bool tcpKeepAlive;
  short port;
  std::string lastError;
  int sock;
  time_t tickStart;
  time_t lastConnectTry;
  char *hostInfo;
  int   hostInfoLength;
  int   hostAddrType;
  

  // Timeout parameters are in millisecond
  bool IsConnected();
  void Disconnect();
  bool Connect(int *retSock);
  int Write(int sock, char *buf, int bufsize,int timeout);
  int Read(int sock, char *buf, int bufsize,int timeout);
  int WaitFor(int sock,int timeout,int mode);
  time_t get_ticks();
   
};


#endif /* _ModbusCore_H */
