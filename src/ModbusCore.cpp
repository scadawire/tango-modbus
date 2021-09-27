//=============================================================================
//
// file :        ModbusCore.cpp
//
// description : Code for implementing the Modbus protocol class in C++
//
// project :     Modbus
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
#include <ModbusCore.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <string.h>

#ifdef WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netdb.h>
#include <netinet/tcp.h>
#endif

using namespace std;

// MODBUS error code and message

const char *modbusError[] = {

  "No error",
  "Function code not allowed",
  "Register address not allowed",
  "Register value not allowed",
  "Unrecoverable error occurred",
  "Request has been accepted but processing may be long",
  "Device is busy",
  "Requested function cannot be performed",
  "Parity error in the memory",
  "Unexpected error code [9]",
  "Gateway Path Unavailable",
  "No response from gateway"

};

const int nbError = sizeof(modbusError)/sizeof(char *);

// ---------------------------------------------------------------------
// Modbus RTU class
// ---------------------------------------------------------------------

// mutex to protect the serial line access in case
// of serveral modbus devices in the same server 
// accessing the same serial line.
// Example : RS485 with serveral nodes
omni_mutex serialAccess;

#define SL_NCHAR 1 // character read/write mode

// -------------------------------------------------------

ModbusRTU::ModbusRTU(std::string serialDevice,short node,std::string logFile) {

  this->state = Tango::UNKNOWN;
  lastError = "";
  serialDS = NULL;
  serialDS = new Tango::DeviceProxy(serialDevice);
  logFileName = logFile;
  this->node = node;

}

// -------------------------------------------------------

ModbusRTU::~ModbusRTU() {
  if( serialDS ) delete serialDS;
}

// -------------------------------------------------------

Tango::DevState ModbusRTU::State() {
  return state;
}

// -------------------------------------------------------

string ModbusRTU::Status() {

  char str[256];
  sprintf(str,"Modbus node address %d protocol RTU",node);
  if(lastError.length()>0) {
    strcat(str,"\n");
    strcat(str,lastError.c_str());
  }
  return string(str);

}

// -------------------------------------------------------

void ModbusRTU::SendGet (unsigned char *query, 
	                 short query_length,
	                 unsigned char *response, 
	                 short response_length) {


  try {
    SendGetInternal(query,query_length,response,response_length);
    state = Tango::ON;
    lastError = "";
  } catch(Tango::DevFailed &e) {
    state = Tango::UNKNOWN;
    lastError = e.errors[0].desc;
    throw e;
  }

}

// -------------------------------------------------------

void ModbusRTU::SendGetInternal (unsigned char *query, 
	                 short query_length,
	                 unsigned char *response, 
	                 short response_length) {

  unsigned char frame[MAX_FRAME_SIZE], crc[2];

  if( serialDS==NULL ) {
    Tango::Except::throw_exception(
   	  (const char *)"ModbusRTU::error_init",
       	  (const char *)"Serial device not imported",
       	  (const char *)"ModbusRTU::SendGet");	  
  }

  // We need to serialize serial line access to handle RS485
  omni_mutex_lock oml(serialAccess);

  Tango::DeviceData argin;
  Tango::DeviceData argout;
  vector<unsigned char> vcharr;

  Send(query,query_length);

  argin << (Tango::DevLong)( (2 << 8) | SL_NCHAR );
  argout = serialDS->command_inout("DevSerReadChar",argin);
  argout >> vcharr;
  frame[0] = vcharr[0];
  frame[1] = vcharr[1];

#ifdef FORCECZ

  // Work around for ELTA MUXBOX !!!
  // FORCECZ is not defined by default
  
  if( frame[0] != node ) {
    // Remove first wrong char
    frame[0] = frame[1];
    argin << (Tango::DevLong)( (1 << 8) | SL_NCHAR );
    argout = serialDS->command_inout("DevSerReadChar",argin);
    argout >> vcharr;
    frame[1] =  vcharr[0];
  }

#endif

  if (frame[1] & 0x80) {

    // We got a modbus error
    argin << (Tango::DevLong)( (3 << 8) | SL_NCHAR );
    argout = serialDS->command_inout("DevSerReadChar",argin);
    argout >> vcharr;

    short errCode = vcharr[0];
    char errStr[256];
    if( errCode<=0 || errCode>=nbError ) {
      sprintf(errStr,"Unknow modbus error code [%d]",errCode);
    } else {
      strcpy(errStr,modbusError[errCode]);
    }

    Tango::Except::throw_exception(
      (const char *)"ModbusRTU::error_read",
      (const char *)errStr,
      (const char *)"ModbusRTU::SendGet");	      
  
  }

  response[0] = frame[1];
  // Function code echoed correctly, read rest of response

  size_t ncharexp = (response_length+1);
  argin << (Tango::DevLong)( (ncharexp << 8) | SL_NCHAR );
  argout = serialDS->command_inout("DevSerReadChar",argin);
  argout >> vcharr;
  size_t nchar = vcharr.size();

  for(size_t i=0;i<nchar;i++)
    frame[2+i] = vcharr[i];
	
  if( ncharexp != nchar ) {
    LogError("Missing char",query,query_length,frame,nchar+2);
    Tango::Except::throw_exception(
   	  (const char *)"ModbusRTU::error_init",
       	  (const char *)"Unexpected message size (missing char)",
       	  (const char *)"ModbusRTU::Send");  
  }

  CalculateCRC(frame, nchar, crc);

  if ((crc[0] != frame[nchar]) && (crc[1] != frame[nchar+1]))
  {	
    LogError("Invalid CRC",query,query_length,frame,response_length+3);
    Tango::Except::throw_exception(
      (const char *)"ModbusRTU::error_read",
      (const char *)"Invalid CRC",
      (const char *)"ModbusRTU::SendGet");    
  }

  for (size_t i=1; i<(size_t)response_length; i++)
    response[i] = frame[i+1];

}

// -------------------------------------------------------

void ModbusRTU::Send ( unsigned char *query, 
  	               short query_length) {

  unsigned char frame[MAX_FRAME_SIZE], crc[2];
  
  if( serialDS==NULL ) {
     Tango::Except::throw_exception(
   	  (const char *)"ModbusRTU::error_init",
       	  (const char *)"Serial device not imported",
       	  (const char *)"ModbusRTU::Send");  
  }
 
  Tango::DeviceData argin;
  vector<unsigned char> vcharr;

  // Add CRC
  size_t iframe = 0;
  frame[iframe++] = node;
  for(size_t i=0; i<(size_t)query_length; i++)
    frame[iframe++] = query[i];
  CalculateCRC(frame, query_length+1, crc);
  frame[iframe++] = crc[0];
  frame[iframe++] = crc[1];

  // Build argin
  vcharr.assign(frame,frame+iframe);
   
  // flush the write and the read buffer to avoid
  // and pending data!
  argin << (Tango::DevLong)2;
  serialDS->command_inout("DevSerFlush", argin);
	 
  // write the frame
  argin << vcharr;
  serialDS->command_inout("DevSerWriteChar",argin);
 
}

// -------------------------------------------------------

void ModbusRTU::CalculateCRC (unsigned char *frame, short frame_length, unsigned char *crc)
{

  // Table of CRC values for high-order byte

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
  };

  // Table of CRC values for low-order byte

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
  };

  unsigned char uchCRCHi = 0xFF ; // high CRC byte initialized
  unsigned char uchCRCLo = 0xFF ; // low CRC byte initialized
  unsigned uIndex ;               // will index into CRC lookup table

  while (frame_length--)  // pass through message buffer
  {
    // calculate the CRC
    uIndex = uchCRCHi ^ *frame++ ;        
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
    uchCRCLo = auchCRCLo[uIndex] ;
  }

  crc[0] = uchCRCHi;
  crc[1] = uchCRCLo;

}

// -------------------------------------------------------

void ModbusRTU::LogError(const char *msg,unsigned char *inFrame,short inFrameLgth,unsigned char *outFrame,short outFrameLgth) {

  if( logFileName.length()==0 )
    return;
    
  time_t now = time(NULL);
  
  FILE *log = fopen(logFileName.c_str(),"a");
  
  if(log==NULL)
    return;
  
  fprintf(log,"-------------------------------------------------\n");
  fprintf(log,"Failure (%s) at %s\n",msg,ctime(&now));
  
  for(int i=0;i<inFrameLgth;i+=16) {
    fprintf(log,"Send: %04X ",i);
    for(int j=0;j<16;j++) {
      int idx = i+j;
      if(idx<inFrameLgth) fprintf(log,"%02X ",inFrame[idx]);
    }
    fprintf(log,"\n");
  }

  if( outFrameLgth==0 ) {
    fprintf(log,"Recv: No response \n");  
  } else {
    for(int i=0;i<outFrameLgth;i+=16) {
      fprintf(log,"Recv: %04X ",i);
      for(int j=0;j<16;j++) {
        int idx = i+j;
        if(idx<outFrameLgth) fprintf(log,"%02X ",outFrame[idx]);
      }
      fprintf(log,"\n");
    }
  }
  
  fclose(log);
  
}

// ---------------------------------------------------------------------
// Modbus TCP class
// ---------------------------------------------------------------------

#define WAIT_FOR_READ  1
#define WAIT_FOR_WRITE 2

#ifndef WIN32
#define closesocket close
#endif

// -------------------------------------------------------

ModbusTCP::ModbusTCP(std::string ipHost,short port,short node,double tcpTimeout,double connectTimeout,bool tcpNoDelay,bool tcpQuickAck, bool tcpKeepAlive) {

  this->ipHost = ipHost;
  this->tcpTimeout = (int)(tcpTimeout * 1000.0);
  this->connectTimeout = (int)(connectTimeout * 1000.0);
  this->tcpNoDelay = tcpNoDelay;
  this->tcpQuickAck = tcpQuickAck;
  this->tcpKeepAlive = tcpKeepAlive;
  this->node = node;
  this->port = port;
  lastError = "";
  sock = -1;
  tickStart = -1;
  lastConnectTry = -5000;

  // Resolve IP
  struct hostent *host_info;
  host_info = gethostbyname(ipHost.c_str());
  if (host_info == NULL) {
     lastError = "ModbusTCP: Unknown host: " + ipHost;
     hostInfo = NULL;
     hostInfoLength = 0;
  } else {
    hostInfoLength = host_info->h_length;
    hostInfo = (char *)malloc(hostInfoLength);
    memcpy(hostInfo,host_info->h_addr,hostInfoLength);
    hostAddrType=host_info->h_addrtype;
  }

}

// -------------------------------------------------------

ModbusTCP::~ModbusTCP() {
  Disconnect();
  if(hostInfo) free(hostInfo);
}

// -------------------------------------------------------

Tango::DevState ModbusTCP::State() {

  if(!IsConnected()) {
    return Tango::UNKNOWN;
  } else {
    return Tango::ON;
  }

}

// -------------------------------------------------------

string ModbusTCP::Status() {

  char str[512];
  sprintf(str,"Modbus node address %d protocol TCP\n",node);
  if(!IsConnected()) {
    strcat(str,lastError.c_str());
  } else {
    char portStr[256];
    sprintf(portStr,":%d",port);
    strcat(str,"Connected to ");
    strcat(str,ipHost.c_str());
    strcat(str,portStr);
  }
  return string(str);

}

// -------------------------------------------------------

int ModbusTCP::WaitFor(int sock,int timeout,int mode) {

  fd_set fdset;
  fd_set *rd = NULL, *wr = NULL;
  struct timeval tmout;
  int result;

  FD_ZERO (&fdset);
  FD_SET (sock, &fdset);
  if (mode == WAIT_FOR_READ)
    rd = &fdset;
  if (mode == WAIT_FOR_WRITE)
    wr = &fdset;

  tmout.tv_sec  = (int)(timeout / 1000);
  tmout.tv_usec = (int)(timeout % 1000) * 1000;

  do
    result = select (sock + 1, rd, wr, NULL, &tmout);
  while (result < 0 && errno == EINTR);

  if( result==0 ) {
    lastError = "ModbusTCP: The operation timed out";
  } else if ( result < 0 ) {
    lastError = "ModbusTCP [";
    if (mode == WAIT_FOR_READ) 
        lastError += "WAIT_FOR_READ";
    else
        lastError += "WAIT_FOR_WRITE";
    lastError += "]: " + string(strerror(errno));
    return 0;
  }

  return result;

}

// ----------------------------------------------------------------------------

int ModbusTCP::Write(int sock, char *buf, int bufsize,int timeout) { // Timeout in millisec

  int total_written = 0;
  int written = 0;

  while( bufsize > 0 )
  {
    // Wait
    if (!WaitFor(sock, timeout, WAIT_FOR_WRITE))
      return -1;

    // Write
    do
      written = send(sock, buf, bufsize, 0);
    while (written == -1 && errno == EINTR);

    if( written < 0 )
       break;

    buf += written;
    total_written += written;
    bufsize -= written;
  }

  if( written < 0 ) {    
    lastError = "ModbusTCP [Write]: " + string(strerror(errno));
    return -1;
  }
  
  if( bufsize != 0 ) {
    lastError = "ModbusTCP [Write]: Failed to send entire buffer";
    return -1;
  }

  return total_written;

}

// ----------------------------------------------------------------------------

int ModbusTCP::Read(int sock, char *buf, int bufsize,int timeout) { // Timeout in millisec

  int rd = 0;
  int total_read = 0;  

#ifndef WIN32
  int optval = 1; 
  socklen_t optlen = sizeof(optval);

  if(tcpQuickAck)
  {
    // Enables TCP Quick Acknowledgements 
    // Since this flag is not permanent, this should be done before each recv call.
    setsockopt(sock, IPPROTO_TCP, TCP_QUICKACK, &optval, optlen);
  }
#endif

  //while( bufsize>0 ) {

    // Wait
    if (!WaitFor(sock, timeout, WAIT_FOR_READ)) {
      return -1;
    }

    // Read
    do
      rd = recv(sock, buf, bufsize, 0);
    while (rd == -1 && errno == EINTR);

    //if( rd <= 0 )
    //  break

    buf += rd;
    total_read += rd;
    bufsize -= rd;
  
  //}

  if( rd < 0 ) {
    lastError = "ModbusTCP [READ]: " + string(strerror(errno));
    return -1;
  }
  
  
  return total_read;

}

// ----------------------------------------------------------------------------

bool ModbusTCP::Connect(int *retSock) {

  *retSock = -1;

  struct sockaddr_in server;

  // Already connected ?
  if( IsConnected() )
    return true;

  // Host resolution succedded ?
  if( hostInfo==NULL )
    return false;

  // Avoid too much connection attemp
  time_t now = get_ticks();
  if( (now - lastConnectTry) < 2000 )
    return false;
  lastConnectTry = now;  

  // Build TCP connection
  int sock = socket(AF_INET, SOCK_STREAM,IPPROTO_TCP);
  if (sock < 0 ) {
    lastError = "ModbusTCP: Socket error: " + string(strerror(errno));
    return false;
  }

  // Use non blocking socket
#ifdef WIN32
  unsigned long iMode = 0;
  if (ioctlsocket(sock,FIONBIO,&iMode) != 0)
  {
#else
  if (fcntl(sock, F_SETFL, O_NONBLOCK) == -1) {
#endif
    lastError = "ModbusTCP: Cannot use non blocking socket";
    Disconnect();
    return false;
  }
  
  // Connect
  memset(&server,0,sizeof(sockaddr_in));
  server.sin_family = hostAddrType;
  memcpy((char*)&server.sin_addr, hostInfo,hostInfoLength);
  server.sin_port=htons(port > 0 ? port : 502);

  int connectStatus = connect(sock,(struct sockaddr *)&server, sizeof(server) );

  if( (connectStatus < 0) && (errno != EINPROGRESS) ) {
    lastError = "ModbusTCP: Cannot connect to host: " + string(strerror(errno));
    Disconnect();
  }

  if( connectStatus<0 ) {

    // Wait for connection
    if (!WaitFor(sock, connectTimeout, WAIT_FOR_WRITE)) {
      lastError = "ModbusTCP: Cannot connect, unreachable host " + ipHost;
      Disconnect();
      return false;
    }

    // Check connection completion
    int socket_err;
#ifdef WIN32
	int serrlen = sizeof socket_err;
	if (getsockopt(sock, SOL_SOCKET, SO_ERROR, (char *)&socket_err, &serrlen) != 0) {
#else
    socklen_t serrlen = sizeof(socket_err);
    if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &socket_err, &serrlen) == -1) {
#endif
      lastError = "ModbusTCP: Cannot connect to host: " + string(strerror(errno));
      Disconnect();
      return false;
    }

    if (socket_err != 0) 	{
      lastError = "ModbusTCP: Cannot connect to host: " + string(strerror(socket_err));
      Disconnect();
      return false;
    }

  }
  
  int on = 1;
  if ( setsockopt (sock, SOL_SOCKET, SO_REUSEADDR, 
		   (const char*) &on, sizeof (on)) == -1) {
    lastError = "ModbusTCP: Socket error: setsockopt error SO_REUSEADDR";
    Disconnect();
    return false; 
  }

  if( tcpNoDelay ) {
    int flag = 1;
    struct protoent *p;
    p = getprotobyname("tcp");
    if ( setsockopt( sock, p->p_proto, TCP_NODELAY, (char *)&flag, sizeof(flag) ) < 0 ) {
      lastError = "ModbusTCP: Socket error: setsockopt error TCP_NODELAY";
      Disconnect();
      return false; 
    }
  }

  if( tcpKeepAlive )
  {
    int optval = 1; 
    socklen_t optlen = sizeof(optval);  
    if ( setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0 ) {
      lastError = "ModbusTCP: Socket error: setsockopt error TCP_KEEPALIVE";
      Disconnect();
      return false; 
    }
  }
  
  *retSock = sock;
  
  return true;
}

// -------------------------------------------------------

void ModbusTCP::Disconnect() {
  if( !IsConnected() ) {
    // best effort close (closesocket certainly don't throw any exception - but anyway, not a big deal)
    try { closesocket(sock); } catch (...) {}
    sock = -1;
  }
}

// -------------------------------------------------------

bool ModbusTCP::IsConnected() {
  return -1 != sock;
}

// -------------------------------------------------------

void ModbusTCP::Send ( unsigned char *query, 
  	               short query_length) {

  unsigned char frame[MAX_FRAME_SIZE];
  int iframe;

  iframe=0;
  frame[iframe++] = node;

  for(int i=0; i<4; i++)
    frame[iframe++] = 0;

  frame[iframe++] = query_length+1; // number of bytes
  frame[iframe++] = node;

  for (int i=0; i<query_length; i++)
    frame[iframe++] = query[i];

  // Connect
  if(!IsConnected()) 
  {
    if( !Connect(&sock) ) 
    {
      Tango::Except::throw_exception(
        (const char *)"ModbusTCP::error_write",
        (const char *)lastError.c_str(),
        (const char *)"ModbusTCP::Send (connect)");
    }
  }

  if( Write( sock , (char *)frame , iframe , tcpTimeout ) < 0 ) 
  {
      // Transmission error, we need to reconnect
      Disconnect();
      Tango::Except::throw_exception(
        (const char *)"ModbusTCP::error_write",
        (const char *)lastError.c_str(),
        (const char *)"ModbusTCP::Send (write)");
 
  }
  
}

// -------------------------------------------------------

time_t ModbusTCP::get_ticks() {
      
#ifdef WIN32
    if(tickStart < 0 )
      tickStart = (time_t)GetTickCount();
	return (time_t)GetTickCount();
#else
    if(tickStart < 0 )
      tickStart = time(NULL);

    struct timeval tv;
    gettimeofday(&tv,NULL);
    return ( (tv.tv_sec-tickStart)*1000 + tv.tv_usec/1000 );
#endif

}  

// -------------------------------------------------------

void ModbusTCP::SendGet (unsigned char *query, 
	                 short query_length,
	                 unsigned char *response, 
	                 short response_length) {

  unsigned char frame[MAX_FRAME_SIZE];

  Send(query,query_length);

  int nbRead = Read( sock , (char *)frame , MAX_FRAME_SIZE , tcpTimeout );

  if( nbRead==0 ) {
    // Connection 'gracefully' closed by peer !
    // Retry
    Disconnect();
    Send(query,query_length);
    nbRead = Read( sock , (char *)frame , MAX_FRAME_SIZE , tcpTimeout );
  }

  if( nbRead < 0 ) {
      // Transmission error, we need to reconnect
      Disconnect();
      Tango::Except::throw_exception(
        (const char *)"ModbusTCP::error_read",
        (const char *)lastError.c_str(),
        (const char *)"ModbusTCP::SendGet");
  }

  if( nbRead < 9 ) {
    char errStr[256];
    sprintf(errStr,"Unexpected response length [%d bytes]",nbRead);
    Tango::Except::throw_exception(
      (const char *)"ModbusTCP::error_read",
      (const char *)errStr,
      (const char *)"ModbusTCP::SendGet");
  }

  if (frame[7] & 0x80) {

    // We got a modbus error

    short errCode = frame[8];
    char errStr[256];
    if( errCode<=0 || errCode>=nbError ) {
      sprintf(errStr,"Unknow modbus error code [%d]",errCode);
    } else {
      strcpy(errStr,modbusError[errCode]);
    }

    Tango::Except::throw_exception(
      (const char *)"ModbusTCP::error_read",
      (const char *)errStr,
      (const char *)"ModbusTCP::SendGet");	      
  
  }

  if(nbRead<response_length+7) {
    char errStr[256];
    sprintf(errStr,"Unexpected response length [%d bytes, %d expected]",nbRead,response_length+7);
    Tango::Except::throw_exception(
      (const char *)"ModbusTCP::error_read",
      (const char *)errStr,
      (const char *)"ModbusTCP::SendGet");
  }

  for (size_t i=0; i<(size_t)response_length; i++)
    response[i] = frame[i+7];

}

