//+*********************************************************************
//
// File:        ModbusCoreSL.cpp
//
// Project:     Device Servers in C++
//
// Description: public include file containing definitions and declarations
//		of serial line access function.
//
// Author(s);   Andy Gotz
//
// Original:    August 2001
//
// $Log: not supported by cvs2svn $
// Revision 1.4  2008/11/19 14:09:06  jean_coquet
// #ifdef changed in case of NOSERIAL special case for SOLEIL (in ModbusCoreSL.cpp .h)
// so now it compiles and runs under W32
//
// Revision 1.3  2008/06/24 10:41:12  buteau
// Soleil added an #ifdef to avoid linking with Serial
//
// Revision 1.2  2005/03/31 15:07:05  jlpons
// Changed namespace name
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

#ifndef _ModbusCoreSL_H
#define _ModbusCoreSL_H

#ifdef TACOSL
#include <DevServer.h>
#else
#include <tango.h>
#endif
#ifndef NOSERIAL
#include <Serial.h>
#else
  #define SL_RAW        0     /* raw read/write mode */
  #define SL_NCHAR      1     /* character read/write mode */
  #define SL_LINE       2     /* line read mode */
  #define SL_RETRY      3     /* retry read mode */
#endif


//+=====================================================================
// Global definitions
//-=====================================================================
#ifndef OK
#define OK              0
#define NOTOK           (-1)
#endif


//+=====================================================================
// Class definition
//-=====================================================================

class ModbusCoreSL {


//
// public members
//
public:

   ModbusCoreSL (
        char  *serialline_name,
        long  *error);
   ~ModbusCoreSL ();
   
   long write(
	unsigned char *frameout, 
	long int ncharout, 
	long *error);
   long read (
	unsigned char *framein,  
	long int ncharexp, 
	long int *ncharin,  
	long *error);

//
// private members
//
private:
#ifdef TACOSL
   Serial               *serialline_device;     /* file descriptor */
   devserver            serialline_ds;          /* device server */
#else
   Tango::DeviceProxy   *serialline_device;     /* Tango device handle */
#endif

//
// protected members
//
protected:
};

#endif /* _ModbusCoreSL_H */
