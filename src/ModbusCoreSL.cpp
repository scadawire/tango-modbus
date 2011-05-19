//+*********************************************************************
//
// File:        ModbusCoreSL.cpp
// 
// Project:     Device Servers in C++
// 
// Description: Code for implementing the ModbusCoreSL serial line access.
//              The current code uses a TANGO Serial Line Device Server.
//
// Author(s);   JL PONS
//
// Original:    Jan 2005
//
// $Log: not supported by cvs2svn $
// Revision 1.1  2009/02/25 13:58:41  buteau
// - files moved to src subdirectory
//
// Revision 1.7  2008/11/19 14:09:06  jean_coquet
// #ifdef changed in case of NOSERIAL special case for SOLEIL (in ModbusCoreSL.cpp .h)
// so now it compiles and runs under W32
//
// Revision 1.6  2008/07/01 10:54:13  taurel
// - Remove some printf
//
// Revision 1.5  2008/06/24 10:40:47  buteau
// Soleil added an #ifdef to avoid linking with Serial
//
// Revision 1.4  2006/11/02 10:33:32  fbecheri
// Try/Catch missing.
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
// 
//
// Copyright (c) 2001 by European Synchrotron Radiation Facility,
//                       Grenoble, France
// 
//-*********************************************************************

#include <ModbusCoreSL.h>


//+======================================================================
// Function:    ModbusCoreSL::ModbusCoreSL()
//
// Description: create a ModbusCoreSL object
//
//-=====================================================================
ModbusCoreSL::ModbusCoreSL (
        char  *serialline_name,
        long  *error)
{

   Tango::DeviceData argin;
	
   *error=0;
   serialline_device = NULL;
	
    // Create the handle
    serialline_device = new Tango::DeviceProxy(serialline_name);
	  
    // Flush the serial
    argin << (long)2;
    try{
	   	if( serialline_device!=NULL )
	   		serialline_device->command_inout("DevSerFlush",argin);
		else 	
			*error=NOTOK;
	}
	catch(Tango::DevFailed &e)
	{
		*error=NOTOK;
	}
		
}


//+=====================================================================
// Function:    ModbusCoreSL::~ModbusCoreSL()
//
// Description: destructor to destroy an object of the ModbusCoreSL class
//
//-=====================================================================
ModbusCoreSL::~ModbusCoreSL()
{
  if(serialline_device)
    delete serialline_device;
}



//+=====================================================================
// Function:    ModbusCoreSL::write()
//
// Description: 
//
// input:       none
//
// Output:      none
//
//-=====================================================================
long ModbusCoreSL::write(
	unsigned char *frame, 
	long          ncharout,
	long          *error)
{

 if( serialline_device==NULL ) {
 	*error=NOTOK;
 	return NOTOK;
 	/*
    Tango::Except::throw_exception(
   	  (const char *)"ModbusCoreSL::error_init",
       	  (const char *)"Serial device not imported.",
       	  (const char *)"ModbusCoreSL::write");   */
	  
 }
 
 Tango::DeviceData argin;
 Tango::DevVarCharArray vcharr;

 vcharr.length(ncharout);
 for(int i=0;i<ncharout;i++) {
   vcharr[i] = frame[i];
  // printf("writeSL Frame[%2d] %02X\n",i,frame[i]);
 }
   
 *error = 0;
  
 try{
 	 // flush the write and the read buffer to avoid
	 // and pending data!
	 argin << (Tango::DevLong)2;
	 serialline_device->command_inout("DevSerFlush", argin);
	 
	 // write the frame
	 argin << vcharr;
	 serialline_device->command_inout("DevSerWriteChar",argin);
 }
 catch(Tango::DevFailed &e)
 {
 	*error=NOTOK;
 	return NOTOK;
 }
 
 return OK;

}

//+=====================================================================
// Function:    ModbusCoreSL::read()
//
//-=====================================================================
long ModbusCoreSL::read(
	unsigned char *frame, 
	long          ncharexp,
	long          *ncharin,
	long          *error)
{

 if( serialline_device==NULL ) {
 	*error=NOTOK;
 	return NOTOK;
 	/*
    Tango::Except::throw_exception(
   	  (const char *)"ModbusCoreSL::error_init",
       	  (const char *)"Serialline device not imported.",
       	  (const char *)"ModbusCoreSL::read");*/

 }
 Tango::DeviceData argin;
 Tango::DeviceData argout;
 const Tango::DevVarCharArray *vcharr;

 argin << (long)((ncharexp << 8) | SL_NCHAR);
 try{
 	argout = serialline_device->command_inout("DevSerReadChar",argin);
 }
 catch(Tango::DevFailed &e)
 {
 	*error=NOTOK;
 	return NOTOK;
 }
 argout >> vcharr;

 *ncharin = vcharr->length();
 for(int i=0;i<vcharr->length();i++) {
   frame[i] = (*vcharr)[i];
  // printf("readSL Frame[%2d] %02X\n",i,frame[i]);
 }
 return OK;

}

