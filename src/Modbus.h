/*----- PROTECTED REGION ID(Modbus.h) ENABLED START -----*/
//=============================================================================
//
// file :        Modbus.h
//
// description : Include file for the Modbus class
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
// $HeadURL:  $
//
//=============================================================================
//                This file is generated by POGO
//        (Program Obviously used to Generate tango Object)
//=============================================================================


#ifndef Modbus_H
#define Modbus_H

#include <tango.h>
#include "ModbusCore.h"
#include "CacheThread.h"


/*----- PROTECTED REGION END -----*/	//	Modbus.h

/**
 *  Modbus class description:
 *    A Class to handle the modbus protocol over TCP/IP or Serial (RTU).
 */

namespace Modbus_ns
{
/*----- PROTECTED REGION ID(Modbus::Additional Class Declarations) ENABLED START -----*/

//	Additional Class Declarations

/*----- PROTECTED REGION END -----*/	//	Modbus::Additional Class Declarations

class Modbus : public TANGO_BASE_CLASS
{

/*----- PROTECTED REGION ID(Modbus::Data Members) ENABLED START -----*/

//	Add your own data members

	ModbusCore *modbusCore;

	CacheThread				*theThread;
	ThreadCmd				thCmd;
	omni_mutex				thCmdMutex;
	vector<CacheDataBlock>			cacheDef;
	int					thId;
	Tango::DevLong				maxDeltaTh;
	omni_mutex				rhr_mutex;
	omni_mutex				rir_mutex;
	omni_mutex				rmcs_mutex;

	std::string error_;

	void check_argin(const Tango::DevVarShortArray *argin,int lgth,const char *where);
	int get_data_block(const char *,short,short);
	void get_cache_data(int data_block,short input_address,short no_inputs,Tango::DevVarShortArray *argout);

/*----- PROTECTED REGION END -----*/	//	Modbus::Data Members

//	Device property data members
public:
	//	Protocol:	RTU => Binary serial communication.
	//  TCP => Communication over ethernet.
	string	protocol;
	//	Iphost:	The host IP address used with the TCP protocol
	string	iphost;
	//	Serialline:	The name of the serial line device used with RTU protocol
	string	serialline;
	//	Address:	Node index used with the RTU or TCP protocol
	Tango::DevShort	address;
	//	CacheConfig:	Describe which data has to be cached.
	//  Each set of cached data is described by 3 parameters which are:
	//  1 - Command to be used to read data (ReadHoldingRegisters, ReadInputStatus
	//  ReadInutRegisters or ReadMultipleCoilStatus)
	//  2 - First address to be read
	//  3 - Number of data to read
	vector<string>	cacheConfig;
	//	CacheSleep:	Cache update thread main loop sleeping time (in ms)CacheSleep
	Tango::DevLong	cacheSleep;
	//	TCPConnectTimeout:	TCP connection timeout (in sec)
	Tango::DevDouble	tCPConnectTimeout;
	//	TCPTimeout:	Timeout used when the TCP protocol is used (in sec)
	Tango::DevDouble	tCPTimeout;
	//	LogFile:	Name of the file where are stored invalid frame
	string	logFile;
	//	TCPNoDelay:	Disable or enable Nagle`s algorithm.
	Tango::DevBoolean	tCPNoDelay;
	//	TCPQuickAck:	Set this property to true to enable TCP quick acknowledgements
	Tango::DevBoolean	tCPQuickAck;
	//	NumberOfRetry:	Number Of Retry for all command in case failed.
	Tango::DevShort	numberOfRetry;
	//	SleepBetweenRetry:	Sleep Between Retry in Miliseconds
	Tango::DevShort	sleepBetweenRetry;
	//	TCPKeepAlive:	Allow to enable/disable the TCP Keep Alive socket option.
	//  Defaults to False.
	Tango::DevBoolean	tCPKeepAlive;


//	Constructors and destructors
public:
	/**
	 * Constructs a newly device object.
	 *
	 *	@param cl	Class.
	 *	@param s 	Device Name
	 */
	Modbus(Tango::DeviceClass *cl,string &s);
	/**
	 * Constructs a newly device object.
	 *
	 *	@param cl	Class.
	 *	@param s 	Device Name
	 */
	Modbus(Tango::DeviceClass *cl,const char *s);
	/**
	 * Constructs a newly device object.
	 *
	 *	@param cl	Class.
	 *	@param s 	Device name
	 *	@param d	Device description.
	 */
	Modbus(Tango::DeviceClass *cl,const char *s,const char *d);
	/**
	 * The device object destructor.
	 */
	~Modbus() {delete_device();};


//	Miscellaneous methods
public:
	/*
	 *	will be called at device destruction or at init command.
	 */
	void delete_device();
	/*
	 *	Initialize the device
	 */
	virtual void init_device();
	/*
	 *	Read the device properties from database
	 */
	void get_device_property();
	/*
	 *	Always executed method before execution command method.
	 */
	virtual void always_executed_hook();


//	Attribute methods
public:
	//--------------------------------------------------------
	/*
	 *	Method      : Modbus::read_attr_hardware()
	 *	Description : Hardware acquisition for attributes.
	 */
	//--------------------------------------------------------
	virtual void read_attr_hardware(vector<long> &attr_list);


	//--------------------------------------------------------
	/**
	 *	Method      : Modbus::add_dynamic_attributes()
	 *	Description : Add dynamic attributes if any.
	 */
	//--------------------------------------------------------
	void add_dynamic_attributes();




//	Command related methods
public:
	/**
	 *	Command ForceSingleCoil related method
	 *	Description: Write single coil (digital I/O) state.
	 *
	 *	@param argin aring[0] = coil address
	 *               argin[1] = coil value (0/1)
	 */
	virtual void force_single_coil(const Tango::DevVarShortArray *argin);
	virtual bool is_ForceSingleCoil_allowed(const CORBA::Any &any);
	/**
	 *	Command ReadCoilStatus related method
	 *	Description: Read coil (digital I/O) status
	 *
	 *	@param argin Coil address
	 *	@returns Coil status (0/1)
	 */
	virtual Tango::DevShort read_coil_status(Tango::DevShort argin);
	virtual bool is_ReadCoilStatus_allowed(const CORBA::Any &any);
	/**
	 *	Command ReadInputStatus related method
	 *	Description: Read discrete input status. Return one boolean per array element.
	 *
	 *	@param argin argin[0] = Input address
	 *               argin[1] = number of inputs
	 *	@returns argout[0..n-1] = Input status (0/1)
	 */
	virtual Tango::DevVarCharArray *read_input_status(const Tango::DevVarShortArray *argin);
	virtual bool is_ReadInputStatus_allowed(const CORBA::Any &any);
	/**
	 *	Command ReadHoldingRegisters related method
	 *	Description: Read multiple 16bits registers.
	 *
	 *	@param argin aring[0] = Register start address
	 *               argin[1] = Number of registers
	 *	@returns argout[0..n-1] Holding 16bits registers.
	 */
	virtual Tango::DevVarShortArray *read_holding_registers(const Tango::DevVarShortArray *argin);
	virtual bool is_ReadHoldingRegisters_allowed(const CORBA::Any &any);
	/**
	 *	Command ReadInputRegisters related method
	 *	Description: Read Multiple 16bits input registers.
	 *
	 *	@param argin argin[0] = Register start address
	 *               argin[1] = Number of registers
	 *	@returns argout[0..n-1] = Input 16bits registers
	 */
	virtual Tango::DevVarShortArray *read_input_registers(const Tango::DevVarShortArray *argin);
	virtual bool is_ReadInputRegisters_allowed(const CORBA::Any &any);
	/**
	 *	Command PresetSingleRegister related method
	 *	Description: Write single 16bits register.
	 *
	 *	@param argin argin[0] = Register address
	 *               argin[1] = Register value
	 */
	virtual void preset_single_register(const Tango::DevVarShortArray *argin);
	virtual bool is_PresetSingleRegister_allowed(const CORBA::Any &any);
	/**
	 *	Command FetchCommEventCtr related method
	 *	Description: Fetch communications event counter.
	 *
	 *	@returns argout[0] = Status
	 *           argout[1] = Event count
	 */
	virtual Tango::DevVarShortArray *fetch_comm_event_ctr();
	virtual bool is_FetchCommEventCtr_allowed(const CORBA::Any &any);
	/**
	 *	Command ForceMultipleCoils related method
	 *	Description: 
	 *
	 *	@param argin argin[0] = Coil start address
	 *               argin[1] = Number of coil
	 *               argin[2..n+1] = Coil values
	 */
	virtual void force_multiple_coils(const Tango::DevVarShortArray *argin);
	virtual bool is_ForceMultipleCoils_allowed(const CORBA::Any &any);
	/**
	 *	Command ReadMultipleCoilsStatus related method
	 *	Description: Read multiple coil (digital I/O) status.
	 *
	 *	@param argin argin[0] = Coil start address
	 *               argin[1] = Number of coils
	 *	@returns argout[0..n-1] = Coil values
	 */
	virtual Tango::DevVarShortArray *read_multiple_coils_status(const Tango::DevVarShortArray *argin);
	virtual bool is_ReadMultipleCoilsStatus_allowed(const CORBA::Any &any);
	/**
	 *	Command PresetMultipleRegisters related method
	 *	Description: Write multiple 16bits registers.
	 *
	 *	@param argin argin[0] = Register start address
	 *               argin[1] = Number of register
	 *               argin[2..n+1] = Register values
	 */
	virtual void preset_multiple_registers(const Tango::DevVarShortArray *argin);
	virtual bool is_PresetMultipleRegisters_allowed(const CORBA::Any &any);
	/**
	 *	Command MaskWriteRegister related method
	 *	Description: Mask write a 16bits register.
	 *
	 *	@param argin argin[0] = Register address
	 *               argin[1] = AND mask
	 *               argin[2] = OR mask
	 */
	virtual void mask_write_register(const Tango::DevVarShortArray *argin);
	virtual bool is_MaskWriteRegister_allowed(const CORBA::Any &any);
	/**
	 *	Command ReadWriteRegister related method
	 *	Description: Read and Write multiple 16bits registers.
	 *
	 *	@param argin argin[0] = Read start address
	 *               argin[1] = Number of registers to read
	 *               argin[2] = Write start address
	 *               argin[3] = Number of registers to write
	 *               argin[4..n+3] = Register values
	 *	@returns argout[0..n-1] = Register values
	 */
	virtual Tango::DevVarShortArray *read_write_register(const Tango::DevVarShortArray *argin);
	virtual bool is_ReadWriteRegister_allowed(const CORBA::Any &any);
	/**
	 *	Command PresetSingleRegisterBroadcast related method
	 *	Description: Write single 16bits register at node 0 (Node reserved for broadcast for RTU protocol on RS485 line)
	 *               Does not wait for the equipment response.
	 *
	 *	@param argin argin[0] = Register address
	 *               argin[1] = Register value
	 */
	virtual void preset_single_register_broadcast(const Tango::DevVarShortArray *argin);
	virtual bool is_PresetSingleRegisterBroadcast_allowed(const CORBA::Any &any);
	/**
	 *	Command ReadExceptionStatus related method
	 *	Description: Read exception status (usually a predefined range of 8 bits
	 *
	 *	@returns Exception status
	 */
	virtual Tango::DevShort read_exception_status();
	virtual bool is_ReadExceptionStatus_allowed(const CORBA::Any &any);


	//--------------------------------------------------------
	/**
	 *	Method      : Modbus::add_dynamic_commands()
	 *	Description : Add dynamic commands if any.
	 */
	//--------------------------------------------------------
	void add_dynamic_commands();

/*----- PROTECTED REGION ID(Modbus::Additional Method prototypes) ENABLED START -----*/

//	Additional Method prototypes
        void SendGet(unsigned char *query, short query_length, 
	         unsigned char *response, short response_length);

/*----- PROTECTED REGION END -----*/	//	Modbus::Additional Method prototypes
};

/*----- PROTECTED REGION ID(Modbus::Additional Classes Definitions) ENABLED START -----*/

//	Additional Classes Definitions

/*----- PROTECTED REGION END -----*/	//	Modbus::Additional Classes Definitions

}	//	End of namespace

#endif   //	Modbus_H
