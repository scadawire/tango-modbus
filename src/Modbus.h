//=============================================================================
//
// file :        Modbus.h
//
// description : Include for the Modbus class.
//
// project :	Modbus
//
// $Author: buteau $
//
// $Revision: 1.1 $
//
// $Log: not supported by cvs2svn $
// Revision 1.10  2008/12/01 08:02:44  taurel
// - Fix a bug in the WriteCoil command
// - Fix warnings generated by gcc 4.3
//
// Revision 1.9  2008/07/18 10:40:19  taurel
// - Add a new TCPTimeout property used during communication between
// the device and the Modbus equipment
//
// Revision 1.8  2008/07/03 09:16:26  jensmeyer
// Added a SocketConnectionSleep property to define the wait time between
// a socket closing and the reopening of a new socket to connect to the hardware.
//
// Revision 1.7  2008/06/06 13:51:13  taurel
// - Better management of error when using the cache thread
// - Fix open file leak in case of TCP protocol and reconnection to the
// modbus device
//
// Revision 1.6  2008/03/17 14:37:35  taurel
// - Add a data cache for the ReadHoldingRegisters, ReadMultipleCoilsStatus, ReadInputRegisters and ReadInputStatus commands
// - Fix some bugs related to closing the sockets in TCP mode
// - The Address property is used for the modbus ID sent at the frame
// beginning
//
// Revision 1.5  2007/08/07 14:57:22  jensmeyer
// Regenerated the source code with pogo to add property descriptions and
// default values for the use with the configuration wizard.
//
// Revision 1.4  2006/12/01 15:04:31  jensmeyer
// Added the command ReadMultipleCoilsStatus to read several coils (bits)
// at the same time.
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
//
// copyleft :    European Synchrotron Radiation Facility
//               BP 220, Grenoble 38043
//               FRANCE
//
//=============================================================================
//
//  		This file is generated by POGO
//	(Program Obviously used to Generate tango Object)
//
//         (c) - Software Engineering Group - ESRF
//=============================================================================
#ifndef _MODBUS_H
#define _MODBUS_H

#include <tango.h>
#include <ModbusCore.h>
#include <CacheThread.h>

//using namespace Tango;

/**
 * @author	$Author: buteau $
 * @version	$Revision: 1.1 $
 */

 //	Add your own constants definitions here.
 //-----------------------------------------------


namespace Modbus_ns
{

/**
 * Class Description:
 * A Class to handle the modbus protocol over TCP/IP or Serial (RTU).
 */

/*
 *	Device States Description:
 */


class Modbus: public Tango::Device_3Impl
{
public :
	//	Add your own data members here
	//-----------------------------------------


	//	Here is the Start of the automatic code generation part
	//-------------------------------------------------------------	
/**
 *	@name attributes
 *	Attributs member data.
 */
//@{
//@}

/**
 *	@name Device properties
 *	Device properties member data.
 */
//@{
/**
 *	RTU'' : Binary serial communication.
 *	''TCP'' : Communication over ethernet.
 */
	string	protocol;
/**
 *	The host IP address  used with the TCP protocol in the form
 *	aa.bb.cc.dd.
 */
	string	iphost;
/**
 *	The name of the serial line device used with RTU protocol.
 *	This can be any device name of a Serial Class object in the Tango
 *	system.
 *	
 */
	string	serialline;
/**
 *	Node index used with the RTU or TCP protocol
 */
	Tango::DevShort	address;
/**
 *	Describe which data has to be cached.
 *	Each set of cached data is described by 3 parameters which are:
 *	1 - Command to be used to read data (ReadHoldingRegisters, ReadInputStatus
 *	ReadInutRegisters or ReadMultipleCoilStatus)
 *	2 - First address to be read
 *	3 - Number of data to read
 */
	vector<string>	cacheConfig;
/**
 *	Cache update thread main loop sleeping time (in ms)
 */
	Tango::DevLong	cacheSleep;
/**
 *	The necessary sleep time between closing a connection (Socket) and 
 *	opening a new connection. To avoid hang-ups a non blocking socket
 *	is used to check the availability on the network. Afterwards the non blocking
 *	socket is closed and a blocking socket will be opened.
 *	The SocketConnectionSleep time specified the wait time in ms between
 *	these two connections.
 */
	Tango::DevLong	socketConnectionSleep;
/**
 *	Timeout used when the TCP protocol is used (in sec)
 */
	Tango::DevLong	tCPTimeout;
//@}

/**@name Constructors
 * Miscellaneous constructors */
//@{
/**
 * Constructs a newly allocated Command object.
 *
 *	@param cl	Class.
 *	@param s 	Device Name
 */
	Modbus(Tango::DeviceClass *cl,string &s);
/**
 * Constructs a newly allocated Command object.
 *
 *	@param cl	Class.
 *	@param s 	Device Name
 */
	Modbus(Tango::DeviceClass *cl,const char *s);
/**
 * Constructs a newly allocated Command object.
 *
 *	@param cl	Class.
 *	@param s 	Device name
 *	@param d	Device description.
 */
	Modbus(Tango::DeviceClass *cl,const char *s,const char *d);
//@}

/**@name Destructor
 * Only one desctructor is defined for this class */
//@{
/**
 * The object desctructor.
 */	
	~Modbus() {delete_device();};
/**
 *	will be called at device destruction or at init command.
 */
	void delete_device();
//@}

	
/**@name Miscellaneous methods */
//@{
/**
 *	Initialize the device
 */
	virtual void init_device();
/**
 *	Always executed method befor execution command method.
 */
	virtual void always_executed_hook();

//@}

/**
 * @name Modbus methods prototypes
 */

//@{
/**
 *	Execution allowed for ForceSingleCoil command.
 */
	virtual bool is_ForceSingleCoil_allowed(const CORBA::Any &any);
/**
 *	Execution allowed for ReadCoilStatus command.
 */
	virtual bool is_ReadCoilStatus_allowed(const CORBA::Any &any);
/**
 *	Execution allowed for ReadInputStatus command.
 */
	virtual bool is_ReadInputStatus_allowed(const CORBA::Any &any);
/**
 *	Execution allowed for ReadHoldingRegisters command.
 */
	virtual bool is_ReadHoldingRegisters_allowed(const CORBA::Any &any);
/**
 *	Execution allowed for ReadInputRegisters command.
 */
	virtual bool is_ReadInputRegisters_allowed(const CORBA::Any &any);
/**
 *	Execution allowed for PresetSingleRegister command.
 */
	virtual bool is_PresetSingleRegister_allowed(const CORBA::Any &any);
/**
 *	Execution allowed for ReadExceptionStatus command.
 */
	virtual bool is_ReadExceptionStatus_allowed(const CORBA::Any &any);
/**
 *	Execution allowed for FetchCommEventCtr command.
 */
	virtual bool is_FetchCommEventCtr_allowed(const CORBA::Any &any);
/**
 *	Execution allowed for ForceMultipleCoils command.
 */
	virtual bool is_ForceMultipleCoils_allowed(const CORBA::Any &any);
/**
 *	Execution allowed for ReadMultipleCoilsStatus command.
 */
	virtual bool is_ReadMultipleCoilsStatus_allowed(const CORBA::Any &any);
/**
 *	Execution allowed for PresetMultipleRegisters command.
 */
	virtual bool is_PresetMultipleRegisters_allowed(const CORBA::Any &any);
/**
 *	Execution allowed for MaskWriteRegister command.
 */
	virtual bool is_MaskWriteRegister_allowed(const CORBA::Any &any);
/**
 *	Execution allowed for ReadWriteRegister command.
 */
	virtual bool is_ReadWriteRegister_allowed(const CORBA::Any &any);
/**
 * Write single coil (digital I/O) state.
 *	@param	argin	coil address, 0/1
 *	@exception DevFailed
 */
	void	force_single_coil(const Tango::DevVarShortArray *);
/**
 * Read coil (digital I/O) status.
 *	@param	argin	coil address
 *	@return	Coil status
 *	@exception DevFailed
 */
	Tango::DevShort	read_coil_status(Tango::DevShort);
/**
 * Read discrete input status. Return one boolean per array element.
 *	@param	argin	input address, no. of inputs
 *	@return	Input status.
 *	@exception DevFailed
 */
	Tango::DevVarCharArray	*read_input_status(const Tango::DevVarShortArray *);
/**
 * Read multiple 16bits registers.
 *	@param	argin	register address, no. of registers
 *	@return	Holding 16bits register.
 *	@exception DevFailed
 */
	Tango::DevVarShortArray	*read_holding_registers(const Tango::DevVarShortArray *);
/**
 * Read Multiple 16bits input registers.
 *	@param	argin	register address, no. of registers
 *	@return	Input 16bits registers
 *	@exception DevFailed
 */
	Tango::DevVarShortArray	*read_input_registers(const Tango::DevVarShortArray *);
/**
 * Write single 16bits register.
 *	@param	argin	Register address, register value.
 *	@exception DevFailed
 */
	void	preset_single_register(const Tango::DevVarShortArray *);
/**
 * Read exception status (usually a predefined range of 8 bits
 *	@return	exception status
 *	@exception DevFailed
 */
	Tango::DevShort	read_exception_status();
/**
 * Fetch communications event counter.
 *	@return	status, event count
 *	@exception DevFailed
 */
	Tango::DevVarShortArray	*fetch_comm_event_ctr();
/**
 * Write multiple coils (digital I/O) state.
 *	argin[0] = coil_address
 *	argin[1] = number of coils
 *	argin[2] = 1st coil state
 *	argin[3] = 2nd coil state
 *	...
 *	@param	argin	coil address, nb of coils, coil states
 *	@exception DevFailed
 */
	void	force_multiple_coils(const Tango::DevVarShortArray *);
/**
 * Read multiple coil (digital I/O) status.
 *	argin[0] = register address
 *	argin[1] = number of registers
 *	@param	argin	coil address, nb of coils
 *	@return	Status of coils
 *	@exception DevFailed
 */
	Tango::DevVarShortArray	*read_multiple_coils_status(const Tango::DevVarShortArray *);
/**
 * Write multiple 16bits registers.
 *	argin[0] = register address
 *	argin[1] = number of registers
 *	argin[2] = 1st register
 *	argin[3] = 2nd register
 *	...
 *	@param	argin	register address, nb of registers, register data
 *	@exception DevFailed
 */
	void	preset_multiple_registers(const Tango::DevVarShortArray *);
/**
 * Mask write a 16bits register.
 *	@param	argin	register address, AND mask, OR mask
 *	@exception DevFailed
 */
	void	mask_write_register(const Tango::DevVarShortArray *);
/**
 * Read and Write multiple 16bits registers.
 *	argin[0] = read address
 *	argin[1] = nb of registers to read
 *	argin[2] = write address,
 *	argin[3] = nb of registers to write,
 *	argin[4] = 1st register value to write
 *	argin[5] = 2nd register value to write
 *	...
 *	@param	argin	read address, no. to read, write address, nb.of write, write data
 *	@return	read registers
 *	@exception DevFailed
 */
	Tango::DevVarShortArray	*read_write_register(const Tango::DevVarShortArray *);

/**
 *	Read the device properties from database
 */
	 void get_device_property();
//@}

	//	Here is the end of the automatic code generation part
	//-------------------------------------------------------------	



protected :	
	//	Add your own data members here
	//-----------------------------------------
	int get_protocol_number();
	void check_argin(const Tango::DevVarShortArray *argin,int lgth,const char *where);
	int data_in_cache(const char *,short,short);

	ModbusCore 				*modbusCore;
	CacheThread				*theThread;
	ThreadCmd				thCmd;
	omni_mutex				thCmdMutex;
	vector<CacheDataBlock>	cacheDef;
	int						thId;
	Tango::DevLong			maxDeltaTh;

	omni_mutex				rhr_mutex;
	omni_mutex				ris_mutex;
	omni_mutex				rir_mutex;
	omni_mutex				rmcs_mutex;
	
};

}	// namespace

#endif	// _MODBUS_H
