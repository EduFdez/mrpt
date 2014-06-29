/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/hwdrivers/CNTRIPEmitter.h>
#include <mrpt/hwdrivers/CGenericSensor.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** A combination of GPS receiver + NTRIP receiver capable of submitting GGA frames to enable RTCM 3.0.
		  * This class holds instances of two classes, publicly exposed as member variables:
		  *  - mrpt::hwdrivers::CGPSInterface  gps;
		  *  - mrpt::hwdrivers::CNTRIPEmitter  ntrip;
		  *
		  * and acts as a "joint sensor", calling both objects' doProcess() inside the doProcess() loop, etc.
		  *
		  * The goal of this class is automatically gather GGA frames from the gps sensor and upload them to the NTRIP server.
		  *
		  * Configuration file format is a combination of the original parameters for both classes, each with
		  * a prefix: "gps_" for CGPSInterface params and "ntrip_" for CNTRIPEmitter.
		  *
		  *  \code
		  *  PARAMETERS IN THE ".INI"-LIKE CONFIGURATION STRINGS:
		  * -------------------------------------------------------
		  *   [supplied_section_name]
		  *    gps_COM_port_WIN = COM3
		  *    gps_COM_port_LIN = ttyS0
		  *    gps_baudRate     = 4800   // The baudrate of the communications (typ. 4800 bauds)
		  *    gps_pose_x       = 0      // 3D position of the sensed point relative to the robot (meters)
		  *    gps_pose_y       = 0
		  *    gps_pose_z       = 0
		  *    # Other params (see CGPSInterface)
		  *
		  *    ntrip_COM_port_WIN = COM1         // Serial port where the NTRIP stream will be dumped to.
		  *    ntrip_COM_port_LIN = ttyUSB0
		  *    ntrip_baudRate     = 38400
		  *
		  *    ntrip_server   = 143.123.9.129    // NTRIP caster IP
		  *    ntrip_port     = 2101
		  *    ntrip_mountpoint = MYPOINT23
		  *    #ntrip_user = pepe            // User & password optional.
		  *    #ntrip_password = loco
		  *
		  *  \endcode
		  *
		  *  \note Verbose debug info will be dumped to cout if the environment variable "MRPT_HWDRIVERS_VERBOSE" is set to "1", or if you call CGenericSensor::enableVerbose(true)
		  *
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP CGPS_NTRIP : public CGenericSensor
		{
			DEFINE_GENERIC_SENSOR(CGPS_NTRIP)

		public:
			mrpt::hwdrivers::CGPSInterface  gps;
			mrpt::hwdrivers::CNTRIPEmitter  ntrip;

			/** Constructor. See mrpt::hwdrivers::CGPSInterface for the meaning of params. */
			CGPS_NTRIP( int BUFFER_LENGTH = 500, mrpt::hwdrivers::CSerialPort *outPort = NULL, mrpt::synch::CCriticalSection *csOutPort = NULL);

			/** Destructor */
			virtual ~CGPS_NTRIP();

			// See docs in parent class
			void  doProcess();

			virtual void initialize();
		protected:
			/** See the class documentation at the top for expected parameters */
			void  loadConfig_sensorSpecific( const mrpt::utils::CConfigFileBase &configSource, const std::string &iniSection );
		}; // end class

	} // end namespace
} // end namespace
