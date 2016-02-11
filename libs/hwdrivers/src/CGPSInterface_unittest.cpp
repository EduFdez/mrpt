/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/hwdrivers/CGPSInterface.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::utils;
using namespace std;

TEST(CGPSInterface, NMEA_parser)
{
	// Test with a correct frame:
	{
		const char *test_cmd = "$GPGGA,101830.00,3649.76162994,N,00224.53709052,W,2,08,1.1,9.3,M,47.4,M,5.0,0120*58";
		mrpt::obs::CObservationGPS obsGPS;
		const bool parse_ret = CGPSInterface::parse_NMEA( test_cmd, obsGPS );
		EXPECT_TRUE(parse_ret) << "Failed parse of: " << test_cmd << endl;

		EXPECT_TRUE(obsGPS.has_GGA_datum);
		const mrpt::obs::gnss::Message_NMEA_GGA &gga = obsGPS.getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();
		EXPECT_NEAR(gga.fields.latitude_degrees, 36+49.76162994/60.0,1e-10);
		EXPECT_NEAR(gga.fields.longitude_degrees, -(002+24.53709052/60.0),1e-10);
		EXPECT_NEAR(gga.fields.altitude_meters, 9.3,1e-10);
	}

	// Test with an empty frame:
	{
		const char *test_cmd = "$GPGGA,,,,,,0,,,,M,,M,,*6";
		mrpt::obs::CObservationGPS obsGPS;
		const bool parse_ret = CGPSInterface::parse_NMEA( test_cmd, obsGPS );
		EXPECT_FALSE(parse_ret);
	}
}
