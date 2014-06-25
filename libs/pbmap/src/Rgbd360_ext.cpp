/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/pbmap/Rgbd360_ext.h>
//#include <mrpt/poses/CPosePDF.h>
//#include <mrpt/utils/CStream.h>

//#include <mrpt/math/CMatrix.h>
//#include <mrpt/math/ops_containers.h> // norm(), etc.
//#include <mrpt/utils/CFileGZInputStream.h>
//#include <mrpt/utils/CFileGZOutputStream.h>
//#include <mrpt/utils/CTimeLogger.h>

using namespace std;
//using namespace mrpt::slam;
//using namespace mrpt::utils;
//using namespace mrpt::poses;
//using namespace mrpt::math;
using namespace mrpt::pbmap;

// This must be added to any CSerializable class implementation file.
//IMPLEMENTS_SERIALIZABLE(Rgbd360_ext, CObservationRGBD360, mrpt::pbmap)
IMPLEMENTS_SERIALIZABLE(Rgbd360_ext, mrpt::pbmap)

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
Rgbd360_ext::Rgbd360_ext( )
//:
//	m_points3D_external_stored(false),
//	m_rangeImage_external_stored(false),
////	hasPoints3D(false),
////	hasRangeImage(false),
////	range_is_depth(true),
////	hasIntensityImage(false),
////	cameraParams(), // The depth measurements are always referred to the RGBD camera, and thus, we use the same parameters for both RGB and Depth images
////	cameraParamsIntensity(),
//	maxRange( 10.0f ),
//	sensorPose(),
//	stdError( 0.01f )
{
//  for(unsigned i=0; i < NUM_SENSORS; i++)
//  {
//
//  }
}

/*---------------------------------------------------------------
							Destructor
 ---------------------------------------------------------------*/

Rgbd360_ext::~Rgbd360_ext()
{

}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  Rgbd360_ext::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
//		// The data
//		out << maxRange << sensorPose;
//
//    for(unsigned i=0; i < NUM_SENSORS; i++)
//    {
//      out << rgbd[i];
//    }

	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  Rgbd360_ext::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
//			in >> maxRange >> sensorPose;
//
//      for (unsigned i=0; i < NUM_SENSORS; i++)
//        in >> rgbd[i];

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}
