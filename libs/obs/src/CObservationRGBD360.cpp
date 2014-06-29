/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

//cd #include <mrpt/obs.h>
//#include <mrpt/slam.h>
#include <mrpt/slam/CObservationRGBD360.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/utils/CStream.h>

#include <mrpt/math/CMatrix.h>
#include <mrpt/math/ops_containers.h> // norm(), etc.
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CTimeLogger.h>

using namespace std;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRGBD360, CObservation, mrpt::slam)

/*---------------------------------------------------------------
							Constructor
 ---------------------------------------------------------------*/
CObservationRGBD360::CObservationRGBD360( ) :
	m_points3D_external_stored(false),
	m_rangeImage_external_stored(false),
//	hasPoints3D(false),
//	hasRangeImage(false),
//	range_is_depth(true),
//	hasIntensityImage(false),
//	cameraParams(), // The depth measurements are always referred to the RGBD camera, and thus, we use the same parameters for both RGB and Depth images
//	cameraParamsIntensity(),
	maxRange( 10.0f ),
	sensorPose(),
	stdError( 0.01f )
{
//  for(unsigned i=0; i < NUM_SENSORS; i++)
//  {
//
//  }
}

/*---------------------------------------------------------------
							Destructor
 ---------------------------------------------------------------*/

CObservationRGBD360::~CObservationRGBD360()
{

}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationRGBD360::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		// The data
		out << maxRange << sensorPose;

    for(unsigned i=0; i < NUM_SENSORS; i++)
    {
      out << rgbd[i];
    }

//		out << hasRangeImage; if (hasRangeImage) for (unsigned i=0; i < NUM_SENSORS; i++) out << rangeImages[i];
//		out << hasIntensityImage; if (hasIntensityImage) for (unsigned i=0; i < NUM_SENSORS; i++) out << intensityImages[i];
//		for (unsigned i=0; i < NUM_SENSORS; i++) out << timestamps[i];
//
		out << stdError;
		out << timestamp;
		out << sensorLabel;

		out << m_points3D_external_stored << m_points3D_external_file;
		out << m_rangeImage_external_stored << m_rangeImage_external_file;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationRGBD360::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			in >> maxRange >> sensorPose;

//      in >> hasRangeImage;
//      if (hasRangeImage) for (unsigned i=0; i < NUM_SENSORS; i++)
//      {
//#ifdef COBS3DRANGE_USE_MEMPOOL
//        // We should call "rangeImage_setSize()" to exploit the mempool:
//        this->rangeImage_setSize(240,320,i);
//#endif
//        in >> rangeImages[i];
//      }
//
//      in >> hasIntensityImage;
//      if (hasIntensityImage) for (unsigned i=0; i < NUM_SENSORS; i++)
//        in >>intensityImages[i];

      for (unsigned i=0; i < NUM_SENSORS; i++)
        in >> rgbd[i];

      in >> stdError;
      in >> timestamp;
      in >> sensorLabel;

      in >> m_points3D_external_stored >> m_points3D_external_file;
      in >> m_rangeImage_external_stored >> m_rangeImage_external_file;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

//void CObservationRGBD360::swap(CObservationRGBD360 &o)
//{
//	CObservation::swap(o);
//
//	std::swap(m_points3D_external_stored,o.m_points3D_external_stored);
//	std::swap(m_points3D_external_file,o.m_points3D_external_file);
//
////	std::swap(hasRangeImage,o.hasRangeImage);
////	rangeImage.swap(o.rangeImage);
////	std::swap(m_rangeImage_external_stored, o.m_rangeImage_external_stored);
////	std::swap(m_rangeImage_external_file, o.m_rangeImage_external_file);
////
////	std::swap(hasIntensityImage,o.hasIntensityImage);
////	std::swap(intensityImageChannel,o.intensityImageChannel);
////	intensityImage.swap(o.intensityImage);
//
////	std::swap(cameraParams,o.cameraParams);
//
//	std::swap(maxRange, o.maxRange);
//	std::swap(sensorPose, o.sensorPose);
//	std::swap(stdError, o.stdError);
//
//  for (unsigned i=0; i < NUM_SENSORS; i++)
//    std::swap(rgbd[i], o.rgbd[i]);
//}
