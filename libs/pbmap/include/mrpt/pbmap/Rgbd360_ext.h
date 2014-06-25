/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef RGBD360_EXT_H
#define RGBD360_EXT_H

//#include "pbmap-precomp.h"   // Precompiled headers
#include <mrpt/pbmap.h>
#include <mrpt/slam/CObservationRGBD360.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h> //Save global map as PCD file
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/fast_bilateral_omp.h>

#include <boost/thread/thread.hpp>

namespace mrpt
{
  namespace pbmap
  {
//	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( Rgbd360_ext, slam::CObservationRGBD360, PBMAP_IMPEXP )
	DEFINE_SERIALIZABLE_PRE_CUSTOM_LINKAGE( Rgbd360_ext, PBMAP_IMPEXP)

	/** Declares a class derived from "CObservationRGBD360" that
	 *      encapsules an omnidirectional RGBD measurement from a set of RGBD sensors.
	 *  This kind of observations can carry one or more of these data fields:
	 *    - 3D point cloud (as float's).
	 *    - 2D range image (as a matrix): Each entry in the matrix "rangeImage(ROW,COLUMN)" contains a distance or a depth (in meters), depending on \a range_is_depth.
	 *    - 2D intensity (grayscale or RGB) image (as a mrpt::utils::CImage): For SwissRanger cameras, a logarithmic A-law compression is used to convert the original 16bit intensity to a more standard 8bit graylevel.
	 *
	 *  The coordinates of the 3D point cloud are in millimeters with respect to the RGB camera origin of coordinates
	 *
	 *  The 2D images and matrices are stored as common images, with an up->down rows order and left->right, as usual.
	 *   Optionally, the intensity and confidence channels can be set to delayed-load images for off-rawlog storage so it saves
	 *   memory by having loaded in memory just the needed images. See the methods load() and unload().
	 *  Due to the intensive storage requirements of this kind of observations, this observation is the only one in MRPT
	 *   for which it's recommended to always call "load()" and "unload()" before and after using the observation, *ONLY* when
	 *   the observation was read from a rawlog dataset, in order to make sure that all the externally stored data fields are
	 *   loaded and ready in memory.
	 *
	 *  Classes that grab observations of this type are:
	 *		- mrpt::hwdrivers::COpenNI2_RGBD360
	 *
	 *
	 *
	 * \sa mrpt::hwdrivers::COpenNI2_RGBD360, CObservation
	 * \ingroup mrpt_obs_grp
	 */
    class PBMAP_IMPEXP Rgbd360_ext : public mrpt::utils::CSerializable//, public mrpt::slam::CObservationRGBD360
    {
      // This must be added to any CSerializable derived class:
      DEFINE_SERIALIZABLE( Rgbd360_ext )

      protected:

        static const unsigned NUM_SENSORS = 4;

        /*! The NUM_SENSORS separate point clouds from each single Asus XPL */
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_[NUM_SENSORS];

      public:
        Rgbd360_ext( );				//!< Default constructor
        virtual ~Rgbd360_ext( ); 	//!< Destructor

        /*! PbMap of the spherical frame */
        mrpt::pbmap::PbMap planes;

    }; // End of class def.

	} // End of namespace

} // End of namespace

#endif
