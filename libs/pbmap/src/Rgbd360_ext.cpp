/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "pbmap-precomp.h"  // Precompiled headers
#include <mrpt/pbmap/Rgbd360_ext.h>
//#include <mrpt/poses/CPosePDF.h>
//#include <mrpt/utils/CStream.h>

//#include <mrpt/math/CMatrix.h>
//#include <mrpt/math/ops_containers.h> // norm(), etc.
//#include <mrpt/utils/CFileGZInputStream.h>
//#include <mrpt/utils/CFileGZOutputStream.h>
//#include <mrpt/utils/CTimeLogger.h>

using namespace std;
using namespace mrpt::slam;
//using namespace mrpt::utils;
//using namespace mrpt::poses;
//using namespace mrpt::math;
using namespace mrpt::pbmap;


/*! Maximum number of planes to match when registering a pair of Spheres */
float max_match_planes = 25;

/*! Maximum curvature to consider the region as planar */
float max_curvature_plane = 0.0013;

/*! Minimum area to consider the planar patch */
float min_area_plane = 0.12;

/*! Maximum elongation to consider the planar patch */
float max_elongation_plane = 6;

/*! Minimum number of matched planes to consider a good registration */
float min_planes_registration = 4;

/*! Minimum distance between keyframes */
float min_dist_keyframes = 0.2;

/*! Maximum distance between two consecutive frames of a RGBD360 video sequence */
float max_translation_odometry = 0.3;

/*! Maximum rotation between two consecutive frames of a RGBD360 video sequence */
float max_rotation_odometry = 1.2;

/*! Maximum conditioning to resolve the calibration equation system. This parameter
    represent the ratio between the maximum and the minimum eigenvalue of the system */
float threshold_conditioning = 8000.0;


// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(Rgbd360_ext, CObservationRGBD360, mrpt::pbmap)
//IMPLEMENTS_SERIALIZABLE(Rgbd360_ext, CObservation, mrpt::pbmap)

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


/*! Create the PbMap of the spherical point cloud */
void Rgbd360_ext::getPlanes()
{
std::cout << "Frame360.getPlanes()\n";
//  double extractPlanes_start = pcl::getTime();
//
////  #pragma omp parallel num_threads(8)
//  for(unsigned sensor_id=0; sensor_id < NUM_SENSORS; sensor_id++)
//  {
////    int sensor_id = omp_get_thread_num();
//    getPlanesInFrame(sensor_id);
//  }
//
//  double segmentation_end = pcl::getTime();
//  std::cout << "Segmentation took " << double (segmentation_end - extractPlanes_start)*1000 << " ms.\n";
//
//  // Merge the big planes
//  groupPlanes(); // Merge planes detected from adjacent sensors, and place them in "planes"
//  mergePlanes(); // Merge big planes
//
//  double extractPlanes_end = pcl::getTime();
//  std::cout << planes.vPlanes.size() << " planes. Extraction took " << double (extractPlanes_end - extractPlanes_start)*1000 << " ms.\n";
}

/*! This function segments planes from the point cloud corresponding to the sensor 'sensor_id',
    in the frame of reference of the omnidirectional camera
*/
void Rgbd360_ext::getPlanesInFrame(int sensor_id)
{
  // Segment planes
//    std::cout << "extractPlaneFeatures, size " << cloud_[sensor_id]->size() << "\n";
//  double extractPlanes_start = pcl::getTime();
assert(cloud_[sensor_id]->height > 1 && cloud_[sensor_id]->width > 1);

  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
//      ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
//      ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
//      ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor (0.02); // For VGA: 0.02f, 10.01
  ne.setNormalSmoothingSize (8.0f);
  ne.setDepthDependentSmoothing (true);

  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
//      mps.setMinInliers (std::max(uint32_t(40),cloud_[sensor_id]->height*2));
  mps.setMinInliers (80);
  mps.setAngularThreshold (0.039812); // (0.017453 * 2.0) // 3 degrees
  mps.setDistanceThreshold (0.02); //2cm
//    cout << "PointCloud size " << cloud_[sensor_id]->size() << endl;

  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
  ne.setInputCloud ( cloud_[sensor_id] );
  ne.compute (*normal_cloud);

  mps.setInputNormals (normal_cloud);
  mps.setInputCloud ( cloud_[sensor_id] );
  std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;
  pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;
  mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

  // Create a vector with the planes detected in this keyframe, and calculate their parameters (normal, center, pointclouds, etc.)
  unsigned single_cloud_size = cloud_[sensor_id]->size();
//  mrpt::math::CMatrixDouble44 pose_mat = rgbd[sensor_id].sensorPose.getHomogeneousMatrixVal();
  Eigen::Matrix4f Rt = rgbd[sensor_id].sensorPose.getHomogeneousMatrixVal().getEigenBase().cast<float>();
  for (size_t i = 0; i < regions.size (); i++)
  {
    mrpt::pbmap::Plane plane;

    plane.v3center = regions[i].getCentroid ();
    plane.v3normal = Eigen::Vector3f(model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2]);
    if( plane.v3normal.dot(plane.v3center) > 0)
    {
      plane.v3normal = -plane.v3normal;
//          plane.d = -plane.d;
    }
    plane.curvature = regions[i].getCurvature ();
//    cout << i << " getCurvature\n";

//        if(plane.curvature > max_curvature_plane)
//          continue;

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud ( cloud_[sensor_id] );
    extract.setIndices ( boost::make_shared<const pcl::PointIndices> (inlier_indices[i]) );
    extract.setNegative (false);
    extract.filter (*plane.planePointCloudPtr);    // Write the planar point cloud
    plane.inliers.resize(inlier_indices[i].indices.size());
    for(size_t j=0; j<inlier_indices[i].indices.size(); j++)
      plane.inliers[j] = inlier_indices[i].indices[j] + sensor_id*single_cloud_size;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr contourPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
    contourPtr->points = regions[i].getContour();

//    cout << "Extract contour\n";
    if(contourPtr->size() != 0)
    {
      plane.calcConvexHull(contourPtr);
    }
    else
    {
//        assert(false);
    std::cout << "HULL 000\n" << plane.planePointCloudPtr->size() << std::endl;
      static pcl::VoxelGrid<pcl::PointXYZRGBA> plane_grid;
      plane_grid.setLeafSize(0.05,0.05,0.05);
      plane_grid.setInputCloud (plane.planePointCloudPtr);
      plane_grid.filter (*contourPtr);
      plane.calcConvexHull(contourPtr);
    }

//        assert(contourPtr->size() > 0);
//        plane.calcConvexHull(contourPtr);
//    cout << "calcConvexHull\n";
    plane.computeMassCenterAndArea();
//    cout << "Extract convexHull\n";
    // Discard small planes
    if(plane.areaHull < min_area_plane)
      continue;

    plane.d = -plane.v3normal .dot( plane.v3center );

    plane.calcElongationAndPpalDir();
    // Discard narrow planes
    if(plane.elongation > max_elongation_plane)
      continue;

//      double color_start = pcl::getTime();
    plane.calcPlaneHistH();
    plane.calcMainColor2();
//      double color_end = pcl::getTime();
//    std::cout << "color in " << (color_end - color_start)*1000 << " ms\n";

//      color_start = pcl::getTime();
    plane.transform(Rt);
//      color_end = pcl::getTime();
//    std::cout << "transform in " << (color_end - color_start)*1000 << " ms\n";

    bool isSamePlane = false;
    if(plane.curvature < max_curvature_plane)
      for (size_t j = 0; j < local_planes_[sensor_id].vPlanes.size(); j++)
        if( local_planes_[sensor_id].vPlanes[j].curvature < max_curvature_plane && local_planes_[sensor_id].vPlanes[j].isSamePlane(plane, 0.99, 0.05, 0.2) ) // The planes are merged if they are the same
        {
//          cout << "Merge local region\n";
          isSamePlane = true;
//            double time_start = pcl::getTime();
          local_planes_[sensor_id].vPlanes[j].mergePlane2(plane);
//            double time_end = pcl::getTime();
//          std::cout << " mergePlane2 took " << double (time_start - time_end) << std::endl;

          break;
        }
    if(!isSamePlane)
    {
//          plane.calcMainColor();
      plane.id = local_planes_[sensor_id].vPlanes.size();
      local_planes_[sensor_id].vPlanes.push_back(plane);
    }
  }
//      double extractPlanes_end = pcl::getTime();
//    std::cout << "getPlanesInFrame in " << (extractPlanes_end - extractPlanes_start)*1000 << " ms\n";
}


/*! Merge the planar patches that correspond to the same surface in the sphere */
void Rgbd360_ext::mergePlanes()
{
//  double time_start = pcl::getTime();
//
//  // Merge repeated planes
//  for(size_t j = 0; j < planes.vPlanes.size(); j++) // numPrevPlanes
//   if(planes.vPlanes[j].curvature < max_curvature_plane)
//    for(size_t k = j+1; k < planes.vPlanes.size(); k++) // numPrevPlanes
//     if(planes.vPlanes[k].curvature < max_curvature_plane)
//    {
//      bool bSamePlane = false;
////        Eigen::Vector3f center_diff = planes.vPlanes[k].v3center - planes.vPlanes[j].v3center;
//      Eigen::Vector3f close_points_diff;
//      float dist, prev_dist = 1;
//      if( planes.vPlanes[j].v3normal.dot(planes.vPlanes[k].v3normal) > 0.99 )
//        if( fabs(planes.vPlanes[j].d - planes.vPlanes[k].d) < 0.45 )
////          if( BhattacharyyaDist_(plane1.hist_H, plane2.hist_H) > configLocaliser.hue_threshold )
////          if( fabs(planes.vPlanes[j].v3normal.dot(center_diff)) < std::max(0.07, 0.03*center_diff.norm() ) )
//        {
//          // Checking distances:
//          // a) Between an vertex and a vertex
//          // b) Between an edge and a vertex
//          // c) Between two edges (imagine two polygons on perpendicular planes)
//          for(unsigned i=1; i < planes.vPlanes[j].polygonContourPtr->size() && !bSamePlane; i++)
//            for(unsigned ii=1; ii < planes.vPlanes[k].polygonContourPtr->size(); ii++)
//            {
//              close_points_diff = mrpt::pbmap::diffPoints(planes.vPlanes[j].polygonContourPtr->points[i], planes.vPlanes[k].polygonContourPtr->points[ii]);
//              dist = close_points_diff.norm();
////                if( dist < prev_dist )
////                  prev_dist = dist;
//              if( dist < 0.3 && fabs(planes.vPlanes[j].v3normal.dot(close_points_diff)) < 0.06)
//              {
//                bSamePlane = true;
//                break;
//              }
//            }
//          // a) & b)
//          if(!bSamePlane)
//          for(unsigned i=1; i < planes.vPlanes[j].polygonContourPtr->size() && !bSamePlane; i++)
//            for(unsigned ii=1; ii < planes.vPlanes[k].polygonContourPtr->size(); ii++)
//            {
//              dist = sqrt(mrpt::pbmap::dist3D_Segment_to_Segment2(mrpt::pbmap::Segment(planes.vPlanes[j].polygonContourPtr->points[i],planes.vPlanes[j].polygonContourPtr->points[i-1]), mrpt::pbmap::Segment(planes.vPlanes[k].polygonContourPtr->points[ii],planes.vPlanes[k].polygonContourPtr->points[ii-1])));
////                if( dist < prev_dist )
////                  prev_dist = dist;
//              if( dist < 0.3)
//              {
//                close_points_diff = mrpt::pbmap::diffPoints(planes.vPlanes[j].polygonContourPtr->points[i], planes.vPlanes[k].polygonContourPtr->points[ii]);
//                if(fabs(planes.vPlanes[j].v3normal.dot(close_points_diff)) < 0.06)
//                {
//                  bSamePlane = true;
//                  break;
//                }
//              }
//            }
//        }
//
//      if( bSamePlane ) // The planes are merged if they are the same
//      {
//          // Update normal and center
//        assert(planes.vPlanes[j].inliers.size() > 0 &&  planes.vPlanes[k].inliers.size() > 0);
//          planes.vPlanes[j].mergePlane2(planes.vPlanes[k]);
//
//        // Update plane index
//        for(size_t h = k+1; h < planes.vPlanes.size(); h++)
//          --planes.vPlanes[h].id;
//
//        // Delete plane to merge
//        std::vector<mrpt::pbmap::Plane>::iterator itPlane = planes.vPlanes.begin();
//        for(size_t i = 0; i < k; i++)
//          itPlane++;
//        planes.vPlanes.erase(itPlane);
//
//        // Re-evaluate possible planes to merge
//        j--;
//        k = planes.vPlanes.size();
//      }
//    }
////    double time_end = pcl::getTime();
////    std::cout << "Merge planes took " << double (time_end - time_start) << std::endl;
}

/*! Group the planes segmented from each single sensor into the common PbMap 'planes' */
void Rgbd360_ext::groupPlanes()
{
////  cout << "groupPlanes...\n";
//  double time_start = pcl::getTime();
//
//  float maxDistHull = 0.5;
//  float maxDistParallelHull = 0.09;
//
////    Eigen::Matrix4f Rt = calib->getRt_id(0);
////    planes.MergeWith(local_planes_[0], Rt);
//  planes = local_planes_[0];
//  std::set<unsigned> prev_planes, first_planes;
//  for(size_t i=0; i < planes.vPlanes.size(); i++)
//    first_planes.insert(planes.vPlanes[i].id);
//  prev_planes = first_planes;
//
//  for(unsigned sensor_id=1; sensor_id < 8; ++sensor_id)
//  {
//   size_t j;
//   std::set<unsigned> next_prev_planes;
//   for(size_t k = 0; k < local_planes_[sensor_id].vPlanes.size(); k++)
//   {
//      bool bSamePlane = false;
//     if(local_planes_[sensor_id].vPlanes[k].areaHull > 0.5 || local_planes_[sensor_id].vPlanes[k].curvature < max_curvature_plane)
//      for(std::set<unsigned>::iterator it = prev_planes.begin(); it != prev_planes.end() && !bSamePlane; it++) // numPrevPlanes
//      {
//        j = *it;
//
//        if(planes.vPlanes[j].areaHull < 0.5 || planes.vPlanes[j].curvature > max_curvature_plane)
//          continue;
//
//      Eigen::Vector3f close_points_diff;
//      float dist, prev_dist = 1;
//      if( fabs(planes.vPlanes[j].d - local_planes_[sensor_id].vPlanes[k].d) < 0.45 )
//        if( planes.vPlanes[j].v3normal.dot(local_planes_[sensor_id].vPlanes[k].v3normal) > 0.99 )
//        {
//          // Checking distances:
//          // a) Between an vertex and a vertex
//          // b) Between an edge and a vertex
//          // c) Between two edges (imagine two polygons on perpendicular planes)
////            if(!planes.vPlanes[j].isPlaneNearby(local_planes_[sensor_id].vPlanes[k],0.2);
////              continue;
//
//          for(unsigned i=1; i < planes.vPlanes[j].polygonContourPtr->size() && !bSamePlane; i++)
//            for(unsigned ii=1; ii < local_planes_[sensor_id].vPlanes[k].polygonContourPtr->size(); ii++)
//            {
//              close_points_diff = mrpt::pbmap::diffPoints(planes.vPlanes[j].polygonContourPtr->points[i], local_planes_[sensor_id].vPlanes[k].polygonContourPtr->points[ii]);
//              dist = close_points_diff.norm();
//              if( dist < maxDistHull && fabs(planes.vPlanes[j].v3normal.dot(close_points_diff)) < maxDistParallelHull)
//              {
//                bSamePlane = true;
//                break;
//              }
//            }
//          // a) & b)
//          if(!bSamePlane)
//          for(unsigned i=1; i < planes.vPlanes[j].polygonContourPtr->size() && !bSamePlane; i++)
//            for(unsigned ii=1; ii < local_planes_[sensor_id].vPlanes[k].polygonContourPtr->size(); ii++)
//            {
//              dist = sqrt(mrpt::pbmap::dist3D_Segment_to_Segment2(mrpt::pbmap::Segment(planes.vPlanes[j].polygonContourPtr->points[i],planes.vPlanes[j].polygonContourPtr->points[i-1]), mrpt::pbmap::Segment(local_planes_[sensor_id].vPlanes[k].polygonContourPtr->points[ii],local_planes_[sensor_id].vPlanes[k].polygonContourPtr->points[ii-1])));
//              if( dist < maxDistHull)
//              {
//                close_points_diff = mrpt::pbmap::diffPoints(planes.vPlanes[j].polygonContourPtr->points[i], local_planes_[sensor_id].vPlanes[k].polygonContourPtr->points[ii]);
//                if(fabs(planes.vPlanes[j].v3normal.dot(close_points_diff)) < maxDistParallelHull)
//                {
//                  bSamePlane = true;
//                  break;
//                }
//              }
//            }
//        }
//        if(bSamePlane)
//          break;
//      }
//      if( bSamePlane ) // The planes are merged if they are the same
//      {
//        next_prev_planes.insert(planes.vPlanes[j].id);
//        planes.vPlanes[j].mergePlane2(local_planes_[sensor_id].vPlanes[k]);
//      }
//      else
//      {
//        next_prev_planes.insert(planes.vPlanes.size());
//        local_planes_[sensor_id].vPlanes[k].id = planes.vPlanes.size();
//        planes.vPlanes.push_back(local_planes_[sensor_id].vPlanes[k]);
//      }
//    }
//    prev_planes = next_prev_planes;
//    if(sensor_id == 6)
//      prev_planes.insert(first_planes.begin(), first_planes.end());
//  }
}
