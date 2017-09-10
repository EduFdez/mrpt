/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*  Plane-based Map (PbMap) library
 *  Construction of plane-based maps and localization in it from RGBD Images.
 *  Writen by Eduardo Fernandez-Moral. See docs for <a href="group__mrpt__pbmap__grp.html" >mrpt-pbmap</a>
 */

#ifndef __PBMAP_H
#define __PBMAP_H

#include <mrpt/config.h>
#if MRPT_HAS_PCL

#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/pbmap/link_pragmas.h>
#include <mrpt/pbmap/Plane.h>
#include <mrpt/pbmap/Miscellaneous.h>  // For typedef PointT;
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
//#include <boost/thread/thread.hpp>
#include <opencv2/core/core.hpp>

namespace mrpt {
    namespace pbmap {
        // This must be added to any CSerializable derived class:
        DEFINE_SERIALIZABLE_PRE_CUSTOM_LINKAGE( PbMap, PBMAP_IMPEXP)

        /** A class used to store a Plane-based Map (PbMap). A PbMap consists of a set of planar patches
        * described by geometric features (shape, relative position, etc.) and/or radiometric features
        * (dominant color). It is organized as an annotated, undirected graph, where nodes stand for planar
        * patches and edges connect neighbor planes when the distance between their closest points is under
        * a threshold. This graph structure permits to find efficiently the closest neighbors of a plane,
        * or to select groups of nearby planes representing part of the scene.
        *
        * \ingroup mrpt_pbmap_grp
        */
        class PBMAP_IMPEXP PbMap : public mrpt::utils::CSerializable
        {
            // This must be added to any CSerializable derived class:
            DEFINE_SERIALIZABLE( PbMap )

            public:
            /*!Constructor.*/
            PbMap();

            /*!Vector to store the 3D-planes which are the basic characteristic of our map.*/
            std::vector<Plane> vPlanes;

            /*!Label to store a semantic attribute*/
            std::string label;

            /*!Floor plane id*/
            int FloorPlane;

            /*!Registered point cloud from the RGB-D or Depth frames and visual odometry.*/
            pcl::PointCloud<PointT>::Ptr globalMapPtr;

            /*!Static function to compute a normal image from an organized point cloud.*/
            static pcl::PointCloud<pcl::Normal>::Ptr computeImgNormal(const pcl::PointCloud<PointT>::Ptr & cloud, const float depth_thres, const float smooth_factor);

            /*!Static function to segment planes from an organized point cloud.*/
            static size_t segmentPlanes(const pcl::PointCloud<PointT>::Ptr & cloud,
                                        std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > & regions,
                                        std::vector<pcl::ModelCoefficients> & model_coefficients,
                                        std::vector<pcl::PointIndices> & inliers,
                                        std::vector<pcl::PointIndices> & boundary_indices,
                                        const float dist_threshold = 0.02, const float angle_threshold = 0.05f, const size_t min_inliers = 2e4);

            /*!This function extracts a PbMap from the given point cloud */
            static void pbMapFromPCloud(pcl::PointCloud<PointT>::Ptr & point_cloud, mrpt::pbmap::PbMap & pbmap,
                                        const float dist_threshold = 0.02f, const float angle_threshold = 0.05f, const size_t min_inliers = 2e4, const float max_curvature_plane = 0.0013f);

            /*!This function displays the PbMap extracted from an organized point_cloud (corresponding to the rgb image) */
            static void displayImagePbMap(const pcl::PointCloud<PointT>::Ptr & point_cloud, const cv::Mat & rgb, const PbMap & pbmap, const bool b_fill_polygon = true, const cv::Point pt = cv::Point(0,0));

            /*!Save PbMap in the given filePath*/
            void savePbMap(std::string filePath);

            /*!Load a PbMap from the given filePath*/
            void loadPbMap(std::string PbMapFile);

            /*!Merge two pbmaps*/
            void MergeWith(const PbMap &pbm, const Eigen::Matrix4f &T);

            /*! Print PbMap content to a text file*/
            void printPbMap(std::string txtFilePbm);

            //    boost::mutex mtx_pbmap_busy;

        };
        DEFINE_SERIALIZABLE_POST_CUSTOM_LINKAGE( PbMap, PBMAP_IMPEXP)

    }
} // End of namespaces

#endif

#endif
