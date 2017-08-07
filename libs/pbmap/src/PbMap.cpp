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
#include "pbmap-precomp.h"  // Precompiled headers

#include <mrpt/pbmap/PbMap.h>
#include <mrpt/pbmap/colors.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/fast_bilateral.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::pbmap;


IMPLEMENTS_SERIALIZABLE(PbMap, CSerializable, mrpt::pbmap)

/*---------------------------------------------------------------
    Constructor
  ---------------------------------------------------------------*/
PbMap::PbMap() :
    FloorPlane(-1),
    globalMapPtr( new pcl::PointCloud<pcl::PointXYZRGBA>() ),
    edgeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>),
    outEdgeCloudPtr(new pcl::PointCloud<pcl::PointXYZRGBA>)
{
}

/*---------------------------------------------------------------
                        writeToStream
 ---------------------------------------------------------------*/
void  PbMap::writeToStream(mrpt::utils::CStream &out, int *out_Version) const
{
    //cout << "Write PbMap. Version " << *out_Version << endl;
    if (out_Version){//cout << "There is version\n";
        *out_Version = 0;}
    else
    {
        // Write label
        out << label;

        // The data
        uint32_t n = uint32_t( vPlanes.size() );
        out << n;
        //  cout << "Write " << n << " planes\n";
        for (uint32_t i=0; i < n; i++)
            out << vPlanes[i];
    }
    //cout << "Exit Write PbMap. " << endl;
}

/*---------------------------------------------------------------
                        readFromStream
 ---------------------------------------------------------------*/
void  PbMap::readFromStream(mrpt::utils::CStream &in, int version)
{
    switch(version)
    {
    case 0:
    {
        //      cout << "Read planes\n";

        // Read label
        in >> label;
        //      cout << "PbMap label " << label << endl;

        // Delete previous content:
        vPlanes.clear();

        // The data
        // First, write the number of planes:
        uint32_t	n;
        in >> n;
        vPlanes.resize(n);
        for (uint32_t i=0; i < n; i++)
        {
            //      cout << "plane\n";

            Plane pl;
            pl.id = i;
            in >> pl;
            vPlanes[i] = pl;
        }
        //        cout << "Finish reading planes\n";
    } break;
    default:
        MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
    };
}

pcl::PointCloud<pcl::Normal>::Ptr PbMap::computeImgNormal(const pcl::PointCloud<PointT>::Ptr & cloud, const float depth_thres, const float smooth_factor)
{
    //ImgRGBD_3D::fastBilateralFilter(cloud, cloud);

    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    //ne.setNormalEstimationMethod(ne.SIMPLE_3D_GRADIENT);
    //ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
    //ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(depth_thres); // For VGA: 0.02f, 10.0f
    ne.setNormalSmoothingSize(smooth_factor);
    ne.setDepthDependentSmoothing(true);

    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(cloud);
    ne.compute(*normal_cloud);
    return normal_cloud;
}

size_t PbMap::segmentPlanes(const pcl::PointCloud<PointT>::Ptr & cloud,
                            vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > & regions,
                            std::vector<pcl::ModelCoefficients> & model_coefficients,
                            std::vector<pcl::PointIndices> & inliers,
                            std::vector<pcl::PointIndices> & boundary_indices,
                            const float dist_threshold, const float angle_threshold, const size_t min_inliers)
{
    pcl::PointCloud<PointT>::Ptr cloud_filtered = cloud;
//    ImgRGBD_3D::fastBilateralFilter(cloud, cloud_filtered, 30, 0.5);
//    computeImgNormal(cloud_filtered);
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud = computeImgNormal(cloud_filtered, 0.02f, 10.0f);

    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers(min_inliers);
    mps.setAngularThreshold(angle_threshold); // (0.017453 * 2.0) // 3 degrees
    mps.setDistanceThreshold(dist_threshold); //2cm
    mps.setInputNormals(normal_cloud);
    mps.setInputCloud(cloud_filtered);

//    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
//    std::vector<pcl::ModelCoefficients> model_coefficients;
//    std::vector<pcl::PointIndices> inliers;
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
//    std::vector<pcl::PointIndices> boundary_indices;
    mps.segmentAndRefine(regions, model_coefficients, inliers, labels, label_indices, boundary_indices);
    size_t n_regions = regions.size();

    return n_regions;
}

/*! This function segments planes from the point cloud */
void PbMap::pbMapFromPCloud(const pcl::PointCloud<PointT>::Ptr & point_cloud, mrpt::pbmap::PbMap & pbmap,
                            const float dist_threshold, const float angle_threshold, const size_t min_inliers, const float max_curvature_plane)
{
    // Downsample and filter point cloud
    //      DownsampleRGBD downsampler(2);
//      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampledCloud = downsampler.downsamplePointCloud(point_cloud);
//      cloudImg.setPointCloud(downsampler.downsamplePointCloud(point_cloud));
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::FastBilateralFilter<pcl::PointXYZRGBA> filter;
    filter.setSigmaS (10.0);
    filter.setSigmaR (0.05);
    filter.setInputCloud(point_cloud);
    filter.filter(*cloud_filtered);

    // Segment planes
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inliers;
    std::vector<pcl::PointIndices> boundary_indices;
    size_t n_planes = segmentPlanes(cloud_filtered, regions, model_coefficients, inliers, boundary_indices, dist_threshold, angle_threshold, min_inliers);
    cout << " number of planes " << n_planes << " cloud size " << point_cloud->size() << "\n";

    // Create a vector with the planes detected in this keyframe, and calculate their parameters (normal, center, pointclouds, etc.)
    cout << "cloud_size " << point_cloud->size() << "\n";
    pbmap.vPlanes.clear();
    for (size_t i = 0; i < regions.size (); i++)
    {
        mrpt::pbmap::Plane plane;

        plane.v3center = regions[i].getCentroid();
        plane.v3normal = Eigen::Vector3f(model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2]);
        if( plane.v3normal.dot(plane.v3center) > 0)
            plane.v3normal = -plane.v3normal;

        plane.curvature = regions[i].getCurvature ();
        //    cout << i << " getCurvature\n";

//        if(plane.curvature > max_curvature_plane)
//          continue;

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        extract.setInputCloud ( point_cloud );
        extract.setIndices ( boost::make_shared<const pcl::PointIndices> (inliers[i]) );
        extract.setNegative (false);
        extract.filter (*plane.planePointCloudPtr);    // Write the planar point cloud
        plane.inliers = inliers[i].indices;

        Eigen::Matrix3f cov_normal = Eigen::Matrix3f::Zero();
        Eigen::Vector3f cov_nd = Eigen::Vector3f::Zero();
        Eigen::Vector3f gravity_center = Eigen::Vector3f::Zero();
        for(size_t j=0; j < inliers[i].indices.size(); j++)
        {
            Eigen::Vector3f pt; pt << plane.planePointCloudPtr->points[j].x, plane.planePointCloudPtr->points[j].y, plane.planePointCloudPtr->points[j].z;
            gravity_center += pt;
        }
        cov_nd = gravity_center;
        gravity_center /= plane.planePointCloudPtr->size();
        //      cout << "gravity_center " << gravity_center.transpose() << "   " << plane.v3center.transpose() << endl;
        //        Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
        for(size_t j=0; j < inliers[i].indices.size(); j++)
        {
            Eigen::Vector3f pt; pt << plane.planePointCloudPtr->points[j].x, plane.planePointCloudPtr->points[j].y, plane.planePointCloudPtr->points[j].z;
            cov_normal += -pt*pt.transpose();// + (plane.v3normal.dot(pt-gravity_center))*(plane.v3normal.dot(pt))*Eigen::Matrix3f::Identity();
//          Eigen::Vector3f pt_rg = (pt-gravity_center);
//          M += pt_rg * pt_rg.transpose();
        }
//            Eigen::JacobiSVD<Eigen::Matrix3f> svdM(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
//            cout << "normalV " << plane.v3normal.transpose() << " covM \n" << svdM.matrixU() << endl;

        Eigen::Matrix4f fim;//, covariance;
        fim.block(0,0,3,3) = cov_normal;
        fim.block(0,3,3,1) = cov_nd;
        fim.block(3,0,1,3) = cov_nd.transpose();
        fim(3,3) = -plane.planePointCloudPtr->size();
        //fim *= 1 / SIGMA2;
        Eigen::JacobiSVD<Eigen::Matrix4f> svd(fim, Eigen::ComputeFullU | Eigen::ComputeFullV);
        //svd.pinv(plane.information);
        plane.information = pseudoinverse(fim);
        //        std::cout << "covariance \n" << plane.information << std::endl;
        plane.information = -fim;
        //
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr contourPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
        contourPtr->points = regions[i].getContour();
        cout << "PbMap::pbMapFromPCloud regions[i].getContour() size " << contourPtr->points.size() << " vs " << boundary_indices[i].indices.size() << endl;
        std::vector<int> indices_hull;

//            cout << "Extract contour\n";
        if(contourPtr->size() != 0)
        {
            //      cout << "Extract contour2 " << contourPtr->size() << "\n";
            plane.calcConvexHull(contourPtr, indices_hull);
        }
        else
        {
            //        assert(false);
            std::cout << "HULL 000\n" << plane.planePointCloudPtr->size() << std::endl;
            static pcl::VoxelGrid<pcl::PointXYZRGBA> plane_grid;
            plane_grid.setLeafSize(0.05,0.05,0.05);
            plane_grid.setInputCloud (plane.planePointCloudPtr);
            plane_grid.filter (*contourPtr);
            plane.calcConvexHull(contourPtr, indices_hull);
        }
        plane.polygon_indices.resize(indices_hull.size());
        for (size_t j = 0; j < indices_hull.size (); j++)
            plane.polygon_indices[j] = boundary_indices[i].indices[indices_hull[j]];


        //        plane.calcConvexHull(contourPtr);
        //      cout << "calcConvexHull\n";
        plane.computeMassCenterAndArea();
        //    cout << "Extract convexHull\n";
        // Discard small planes

        plane.d = -plane.v3normal .dot( plane.v3center );

        //        // Discard narrow planes
        //        plane.calcElongationAndPpalDir();
        //        if(plane.elongation > max_elongation_plane)
        //          continue;

        bool isSamePlane = false;
        if(plane.curvature < max_curvature_plane)
            for (size_t j = 0; j < pbmap.vPlanes.size(); j++)
                if( pbmap.vPlanes[j].curvature < max_curvature_plane && pbmap.vPlanes[j].isSamePlane(plane, 0.99, 0.05, 0.2) ) // The planes are merged if they are the same
                {
                    //            cout << "Merge local region\n";
                    isSamePlane = true;
                    //            double time_start = pcl::getTime();
                    pbmap.vPlanes[j].mergePlane2(plane);
                    //            double time_end = pcl::getTime();
                    //          std::cout << " mergePlane2 took " << double (time_start - time_end) << std::endl;

                    break;
                }
        if(!isSamePlane)
        {
            //          cout << "New plane\n";
            //          plane.calcMainColor();
            plane.id = pbmap.vPlanes.size();
            pbmap.vPlanes.push_back(plane);
        }
    }

    //double extractPlanes_end = pcl::getTime();
    //std::cout << "PlaneFeatures::pbMapFromPCloud in " << (extractPlanes_end - extractPlanes_start)*1000 << " ms\n";
}

void PbMap::displayImagePbMap(const pcl::PointCloud<PointT>::Ptr & point_cloud, const cv::Mat & rgb, const PbMap & pbmap)
{
    cout << " PbMap::displayImagePbMap... " << point_cloud->width << " pbmap " << pbmap.vPlanes.size() << std::endl;

    if(pbmap.vPlanes.empty())
    {
        cout << "PbMap::displayImagePbMap: pbmap is empty -> do not display\n\n";
        return;
    }

    cv::Mat img_regions = rgb.clone();
    if( rgb.empty() )
        img_regions = cv::Mat(point_cloud->height, point_cloud->width, CV_8UC3, cv::Scalar(0,0,0));
    else
        cv::imshow( "rgb", rgb );

    // Color the planar regions
    cv::Vec3b * pixel = reinterpret_cast<cv::Vec3b*>(img_regions.data);
    for(size_t i=0; i < pbmap.vPlanes.size(); i++)
    {
        const cv::Vec3b color = cv::Vec3b(blu[i%10], grn[i%10], red[i%10]);
        for(size_t j=0; j < pbmap.vPlanes[i].inliers.size(); j++)
            pixel[pbmap.vPlanes[i].inliers[j]] = color;

        // Draw the polygonal contour of the planar regions
        cout << i << " polygon_indices ";
        for (auto& k: pbmap.vPlanes[i].polygon_indices)
            cout << k << " ";
        for(size_t j=0; j < pbmap.vPlanes[i].polygon_indices.size()-1; j++)
        {
            int x1 = pbmap.vPlanes[i].polygon_indices[j] % img_regions.cols;
            int y1 = pbmap.vPlanes[i].polygon_indices[j] / img_regions.cols;
            int x2 = pbmap.vPlanes[i].polygon_indices[j+1] % img_regions.cols;
            int y2 = pbmap.vPlanes[i].polygon_indices[j+1] / img_regions.cols;
            cv::line(img_regions, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(color[0],color[1],color[2]), 2);
        }
    }

    cv::imshow( "displayRegions", img_regions );
    cv::waitKey(0);
}

void PbMap::savePbMap(string filePath)
{
    //  boost::mutex::scoped_lock lock (mtx_pbmap_busy);

    //cout << "PbMap::savePbMap\n";
    // Serialize PbMap
    mrpt::utils::CFileGZOutputStream serialize_pbmap(filePath + "/planes.pbmap");
    serialize_pbmap << *this;
    serialize_pbmap.close();

    // Save reconstructed point cloud
    pcl::io::savePCDFile(filePath + "/cloud.pcd", *this->globalMapPtr);
}

void PbMap::loadPbMap(std::string filePath)
{
    // Read in the cloud data
    pcl::PCDReader reader;
    string PbMapFile = filePath;
    reader.read (PbMapFile.append("/cloud.pcd"), *(this->globalMapPtr));
    //  cout << "Size " << globalMapPtr->size() << " " << globalMapPtr->empty() << endl;

    // Load Previous Map
    PbMapFile = filePath;
    mrpt::utils::CFileGZInputStream serialized_pbmap;
    if (serialized_pbmap.open(PbMapFile.append("/planes.pbmap")))
    {
        serialized_pbmap >> *this;
    }
    else
        cout << "Error: cannot open " << PbMapFile << "\n";
    serialized_pbmap.close();

    //  std::cout << "Load PbMap from " << filePath << "\n";
}


// Merge two pbmaps.
void PbMap::MergeWith(PbMap &pbm, Eigen::Matrix4f &T)
{
    // Rotate and translate PbMap
    for(size_t i = 0; i < pbm.vPlanes.size(); i++)
    {
        Plane plane = pbm.vPlanes[i];
        //    Plane plane = &pbm.vPlanes[i]; //Warning: It modifies the source!!!

        // Transform normal and ppal direction
        plane.v3normal = T.block(0,0,3,3) * plane.v3normal;
        plane.v3PpalDir = T.block(0,0,3,3) * plane.v3PpalDir;

        // Transform centroid
        plane.v3center = T.block(0,0,3,3) * plane.v3center + T.block(0,3,3,1);

        // Transform convex hull points
        pcl::transformPointCloud(*plane.polygonContourPtr, *plane.polygonContourPtr, T);

        pcl::transformPointCloud(*plane.planePointCloudPtr, *plane.planePointCloudPtr, T);

        plane.id = vPlanes.size();

        vPlanes.push_back(plane);
    }

    // Rotate and translate the point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr alignedPointCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*pbm.globalMapPtr,*alignedPointCloud,T);

    *globalMapPtr += *alignedPointCloud;

}

#include <fstream>
// Print PbMap content to a text file
void PbMap::printPbMap(string txtFilePbm)
{
    cout << "PbMap 0.2\n\n";

    ofstream pbm;
    pbm.open(txtFilePbm.c_str());
    pbm << "PbMap 0.2\n\n";
    pbm << "MapPlanes " << vPlanes.size() << endl;
    for(unsigned i=0; i < vPlanes.size(); i++)
    {
        pbm << " ID " << vPlanes[i].id << " obs " << vPlanes[i].numObservations;
        pbm << " areaVoxels " << vPlanes[i].areaVoxels << " areaHull " << vPlanes[i].areaHull;
        pbm << " ratioXY " << vPlanes[i].elongation << " structure " << vPlanes[i].bFromStructure << " label " << vPlanes[i].label;
        pbm << "\n normal\n" << vPlanes[i].v3normal << "\n center\n" << vPlanes[i].v3center;
        pbm << "\n PpalComp\n" << vPlanes[i].v3PpalDir << "\n RGB\n" << vPlanes[i].v3colorNrgb;
        pbm << "\n Neighbors (" << vPlanes[i].neighborPlanes.size() << "): ";
        for(map<unsigned,unsigned>::iterator it=vPlanes[i].neighborPlanes.begin(); it != vPlanes[i].neighborPlanes.end(); it++)
            pbm << it->first << " ";
        pbm << "\n CommonObservations: ";
        for(map<unsigned,unsigned>::iterator it=vPlanes[i].neighborPlanes.begin(); it != vPlanes[i].neighborPlanes.end(); it++)
            pbm << it->second << " ";
        pbm << "\n ConvexHull (" << vPlanes[i].polygonContourPtr->size() << "): \n";
        for(unsigned j=0; j < vPlanes[i].polygonContourPtr->size(); j++)
            pbm << "\t" << vPlanes[i].polygonContourPtr->points[j].x << " " << vPlanes[i].polygonContourPtr->points[j].y << " " << vPlanes[i].polygonContourPtr->points[j].z << endl;
        pbm << endl;
    }
    pbm.close();
}
