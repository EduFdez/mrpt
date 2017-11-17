/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
    APPLICATION: camera-rot-tracker
    FILE: camera-rot-tracker_main.cpp
    AUTHOR: Eduardo Fernández-Moral <efernandezmoral@gmail.com>

    See README.txt for instructions or
          http://www.mrpt.org/Application:camera-rot-tracker
  ---------------------------------------------------------------*/

#ifndef CAMERA_ROT_TRACKER_H
#define CAMERA_ROT_TRACKER_H

#include "../kinect-rig-calib/extrinsic_calib_lines.h"
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/system/os.h>
//#include <mrpt/system/threads.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/vision/chessboard_stereo_camera_calib.h>
#include <mrpt/vision/CUndistortMap.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/mutex.hpp>

#include <omp.h>
#include <numeric>

typedef double T;

/*! This class calibrates the extrinsic parameters of the omnidirectional RGB-D sensor. For that, the sensor is accessed
 *  and big planes are segmented and matched between different single sensors.
*/
//class CameraRotTracker : public ExtrinsicCalib<T>
//template<typename T>
class CameraRotTracker : public ExtrinsicCalibLines
{

    /*! Visualization elements */
    //pcl::visualization::CloudViewer viewer;
    bool b_exit;
    bool b_viz_init;
    bool b_freeze;
    bool b_show_corresp;
    bool b_pause;

    /*! Visualization callbacks */
    void viz_cb (pcl::visualization::PCLVisualizer& viz);
    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

    /*! Display the RGB-D images (This function has been written for adjacent sensors from RGBD360) */
    void displayObservation();

  public:

    boost::mutex visualizationMutex;

    // Sensor parameters
    mrpt::utils::TCamera intrinsics;
    mrpt::poses::CPose3D pose_init;

    // Observation parameters (current:0 and previous:1)
    std::vector<mrpt::obs::CObservation3DRangeScanPtr> obsRGBD;  // The RGBD observation
    std::vector<cv::Mat> v_rgb;
    std::vector<cv::Mat> v_gray;
    std::vector<cv::Mat> v_depth;
    //std::vector<cv::Mat> depth_reg; // Depth image registered to RGB pose
    std::vector<pcl::PointCloud<PointT>::Ptr> v_cloud;
    std::vector<pcl::PointCloud<pcl::Normal>::Ptr> v_normal_cloud;
//    std::vector<mrpt::vision::FeatureList> points;
    mrpt::vision::CUndistortMap rgb_undist;

    // SurfMatch data structures
    std::vector<std::vector<cv::Vec2i> > vv_pt_coor;
    std::vector<std::vector<Eigen::Vector3f> > vv_pt_normal;
    std::vector<std::vector<size_t> > vv_pt_low_curv;

    std::string rawlog_file;
    std::string output_dir;

    size_t decimation;
    bool display;
    bool verbose;

    CameraRotTracker() :
        //viewer("camera-rot-tracker"),
        b_exit(false),
        b_viz_init(false),
        b_freeze(true),
        b_pause(false)
    {
        num_sensors = 2;
        obsRGBD.resize(num_sensors);
        v_rgb.resize(num_sensors);
        v_gray.resize(num_sensors);
        v_depth.resize(num_sensors);
        //depth_reg.resize(num_sensors); // Depth image registered to RGB pose
        v_cloud.resize(num_sensors);
        v_normal_cloud.resize(num_sensors);
        v_pbmap.resize(num_sensors);
        vv_segments2D.resize(num_sensors);
        vv_segmentsDesc.resize(num_sensors);
        vv_length.resize(num_sensors);
        vv_seg_contrast.resize(num_sensors);
        vv_segment_n.resize(num_sensors);
        vv_pt_coor.resize(num_sensors);
        vv_pt_normal.resize(num_sensors);
        vv_pt_low_curv.resize(num_sensors);

//        // Initialize visualizer
//        viewer.runOnVisualizationThread (boost::bind(&CameraRotTracker::viz_cb, this, _1), "viz_cb");
//        viewer.registerKeyboardCallback ( &CameraRotTracker::keyboardEventOccurred, *this );
//        b_show_corresp = false;
    }

    /*! Load a config which indicates the system to calibrate: input rawlog dataset, initial poses with uncertainty, plus other parameters. */
    void loadConfiguration(const std::string & config_file);

    /*! Swap images and features from observation [0] (current) to [1] (previous). */
    void setNewFrame();

    static bool computeRobustNormal(const cv::Mat & depth, const mrpt::utils::TCamera &cam, const int u, const int v, Eigen::Vector3f & normal, const int radius = 2, const float max_angle_cos = 0.996194698f ); // cos(mrptDEG2RAD(5))=0.996194698

    static bool computeRobustNormal(const cv::Mat & depth, const float fx, const float fy, const int u, const int v, Eigen::Vector3f & normal, const int radius = 2, const float max_angle_cos = 0.996194698f);

    std::vector< std::vector<Eigen::Matrix<T,3,1> > > vv_point_n; // The normal vector to the surface containing the 2D point

    std::vector<cv::Vec2i> getDistributedNormals(cv::Mat & depth, const mrpt::utils::TCamera & cam, std::vector<Eigen::Vector3f> & v_normal, std::vector<bool> & v_low_curv, const int h_divisions = 4, const int v_divisions = 3);

    static bool getRobustNormal(const pcl::PointCloud<pcl::Normal>::Ptr img_normals, const int u, const int v, Eigen::Vector3f & normal, const int radius, const float max_angle_cos);

    std::vector<cv::Vec2i> getDistributedNormals2(pcl::PointCloud<pcl::Normal>::Ptr & img_normals, std::vector<Eigen::Vector3f> & v_normal, std::vector<size_t> &v_low_curv, const int h_divisions, const int v_divisions);

    std::vector<cv::Vec2i> getNormalsOfPixels_trg ( const std::vector<cv::Vec2i> & v_pixels, const Eigen::Matrix<T,3,3> & H, pcl::PointCloud<pcl::Normal>::Ptr img_normals,
                                                    std::vector<Eigen::Vector3f> & v_normal, std::vector<size_t> & v_low_curv);

//    /*! This function encapsulates the main functionality of the calibration process:
//     *  parse the dataset to find geometric correspondences between the sensors, and estimate the calibration */
//    void run2();

};

#endif
