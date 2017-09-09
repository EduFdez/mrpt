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
#include <numeric>
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
    std::vector<cv::Mat> v_depth;
    //std::vector<cv::Mat> depth_reg; // Depth image registered to RGB pose
    std::vector<pcl::PointCloud<PointT>::Ptr> v_cloud;
//    std::vector<mrpt::vision::FeatureList> points;

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
//        // Initialize visualizer
//        viewer.runOnVisualizationThread (boost::bind(&CameraRotTracker::viz_cb, this, _1), "viz_cb");
//        viewer.registerKeyboardCallback ( &CameraRotTracker::keyboardEventOccurred, *this );
//        b_show_corresp = false;
    }

    /*! Load a config which indicates the system to calibrate: input rawlog dataset, initial poses with uncertainty, plus other parameters. */
    void loadConfiguration(const std::string & config_file);

    /*! Transfer images and features from observation [0] (current) to [1] (previous). */
    void setNewFrame();

//    /*! This function encapsulates the main functionality of the calibration process:
//     *  parse the dataset to find geometric correspondences between the sensors, and estimate the calibration */
//    void run2();

};

#endif
