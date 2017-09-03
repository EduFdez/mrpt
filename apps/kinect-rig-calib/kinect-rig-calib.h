/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
    APPLICATION: kinect-rig-calib
    FILE: kinect-rig-calib_main.cpp
    AUTHOR: Eduardo Fernández-Moral <efernandezmoral@gmail.com>

    See README.txt for instructions or
          http://www.mrpt.org/Application:kinect-rig-calib
  ---------------------------------------------------------------*/

#ifndef KINECT_RIG_CALIB_H
#define KINECT_RIG_CALIB_H

#include "extrinsic_calib_planes.h"
#include "extrinsic_calib_lines.h"

#include <numeric>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/utils/CConfigFile.h>
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
//class KinectRigCalib : public ExtrinsicCalib<T>
//template<typename T>
class KinectRigCalib : public ExtrinsicCalibPlanes, public ExtrinsicCalibLines
{

    /*! Visualization elements */
    pcl::visualization::CloudViewer viewer;
    bool b_exit;
    bool b_viz_init;
    bool b_freeze;

    void viz_cb (pcl::visualization::PCLVisualizer& viz);
    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

    void setNumSensors(const size_t n_sensors);

  public:

    boost::mutex visualizationMutex;

    /*! The type of features used for calibrate */
    enum strategy{ PLANES, LINES, PLANES_AND_LINES } s_type;

//    ExtrinsicCalib<T> calib;

//    ExtrinsicCalibPlanes<T> calib_planes;

//    ExtrinsicCalibLines<T> calib_lines;

    // Sensor parameters
    std::vector<std::string> sensor_labels;
    std::vector<mrpt::poses::CPose3D> init_poses;
    double max_diff_sync;

    // Observation parameters
    std::vector<cv::Mat> rgb;
    std::vector<cv::Mat> depth;
    std::vector<cv::Mat> depth_reg; // Depth image registered to RGB pose
    std::vector<pcl::PointCloud<PointT>::Ptr> cloud;
//    std::vector<mrpt::pbmap::PbMap> v_pbmap;

    Eigen::Matrix<T,4,4> initOffset;
    std::vector<Eigen::Matrix<T,4,4>, Eigen::aligned_allocator<Eigen::Matrix<T,4,4> > > initOffsets;

    std::string rawlog_file;
    std::string output_dir;
    size_t decimation;
    bool display;
    bool verbose;

    KinectRigCalib() :
        viewer("kinect-rig-calib"),
        b_exit(false),
        b_viz_init(false),
        b_freeze(true),
        s_type(PLANES_AND_LINES),
        max_diff_sync(0.005)
    {
        // Initialize visualizer
        viewer.runOnVisualizationThread (boost::bind(&KinectRigCalib::viz_cb, this, _1), "viz_cb");
        viewer.registerKeyboardCallback ( &KinectRigCalib::keyboardEventOccurred, *this );
    }

    /*! Load a config which indicates the system to calibrate: input rawlog dataset, initial poses with uncertainty, plus other parameters. */
    void loadConfiguration(const std::string & config_file);

    /*! Extract plane and/or line correspondences to estimate the calibration. */
    void getCorrespondences();

    /*! Perform the calibration from plane and/or line correspondences. */
    void calibrate(const bool save_corresp = true);

    /*! This function encapsulates the main functionality of the calibration process:
     *  parse the dataset to find geometric correspondences between the sensors, and estimate the calibration */
    void run();

};

#endif
