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
//#include <mrpt/math/CMatrixFixedNumeric.h>
//#include <mrpt/utils/CArray.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/utils/CConfigFile.h>
//#include <mrpt/utils/CFileGZInputStream.h>
////#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
//#include <mrpt/system/threads.h>
#include <mrpt/system/filesystem.h>
//#include <mrpt/opengl/CPlanarLaserScan.h> // This class lives in the lib [mrpt-maps] and must be included by hand
//#include <mrpt/math/ransac_applications.h>
#include <mrpt/vision/chessboard_stereo_camera_calib.h>
#include <mrpt/vision/CUndistortMap.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <boost/thread/mutex.hpp>

#include <omp.h>

typedef float T;

/*! This class calibrates the extrinsic parameters of the omnidirectional RGB-D sensor. For that, the sensor is accessed
 *  and big planes are segmented and matched between different single sensors.
*/
//template<typename T>
//class KinectRigCalib : public ExtrinsicCalib<T>
class KinectRigCalib : public ExtrinsicCalibPlanes<T>, public ExtrinsicCalibLines<T>
{
    using ExtrinsicCalib<T>::num_sensors;
    using ExtrinsicCalib<T>::Rt_estimated;

  public:

    boost::mutex visualizationMutex;

    /*! The type of features used for calibrate */
    enum strategy{ PLANES, LINES, PLANES_AND_LINES } s_type;

//    ExtrinsicCalib<T> calib;

//    ExtrinsicCalibPlanes<T> calib_planes;

//    ExtrinsicCalibLines<T> calib_lines;

    // Sensor parameters
    std::vector<double> weight_pair;
    std::vector<std::string> sensor_labels;
    std::vector<mrpt::utils::TStereoCamera> rgbd_intrinsics;
    std::vector<double> mxmy; // pixel relation (fx=mx*f; fy=my*f)
    std::vector<mrpt::poses::CPose3D> init_poses;
    std::vector<double> v_approx_trans;
    std::vector<double> v_approx_rot;
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
        s_type(PLANES_AND_LINES),
        max_diff_sync(0.005)
    {
    }

    /*! Load a config which indicates the system to calibrate: input rawlog dataset, initial poses with uncertainty, plus other parameters. */
    void loadConfiguration(const std::string & config_file);

    /*! Load an initial estimation of Rt between the pair of Asus sensors from file */
    inline void setInitRt(const std::string Rt_file, const size_t sensor_id = 1)
    {
        if( !mrpt::system::fileExists(Rt_file) || sensor_id >= calib.Rt_estimated.size() )
            throw std::runtime_error("\nERROR...");

        calib.Rt_estimated[sensor_id].loadFromTextFile(Rt_file);
    }

    /*! Load an initial estimation of Rt between the pair of Asus sensors from file */
    inline void setInitRt(Eigen::Matrix<T,4,4> initRt, const size_t sensor_id = 1)
    {
        if( sensor_id >= calib.Rt_estimated.size() )
            throw std::runtime_error("\nERROR...");

        calib.Rt_estimated[sensor_id] = initRt;
    }

//    void trimOutliersRANSAC(mrpt::math::CMatrixDouble &matched_planes, mrpt::math::CMatrixDouble &FIM_values);

    /*! Load a config which indicates the system to calibrate: input rawlog dataset, initial poses with uncertainty, plus other parameters. */
    void loadConfiguration(const std::string & config_file);

    /*! Extract plane and/or line correspondences to estimate the calibration. */
    void getCorrespondences();

    /*! Perform the calibration from plane and/or line correspondences. */
    void calibrate();

    /*! This function encapsulates the main functionality of the calibration process:
     *  parse the dataset to find geometric correspondences between the sensors, and estimate the calibration */
    void run();

};

#endif
