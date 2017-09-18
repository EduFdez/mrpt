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

#include "camera-rot-tracker.h"
#include "../kinect-rig-calib/kinect-rig-calib_misc.h"
#include "DifOdometry_Datasets_RGBD.h"
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <boost/thread/mutex.hpp>

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace std;
using namespace Eigen;


void CameraRotTracker::displayObservation()
{
    cout << "CameraRotTracker::displayObservation...\n";

    for(size_t i=1; i < num_sensors; i++)
    {
//        string win_name = format("rgb_%lu",v_rgb[i]);
        cv::imshow(mrpt::format("rgb_%lu",i), v_rgb[i] ); //cv::moveWindow("rgb", 20,20);
    }
    cv::waitKey(0);
}

void CameraRotTracker::loadConfiguration(const string & config_file)
{
    //Initial steps. Load configuration file
    //-------------------------------------------
    utils::CConfigFile cfg(config_file);
    rawlog_file = cfg.read_string("GLOBAL", "rawlog_file", "no file", true);
    output_dir = cfg.read_string("GLOBAL", "output_dir", "no dir", true);

    decimation = cfg.read_int("GLOBAL", "decimation", 0, true);
    display = cfg.read_bool("GLOBAL", "display", 0, true);
    verbose = cfg.read_bool("GLOBAL", "verbose", 0, true);

    line_extraction = cfg.read_int("GLOBAL", "line_extraction", 2, true);
    min_pixels_line = cfg.read_int("GLOBAL", "min_pixels_line", 100, true);
    max_lines       = cfg.read_int("GLOBAL", "max_lines", 20, true);
    min_angle_diff = cfg.read_float("GLOBAL", "min_angle_diff", 1.2, true);

    if(verbose)
        cout << "loadConfiguration -> dataset: " << rawlog_file << "\toutput: " << output_dir << "\tdecimation: " << decimation << endl;

    //string sensor_label = "RGBD";
    intrinsics.loadFromConfigFile("GLOBAL", cfg);

    cout << "...CameraRotTracker::loadConfiguration\n";
}

void CameraRotTracker::setNewFrame()
{
    //cout << "CameraRotTracker::setNewFrame..." << num_sensors << endl;
    for(size_t i=1; i < num_sensors; i++)
    {
//        if(obsRGBD[i])
//            obsRGBD[i]->unload();
        obsRGBD[i] = obsRGBD[i-1];
//        v_rgb[i] = v_rgb[i-1];
        cv::swap(v_rgb[i], v_rgb[i-1]);
        cv::swap(v_gray[i], v_gray[i-1]);
        cv::swap(v_depth[i], v_depth[i-1]);
//        v_cloud[i].swap(v_cloud[i-1]);
        vv_segments2D[i] = vv_segments2D[i-1];
        vv_segment_n[i] = vv_segment_n[i-1];
    }
}
