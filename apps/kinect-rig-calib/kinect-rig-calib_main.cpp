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

// 0 load init file with estimated init_poses and errors. display/verbose parameters    -OK
// 1 open rawlog                                                                    -OK
// 2 identify observation pairs/triples...                                          -OK
// 3 segment planes and lines
// 4 get correspondences
// 5 perform calibration. calibration algorithm in a different file
// 6 visualize and evaluate

//#include "DownsampleRGBD.h"
#include "kinect-rig-calib.h"
#include "kinect-rig-calib_misc.h"
//#include "kinect-rig-calib_display.h"
#include <mrpt/system/os.h>

using namespace std;
using namespace mrpt::system;

void print_help(char ** argv)
{
    cout << "This program computes the extrinsic calibration of several RGB-D sensors (e.g. Kinect) based on plane and line correspondences." << endl
         << "No overlapping required for the sensor FOV, as far as the planes/lines can be observed simultaneously by pairs of sensors." << endl
         << "This program accepts a single argument specifying a configuration file which indicates the rawlog file of sensor observations" << endl
         << "to compute the calibration, the approximated sensor init_poses, and a conservative approximation of their accuracy. The output text" << endl
         << "file with the calibration results, and display/verbose parameters can also be set in the configuration.\n" << endl;

    cout << "Usage: " << argv[0] << " [config_file] \n";
    cout << "    [config_file] optional configuration file which contains the information of the RGBD sequences and estimated calibration" << endl;
    cout << "         " << argv[0] << " -h | --help : shows this help" << endl;
}

// ------------------------------------------------------
//                         MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
    try
    {
        printf(" kinect-rig-calib - Part of the MRPT\n");
        printf(" MRPT C++ Library: %s - BUILD DATE %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
        printf("-------------------------------------------------------------------\n");

        // Process arguments:
        bool showHelp = (argc == 2 && (!os::_strcmp(argv[1],"-h") || !os::_strcmp(argv[1],"--help")));
        if(showHelp)
        {
            if(argc == 2 && !fileExists(argv[1]))
                cout << "config_file: " << argv[1] << " does not exist\n\n";
            print_help(argv);
            mrpt::system::pause();
            return -1;
        }
        string config_file = find_mrpt_shared_dir() + std::string("config_files/calibration/extrinsic_calib_2rgbd.ini"); // Default config file
        if(argc == 2)
            config_file = argv[1];

        if(!fileExists(config_file))
        {
            cout << "config_file: " << config_file << " does not exist\n\n";
            print_help(argv);
            mrpt::system::pause();
            return -1;
        }

        KinectRigCalib calibrator;
        calibrator.loadConfiguration(config_file);
        calibrator.run();

        return 0;
    }
    catch (std::exception &e)
    {
            std::cout << "MRPT exception caught: " << e.what() << std::endl;
            return -1;
    }
    catch (...)
    {
            printf("Untyped exception!!");
            return -1;
    }
}
