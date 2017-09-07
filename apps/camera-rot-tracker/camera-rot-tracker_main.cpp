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
#include "camera-rot-tracker_misc.h"
#include <mrpt/system/os.h>

using namespace std;
using namespace mrpt::system;

void print_help(char ** argv)
{
    cout << "This program tracks the rotation of a camera by matching point and lines along consecutive frames." << endl
         << "The overlap may be lost as far as corresponding lines are still visible in the two views." << endl
         << "This program accepts a single argument specifying a configuration file which indicates the rawlog file of a camera sequence." << endl
         << "The step among frames can be specified, and also the possibility to rely only on points, lines or both together. The output text" << endl
         << "file with the \"angular trajectory\", and display/verbose parameters can also be set in the configuration.\n" << endl;

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
        printf(" camera-rot-tracker - Part of the MRPT\n");
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
        string config_file = find_mrpt_shared_dir() + std::string("config_files/camera-tracking/rot_TUM_RGBD_rpy.ini"); // Default config file
        if(argc == 2)
            config_file = argv[1];

        if(!fileExists(config_file))
        {
            cout << "config_file: " << config_file << " does not exist\n\n";
            print_help(argv);
            mrpt::system::pause();
            return -1;
        }

        CameraRotTracker camera_tracker;
        camera_tracker.loadConfiguration(config_file);
        camera_tracker.run();

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
