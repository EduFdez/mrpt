/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#pragma once

#include "kinect-rig-calib.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

class KinectRigCalib_display
{
    KinectRigCalib krc;

    pcl::visualization::CloudViewer viewer;

    bool b_confirm_visually;
    bool b_exit;
    bool b_viz_init;
    bool b_freeze;

    void viz_cb (pcl::visualization::PCLVisualizer& viz);

    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

  public:

    KinectRigCalib_display(const KinectRigCalib & calib) :
        viewer("kinect-rig-calib"),
        b_confirm_visually(false),
        b_exit(false),
        b_viz_init(false),
        b_freeze(false)
    {
        // Initialize visualizer
        viewer.runOnVisualizationThread (boost::bind(&KinectRigCalib_display::viz_cb, this, _1), "viz_cb");
        viewer.registerKeyboardCallback ( &KinectRigCalib_display::keyboardEventOccurred, *this );
    }

    ~KinectRigCalib_display()
    {
    }
};

