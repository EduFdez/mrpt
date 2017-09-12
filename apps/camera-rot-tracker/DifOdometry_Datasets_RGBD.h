/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "camera-rot-tracker.h"
#include <mrpt/vision/CDifodo.h>
#include <mrpt/utils/types_math.h> // Eigen (with MRPT "plugin" in BaseMatrix<>)
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/CImage.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
//#include <mrpt/opengl/COpenGLScene.h>
//#include <mrpt/gui.h>
#include <iostream>

class CDifodoDatasets_RGBD : public CameraRotTracker, mrpt::vision::CDifodo {
public:

	mrpt::poses::CPose3D gt_pose;		//!< Groundtruth camera pose
	mrpt::poses::CPose3D gt_oldpose;	//!< Groundtruth camera previous pose
    mrpt::poses::CPose3D gt_rel_pose;   //!< Groundtruth of the relative camera pose wrt its previous observation
    mrpt::poses::CPose3D gt_rel_pose_TUM;   //!< Groundtruth of the relative camera pose wrt its previous observation
    mrpt::poses::CPose3D transf;        //!< The relative pose between TUM and MRPT reference frames
    mrpt::poses::CPose3D rel_pose_TUM;	//!< Estimated relative camera pose in the camera reference frame

//	mrpt::opengl::COpenGLScenePtr scene;	//!< Opengl scene
//	mrpt::gui::CDisplayWindow3D	window;
	mrpt::obs::CRawlog	dataset;
	std::ifstream		f_gt;
	std::ofstream		f_res;

	unsigned int repr_level;
	unsigned int rawlog_count;
	bool first_pose;
	bool save_results;
	bool dataset_finished;

	/** Constructor. */
    CDifodoDatasets_RGBD() : mrpt::vision::CDifodo()
	{
		save_results = 0;
		first_pose = false;
		dataset_finished = false;
        transf.setFromValues(0,0,0,0.5*M_PI, -0.5*M_PI, 0);
        std::cout << "transf \n" << transf.getHomogeneousMatrixVal() << std::endl;
	}

	/** Initialize the visual odometry method and loads the rawlog file */
    void loadConfiguration(const std::string & config_file);

	/** Load the depth image and the corresponding groundtruth pose */
	void loadFrame();

    /** Read the dataset and compute odometry */
    void run(const std::string & config_file);

	/** Create a file to save the trajectory estimates */
	void CreateResultsFile();

//	/** Initialize the opengl scene */
//	void initializeScene();

//	/** Update the opengl scene */
//	void updateScene();

	/** A pre-step that should be performed before starting to estimate the camera speed
	  * As a couple of frames are necessary to estimate the camera motion, this methods loads the first frame
	  * before any motion can be estimated.*/
	void reset();

	/** Save the pose estimation following the format of the TUM datasets:
	  * 
	  * 'timestamp tx ty tz qx qy qz qw'
	  *
	  * Please visit http://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats for further details.*/  
	void writeTrajectoryFile();

private:

	// Used to interpolate grountruth poses
	bool groundtruth_ok;
	bool last_groundtruth_ok;

	double last_groundtruth;	//!< Timestamp of the last groundtruth read
	double timestamp_obs;		//!< Timestamp of the last observation
	double last_gt_data[7];		//!< Last ground truth read (x y z qx qy qz w)
};
