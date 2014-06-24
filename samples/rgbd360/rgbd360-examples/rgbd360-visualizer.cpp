/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>
#include <mrpt/gui.h>
#include <mrpt/opengl.h>
//#include <mrpt/maps.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/slam/CObservationRGBD360.h>
//#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/slam/CObservationIMU.h>
#include <mrpt/obs.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
//using namespace mrpt::hwdrivers;

// This simple demo opens a rawlog dataset containing the data recorded
// by several rgbd sensors and build the calibrated rgbd images from it

int main ( int argc, char** argv )
{
  try
  {
    if (argc != 2)
    {
      cerr << "Usage: " << argv[0] << " <path_to_rawlog_dataset\n";
      return 1;
    }

    const string RAWLOG_FILENAME = string( argv[1] );
    const unsigned num_sensors = 4;

    unsigned SensorArrangement[] = {1,2,3,4};

//    // Set the sensor poses (Extrinsic calibration)
//    mrpt::poses::CPose3D sensorPoses[num_sensors];
//    Eigen::Matrix4d pose_sensor1 = Eigen::Matrix4d::Identity(); pose_sensor1.block(0,3,3,1) << 0, 0, 0.055;
//    //math::CMatrixDouble44 pose_sensor[num_sensors];
//    sensorPoses[0] = mrpt::poses::CPose3D(pose_sensor1);
//
//    Eigen::Matrix4d Rt_45 = Eigen::Matrix4d::Identity();
//    Rt_45(1,1) = Rt_45(2,2) = cos(45*PI/180);
//    Rt_45(1,2) = -sin(45*PI/180);
//    Rt_45(2,1) = -Rt_45(1,2);
//
//    for(unsigned i=1; i < num_sensors; i++)
//    {
////      pose_mat
//      Eigen::Matrix4d pose_sensor = Rt_45 * sensorPoses[SensorArrangement[i-1]]*getHomogeneousMatrixVal();
//      sensorPoses[SensorArrangement[i]] = Rt_45*mrpt::poses::CPose3D(pose_sensor);
//    }

    CFileGZInputStream rawlogFile(RAWLOG_FILENAME);
    CActionCollectionPtr action;
    CSensoryFramePtr observations;
    CObservationPtr observation;
    size_t rawlogEntry=0;
    //bool end = false;

    CObservation3DRangeScanPtr obsRGBD[4];  // Pointers to the 4 images that compose an observation
    bool rgbd1 = false, rgbd2 = false, rgbd3 = false, rgbd4 = false;
    CObservation2DRangeScanPtr laserObs;    // Pointer to the laser observation
    const int decimation = 1;
    int num_observations = 0, num_rgbd360_obs = 0;

    // Create window and prepare OpenGL object in the scene:
    // --------------------------------------------------------
    bool bVisualize = true;
    mrpt::gui::CDisplayWindow3D  win3D("OpenNI2 3D view",800,600);

    win3D.setCameraAzimuthDeg(140);
    win3D.setCameraElevationDeg(20);
    win3D.setCameraZoom(8.0);
    win3D.setFOV(90);
    win3D.setCameraPointingToPoint(2.5,0,0);

    mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
    gl_points->setPointSize(2.5);

    opengl::COpenGLViewportPtr viewInt; // Extra viewports for the RGB images.
    {
      mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();

      // Create the Opengl object for the point cloud:
      scene->insert( gl_points );
      scene->insert( mrpt::opengl::CGridPlaneXY::Create() );
      scene->insert( mrpt::opengl::stock_objects::CornerXYZ() );

      const double aspect_ratio =  480.0 / 640.0;
      const int VW_WIDTH = 400;	// Size of the viewport into the window, in pixel units.
      const int VW_HEIGHT = aspect_ratio*VW_WIDTH;

      // Create an extra opengl viewport for the RGB image:
      viewInt = scene->createViewport("view2d_int");
      viewInt->setViewportPosition(5, 30, VW_WIDTH,VW_HEIGHT );
      win3D.addTextMessage(10, 30+VW_HEIGHT+10,"Intensity data",TColorf(1,1,1), 2, MRPT_GLUT_BITMAP_HELVETICA_12 );

      win3D.addTextMessage(5,5,
        format("'o'/'i'-zoom out/in, ESC: quit"),
          TColorf(0,0,1), 110, MRPT_GLUT_BITMAP_HELVETICA_18 );

      win3D.unlockAccess3DScene();
      win3D.repaint();
    }


    while ( CRawlog::getActionObservationPairOrObservation(
                                                 rawlogFile,      // Input file
                                                 action,            // Possible out var: action of a pair action/obs
                                                 observations,  // Possible out var: obs's of a pair action/obs
                                                 observation,    // Possible out var: a single obs.
                                                 rawlogEntry    // Just an I/O counter
                                                 ) )
    {
      // Process action & observations
      if (observation)
      {
        // assert(IS_CLASS(observation, CObservation2DRangeScan) || IS_CLASS(observation, CObservation3DRangeScan));
        cout << "Observation " << num_observations++ << " timestamp " << observation->timestamp << endl;

        if(observation->sensorLabel == "RGBD1")
        {
          obsRGBD[0] = CObservation3DRangeScanPtr(observation);
          rgbd1 = true;
        }
        if(observation->sensorLabel == "RGBD2")
        {
          obsRGBD[1] = CObservation3DRangeScanPtr(observation);
          rgbd2 = true;
        }
        if(observation->sensorLabel == "RGBD3")
        {
          obsRGBD[2] = CObservation3DRangeScanPtr(observation);
          rgbd3 = true;
        }
        if(observation->sensorLabel == "RGBD4")
        {
          obsRGBD[3] = CObservation3DRangeScanPtr(observation);
          rgbd4 = true;
        }
        else if(observation->sensorLabel == "LASER")
        {
          laserObs = CObservation2DRangeScanPtr(observation);
        }
      }
      else
      {
      // action, observations should contain a pair of valid data (Format #1 rawlog file)
        THROW_EXCEPTION("Not a valid observation");
      }

      if(!(rgbd1 && rgbd2 && rgbd3 && rgbd4))
        continue;

      rgbd1 = rgbd2 = rgbd3 = rgbd4 = false; // Reset the counter of simultaneous observations

      // Apply decimation
      num_rgbd360_obs++;
      if(num_rgbd360_obs%decimation != 0)
        continue;

//      // Fill the frame180 structure
//      CObservationRGBD360 obs360;
//      for(unsigned i=0; i<obs360.NUM_SENSORS; i++)
//      {
//        obs360.rgbd[i] = *obsRGBD[SensorArrangement[i]];
//      }

      // Segment surfaces (planes and curve regions)


      // Visualize the data
      if(bVisualize)
      {
        // It IS a new observation:
        mrpt::system::TTimeStamp last_obs_tim = observation->timestamp;

        // Update visualization ---------------------------------------

        win3D.get3DSceneAndLock();

        // Estimated grabbing rate:
        win3D.addTextMessage(-350,-13, format("Timestamp: %s", mrpt::system::dateTimeLocalToString(last_obs_tim).c_str()), TColorf(0.6,0.6,0.6),"mono",10,mrpt::opengl::FILL, 100);

        // Show intensity image:
        if (obsRGBD[0]->hasIntensityImage )
        {
          viewInt->setImageView(obsRGBD[0]->intensityImage); // This is not "_fast" since the intensity image may be needed later on.
        }
        win3D.unlockAccess3DScene();

        // -------------------------------------------------------
        //           Create 3D points from RGB+D data
        //
        // There are several methods to do this.
        //  Switch the #if's to select among the options:
        // See also: http://www.mrpt.org/Generating_3D_point_clouds_from_RGB_D_observations
        // -------------------------------------------------------
        {
          win3D.get3DSceneAndLock();
            obsRGBD[0]->project3DPointsFromDepthImageInto(*gl_points, false /* without obs.sensorPose */);
          win3D.unlockAccess3DScene();
        }

        win3D.repaint();
      }

    };



//    cout << "OK " << rgbd_sensor.numDevices << " available devices."  << endl;
//    cout << "\nUse device " << sensor_id_or_serial << endl << endl;
//
//    bool showImages = false;
//   int cloud and images
//    {
//      // Create window and prepare OpenGL object in the scene:
//      // --------------------------------------------------------
//      mrpt::gui::CDisplayWindow3D  win3D("OpenNI2 3D view",800,600);
//
//      win3D.setCameraAzimuthDeg(140);
//      win3D.setCameraElevationDeg(20);
//      win3D.setCameraZoom(8.0);
//      win3D.setFOV(90);
//      win3D.setCameraPointingToPoint(2.5,0,0);
//
//      mrpt::opengl::CPointCloudColouredPtr gl_points = mrpt::opengl::CPointCloudColoured::Create();
//      gl_points->setPointSize(2.5);
//
//      opengl::COpenGLViewportPtr viewInt; // Extra viewports for the RGB images.
//      {
//        mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();
//
//        // Create the Opengl object for the point cloud:
//        scene->insert( gl_points );
//        scene->insert( mrpt::opengl::CGridPlaneXY::Create() );
//        scene->insert( mrpt::opengl::stock_objects::CornerXYZ() );
//
//        const double aspect_ratio =  480.0 / 640.0;
//        const int VW_WIDTH = 400;	// Size of the viewport into the window, in pixel units.
//        const int VW_HEIGHT = aspect_ratio*VW_WIDTH;
//
//        // Create an extra opengl viewport for the RGB image:
//        viewInt = scene->createViewport("view2d_int");
//        viewInt->setViewportPosition(5, 30, VW_WIDTH,VW_HEIGHT );
//        win3D.addTextMessage(10, 30+VW_HEIGHT+10,"Intensity data",TColorf(1,1,1), 2, MRPT_GLUT_BITMAP_HELVETICA_12 );
//
//        win3D.addTextMessage(5,5,
//          format("'o'/'i'-zoom out/in, ESC: quit"),
//            TColorf(0,0,1), 110, MRPT_GLUT_BITMAP_HELVETICA_18 );
//
//        win3D.unlockAccess3DScene();
//        win3D.repaint();
//      }
//
//      //							Grab frames continuously and show
//      //========================================================================================
//
//      bool bObs = false, bError = true;
//      mrpt::system::TTimeStamp  last_obs_tim = INVALID_TIMESTAMP;
//
//      while (!win3D.keyHit())	//Push any key to exit // win3D.isOpen()
//      {
//  //    cout << "Get new observation\n";
//        CObservation3DRangeScanPtr obsRGBD[0] = CObservation3DRangeScan::Create();
//        rgbd_sensor.getNextObservation(*obsRGBD[0], bObs, bError);
//
//        if (bObs && !bError && obsRGBD[0] && obsRGBD[0]->timestamp!=INVALID_TIMESTAMP && obsRGBD[0]->timestamp!=last_obs_tim )
//        {
//          // It IS a new observation:
//          last_obs_tim = obsRGBD[0]->timestamp;
//
//          // Update visualization ---------------------------------------
//
//          win3D.get3DSceneAndLock();
//
//          // Estimated grabbing rate:
//          win3D.addTextMessage(-350,-13, format("Timestamp: %s", mrpt::system::dateTimeLocalToString(last_obs_tim).c_str()), TColorf(0.6,0.6,0.6),"mono",10,mrpt::opengl::FILL, 100);
//
//          // Show intensity image:
//          if (obsRGBD[0]->hasIntensityImage )
//          {
//            viewInt->setImageView(obsRGBD[0]->intensityImage); // This is not "_fast" since the intensity image may be needed later on.
//          }
//          win3D.unlockAccess3DScene();
//
//          // -------------------------------------------------------
//          //           Create 3D points from RGB+D data
//          //
//          // There are several methods to do this.
//          //  Switch the #if's to select among the options:
//          // See also: http://www.mrpt.org/Generating_3D_point_clouds_from_RGB_D_observations
//          // -------------------------------------------------------
//          if (obsRGBD[0]->hasRangeImage)
//          {
//    #if 0
//            static pcl::PointCloud<pcl::PointXYZRGB> cloud;
//            obsRGBD[0]->project3DPointsFromDepthImageInto(cloud, false /* without obs.sensorPose */);
//
//            win3D.get3DSceneAndLock();
//              gl_points->loadFromPointsMap(&cloud);
//            win3D.unlockAccess3DScene();
//    #endif
//
//    // Pathway: RGB+D --> XYZ+RGB opengl
//    #if 1
//            win3D.get3DSceneAndLock();
//              obsRGBD[0]->project3DPointsFromDepthImageInto(*gl_points, false /* without obs.sensorPose */);
//            win3D.unlockAccess3DScene();
//    #endif
//          }
//
//          win3D.repaint();
//        } // end update visualization:
//      }
//    }

    cout << "\n ... END rgbd360-visualizer ...\n";

    return 0;

	} catch (std::exception &e)
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
