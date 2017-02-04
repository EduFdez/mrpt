/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


//#include <mrpt/math/CMatrixFixedNumeric.h>
//#include <mrpt/utils/CArray.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/gui/gui.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/gui/CDisplayWindow3D.h>
//#include <mrpt/gui/CDisplayWindowPlots.h>
//#include <mrpt/opengl/CGridPlaneXY.h>
//#include <mrpt/opengl/CPointCloud.h>
//#include <mrpt/opengl/stock_objects.h>
//#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/utils/CConfigFile.h>
//#include <mrpt/utils/CFileGZInputStream.h>
////#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
//#include <mrpt/system/threads.h>
#include <mrpt/system/filesystem.h>
//#include <mrpt/opengl/CPlanarLaserScan.h>  // This class lives in the lib [mrpt-maps] and must be included by hand
//#include <mrpt/math/ransac_applications.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace std;
using namespace Eigen;



static mrpt::opengl::COpenGLScenePtr scene;	//!< Opengl scene
static mrpt::gui::CDisplayWindow3D	window;

static poses::CPose3D cam_pose(0,0,0,0,0,0);

void initializeScene()
{
    poses::CPose3D rel_lenspose(0,-0.05,0,0,0,0);

    global_settings::OCTREE_RENDER_MAX_POINTS_PER_NODE = 1000000;
    window.resize(1000,900);
    window.setPos(900,0);
    window.setCameraZoom(16);
    window.setCameraAzimuthDeg(0);
    window.setCameraElevationDeg(90);
    window.setCameraPointingToPoint(0,0,0);
    window.setCameraPointingToPoint(0,0,1);

    scene = window.get3DSceneAndLock();

    // Lights:
    scene->getViewport()->setNumberOfLights(1);
    CLight & light0 = scene->getViewport()->getLight(0);
    light0.light_ID = 0;
    light0.setPosition(0,0,1,1);

    //Grid (ground)
    CGridPlaneXYPtr ground = CGridPlaneXY::Create();
    scene->insert( ground );

    //Reference
    CSetOfObjectsPtr reference = stock_objects::CornerXYZ();
    scene->insert( reference );

    //					Cameras and points
    //------------------------------------------------------

    //DifOdo camera
    CBoxPtr camera_odo = CBox::Create(math::TPoint3D(-0.02,-0.1,-0.01),math::TPoint3D(0.02,0.1,0.01));
    camera_odo->setPose(cam_pose + rel_lenspose);
    camera_odo->setColor(0,1,0);
    scene->insert( camera_odo );

    //Frustum
    opengl::CFrustumPtr FOV = opengl::CFrustum::Create(0.3, 2, 57.3*fovh, 57.3*fovv, 1.f, true, false);
    FOV->setColor(0.7,0.7,0.7);
    FOV->setPose(cam_pose);
    scene->insert( FOV );

    //Reference cam
    CSetOfObjectsPtr reference_gt = stock_objects::CornerXYZ();
    reference_gt->setScale(0.2);
    reference_gt->setPose(cam_pose);
    scene->insert( reference_gt );

    //Camera points
    CPointCloudColouredPtr cam_points = CPointCloudColoured::Create();
    cam_points->setPointSize(2);
    cam_points->enablePointSmooth(1);
    cam_points->setPose(cam_pose);
    scene->insert( cam_points );

    window.unlockAccess3DScene();
    window.repaint();
}

void CDifodoDatasets::updateScene(CColouredPointsMap & pts_map)
{
    CPose3D rel_lenspose(0,-0.022,0,0,0,0);

    scene = window.get3DSceneAndLock();

    //Reference gt
    CSetOfObjectsPtr reference_gt = scene->getByClass<CSetOfObjects>(0);
    reference_gt->setPose(gt_pose);

    //Camera points
    CPointCloudColouredPtr cam_points = scene->getByClass<CPointCloudColoured>(0);
    cam_points->clear();
    cam_points->setPose(gt_pose);
    cam_points->loadFromPointsMap(&pts_map);
//    for (unsigned int y=0; y<cols; y++)
//        for (unsigned int z=0; z<rows; z++)
//            cam_points->push_back(depth[repr_level](z,y), xx[repr_level](z,y), yy[repr_level](z,y),
//                                    1.f-sqrt(weights(z,y)), sqrt(weights(z,y)), 0);


    //Frustum
    CFrustumPtr FOV = scene->getByClass<CFrustum>(0);
    FOV->setPose(gt_pose);

//    //Ellipsoid showing covariance
//    math::CMatrixFloat33 cov3d = 20.f*est_cov.topLeftCorner(3,3);
//    CEllipsoidPtr ellip = scene->getByClass<CEllipsoid>(0);
//    ellip->setCovMatrix(cov3d);
//    ellip->setPose(cam_pose + rel_lenspose);

    window.unlockAccess3DScene();
    window.repaint();
}
