/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "DifOdometry_Datasets_RGBD.h"
#include "pinhole_model_warp.h"
//#include "../DifOdometry-Datasets/legend.xpm"
#include "../kinect-rig-calib/kinect-rig-calib_misc.h"
#include <mrpt/system/filesystem.h>
#include <mrpt/system/threads.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/vision/CFeatureLines.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/vision/tracking.h>
#include <mrpt/pbmap/PbMap.h>
#include <mrpt/pbmap/colors.h>
#include <mrpt/slam/MetricRegistration.h>
//#include <mrpt/opengl/CFrustum.h>
//#include <mrpt/opengl/CGridPlaneXY.h>
//#include <mrpt/opengl/CBox.h>
//#include <mrpt/opengl/CPointCloudColoured.h>
//#include <mrpt/opengl/CPointCloud.h>
//#include <mrpt/opengl/CSetOfLines.h>
//#include <mrpt/opengl/CEllipsoid.h>
//#include <mrpt/opengl/stock_objects.h>

#include "opencv2/core/cuda.hpp"
#include "opencv2/cudawarping.hpp"
#include "opencv2/cudaarithm.hpp"

#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/fast_bilateral_omp.h>

using namespace Eigen;
using namespace std;
using namespace mrpt;
using namespace mrpt::opengl;

//using namespace mrpt::gui;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::vision;
using namespace mrpt::slam;

void CDifodoDatasets_RGBD::loadConfiguration(const string & config_file)
{	
    cout << "CDifodoDatasets_RGBD::loadConfiguration...\n";

    CameraRotTracker::loadConfiguration(config_file);

    CConfigFile ini(config_file);

	fovh = M_PIf*62.5f/180.0f;	//Larger FOV because depth is registered with color
	fovv = M_PIf*48.5f/180.0f;
	cam_mode = 1;
	fast_pyramid = false;
	downsample = ini.read_int("DIFODO_CONFIG", "downsample", 2, true);
	rows = ini.read_int("DIFODO_CONFIG", "rows", 240, true);
	cols = ini.read_int("DIFODO_CONFIG", "cols", 320, true);
    ctf_levels = ini.read_int("DIFODO_CONFIG", "ctf_levels", 5, true);
    if(b_initial_rot)
        ctf_levels = 2;

	//						Open Rawlog File
	//==================================================================
    if (!dataset.loadFromRawLogFile(rawlog_file))
		throw std::runtime_error("\nCouldn't open rawlog dataset file for input...");

	rawlog_count = 0;

	// Set external images directory:
    const string imgsPath = CRawlog::detectImagesDirectory(rawlog_file);
	CImage::IMAGES_PATH_BASE = imgsPath;

	//					Load ground-truth
	//=========================================================
    string gt_file = system::extractFileDirectory(rawlog_file);
    gt_file.append("/groundtruth.txt");
    f_gt.open(gt_file.c_str());

	if (f_gt.fail())
		throw std::runtime_error("\nError finding the groundtruth file: it should be contained in the same folder than the rawlog file");

	char aux[100];
	f_gt.getline(aux, 100);
	f_gt.getline(aux, 100);
	f_gt.getline(aux, 100);
	f_gt >> last_groundtruth;
	f_gt >> last_gt_data[0]; f_gt >> last_gt_data[1]; f_gt >> last_gt_data[2];
	f_gt >> last_gt_data[3]; f_gt >> last_gt_data[4]; f_gt >> last_gt_data[5]; f_gt >> last_gt_data[6];
	last_groundtruth_ok = 1;

	//			Resize matrices and adjust parameters
	//=========================================================
	width = 640/(cam_mode*downsample);
	height = 480/(cam_mode*downsample);
	repr_level = utils::round(log(float(width/cols))/log(2.f));

	//Resize pyramid
    const unsigned int pyr_levels = round(log(float(width/cols))/log(2.f)) + ctf_levels;
    cout << "x pyr_levels " << pyr_levels << endl;
    depth.resize(pyr_levels);
    depth_old.resize(pyr_levels);
    depth_inter.resize(pyr_levels);
	depth_warped.resize(pyr_levels);
    xx.resize(pyr_levels);
    xx_inter.resize(pyr_levels);
    xx_old.resize(pyr_levels);
	xx_warped.resize(pyr_levels);
    yy.resize(pyr_levels);
    yy_inter.resize(pyr_levels);
    yy_old.resize(pyr_levels);
	yy_warped.resize(pyr_levels);
	transformations.resize(pyr_levels);

	for (unsigned int i = 0; i<pyr_levels; i++)
    {
        unsigned int s = pow(2.f,int(i));
        cols_i = width/s; rows_i = height/s;
        depth[i].resize(rows_i, cols_i);
        depth_inter[i].resize(rows_i, cols_i);
        depth_old[i].resize(rows_i, cols_i);
        depth[i].assign(0.0f);
        depth_old[i].assign(0.0f);
        xx[i].resize(rows_i, cols_i);
        xx_inter[i].resize(rows_i, cols_i);
        xx_old[i].resize(rows_i, cols_i);
        xx[i].assign(0.0f);
        xx_old[i].assign(0.0f);
        yy[i].resize(rows_i, cols_i);
        yy_inter[i].resize(rows_i, cols_i);
        yy_old[i].resize(rows_i, cols_i);
        yy[i].assign(0.0f);
        yy_old[i].assign(0.0f);

		if (cols_i <= cols)
		{
			depth_warped[i].resize(rows_i,cols_i);
			xx_warped[i].resize(rows_i,cols_i);
			yy_warped[i].resize(rows_i,cols_i);
		}
    }

	//Resize matrix that store the original depth image
    depth_wf.setSize(height,width);
}

void CDifodoDatasets_RGBD::CreateResultsFile()
{
	try
	{
		// Open file, find the first free file-name.
		char	aux[100];
		int     nFile = 0;
		bool    free_name = false;

        system::createDirectory("./difresults");

		while (!free_name)
		{
			nFile++;
            sprintf(aux, "./difresults/experiment_%03u.txt", nFile );
			free_name = !system::fileExists(aux);
		}

		// Open log file:
		f_res.open(aux);
		printf(" Saving results to file: %s \n", aux);
	}
	catch (...)
	{
		printf("Exception found trying to create the 'results file' !!\n");
	}

}
/*
void CDifodoDatasets_RGBD::initializeScene()
{
	CPose3D rel_lenspose(0,-0.022,0,0,0,0);
	
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
	//CSetOfObjectsPtr reference = stock_objects::CornerXYZ();
	//scene->insert( reference );

	//					Cameras and points
	//------------------------------------------------------

	//DifOdo camera
	CBoxPtr camera_odo = CBox::Create(math::TPoint3D(-0.02,-0.1,-0.01),math::TPoint3D(0.02,0.1,0.01));
	camera_odo->setPose(cam_pose + rel_lenspose);
	camera_odo->setColor(0,1,0);
	scene->insert( camera_odo );

	//Groundtruth camera
	CBoxPtr camera_gt = CBox::Create(math::TPoint3D(-0.02,-0.1,-0.01),math::TPoint3D(0.02,0.1,0.01));
	camera_gt->setPose(gt_pose + rel_lenspose);
	camera_gt->setColor(1,0,0);
	scene->insert( camera_gt );

	//Frustum
	opengl::CFrustumPtr FOV = opengl::CFrustum::Create(0.3f, 2, 57.3f*fovh, 57.3f*fovv, 1.f, true, false);
	FOV->setColor(0.7,0.7,0.7);
	FOV->setPose(gt_pose);
	scene->insert( FOV );

	//Reference gt
	CSetOfObjectsPtr reference_gt = stock_objects::CornerXYZ();
	reference_gt->setScale(0.2f);
	reference_gt->setPose(gt_pose);
	scene->insert( reference_gt );

	//Camera points
	CPointCloudColouredPtr cam_points = CPointCloudColoured::Create();
	cam_points->setColor(1,0,0);
	cam_points->setPointSize(2);
	cam_points->enablePointSmooth(1);
	cam_points->setPose(cam_pose);
	scene->insert( cam_points );


	//					Trajectories and covariance
	//-------------------------------------------------------------

	//Dif Odometry
	CSetOfLinesPtr traj_lines_odo = CSetOfLines::Create();
	traj_lines_odo->setLocation(0,0,0);
	traj_lines_odo->setColor(0,0.6,0);
	traj_lines_odo->setLineWidth(6);
	scene->insert( traj_lines_odo );
	CPointCloudPtr traj_points_odo = CPointCloud::Create();
	traj_points_odo->setColor(0,0.6,0);
	traj_points_odo->setPointSize(4);
	traj_points_odo->enablePointSmooth(1);
	scene->insert( traj_points_odo );

	//Groundtruth
	CSetOfLinesPtr traj_lines_gt = CSetOfLines::Create();
	traj_lines_gt->setLocation(0,0,0);
	traj_lines_gt->setColor(0.6,0,0);
	traj_lines_gt->setLineWidth(6);
	scene->insert( traj_lines_gt );
	CPointCloudPtr traj_points_gt = CPointCloud::Create();
	traj_points_gt->setColor(0.6,0,0);
	traj_points_gt->setPointSize(4);
	traj_points_gt->enablePointSmooth(1);
	scene->insert( traj_points_gt );

	//Ellipsoid showing covariance
	math::CMatrixFloat33 cov3d = 20.f*est_cov.topLeftCorner(3,3);
	CEllipsoidPtr ellip = CEllipsoid::Create();
	ellip->setCovMatrix(cov3d);
	ellip->setQuantiles(2.0);
	ellip->setColor(1.0, 1.0, 1.0, 0.5);
	ellip->enableDrawSolid3D(true);
	ellip->setPose(cam_pose + rel_lenspose);
	scene->insert( ellip );

	//User-interface information
	utils::CImage img_legend;
	img_legend.loadFromXPM(legend_xpm);
	COpenGLViewportPtr legend = scene->createViewport("legend");
	legend->setViewportPosition(20, 20, 332, 164);
	legend->setImageView(img_legend);

	window.unlockAccess3DScene();
	window.repaint();
}

void CDifodoDatasets_RGBD::updateScene()
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
	for (unsigned int y=0; y<cols; y++)
		for (unsigned int z=0; z<rows; z++)
			cam_points->push_back(depth[repr_level](z,y), xx[repr_level](z,y), yy[repr_level](z,y),
									1.f-sqrt(weights(z,y)), sqrt(weights(z,y)), 0);

	//DifOdo camera
	CBoxPtr camera_odo = scene->getByClass<CBox>(0);
	camera_odo->setPose(cam_pose + rel_lenspose);

	//Groundtruth camera
	CBoxPtr camera_gt = scene->getByClass<CBox>(1);
	camera_gt->setPose(gt_pose + rel_lenspose);

	//Frustum
	CFrustumPtr FOV = scene->getByClass<CFrustum>(0);
	FOV->setPose(gt_pose);

	if ((first_pose == true)&&((gt_pose-gt_oldpose).norm() < 0.5))
	{
		//Difodo traj lines
		CSetOfLinesPtr traj_lines_odo = scene->getByClass<CSetOfLines>(0);
		traj_lines_odo->appendLine(cam_oldpose.x(), cam_oldpose.y(), cam_oldpose.z(), cam_pose.x(), cam_pose.y(), cam_pose.z());

		//Difodo traj points
		CPointCloudPtr traj_points_odo = scene->getByClass<CPointCloud>(0);
		traj_points_odo->insertPoint(cam_pose.x(), cam_pose.y(), cam_pose.z());

		//Groundtruth traj lines
		CSetOfLinesPtr traj_lines_gt = scene->getByClass<CSetOfLines>(1);
		traj_lines_gt->appendLine(gt_oldpose.x(), gt_oldpose.y(), gt_oldpose.z(), gt_pose.x(), gt_pose.y(), gt_pose.z());

		//Groundtruth traj points
		CPointCloudPtr traj_points_gt = scene->getByClass<CPointCloud>(1);
		traj_points_gt->insertPoint(gt_pose.x(), gt_pose.y(), gt_pose.z());
	}

	//Ellipsoid showing covariance
	math::CMatrixFloat33 cov3d = 20.f*est_cov.topLeftCorner(3,3);
	CEllipsoidPtr ellip = scene->getByClass<CEllipsoid>(0);
	ellip->setCovMatrix(cov3d);
	ellip->setPose(cam_pose + rel_lenspose);

	window.unlockAccess3DScene();
	window.repaint();
}
*/
void CDifodoDatasets_RGBD::loadFrame()
{
    //cout << "CDifodoDatasets_RGBD::loadFrame...\n";
    CObservationPtr observation;
    for(size_t step = 0; step < decimation; step++ )
    {
        observation = dataset.getAsObservation(rawlog_count++);
        while (!IS_CLASS(observation, CObservation3DRangeScan))
        {
            if (dataset.size() <= rawlog_count)
            {
                dataset_finished = true;
                return;
            }
            observation = dataset.getAsObservation(rawlog_count++);
        }
    }
    cout << "\nCDifodoDatasets_RGBD::loadFrame... " << rawlog_count << " / " << dataset.size() << endl;

    obsRGBD[0] = CObservation3DRangeScanPtr(observation);
    obsRGBD[0]->load();
    const CMatrix range = obsRGBD[0]->rangeImage;
	const unsigned int height = range.getRowCount();
	const unsigned int width = range.getColCount();
    v_rgb[0] = cv::cvarrToMat(obsRGBD[0]->intensityImage.getAs<IplImage>());
    //rgb_undist.undistort(cv::cvarrToMat(obsRGBD[0]->intensityImage.getAs<IplImage>()), v_rgb[0]);
    cv::cvtColor( v_rgb[0], v_gray[0], cv::COLOR_BGR2GRAY );
    convertRange_mrpt2cvMat(obsRGBD[0]->rangeImage, v_depth[0]);
    v_cloud[0] = getPointCloud<PointT>(cv::cvarrToMat(obsRGBD[0]->intensityImage.getAs<IplImage>()), v_depth[0], intrinsics);
//    cv::imshow("rgb", v_rgb[0]);
//    cv::waitKey();

	for (unsigned int j = 0; j<cols; j++)
		for (unsigned int i = 0; i<rows; i++)
		{
			const float z = range(height-downsample*i-1, width-downsample*j-1);
			if (z < 4.5f)	depth_wf(i,j) = z;
			else			depth_wf(i, j) = 0.f;
		}


	double timestamp_gt;
    timestamp_obs = mrpt::system::timestampTotime_t(obsRGBD[0]->timestamp);

	//Exit if there is no groundtruth at this time
	if (last_groundtruth > timestamp_obs)
	{
		groundtruth_ok = 0;
        obsRGBD[0]->unload();
		return;
	}

	//Search the corresponding groundtruth data and interpolate
	bool new_data = 0;
	last_groundtruth_ok = groundtruth_ok;
	while (last_groundtruth < timestamp_obs - 0.01)
	{
		f_gt.ignore(100,'\n');
		f_gt >> timestamp_gt;
		last_groundtruth = timestamp_gt;
		new_data = 1;

		if (f_gt.eof())
		{
			dataset_finished = true;
			return;
		}
	}

	//Read the inmediatly previous groundtruth
	double x0,y0,z0,qx0,qy0,qz0,w0,t0;
	if (new_data == 1)
	{
		f_gt >> x0; f_gt >> y0; f_gt >> z0;
		f_gt >> qx0; f_gt >> qy0; f_gt >> qz0; f_gt >> w0;
	}
	else
	{
		x0 = last_gt_data[0]; y0 = last_gt_data[1]; z0 = last_gt_data[2];
		qx0 = last_gt_data[3]; qy0 = last_gt_data[4]; qz0 = last_gt_data[5]; w0 = last_gt_data[6];
	}

	t0 = last_groundtruth;

	//Read the inmediatly posterior groundtruth
	f_gt.ignore(10,'\n');
	f_gt >> timestamp_gt;
	if (f_gt.eof())
	{
		dataset_finished = true;
		return;
	}
	last_groundtruth = timestamp_gt;

	//last_gt_data = [x y z qx qy qz w]
	f_gt >> last_gt_data[0]; f_gt >> last_gt_data[1]; f_gt >> last_gt_data[2];
	f_gt >> last_gt_data[3]; f_gt >> last_gt_data[4]; f_gt >> last_gt_data[5]; f_gt >> last_gt_data[6];

	if (last_groundtruth - timestamp_obs > 0.01)
		groundtruth_ok = 0;

	else
	{
		gt_oldpose = gt_pose;

		//							Update pose
		//-----------------------------------------------------------------
		const float incr_t0 = timestamp_obs - t0;
		const float incr_t1 = last_groundtruth - timestamp_obs;
		const float incr_t = incr_t0 + incr_t1;

		if (incr_t == 0.f) //Deal with defects in the groundtruth files
		{
			groundtruth_ok = 0;
			return;
		}

		//Sometimes the quaternion sign changes in the groundtruth
		if (abs(qx0 + last_gt_data[3]) + abs(qy0 + last_gt_data[4]) + abs(qz0 + last_gt_data[5]) + abs(w0 + last_gt_data[6]) < 0.05)
		{
			qx0 = -qx0; qy0 = -qy0; qz0 = -qz0; w0 = -w0;
		}

		double x,y,z,qx,qy,qz,w;
		x = (incr_t0*last_gt_data[0] + incr_t1*x0)/(incr_t);
		y = (incr_t0*last_gt_data[1] + incr_t1*y0)/(incr_t);
		z = (incr_t0*last_gt_data[2] + incr_t1*z0)/(incr_t);
		qx = (incr_t0*last_gt_data[3] + incr_t1*qx0)/(incr_t);
		qy = (incr_t0*last_gt_data[4] + incr_t1*qy0)/(incr_t);
		qz = (incr_t0*last_gt_data[5] + incr_t1*qz0)/(incr_t);
		w = (incr_t0*last_gt_data[6] + incr_t1*w0)/(incr_t);

		CMatrixDouble33 mat;
		mat(0,0) = 1- 2*qy*qy - 2*qz*qz;
		mat(0,1) = 2*(qx*qy - w*qz);
		mat(0,2) = 2*(qx*qz + w*qy);
		mat(1,0) = 2*(qx*qy + w*qz);
		mat(1,1) = 1 - 2*qx*qx - 2*qz*qz;
		mat(1,2) = 2*(qy*qz - w*qx);
		mat(2,0) = 2*(qx*qz - w*qy);
		mat(2,1) = 2*(qy*qz + w*qx);
		mat(2,2) = 1 - 2*qx*qx - 2*qy*qy;

        CPose3D gt;
		gt.setFromValues(x,y,z,0,0,0);
		gt.setRotationMatrix(mat);

		//Alternative - directly quaternions
		//vector<float> quat;
		//quat[0] = x, quat[1] = y; quat[2] = z;
		//quat[3] = w, quat[4] = qx; quat[5] = qy; quat[6] = qz;
		//gt.setFromXYZQ(quat);

		//Set the initial pose (if appropiate)
		if (first_pose == false)
		{
			cam_pose = gt + transf;
			first_pose = true;
		}

		gt_pose = gt + transf;
        gt_rel_pose = -gt_oldpose + gt_pose;
        gt_rel_pose_TUM = transf + gt_rel_pose + (-transf);
        groundtruth_ok = 1;
    }

    //obsRGBD[0]->unload();

	if (dataset.size() <= rawlog_count)
		dataset_finished = true;

    //cout << "...CDifodoDatasets_RGBD::loadFrame\n";
}

void CDifodoDatasets_RGBD::run()
{
    int pushed_key = 0;
    bool working = 0, stop = 0;

    // Initialize
//    initializeScene();
    reset();

    T max_angle_rad = DEG2RAD(min_angle_diff);

    CFeatureLines featLines;
    //featLines.extractLinesDesc(v_gray[0], vv_segments2D[0], vv_segmentsDesc[0], line_extraction, min_pixels_line, max_lines); //, display);
    //featLines.extractLines(cv::cvarrToMat(obsRGBD[0]->intensityImage.getAs<IplImage>()), vv_segments2D[0], line_extraction, min_pixels_line, max_lines); //, display);
    featLines.extractLines(v_gray[0], vv_segments2D[0], vv_length[0], vv_seg_contrast[0], line_extraction, min_pixels_line, max_lines, true); //, display);
    //featLines.extractLines(v_gray[0], vv_segments2D[0], line_extraction, min_pixels_line, max_lines); //, display);
    ExtrinsicCalibLines::getProjPlaneNormals(intrinsics, vv_segments2D[0], vv_segment_n[0]);
    if(verbose)
        cout << "CDifodoDatasets_RGBD initialize. lines " << vv_segments2D[0].size() << endl;

    pcl::FastBilateralFilter<PointT> filter;
    filter.setSigmaS(10.0);
    filter.setSigmaR(0.5);
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    CTicTac clock1;
    clock1.Tic(); //Clock to measure the runtime
    filter.setInputCloud(v_cloud[0]);
    filter.filter(*cloud);
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud = mrpt::pbmap::PbMap::computeImgNormal(cloud);
    cout << "  normal_cloud took " << 1000*clock1.Tac() << " ms \n";
    clock1.Tic(); //Clock to measure the runtime
//    vv_pt_coor[0] = getDistributedNormals(v_depth[0], intrinsics, vv_pt_normal[0], vv_pt_robust[0], 4, 3);
    vv_pt_coor[0] = getDistributedNormals(normal_cloud, vv_pt_normal[0], vv_pt_robust[0], 5, 4);
    cout << "  getDistributedNormals took " << 1000*clock1.Tac() << " ms \n";


//    CFeatureExtraction featPoints;
//    CFeatureList points1, points2;
//    CMatchedFeatureList point_matches;
//    featPoints.options.featsType = featFAST;// featORB;
//    //IplImage ipl_img = v_rgb[0]; CImage im1(&ipl_img), im2;
//    featPoints.detectFeatures( obsRGBD[0]->intensityImage, points1 );
//    cout << "CDifodoDatasets_RGBD initialize. points " << points1.size() << endl;

    // Results
    Eigen::Matrix<T,3,3> isometry;
    Eigen::Matrix<T,4,4> approx_rot = Eigen::Matrix<T,4,4>::Identity(), approx_rot_tf = Eigen::Matrix<T,4,4>::Identity(); //rel_pose_diff_difodo
    vector<float> v_rot, v_trans;
    vector<float> v_rot_error, v_rot_error_difodo, v_trans_error_difodo;
    vector<float> v_time_lines, v_time_rot, v_time_difodo;
    vector<float> v_median_depth;
    size_t n_frames(0), n_success_rot(0), n_success_rot_difodo(0);
    float th_good_rot_deg = 3, th_good_trans = 0.03f;
    T rot_error, trans_error;

    while (!stop)
    {
//        pushed_key = window.waitForKey();
//        if (window.keyHit())
//            pushed_key = window.getPushedKey();
//        else
//            pushed_key = 0;

//        cout << "Pause until getchar (n,s,q): ";
//        pushed_key = getchar();

        pushed_key = 'n';
        switch (pushed_key) {

        //Capture 1 new frame and calculate odometry
        case  'n':
            setNewFrame();
            loadFrame();

            if (dataset_finished)
            {
                working = 0;
                stop = true;
                cout << endl << "End of dataset.";
                if (f_res.is_open())
                    f_res.close();
            }
            else
            {
                if(verbose)
                    cout << "CDifodoDatasets_RGBD. process frame \n";
                ++n_frames;
                v_median_depth.push_back( getMatMedian<float>(v_depth[0], 0.4, 10) );
                v_rot.push_back( RAD2DEG( getRotationVector<T,3>( gt_rel_pose_TUM.getRotationMatrix() ).norm() ) );
                v_trans.push_back( 1000*sqrt(gt_rel_pose_TUM.m_coords[0]*gt_rel_pose_TUM.m_coords[0] + gt_rel_pose_TUM.m_coords[1]*gt_rel_pose_TUM.m_coords[1] + gt_rel_pose_TUM.m_coords[2]*gt_rel_pose_TUM.m_coords[2]) );
                if(verbose)
                    cout << "groundtruth TUM : " << v_trans.back() << " mm " << v_rot.back() << " deg. rotation " << gt_rel_pose_TUM.yaw() << " " << gt_rel_pose_TUM.pitch() << " " << gt_rel_pose_TUM.roll() << endl;

//                featLines.extractLinesDesc(v_gray[0], vv_segments2D[0], vv_segmentsDesc[0], line_extraction, min_pixels_line, max_lines, true);
//                featLines.extractLines(cv::cvarrToMat(obsRGBD[0]->intensityImage.getAs<IplImage>()), vv_segments2D[0], line_extraction, min_pixels_line, max_lines); //, display);
//                featLines.extractLines(v_gray[0], vv_segments2D[0], line_extraction, min_pixels_line, max_lines); //, display);
                featLines.extractLines(v_gray[0], vv_segments2D[0], vv_length[0], vv_seg_contrast[0], line_extraction, min_pixels_line, max_lines);//, true); //, display);
                v_time_lines.push_back(featLines.time);
                T condition;
                Matrix<T,3,3> rot;
                ExtrinsicCalibLines::getProjPlaneNormals(intrinsics, vv_segments2D[0], vv_segment_n[0]);                
                CTicTac clock; clock.Tic(); //Clock to measure the runtime
                map<size_t,size_t> line_matches = mrpt::slam::MetricRegistration<T>::registerNormalVectors(vv_segment_n[1], vv_segment_n[0], rot, condition, max_angle_rad, DEG2RAD(10));
//                map<size_t,size_t> line_matches = MetricRegistration<T>::registerNormalVectorsContrast(vv_segment_n[1], vv_length[1], vv_seg_contrast[1],
//                                                                                                        vv_segment_n[0], vv_length[0], vv_seg_contrast[0],
//                                                                                                        rot, condition, max_angle_rad, DEG2RAD(10), 60);
                v_time_rot.push_back(1000*clock.Tac());
                cout << "match normals " << v_time_rot.back() << "ms\n";
                approx_rot.block<3,3>(0,0) = rot;
                rot_error = RAD2DEG( getRotationVector<T,3>( rot.transpose() * gt_rel_pose_TUM.getRotationMatrix() ).norm() );
                rel_pose_TUM.setRotationMatrix(rot);
                if(verbose)
                {
                    cout << "CDifodoDatasets_RGBD. lines " << vv_segments2D[0].size() << endl;
                    cout << "line_matches " << line_matches.size() << endl;
                    cout << "Approx rotation \n" << rot << endl;
                    cout << "ERROR " << rot_error << " Approx rotation : " << rel_pose_TUM.yaw() << " " << rel_pose_TUM.pitch() << " " << rel_pose_TUM.roll() << endl;
                }

                if( rot_error < th_good_rot_deg)
                {
                    v_rot_error.push_back(rot_error);
                    ++n_success_rot;
                }

                clock.Tic();
                map<size_t,size_t> line_matches2 = mrpt::slam::MetricRegistration<T>::registerNormalVectorsStop(vv_segment_n[1], vv_segment_n[0], rot, condition, max_angle_rad, DEG2RAD(10));
//                map<size_t,size_t> line_matches2 = MetricRegistration<T>::registerNormalVectorsContrastStop(vv_segment_n[1], vv_length[1], vv_seg_contrast[1],
//                                                                                                            vv_segment_n[0], vv_length[0], vv_seg_contrast[0],
//                                                                                                            rot, condition, max_angle_rad, DEG2RAD(10), 60);
                double time = 1000*clock.Tac();
                cout << "match normals contrast " << time << "ms\n";
                approx_rot.block<3,3>(0,0) = rot;
                rot_error = RAD2DEG( getRotationVector<T,3>( rot.transpose() * gt_rel_pose_TUM.getRotationMatrix() ).norm() );
                cout << "ERROR " << rot_error << endl;
                if( line_matches != line_matches2 )
                {
                    std::cout << "line_matches " << line_matches.size() << " \n";
                    for(map<size_t,size_t>::iterator it=line_matches.begin(); it != line_matches.end(); it++)
                        std::cout << " match " << it->first << " - " << it->second << endl;

                    std::cout << "line_matches2 " << line_matches2.size() << " \n";
                    for(map<size_t,size_t>::iterator it=line_matches2.begin(); it != line_matches2.end(); it++)
                        std::cout << " match " << it->first << " - " << it->second << endl;
                }


                // Match surface patches from the depth images
                filter.setInputCloud(v_cloud[0]);
                filter.filter(*cloud);
                pcl::PointCloud<pcl::Normal>::Ptr normal_cloud = mrpt::pbmap::PbMap::computeImgNormal(cloud);
                vv_pt_coor[0] = getDistributedNormals(normal_cloud, vv_pt_normal[0], vv_pt_robust[0], 5, 4);

                float condition_pts;
                Matrix<float,3,3> rot_pts(Matrix<float,3,3>::Identity());
                map<size_t,size_t> point_depth_matches = mrpt::slam::MetricRegistration<float>::registerNormalVectors(vv_pt_normal[1], vv_pt_normal[0], rot_pts, condition_pts, max_angle_rad, DEG2RAD(10));
                T rot_pts_error = RAD2DEG( getRotationVector<T,3>( rot_pts.transpose().cast<T>() * gt_rel_pose_TUM.getRotationMatrix() ).norm() );
                cout << "Point matches " << point_depth_matches.size() << " condition_pts " << condition_pts << " rot_pts_error " << rot_pts_error << endl;


//                points2 = points1;
//                featPoints.detectFeatures( obsRGBD[0]->intensityImage, points1 );//
//                matchFeatures( points1, points2, point_matches );
//                cout << "point_matches " << point_matches.size() << endl;
//                if( display )
//                {
//                    CDisplayWindow feature_matches;
//                    feature_matches.setWindowTitle("feature_matches");
//                    feature_matches.showImagesAndMatchedPoints( obsRGBD[0]->intensityImage, obsRGBD[1]->intensityImage, point_matches, TColor(0,0,255) );
//                    mrpt::system::pause();
//                }

//                std::vector<Point2f> obj;
//                std::vector<Point2f> scene;
//                  Mat H = findHomography( obj, scene, CV_RANSAC );

//                cout << "odometryCalculation \n";
                if(b_initial_rot)
                {
                    //                approx_rot_tf = (-transf).getHomogeneousMatrixVal() * approx_rot * (transf).getHomogeneousMatrixVal();
                    //                approx_rot_tf = gt_rel_pose.getHomogeneousMatrixVal();
                    approx_rot_tf.block<3,3>(0,0) = gt_rel_pose.getRotationMatrix();
                    odometryCalculation( approx_rot_tf.cast<float>() );
                }
                else
                    odometryCalculation();
                v_time_difodo.push_back(execution_time);
                rot_error = RAD2DEG( getRotationVector<T,3>( (rel_pose - gt_rel_pose).getRotationMatrix() ).norm() );
                trans_error = (rel_pose - gt_rel_pose).getHomogeneousMatrixVal().block<3,1>(0,3).norm();
                if( rot_error < th_good_rot_deg && trans_error < th_good_trans )
                {
                    v_rot_error_difodo.push_back(rot_error);
                    v_trans_error_difodo.push_back(trans_error);
                    ++n_success_rot_difodo;
                }
                rel_pose_TUM = transf + rel_pose + (-transf);
                if(verbose)
                {
                    cout << "\nERROR DifOdo " << rot_error << " trans error " << 1000*trans_error << endl;
                    cout << "Difodo runtime(ms): " << execution_time << " rot Euler angles : " << rel_pose_TUM.yaw() << " " << rel_pose_TUM.pitch() << " " << rel_pose_TUM.roll() << endl;
                    //                cout << "groundtruth \n" << gt_rel_pose.getHomogeneousMatrixVal() << endl;
                    //                cout << "DifOdo pose \n" << rel_pose.getHomogeneousMatrixVal() << endl;
                }

                if (save_results == 1)
                    writeTrajectoryFile();

                if( display )
                {
                    cv::Mat diff;
                    cv::Mat gray_warped1, diff1;
                    cv::Mat gray_warped2, diff2;
                    cv::Mat gray_warped3, diff3;
                    cv::Mat gray_warped4, diff4;

                    //cv::cvtColor( cv::cvarrToMat(obsRGBD[0]->intensityImage.getAs<IplImage>()), gray, cv::COLOR_BGR2GRAY );
                    warp(v_gray[0], v_depth[0], intrinsics, getPoseEigen<float>(gt_rel_pose_TUM), gray_warped1);
                    warp(v_gray[0], v_depth[0], intrinsics, approx_rot.cast<float>(), gray_warped2);
                    warp(v_gray[0], v_depth[0], intrinsics, getPoseEigen<float>(transf+rel_pose+(-transf)), gray_warped3);
                    mrpt::slam::MetricRegistration<T>::rotation2homography( rot, intrinsics.intrinsicParams.cast<T>(), isometry );
                    //cout << "Isometry\n" << isometry << endl;
                    cv::Mat cvH;
                    cv::Mat imgMask = cv::Mat(v_gray[0].size(), CV_8UC1, cv::Scalar(1));
                    cv::Mat imgMaskWarped;
                    Eigen::Matrix<T,2,3> isometry_2x3 = isometry.block<2,3>(0,0);
                    cv::eigen2cv(isometry_2x3, cvH);
                    //cv::cuda::warpAffine(imgMask , imgMaskWarped, cvH, imgMaskWarped.size());
                    cv::warpAffine(imgMask , imgMaskWarped, cvH, imgMaskWarped.size());
                    cv::warpAffine(v_gray[0], gray_warped4, cvH, v_gray[0].size());
                    gray_warped4.copyTo(gray_warped4, imgMaskWarped);
                    cv::Scalar sum_abs_diff = cv::sum(gray_warped4); // cv::cuda::sum(gray_warped4, imgMaskWarped);
                    cv::Scalar sum_warped = cv::sum(imgMaskWarped);
                    T MAD = sum_abs_diff[0] / T(sum_warped[0]);
                    cout << "MAD " << MAD << endl;
                    //cv::cuda::absdiff(v_gray[0], v_gray[1], diff);
                    cv::absdiff(v_gray[0], v_gray[1], diff);
                    cv::absdiff(v_gray[1], gray_warped1, diff1);
                    cv::absdiff(v_gray[1], gray_warped2, diff2);
                    cv::absdiff(v_gray[1], gray_warped3, diff3);
                    cv::absdiff(v_gray[1], gray_warped4, diff4);

                    cv::Mat image_lines_3[2];
                    cv::Mat image_lines_2[2];
                    cv::Mat image_lines[2];
                    cv::Mat image_pts[2];
                    for(size_t i=0; i < 2; i++)
                    {
                        cv::cvtColor( v_gray[i], image_lines[i], cv::COLOR_GRAY2BGR );
                        cv::cvtColor( v_gray[i], image_lines_2[i], cv::COLOR_GRAY2BGR );
                        cv::cvtColor( v_gray[i], image_lines_3[i], cv::COLOR_GRAY2BGR );
//                        v_rgb[i].copyTo(image_lines_2[i]);
//                        image_lines_2[i] = v_gray[i].clone();
//                        image_lines[i] = v_gray[i].clone();
//                        if( image_lines[i].channels() == 1 )
//                            cv::cvtColor( image_lines[i], image_lines[i], cv::COLOR_GRAY2BGR );
//                            cv::cvtColor( image_lines_2[i], image_lines_2[i], cv::COLOR_GRAY2BGR );
                        for(auto it = begin(line_matches); it != end(line_matches); ++it)
                        {
                            size_t m = (i == 0) ? it->second : it->first;
                            cv::Scalar color(blu[distance(begin(line_matches),it)%10],grn[distance(begin(line_matches),it)%10],red[distance(begin(line_matches),it)%10]);
                            cv::line(image_lines[i], cv::Point(cv::Point2f(vv_segments2D[i][m][0], vv_segments2D[i][m][1])), cv::Point2f(cv::Point(vv_segments2D[i][m][2], vv_segments2D[i][m][3])), color, 2 );
                            //cv::circle(image_lines[i], cv::Point(cv::Point2f(vv_segments2D[i][m][0], vv_segments2D[i][m][1])), 3, color, 3);
                            cv::putText(image_lines[i], to_string(m), cv::Point2f(vv_segments2D[i][m][0], vv_segments2D[i][m][1]), 0, 1.0, color, 1 );
                        }

                        for(auto it = begin(line_matches2); it != end(line_matches2); ++it)
                        {
                            //cout << "match " << it->second << " / " << vv_segments2D[it->second].size() << " " << it->first << " / " << vv_segments2D[it->first].size() << "\n";
                            size_t m = (i == 0) ? it->second : it->first;
                            cv::Scalar color(blu[distance(begin(line_matches2),it)%10],grn[distance(begin(line_matches2),it)%10],red[distance(begin(line_matches2),it)%10]);
                            cv::line(image_lines_2[i], cv::Point(cv::Point2f(vv_segments2D[i][m][0], vv_segments2D[i][m][1])), cv::Point2f(cv::Point(vv_segments2D[i][m][2], vv_segments2D[i][m][3])), color, 2 );

                            Eigen::Vector2f seg(vv_segments2D[i][m][2]-vv_segments2D[i][m][0], vv_segments2D[i][m][3]-vv_segments2D[i][m][1]);
                            int l = seg.norm();
                            seg.normalize();
                            Eigen::Vector2f n(vv_segments2D[i][m][3]-vv_segments2D[i][m][1], vv_segments2D[i][m][0]-vv_segments2D[i][m][2]);
                            n.normalize();
                            int d = 10 + rand() % 2;
                            int rand_line = 0.5*(rand() % l);
                            cv::Point pt_line(vv_segments2D[i][m][0]+rand_line*seg[0], vv_segments2D[i][m][1]+rand_line*seg[1]);
                            cv::Point n1(d*n[0], d*n[1]);
                            cv::Point pt_txt = pt_line + n1;
                            cv::line( image_lines_2[i], pt_line, pt_txt, color, 1 );
                            cv::putText(image_lines_2[i], to_string(int(vv_seg_contrast[i][m])), pt_txt, 0, 1.0, color, 1 );

//                            cv::circle(image_lines_2[i], cv::Point(cv::Point2f(vv_segments2D[i][m][0], vv_segments2D[i][m][1])), 3, color, 3);
                            // cv::putText(image_lines_2[i], to_string(distance(begin(segments),line)), cv::Point2f(vv_segments2D[i][m][0], vv_segments2D[i][m][1])), 0, 1.2, color, 3 );
                        }

                        for(auto line = begin(vv_segments2D[i]); line != end(vv_segments2D[i]); ++line)
                        {
                            size_t j = distance(begin(vv_segments2D[i]),line);
                            cv::Scalar color(blu[j%10],grn[j%10],red[j%10]);
                            cv::line(image_lines_3[i], cv::Point(cv::Point2f((*line)[0], (*line)[1])), cv::Point2f(cv::Point((*line)[2], (*line)[3])), color, 3 );
                            cv::circle(image_lines_3[i], cv::Point(cv::Point2f((*line)[0], (*line)[1])), 3, color, 3);
                            cv::putText(image_lines_3[i], to_string(int(vv_seg_contrast[i][j])), cv::Point(cv::Point2f((*line)[0], (*line)[1])), 0, 1.0, color, 1 );
                            cv::putText(image_lines_3[i], to_string(j), cv::Point(cv::Point2f((*line)[2], (*line)[3])), 0, 1.0, color, 1 );
                        }

                        //image_pts[i] = v_rgb[i].clone();
                        cv::Mat mask = cv::Mat::zeros(v_rgb[i].size(), CV_8UC1);
                        mask.setTo(255, v_depth[i] > 0.3f);
                        v_rgb[i].copyTo(image_pts[i], mask);
                        for(size_t ii=0; ii < vv_pt_coor[i].size(); ++ii)
                        {
                            //cout << "it: " << vv_pt_coor[i][ii][0] << " " << vv_pt_coor[i][ii][1] << " \n";
                            if( vv_pt_robust[i][ii] )
                                cv::circle(image_pts[i], cv::Point(vv_pt_coor[i][ii][0], vv_pt_coor[i][ii][1]), 3, cv::Scalar(0,255,0), 3); // Green
                            else
                                cv::circle(image_pts[i], cv::Point(vv_pt_coor[i][ii][0], vv_pt_coor[i][ii][1]), 3, cv::Scalar(0,0,255), 3); // Red
                        }
                    }

                    cv::imshow("lines0", image_lines[0]);        cv::moveWindow("lines0", 100, 50);
                    cv::imshow("lines1", image_lines[1]);        cv::moveWindow("lines1", 760, 50);
//                    cv::imshow("gray0", v_gray[0]);             cv::moveWindow("gray0", 100, 50);
//                    cv::imshow("gray1", v_gray[1]);             cv::moveWindow("gray1", 760, 50);
                    cv::imshow("diff", diff);                   cv::moveWindow("diff", 1400, 50);
                    cv::imshow("gray_warped1", gray_warped1);   cv::moveWindow("gray_warped1", 100, 570);
                    cv::imshow("gray_warped2", gray_warped2);   cv::moveWindow("gray_warped2", 760, 570);
                    cv::imshow("gray_warped3", gray_warped3);   cv::moveWindow("gray_warped3", 1400, 570);
                    cv::imshow("gray_warped4", gray_warped4);   cv::moveWindow("gray_warped4", 2040, 570);
                    cv::imshow("diff1", diff1);                 cv::moveWindow("diff1", 100, 1100);
                    cv::imshow("diff2", diff2);                 cv::moveWindow("diff2", 760, 1100);
                    cv::imshow("diff3", diff3);                 cv::moveWindow("diff3", 1400, 1100);
                    cv::imshow("diff4", diff4);                 cv::moveWindow("diff4", 2040, 1100);
                    cv::imshow("points0", image_pts[0]);        cv::moveWindow("points0", 100, 1630);
                    cv::imshow("points1", image_pts[1]);        cv::moveWindow("points1", 760, 1630);
                    cv::imshow("lines_0", image_lines_2[0]);     cv::moveWindow("lines_0", 100, 570);
                    cv::imshow("lines_1", image_lines_2[1]);     cv::moveWindow("lines_1", 760, 570);
                    cv::imshow("lines_x0", image_lines_3[0]);     cv::moveWindow("lines_x0", 100, 1630);
                    cv::imshow("lines_x1", image_lines_3[1]);     cv::moveWindow("lines_x1", 760, 1630);

//                    cv::imwrite("gray0.png", v_gray[0]);
//                    cv::imwrite("gray1.png", v_gray[0]);
//                    cv::imwrite("lines0.png", image_lines[0]);
//                    cv::imwrite("lines1.png", image_lines[1]);
//                    cv::imwrite("warp.png", gray_warped1);
//                    cv::imwrite("diff_gt.png", diff1);
//                    cv::imwrite("diff_approx_rot.png", diff2);
//                    cv::imwrite("diff_identity.png", diff);

                    cv::waitKey(0);
//                    updateScene();
                }
            }

            break;

        //Start and stop continuous odometry
        case 's':
            working = !working;
            break;

        //Close the program
        case 'e':
            stop = 1;
            if (f_res.is_open())
                f_res.close();
            break;

        }

        if (working == 1)
        {
            if (dataset_finished)
            {
                working = 0;
                cout << endl << "End of dataset.";
                if (f_res.is_open())
                    f_res.close();
            }
            else
            {
                setNewFrame();
                loadFrame();
                odometryCalculation();
                if (save_results == 1)
                    writeTrajectoryFile();

                cout << endl << "Difodo runtime(ms): " << execution_time;
//                updateScene();
            }
        }
    }

    // Show results
    cout << "Sequence information: \n" << rawlog_file << "\ndecimation " << decimation << " b_initial_rot " << b_initial_rot << " av median depth " << mean(v_median_depth)
         << " rot " << mean(v_rot) << " " << stdev(v_rot) << " " << median(v_rot) << " trans " << mean(v_trans) << " " << stdev(v_trans) << " " << median(v_trans) << endl;
    cout << " Approx ROT error " << mean(v_rot_error) << " " << stdev(v_rot_error) << " " << median(v_rot_error) << " success rate " << n_success_rot / float(n_frames)
         << " time lines " << mean(v_time_lines) << " " << median(v_time_lines) << " time rot " << mean(v_time_rot) << " " << median(v_time_rot) << endl;
    cout << " DifOdo ROT error " << mean(v_rot_error_difodo) << " " << stdev(v_rot_error_difodo) << " " << median(v_rot_error_difodo) << " success rate " << n_success_rot_difodo / float(n_frames)
         << " time " << mean(v_time_difodo) << " " << median(v_time_difodo) << endl;

    size_t pos = rawlog_file.find_last_of('/')+14;
    string name_seq = rawlog_file.substr(pos,rawlog_file.length()-7-pos);
    std::ofstream file("results.txt", std::ios_base::app);
    file << "=============================================================================\n"
         << name_seq << "\tdecimation " << decimation << " b_init " << b_initial_rot << " av_median_depth " << mean(v_median_depth)
         << " rot " << mean(v_rot) << " " << stdev(v_rot) << " " << median(v_rot) << " trans " << mean(v_trans) << " " << stdev(v_trans) << " " << median(v_trans)
         << " ROT_error " << mean(v_rot_error) << " " << stdev(v_rot_error) << " " << median(v_rot_error) << " success " << n_success_rot / float(n_frames)
//         << " time lines " << mean(v_time_lines) << " " << median(v_time_lines) << " time rot " << mean(v_time_rot) << " " << median(v_time_rot)
         << " DifOdo_ROT_error " << mean(v_rot_error_difodo) << " " << stdev(v_rot_error_difodo) << " " << median(v_rot_error_difodo)
         << " DifOdo_TRANS_error " << mean(v_trans_error_difodo) << " " << stdev(v_trans_error_difodo) << " " << median(v_trans_error_difodo)
         << " DifOdo_success " << n_success_rot_difodo / float(n_frames) << " time " << mean(v_time_difodo) << " " << median(v_time_difodo) << endl << endl;

}

void CDifodoDatasets_RGBD::reset()
{
	loadFrame();
	if (fast_pyramid)	buildCoordinatesPyramidFast();
	else				buildCoordinatesPyramid();

	cam_oldpose = cam_pose;
	gt_oldpose = gt_pose;
}

void CDifodoDatasets_RGBD::writeTrajectoryFile()
{	
	//Don't take into account those iterations with consecutive equal depth images
	if (abs(dt.sumAll()) > 0)
	{		
		mrpt::math::CQuaternionDouble quat;
		CPose3D auxpose, transf;
		transf.setFromValues(0,0,0,0.5*M_PI, -0.5*M_PI, 0);

		auxpose = cam_pose - transf;
		auxpose.getAsQuaternion(quat);
	
		char aux[24];
		sprintf(aux,"%.04f", timestamp_obs);
		f_res << aux << " ";
		f_res << cam_pose[0] << " ";
		f_res << cam_pose[1] << " ";
		f_res << cam_pose[2] << " ";
		f_res << quat(2) << " ";
		f_res << quat(3) << " ";
		f_res << -quat(1) << " ";
		f_res << -quat(0) << endl;
	}
}


