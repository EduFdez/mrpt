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

#include "kinect-rig-calib.h"
#include "kinect-rig-calib_misc.h"
//#include "kinect-rig-calib_display.h"
#include <mrpt/pbmap/PbMap.h>
#include <mrpt/pbmap/colors.h>
#include <mrpt/pbmap/DisplayCloudPbMap.h>
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
using namespace mrpt::pbmap;
using namespace std;
using namespace Eigen;

void KinectRigCalib::setNumSensors(const size_t n_sensors)
{
    cout << "KinectRigCalib::setNumSensors... " << n_sensors << " sensors" << endl;
    num_sensors = n_sensors;
    Rt_estimated = vector<Eigen::Matrix<T,4,4> , Eigen::aligned_allocator<Eigen::Matrix<T,4,4> > >(num_sensors, Eigen::Matrix<T,4,4>::Identity());
    intrinsics = vector<mrpt::utils::TStereoCamera>(num_sensors);
    init_poses = vector<CPose3D>(num_sensors);
    v_approx_trans = vector<double>(num_sensors);
    v_approx_rot = vector<double>(num_sensors);
    rgb.resize(num_sensors);
    depth.resize(num_sensors);
    depth_reg.resize(num_sensors); // Depth image registered to RGB pose
    cloud.resize(num_sensors);
    v_pbmap.resize(num_sensors);
    if(s_type == PLANES || s_type == PLANES_AND_LINES)
    {
        planes = FeatCorresp(num_sensors, 8);
    }
    if(s_type == LINES || s_type == PLANES_AND_LINES)
    {
        lines = FeatCorresp(num_sensors, 18);
        vv_segments2D.resize(num_sensors);
        vv_segment_n.resize(num_sensors);
        vv_segments3D.resize(num_sensors);
        vv_line_has3D.resize(num_sensors);
        vv_lines3D.resize(num_sensors);
    }
}

void KinectRigCalib::displayObservation()
{
    cout << "KinectRigCalib::displayObservation...\n";

    // Note that the RGB-D camera is in a vertical configuration in the rig RGBD360
    cv::Mat img_transposed, img_rotated;
    cv::transpose(rgb[0], img_transposed);
    cv::flip(img_transposed, img_rotated, 0);
    cv::Mat rgb_concat(img_transposed.rows, 2*img_transposed.cols+20, CV_8UC3, cv::Scalar(155,100,255));
    cv::Mat tmp = rgb_concat(cv::Rect(0, 0, img_transposed.cols, img_transposed.rows));
    img_rotated.copyTo(tmp);
    cv::transpose(rgb[1], img_transposed);
    cv::flip(img_transposed, img_rotated, 0);
    tmp = rgb_concat(cv::Rect(img_transposed.cols+20, 0, img_transposed.cols, img_transposed.rows));
    img_rotated.copyTo(tmp);
    //cv::circle(rgb_concat, cv::Point(240,320),5,cv::Scalar(0, 0, 200));
    //cout << "Pixel values " << rgb_concat.at<cv::Vec3b>(5,5) << " " << rgb_concat.at<cv::Vec3b>(5,475) << " " << rgb_concat.at<cv::Vec3b>(5,485) << " " << endl;
    cv::imshow("rgb", rgb_concat ); cv::moveWindow("rgb", 20,20);

//            // Show depth
//            cv::transpose(depth[0], img_transposed);
//            cv::flip(img_transposed, img_rotated, 0);
//            cv::Mat depth_concat(img_transposed.rows, 2*img_transposed.cols+20, CV_32FC1, cv::Scalar(0.f));
//            tmp = depth_concat(cv::Rect(0, 0, img_transposed.cols, img_transposed.rows));
//            img_rotated.copyTo(tmp);
//            cv::transpose(depth[1], img_transposed);
//            cv::flip(img_transposed, img_rotated, 0);
//            tmp = depth_concat(cv::Rect(img_transposed.cols+20, 0, img_transposed.cols, img_transposed.rows));
//            img_rotated.copyTo(tmp);
//            cv::Mat img_depth_display;
//            depth_concat.convertTo(img_depth_display, CV_32FC1, 0.3 );
//            cv::Mat depth_display = cv::Mat(depth_concat.rows, depth_concat.cols, depth_concat.type(), cv::Scalar(1.f)) - img_depth_display;
//            depth_display.setTo(cv::Scalar(0.f), depth_concat < 0.1f);
//            cv::imshow("depth", depth_display ); cv::moveWindow("depth", 20,100+640);
//            //cout << "Some depth values: " << depth_concat.at<float>(200, 320) << " " << depth_concat.at<float>(50, 320) << " " << depth[0].at<float>(200, 320) << " " << depth[0].at<float>(50, 320) << " " << depth[1].at<float>(430, 320) << " " << depth[1].at<float>(280, 320) << "\n";

    // Show registered depth
    cv::transpose(depth_reg[0], img_transposed);
    cv::flip(img_transposed, img_rotated, 0);
    cv::Mat depth_reg_concat(img_transposed.rows, 2*img_transposed.cols+20, CV_32FC1, cv::Scalar(0.f));
    tmp = depth_reg_concat(cv::Rect(0, 0, img_transposed.cols, img_transposed.rows));
    img_rotated.copyTo(tmp);
    cv::transpose(depth_reg[1], img_transposed);
    cv::flip(img_transposed, img_rotated, 0);
    tmp = depth_reg_concat(cv::Rect(img_transposed.cols+20, 0, img_transposed.cols, img_transposed.rows));
    img_rotated.copyTo(tmp);
    cv::Mat img_depth_display;
    depth_reg_concat.convertTo(img_depth_display, CV_32FC1, 0.3 );
    cv::Mat depth_reg_display = cv::Mat(depth_reg_concat.rows, depth_reg_concat.cols, depth_reg_concat.type(), cv::Scalar(1.f)) - img_depth_display;
    depth_reg_display.setTo(cv::Scalar(0.f), depth_reg_concat < 0.1f);
    cv::imshow("depth_reg", depth_reg_display ); cv::moveWindow("depth_reg", 20,100+640);

    cv::waitKey(0);
}

void KinectRigCalib::loadConfiguration(const string & config_file)
{
    //Initial steps. Load configuration file
    //-------------------------------------------
    utils::CConfigFile cfg(config_file);
    rawlog_file = cfg.read_string("GLOBAL", "rawlog_file", "no file", true);
//    sensor_labels = cfg.read_string("GLOBAL", "sensor_labels", "", true);
    output_dir = cfg.read_string("GLOBAL", "output_dir", "no dir", true);
    decimation = cfg.read_int("GLOBAL", "decimation", 0, true);
    display = cfg.read_bool("GLOBAL", "display", 0, true);
    verbose = cfg.read_bool("GLOBAL", "verbose", 0, true);
    min_pixels_line = cfg.read_int("GLOBAL", "min_pixels_line", 100, true);
    th_dist_plane = cfg.read_float("GLOBAL", "th_dist_plane", 0.02, true);
    th_angle_plane = cfg.read_float("GLOBAL", "th_angle_plane", 0.04, true);

    if(verbose)
        cout << "loadConfiguration -> dataset: " << rawlog_file << "\toutput: " << output_dir << "\tdecimation: " << decimation << endl;

    // Get sensor labels
    cfg.getAllSections(sensor_labels);
    sensor_labels.erase(sensor_labels.begin()); // Remove the GLOBAL section
    vector<string>::iterator it_sensor_label = sensor_labels.begin(), it_sensor_label2 = sensor_labels.begin();
    ++it_sensor_label2;
    while(it_sensor_label2 != sensor_labels.end())
    {
        if(it_sensor_label2->find(*it_sensor_label) == 0)
            sensor_labels.erase(it_sensor_label2);
        else
        {
            ++it_sensor_label;
            ++it_sensor_label2;
        }
    }
    setNumSensors(sensor_labels.size());
    for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++) // Load the approximated init_poses of each sensor and the accuracy of such approximation
    {
        if(verbose)
            cout << "sensor: " << sensor_labels[sensor_id] << endl;

        //cfg.read_matrix(sensor_labels[sensor_id],"pose",init_poses[sensor_id]); // Matlab's matrix format
        init_poses[sensor_id] = CPose3D(
                                cfg.read_double(sensor_labels[sensor_id],"pose_x",0,true),
                                cfg.read_double(sensor_labels[sensor_id],"pose_y",0,true),
                                cfg.read_double(sensor_labels[sensor_id],"pose_z",0,true),
                                DEG2RAD( cfg.read_double(sensor_labels[sensor_id],"pose_yaw",0,true) ),
                                DEG2RAD( cfg.read_double(sensor_labels[sensor_id],"pose_pitch",0,true) ),
                                DEG2RAD( cfg.read_double(sensor_labels[sensor_id],"pose_roll",0,true) ) );
        Rt_estimated[sensor_id] = init_poses[sensor_id].getHomogeneousMatrixVal();
        v_approx_trans[sensor_id] = cfg.read_double(sensor_labels[sensor_id],"approx_translation",0.f,true);
        v_approx_rot[sensor_id] = cfg.read_double(sensor_labels[sensor_id],"approx_rotation",0.f,true);

        //mm_conditioning[sensor_id] = map<size_t, double>();
        mm_covariance[sensor_id] = map<size_t, Eigen::Matrix<T,3,3> >();
        for(size_t sensor2=sensor_id+1; sensor2 < num_sensors; sensor2++)
        {
            mm_conditioning[sensor_id][sensor2] = 0.0;
            mm_covariance[sensor_id][sensor2] = Eigen::Matrix<T,3,3>::Zero();
        }

        cout << sensor_labels[sensor_id] << " v_approx_trans " << v_approx_trans[sensor_id] << " v_approx_rot " << v_approx_rot[sensor_id] << "\n" << init_poses[sensor_id] << endl;
        cout << Rt_estimated[sensor_id] << endl;

        intrinsics[sensor_id].loadFromConfigFile(sensor_labels[sensor_id],cfg);
//        string calib_path = mrpt::system::extractFileDirectory(rawlog_file) + "asus_" + to_string(sensor_id+1) + "/ini";
//        mrpt::utils::CConfigFile calib_RGBD(calib_path);
//        intrinsics[sensor_id].loadFromConfigFile("CAMERA_PARAMS", calib_RGBD);
//        cout << "right2left_camera_pose \n" << intrinsics[sensor_id].rightCameraPose << endl;

//        cout << sensor_labels[sensor_id] << "RGB params: fx=" << intrinsics[sensor_id].rightCamera.fx() << " fy=" << intrinsics[sensor_id].rightCamera.fy() << " cx=" << intrinsics[sensor_id].rightCamera.cx() << " cy=" << intrinsics[sensor_id].rightCamera.cy() << endl;
//        cout << sensor_labels[sensor_id] << "RGB params: fx=" << intrinsics2[sensor_id].cam_params.rightCamera.fx() << " fy=" << intrinsics2[sensor_id].cam_params.rightCamera.fy() << " cx=" << intrinsics2[sensor_id].cam_params.rightCamera.cx() << " cy=" << intrinsics2[sensor_id].cam_params.rightCamera.cy() << endl;
    }

    cout << "...loadConfiguration\n";
}

void KinectRigCalib::getCorrespondences()
{
    if(s_type == PLANES_AND_LINES || s_type == PLANES)
        ExtrinsicCalibPlanes::getCorrespondences(cloud);
//    calib_planes.getCorrespondences(cloud);
    if(s_type == PLANES_AND_LINES || s_type == LINES)
        ExtrinsicCalibLines::getCorrespondences(rgb, cloud);
//    getCorrespondences();
}

void KinectRigCalib::calibrate(const bool save_corresp)
{
    if(s_type == PLANES_AND_LINES || s_type == PLANES)
    {
        ExtrinsicCalibPlanes::Calibrate();

        if( save_corresp )
        {
            cout << "\tSave Plane Correspondences\n";
            planes.saveCorrespondences( output_dir.c_str(), "planes" );
            //    conditioningFIM.saveToTextFile( mrpt::format("%s/conditioningFIM.txt", output_dir.c_str()) );
        }
    }
    if(s_type == PLANES_AND_LINES || s_type == LINES)
    {
        ExtrinsicCalibLines::Calibrate();

        if( save_corresp )
        {
            cout << "\tSave Line Correspondences\n";
            lines.saveCorrespondences( output_dir.c_str(), "planes" );
        }
    }
}

void KinectRigCalib::run()
{
    //						Open Rawlog File
    //==================================================================
    mrpt::obs::CRawlog dataset;
    if (!dataset.loadFromRawLogFile(rawlog_file))
        throw runtime_error("\nCouldn't open rawlog dataset file for input...");

//        // Set external images directory:
//        const string imgsPath = CRawlog::detectImagesDirectory(filename);
//        CImage::IMAGES_PATH_BASE = imgsPath;

    cout << "dataset size " << dataset.size() << "\n";

    // Observation parameters
    vector<mrpt::obs::CObservation3DRangeScanPtr> obsRGBD(num_sensors);  // The RGBD observation
    vector<bool> obs_sensor(num_sensors,false);
    vector<double> obs_sensor_time(num_sensors);
    vector<mrpt::vision::CUndistortMap> undist_rgb(num_sensors);
    vector<mrpt::vision::CUndistortMap> undist_depth(num_sensors);
    for(size_t i=0; i < num_sensors; i++)
    {
        undist_rgb[i].setFromCamParams(intrinsics[i].rightCamera);
        undist_depth[i].setFromCamParams(intrinsics[i].leftCamera);
    }

    // Calibration parameters
    float angle_offset = 20;
//        float angle_offset = 180;
    initOffset = Eigen::Matrix<T,4,4>::Identity();
    initOffset(1,1) = initOffset(2,2) = cos(angle_offset*3.14159/180);
    initOffset(1,2) = -sin(angle_offset*3.14159/180);
    initOffset(2,1) = -initOffset(1,2);
    cout << "initOffset\n" << initOffset << endl;

    // Get the plane and line correspondences
//    calib_planes = ExtrinsicCalibPlanes<T>(num_sensors);
//    calib_lines = ExtrinsicCalibLines<T>(num_sensors);

//    CMatrixDouble conditioningFIM(0,6);
//    Eigen::Matrix<T,3,3> FIMrot = Eigen::Matrix<T,3,3>::Zero();
//    Eigen::Matrix<T,3,3> FIMtrans = Eigen::Matrix<T,3,3>::Zero();

//    KinectRigCalib calib;
//    KinectRigCalib_display v3D(calib);

    //==============================================================================
    //									Main operation
    //==============================================================================
    size_t n_obs = 0, n_frame_sync = 0;
    CObservationPtr observation;
    while( !b_exit && n_obs < dataset.size() )
    {
        observation = dataset.getAsObservation(n_obs);
        ++n_obs;
        if(verbose)
            cout << n_obs << " observation: " << observation->sensorLabel << endl; //<< ". Timestamp " << timestampTotime_t(observation->timestamp) << endl;

        if(!IS_CLASS(observation, CObservation3DRangeScan))
            continue;

        size_t sensor_id = 10e6; // Initialize with a non-valid sensor index
        for(size_t i=0; i < num_sensors; i++) // Identify the sensor which captured the current obs
            if(observation->sensorLabel == sensor_labels[i])
            {
                sensor_id = i;
                break;
            }
        if(sensor_id == 10e6) // This RGBD sensor is not taken into account for calibration
            continue;

        obs_sensor[sensor_id] = true;
        obs_sensor_time[sensor_id] = timestampTotime_t(observation->timestamp);
        //cout << sensor_id << " diff " << timestampTotime_t(observation->timestamp)-obs_sensor_time[sensor_id] << endl;
        obsRGBD[sensor_id] = CObservation3DRangeScanPtr(observation);
        obsRGBD[sensor_id]->load();


        // Get synchronized observations (aproximate synchronization)
        if( !accumulate(begin(obs_sensor), end(obs_sensor), true, logical_and<bool>()) )
            continue;
        vector<double>::iterator max_time = max_element(obs_sensor_time.begin(), obs_sensor_time.end());
        vector<double>::iterator min_time = min_element(obs_sensor_time.begin(), obs_sensor_time.end());
        if(verbose)
        {
            cout << "max diff " << (*max_time - *min_time)*1e3 << endl;
//                for(size_t i=1; i < num_sensors; i++)
//                    cout << i << " time diff to ref (sensor_id=0) " << (obs_sensor_time[i]-obs_sensor_time[0])*1e3 << " ms." << endl;
        }
        if( (*max_time - *min_time) > max_diff_sync ) // maximum de-synchronization in seconds
            continue;
        ++n_frame_sync;
        cout << n_frame_sync << " frames sync\n";
        fill(obs_sensor.begin(), obs_sensor.end(), false);

        // Apply decimation
        if( n_frame_sync % decimation != 0)
            continue;
        cout << n_obs << " use this sync frames \n";


        // Get the rgb and depth images, and the point clouds
        for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++)
        {
            boost::mutex::scoped_lock updateLock(visualizationMutex);
//            rgb[sensor_id] = cv::Mat(obsRGBD[sensor_id]->intensityImage.getAs<IplImage>());
//            convertRange_mrpt2cvMat(obsRGBD[sensor_id]->rangeImage, depth[sensor_id]);
            // Image rectification (to reduce radial distortion)
            cv::Mat src(obsRGBD[sensor_id]->intensityImage.getAs<IplImage>());
            undist_rgb[sensor_id].undistort(src, rgb[sensor_id]);
            cv::Mat raw_depth;
            convertRange_mrpt2cvMat(obsRGBD[sensor_id]->rangeImage, raw_depth);
            undist_depth[sensor_id].undistort(raw_depth, depth[sensor_id]);
            cloud[sensor_id] = getPointCloudRegistered<PointT>(rgb[sensor_id], depth[sensor_id], intrinsics[sensor_id], depth_reg[sensor_id]);
//            pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//            viewer.showCloud(cloud[sensor_id]);
//            while (!viewer.wasStopped ())
//                boost::this_thread::sleep (boost::posix_time::milliseconds (10));
            updateLock.unlock();
        }
        b_freeze = false; // Update 3D visualization

        // Display color and depth images
        //displayObservation();

        //						Segment local planes
        //==================================================================
        // #pragma omp parallel num_threads(num_sensors)
        for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++)
        {
            //boost::mutex::scoped_lock updateLock(visualizationMutex);
            PbMap::pbMapFromPCloud(cloud[sensor_id], v_pbmap[sensor_id], th_dist_plane, th_angle_plane, min_inliers);
            //DisplayCloudPbMap::displayAndPause(cloud[sensor_id], v_pbmap[sensor_id]); // This 3D visualization is not compatible with KinectRigCalib's one
            //PbMap::displayImagePbMap(cloud[sensor_id], rgb[sensor_id], v_pbmap[sensor_id]);
            //updateLock.unlock();
        }

        //==============================================================================
        //								Get Correspondences
        //==============================================================================
        cout << "Get Correspondences\n";
        b_freeze = false; // Update 3D visualization
        getCorrespondences(); // Both planes and lines according to s_type strategy

        b_show_corresp = true;
        b_freeze = false; // Update 3D visualization
        b_pause = true;
        while(b_pause)
            boost::this_thread::sleep (boost::posix_time::milliseconds (50));

        // Compute the calibration if there are enough measurements
        Matrix<T,3,3> R_planes = CalibrateRotationPair();
        Matrix<T,3,3> R_lines_n = ApproximateRotationZeroTrans();
        cout << "ROTATION diff " << RAD2DEG(acos( (trace<T,3>(R_planes * R_lines_n.transpose()) - 1) / 2)) << endl;
    }

    //========================== Perform calibration ===============================
    bool save_corresp = true;
    calibrate(save_corresp);
}

void KinectRigCalib::viz_cb (pcl::visualization::PCLVisualizer& viz)
{
    //cout << "ExtrinsicRgbdCalibration::viz_cb(...) " << b_freeze << "\n";
    if(!b_viz_init)
    {
        viz.setSize(1280,960); // Set the window size
        viz.setPosition(1100,740);
        viz.setCameraPosition(0,-2,-3,1,0,0);
        //viz.setCameraPosition(0,0,-5,0,-0.707107,0.707107,1,0,0);
        b_viz_init = true;
    }
    if( b_freeze || cloud.empty() || !cloud[0] || cloud[0]->empty() )
    {
        boost::this_thread::sleep (boost::posix_time::milliseconds (10));
        return;
    }
    //cout << "   ::viz_cb(...) Update visualization \n";

    viz.removeAllShapes();
    viz.removeAllPointClouds();
    //viz.removeCoordinateSystem();

    { //mrpt::synch::CCriticalSectionLocker csl(&CS_visualize);
        boost::mutex::scoped_lock updateLock(visualizationMutex);

        char name[1024];
        Eigen::Affine3f Rt;

        sprintf (name, "%zu pts. Params ...", cloud[1]->size());
        viz.addText (name, 20, 20, "params");
        viz.addCoordinateSystem(0.1, Eigen::Affine3f::Identity());

        // Draw camera systems and point clouds
        for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++)
            if (!viz.updatePointCloud (cloud[sensor_id], sensor_labels[sensor_id]))
            {
                Rt.matrix() = Rt_estimated[sensor_id].cast<float>();
                viz.addCoordinateSystem(0.2, Rt);
                viz.addPointCloud (cloud[sensor_id], sensor_labels[sensor_id]);
                viz.updatePointCloudPose(sensor_labels[sensor_id], Rt);
            }

        // Confirm match candidates
        if(b_confirm_visually)
        {
            if(b_wait_plane_confirm)
            {
                //cout << "Draw a pair of candidate planes to match \n";
                // Draw a pair of candidate planes to match
                for(size_t i=0; i < 2; i++)
                {
                    // Draw camera system
                    sprintf (name, "ref_%lu", i);
                    Rt.matrix() = Rt_estimated[sensor_pair[i]].cast<float>();
                    viz.removeCoordinateSystem();
                    viz.addCoordinateSystem(0.05, Rt);

                    mrpt::pbmap::Plane &plane = all_planes.vPlanes[plane_candidate_all[i]];
                    sprintf (name, "m_normal_%lu", i);
                    pcl::PointXYZ pt1(plane.v3center[0], plane.v3center[1], plane.v3center[2]); // Begin and end points of normal's arrow for visualization
                    pcl::PointXYZ pt2(plane.v3center[0] + (0.2f*plane.v3normal[0]), plane.v3center[1] + (0.2f*plane.v3normal[1]), plane.v3center[2] + (0.2f*plane.v3normal[2]));
                    viz.addArrow (pt2, pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);
                    sprintf (name, "m_plane_contour_%lu", i);
                    viz.addPolygon<PointT> (plane.polygonContourPtr, red[0], grn[0], blu[0], name);
                    sprintf (name, "m_inliers_%lu", i);
                    pcl::PointCloud<PointT>::Ptr planeCloudColoured = colourPointCloud<PointT>(plane.planePointCloudPtr, red[0], grn[0], blu[0], 0.5f);
                    viz.addPointCloud (planeCloudColoured, name);
                    viz.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
                    //pcl::visualization::PointCloudColorHandlerCustom <PointT> color (plane.planePointCloudPtr, red[0], grn[0], blu[0]);
                    //viz.addPointCloud (plane.planePointCloudPtr, color, name);
                }
                while(confirm_corresp == 0){
                    viz.spinOnce(50);
                    boost::this_thread::sleep (boost::posix_time::milliseconds (50));
                }

                updateLock.unlock();
                return;
            }

            //        if(b_wait_line_confirm)
            //        {
            //            // Draw a pair of candidate lines to match
            //            for(size_t i=0; i < 2; i++)
            //            {
            ////                if(vv_line_has3D[sensor_pair[i]][line_candidate[i]])
            //                {
            ////                    vector<mrpt::math::TLine3D> & seg3D = vv_segments3D[sensor_pair[i]][line_candidate[i]];
            //                    Eigen::Matrix<T,6,1> & seg3D = vv_segments3D[sensor_pair[i]][line_candidate[i]];

            //                    pcl::PointXYZ pt1(seg3D[0], seg3D[1], seg3D[2]);
            //                    pcl::PointXYZ pt2(seg3D[3], seg3D[4], seg3D[5]);
            //                    sprintf (name, "m_line_%lu_%u", sensor_pair[i], static_cast<unsigned>(i));
            //                    viz.addLine<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], name);
            //    //                viz.removeShape("line");
            //    //                viz.addLine<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], "line");

            //                    viz.removeShape("pt1");
            //                    viz.addSphere<pcl::PointXYZ>(pt1, 0.02, ared[i%10], agrn[i%10], ablu[i%10], "sp1");
            //                    viz.removeShape("pt2");
            //                    viz.addSphere<pcl::PointXYZ>(pt2, 0.02, ared[i%10], agrn[i%10], ablu[i%10], "sp2");

            //        //                size_t p1 = vv_segments2D[sensor1][i][0] + vv_segments2D[sensor1][i][1] * cloud[sensor1]->width;
            //                    size_t p1 = line_match1[0] + line_match1[1] * cloud[sensor1]->width;
            //                    if(cloud[sensor1]->points[p1].z > 0.3f && cloud[sensor1]->points[p1].z < 10.f)
            //                    {
            //                        viz.removeShape("sp1");
            //                        viz.addSphere<PointT>(cloud[sensor1]->points[p1], 0.02, ared[i%10], agrn[i%10], ablu[i%10], "sp1");
            //        //                    pcl::ModelCoefficients circle_coeff;
            //        //                    circle_coeff.values.resize(3);    // We need 3 values
            //        //                    circle_coeff.values[0] = x;
            //        //                    circle_coeff.values[1] = y;
            //        //                    circle_coeff.values[2] = radius;
            //        //                    viz.addCircle<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], "line");
            //                    }
            //                }
            //            }
            //            while(confirm_corresp == 0)
            //                boost::this_thread::sleep (boost::posix_time::milliseconds (10));

            //            size_t sensor2 = 1;
            //            Rt.matrix() = Rt_estimated[sensor2];
            //            size_t p1 = line_match2[0] + line_match2[1] * cloud[sensor2]->width;
            //            if(cloud[sensor2]->points[p1].z > 0.3f && cloud[sensor2]->points[p1].z < 10.f)
            //            {
            //                viz.removeShape("l2_sp1");
            //                viz.addSphere<PointT>(cloud[sensor2]->points[p1], 0.02, ared[1], agrn[1], ablu[1], "l2_sp1");
            //                viz.updateShapePose("l2_sp1", Rt);
            //            }
            //            size_t p2 = line_match2[2] + line_match2[3] * cloud[sensor2]->width;
            //            if(cloud[sensor2]->points[p2].z > 0.3f && cloud[sensor2]->points[p2].z < 10.f)
            //            {
            //                viz.removeShape("l2_sp2");
            //                viz.addSphere<PointT>(cloud[sensor2]->points[p2], 0.02, ared[1], agrn[1], ablu[1], "l2_sp2");
            //                viz.updateShapePose("l2_sp2", Rt);
            //            }
            //            return;
            //        }
        }

        if(b_show_corresp)
        {
//            cout << "   ::viz_cb(...) Draw correspondences \n";
//            // Draw planes
//            for(size_t i=0; i < all_planes.vPlanes.size(); i++)
//            {
//                //            for(map<unsigned, unsigned>::iterator it=plane_corresp.begin(); it!=plane_corresp.end(); it++)
//                //              if(it->first == i)
//                //              {
//                mrpt::pbmap::Plane &plane = all_planes.vPlanes[i];
//                sprintf (name, "normal_%u", static_cast<unsigned>(i));
//                pcl::PointXYZ pt1(plane.v3center[0], plane.v3center[1], plane.v3center[2]); // Begin and end points of normal's arrow for visualization
//                pcl::PointXYZ pt2(plane.v3center[0] + (0.2f*plane.v3normal[0]), plane.v3center[1] + (0.2f*plane.v3normal[1]), plane.v3center[2] + (0.2f*plane.v3normal[2]));
//                viz.addArrow (pt2, pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);

//                sprintf (name, "plane_contour_%02d", int (i));
//                // viz.addPolygon<PointT> (plane.polygonContourPtr, 0.5 * red[5], 0.5 * grn[5], 0.5 * blu[5], name);
//                viz.addPolygon<PointT> (plane.polygonContourPtr, red[i%10], grn[i%10], blu[i%10], name);

//            }
            // Draw plane matches
            set<size_t> drawn_planes;
            string matched_planes = "Matched planes:\n";
            for(map< size_t, map< size_t, map<size_t, size_t> > >::iterator it1=mmm_plane_matches_all.begin(); it1 != mmm_plane_matches_all.end(); it1++)
                for(map< size_t, map<size_t, size_t> >::iterator it2=it1->second.begin(); it2 != it1->second.end(); it2++)
                    for(map<size_t, size_t>::iterator it3=it2->second.begin(); it3 != it2->second.end(); it3++)
                    {
                        matched_planes += to_string(it3->first) + " - " + to_string(it3->second);
                        size_t col = it3->first%10; // Colour index
                        array<size_t,2> idx = {it3->first, it3->second};
                        for(size_t i=0; i < 2; i++)
                        {
                            if(drawn_planes.count(idx[i]))
                                continue;
                            drawn_planes.insert(idx[i]);
                            mrpt::pbmap::Plane &plane = all_planes.vPlanes[idx[i]];
                            sprintf (name, "normal_%lu", idx[i]);
                            pcl::PointXYZ pt1(plane.v3center[0], plane.v3center[1], plane.v3center[2]);
                            pcl::PointXYZ pt2(plane.v3center[0] + (0.2f*plane.v3normal[0]), plane.v3center[1] + (0.2f*plane.v3normal[1]), plane.v3center[2] + (0.2f*plane.v3normal[2]));
                            viz.addArrow (pt2, pt1, ared[col], agrn[col], ablu[col], false, name);

                            sprintf (name, "plane_contour_%lu", idx[i]);
                            viz.addPolygon<PointT> (plane.polygonContourPtr, red[col], grn[col], blu[col], name);

                            sprintf (name, "inliers_%lu", idx[i]);
                            pcl::PointCloud<PointT>::Ptr planeCloudColoured = colourPointCloud<PointT>(plane.planePointCloudPtr, red[col], grn[col], blu[col], 0.5f);
                            viz.addPointCloud (planeCloudColoured, name);
                        }
                    }
            viz.addText (matched_planes.c_str(), 220, 20, "matched_planes");

//            // Draw lines
//            for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++)
//            {
//                for(size_t i=0; i < vv_segments3D[sensor_id].size(); i++)
//                    //for(size_t sensor2=sensor1+1; sensor2 < num_sensors; sensor2++)
//                {
//                    //vector<mrpt::math::TLine3D> seg3D = vv_segments3D[sensor_id];
//                    Matrix<T,3,1> seg3D_1(Rt_estimated[sensor_id].block<3,3>(0,0)*vv_segments3D[sensor_id][i].block<3,1>(0,0) + Rt_estimated[sensor_id].block<3,1>(0,3));
//                    Matrix<T,3,1> seg3D_2(Rt_estimated[sensor_id].block<3,3>(0,0)*vv_segments3D[sensor_id][i].block<3,1>(3,0) + Rt_estimated[sensor_id].block<3,1>(0,3));
//                    pcl::PointXYZ pt1(seg3D_1[0], seg3D_1[1], seg3D_1[2]);
//                    pcl::PointXYZ pt2(seg3D_2[0], seg3D_2[1], seg3D_2[2]);
//                    sprintf (name, "line_%lu_%lu", sensor_id, i);
//                    viz.removeShape(name);
//                    viz.addLine<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], name);
//                }
//            }
            // Draw line matches
            //cout << "   ::viz_cb(...) Draw line matches \n";
            vector< set<size_t> > drawn_lines(num_sensors);
            string matched_lines = "Matched lines:\n";
            for(map< size_t, map< size_t, map<size_t, size_t> > >::iterator it1=mmm_line_matches.begin(); it1 != mmm_line_matches.end(); it1++)
                for(map< size_t, map<size_t, size_t> >::iterator it2=it1->second.begin(); it2 != it1->second.end(); it2++)
                    for(map<size_t, size_t>::iterator it3=it2->second.begin(); it3 != it2->second.end(); it3++)
                    {
                        matched_lines += to_string(it1->first) + "." + to_string(it3->first) + " - " + to_string(it2->first) + "." + to_string(it3->second);
                        size_t col = it3->first%10; // Colour index
                        array<size_t,2> idx = {it3->first, it3->second};
                        array<size_t,2> sensor = {it1->first, it2->first};
                        for(size_t i=0; i < 2; i++)
                        {
                            //cout << i << " line " << vv_segments3D[sensor[i]][idx[i]].transpose() << endl;
                            if(drawn_lines[sensor[i]].count(idx[i]))
                                continue;
                            drawn_lines[sensor[i]].insert(idx[i]);

                            Matrix<T,3,1> seg3D_1(Rt_estimated[sensor[i]].block<3,3>(0,0)*vv_segments3D[sensor[i]][idx[i]].block<3,1>(0,0) + Rt_estimated[sensor[i]].block<3,1>(0,3));
                            Matrix<T,3,1> seg3D_2(Rt_estimated[sensor[i]].block<3,3>(0,0)*vv_segments3D[sensor[i]][idx[i]].block<3,1>(3,0) + Rt_estimated[sensor[i]].block<3,1>(0,3));
                            pcl::PointXYZ pt1(seg3D_1[0], seg3D_1[1], seg3D_1[2]);
                            pcl::PointXYZ pt2(seg3D_2[0], seg3D_2[1], seg3D_2[2]);
                            sprintf (name, "line_%lu_%lu", sensor[i], idx[i]);
                            viz.removeShape(name);
                            viz.addLine<pcl::PointXYZ>(pt1, pt2, red[col], grn[col], blu[col], name);
                            sprintf (name, "sp1_%lu_%lu", sensor[i], idx[i]);
                            viz.addSphere<pcl::PointXYZ>(pt1, 0.01, red[col], grn[col], blu[col], name);
                            sprintf (name, "sp2_%lu_%lu", sensor[i], idx[i]);
                            viz.addSphere<pcl::PointXYZ>(pt2, 0.01, red[col], grn[col], blu[col], name);

                            // Draw virtual plane
                            sprintf (name, "virtual_plane_%lu_%lu", sensor[i], idx[i]);
                            pcl::PointCloud<pcl::PointXYZ>::Ptr triangle(new pcl::PointCloud<pcl::PointXYZ>());
                            triangle->points.resize(3);
                            triangle->points[0].getVector3fMap() = Rt_estimated[sensor[i]].block<3,1>(0,3).cast<float>(); // Set the optical center of the camera
                            triangle->points[1] = pt1;
                            triangle->points[2] = pt2;
                            viz.addPolygon<pcl::PointXYZ> (triangle, red[col], grn[col], blu[col], name);

//                            pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
//                            plane->values.resize(4);
//                            plane->values[0] = vv_segment_n[sensor[i]][idx[i]][0];
//                            plane->values[1] = vv_segment_n[sensor[i]][idx[i]][1];
//                            plane->values[2] = vv_segment_n[sensor[i]][idx[i]][2];
//                            plane->values[3] = 0;
//                            viz.addPlane (*plane, name, 0);
//                            viz.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,  ared[col], agrn[col], ablu[col], name, 0);
//                            viz.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.4, name, 0);
//                            //viz.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name, 0);

                            pcl::PointXYZ pt1_n = p1;//(triangle->points[0]); // Begin and end points of normal's arrow for visualization
//                            pt1_n.getVector3fMap() += triangle->points[1].getVector3fMap() + triangle->points[2].getVector3fMap(); pt1_n.getVector3fMap() /= 3.f;
                            pcl::PointXYZ pt2_n(pt1_n); pt2_n.getVector3fMap() += 0.1f*vv_segment_n[sensor[i]][idx[i]].cast<float>();
                            sprintf (name, "seg_n_%lu_%lu", sensor[i], idx[i]);
                            viz.addArrow (pt2_n, pt1_n, ared[col], agrn[col], ablu[col], false, name);
                        }
                    }
            viz.addText (matched_lines.c_str(), 420, 20, "matched_lines");
            b_show_corresp = false;
        }

#if RECORD_VIDEO
        string screenshotFile = mrpt::format("im_%04u.png", ++numScreenshot);
        viz.saveScreenshot(screenshotFile);
#endif
        b_freeze = true;
        updateLock.unlock();
    }
}

void KinectRigCalib::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    if ( event.keyDown () )
    {
        cout << "KinectRigCalib::keyboardEventOccurred : " << event.getKeySym () << endl;
        if(event.getKeySym () == "e" || event.getKeySym () == "E")
            b_exit = true;
        else if(event.getKeySym () == "k" || event.getKeySym () == "K"){
            confirm_corresp = 1;
        }
        else if(event.getKeySym () == "l" || event.getKeySym () == "L"){
            confirm_corresp = -1;
//            b_freeze = !b_freeze;
        }
        else if(event.getKeySym () == "Return"){
            cout << "PAUSE\n";
            b_pause = !b_pause;
        }
    }
}
