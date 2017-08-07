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
#include "kinect-rig-calib_display.h"
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
//    calib = ExtrinsicCalib( sensor_labels.size() );
//    size_t num_sensors = num_sensors;
//    vector<Eigen::Matrix<T,4,4> , Eigen::aligned_allocator<Eigen::Matrix<T,4,4> > > Rt_estimated = Rt_estimated;

    if(verbose)
    {
        for(size_t i=0; i < num_sensors; i++)
        cout << "num_sensors: " << num_sensors << endl;
    }

    num_sensors = sensor_labels.size();
    Rt_estimated = vector<Eigen::Matrix<T,4,4> , Eigen::aligned_allocator<Eigen::Matrix<T,4,4> > >(num_sensors, Eigen::Matrix<T,4,4>::Identity());
    rgbd_intrinsics = vector<mrpt::utils::TStereoCamera>(num_sensors);
    init_poses = vector<CPose3D>(num_sensors);
    v_approx_trans = vector<double>(num_sensors);
    v_approx_rot = vector<double>(num_sensors);
    rgb.resize(num_sensors);
    depth.resize(num_sensors);
    depth_reg.resize(num_sensors); // Depth image registered to RGB pose
    cloud.resize(num_sensors);
    v_pbmap.resize(num_sensors);
    if(s_type == LINES || s_type == PLANES_AND_LINES)
    {
        v_segments2D.resize(num_sensors);
        v_segments3D.resize(num_sensors);
        v_line_has3D.resize(num_sensors);
        v_lines3D.resize(num_sensors);
    }

    for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++) // Load the approximated init_poses of each sensor and the accuracy of such approximation
    {
        if(verbose)
            cout << num_sensors << " sensor: " << sensor_labels[i] << endl;

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

        mm_conditioning[sensor_id] = std::map<size_t, double>;
        mm_covariance[sensor_id] = std::map<size_t, Eigen::Matrix<T,3,3> >;
        for(size_t sensor2=sensor_id+1; sensor2 < num_sensors; sensor2++)
        {
            mm_conditioning[sensor_id][sensor2] = 0.0;
            mm_covariance[sensor_id][sensor2] = Eigen::Matrix<T,3,3>::Zero();
        }

        cout << sensor_labels[sensor_id] << " v_approx_trans " << v_approx_trans[sensor_id] << " v_approx_rot " << v_approx_rot[sensor_id] << "\n" << init_poses[sensor_id] << endl;
        cout << Rt_estimated[sensor_id] << endl;

        rgbd_intrinsics[sensor_id].loadFromConfigFile(sensor_labels[sensor_id],cfg);
//        string calib_path = mrpt::system::extractFileDirectory(rawlog_file) + "asus_" + std::to_string(sensor_id+1) + "/ini";
//        mrpt::utils::CConfigFile calib_RGBD(calib_path);
//        rgbd_intrinsics[sensor_id].loadFromConfigFile("CAMERA_PARAMS", calib_RGBD);
//        cout << "right2left_camera_pose \n" << rgbd_intrinsics[sensor_id].rightCameraPose << endl;

//        cout << sensor_labels[sensor_id] << "RGB params: fx=" << rgbd_intrinsics[sensor_id].rightCamera.fx() << " fy=" << rgbd_intrinsics[sensor_id].rightCamera.fy() << " cx=" << rgbd_intrinsics[sensor_id].rightCamera.cx() << " cy=" << rgbd_intrinsics[sensor_id].rightCamera.cy() << endl;
//        cout << sensor_labels[sensor_id] << "RGB params: fx=" << rgbd_intrinsics2[sensor_id].cam_params.rightCamera.fx() << " fy=" << rgbd_intrinsics2[sensor_id].cam_params.rightCamera.fy() << " cx=" << rgbd_intrinsics2[sensor_id].cam_params.rightCamera.cx() << " cy=" << rgbd_intrinsics2[sensor_id].cam_params.rightCamera.cy() << endl;
    }
    cout << "...loadConfiguration\n";
}

void KinectRigCalib::getCorrespondences()
{
    if(s_type == PLANES_AND_LINES || s_type == PLANES)
        ExtrinsicCalibPlanes<T>::getCorrespondences(cloud);
//    calib_planes.getCorrespondences(cloud);
    if(s_type == PLANES_AND_LINES || s_type == LINES)
        ExtrinsicCalibLines<T>::getCorrespondences(rgb, cloud);
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
//    {

//    }
}

void KinectRigCalib::run()
{
    //						Open Rawlog File
    //==================================================================
    mrpt::obs::CRawlog dataset;
    if (!dataset.loadFromRawLogFile(rawlog_file))
        throw std::runtime_error("\nCouldn't open rawlog dataset file for input...");

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
        undist_rgb[i].setFromCamParams(rgbd_intrinsics[i].rightCamera);
        undist_depth[i].setFromCamParams(rgbd_intrinsics[i].leftCamera);
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
        if( !std::accumulate(begin(obs_sensor), end(obs_sensor), true, logical_and<bool>()) )
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
        std::fill(obs_sensor.begin(), obs_sensor.end(), false);

        // Apply decimation
        if( n_frame_sync % decimation != 0)
            continue;
        cout << n_obs << " use this sync frames \n";


        // Get the rgb and depth images, and the point clouds
        for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++)
        {
//            rgb[sensor_id] = cv::Mat(obsRGBD[sensor_id]->intensityImage.getAs<IplImage>());
//            convertRange_mrpt2cvMat(obsRGBD[sensor_id]->rangeImage, depth[sensor_id]);
            // With image rectification (to reduce radial distortion)
            cv::Mat src(obsRGBD[sensor_id]->intensityImage.getAs<IplImage>());
            undist_rgb[sensor_id].undistort(src, rgb[sensor_id]);
            cv::Mat raw_depth;
            convertRange_mrpt2cvMat(obsRGBD[sensor_id]->rangeImage, raw_depth);
            undist_depth[sensor_id].undistort(raw_depth, depth[sensor_id]);
            cloud[sensor_id] = getPointCloudRegistered(rgb[sensor_id], depth[sensor_id], rgbd_intrinsics[sensor_id], depth_reg[sensor_id]);
//            pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//            viewer.showCloud(cloud[sensor_id]);
//            while (!viewer.wasStopped ())
//                boost::this_thread::sleep (boost::posix_time::milliseconds (10));
        }
        int height = rgb[0].cols; // Note that the RGB-D camera is in a vertical configuration
        int width = rgb[0].rows;

        // Display color and depth images
        if(display)
        {
            cv::Mat rgb_concat(height, 2*width+20, CV_8UC3, cv::Scalar(155,100,255));
            cv::Mat img_transposed, img_rotated;
            cv::transpose(rgb[0], img_transposed);
            cv::flip(img_transposed, img_rotated, 0);
            cv::Mat tmp = rgb_concat(cv::Rect(0, 0, width, height));
            img_rotated.copyTo(tmp);
            cv::transpose(rgb[1], img_transposed);
            cv::flip(img_transposed, img_rotated, 0);
            tmp = rgb_concat(cv::Rect(width+20, 0, width, height));
            img_rotated.copyTo(tmp);
            //cv::circle(rgb_concat, cv::Point(240,320),5,cv::Scalar(0, 0, 200));
            //cout << "Pixel values " << rgb_concat.at<cv::Vec3b>(5,5) << " " << rgb_concat.at<cv::Vec3b>(5,475) << " " << rgb_concat.at<cv::Vec3b>(5,485) << " " << endl;
            cv::imshow("rgb", rgb_concat ); cv::moveWindow("rgb", 20,20);
            //cv::waitKey(0);
            cv::Mat depth_concat(height, 2*width+20, CV_32FC1, cv::Scalar(0.f));
            cv::transpose(depth[0], img_transposed);
            cv::flip(img_transposed, img_rotated, 0);
            tmp = depth_concat(cv::Rect(0, 0, width, height));
            img_rotated.copyTo(tmp);
            cv::transpose(depth[1], img_transposed);
            cv::flip(img_transposed, img_rotated, 0);
            tmp = depth_concat(cv::Rect(width+20, 0, width, height));
            img_rotated.copyTo(tmp);
            cv::Mat img_depth_display;
            depth_concat.convertTo(img_depth_display, CV_32FC1, 0.3 );
            cv::Mat depth_display = cv::Mat(depth_concat.rows, depth_concat.cols, depth_concat.type(), cv::Scalar(1.f)) - img_depth_display;
            depth_display.setTo(cv::Scalar(0.f), depth_concat < 0.1f);
            cv::imshow("depth", depth_display ); cv::moveWindow("depth", 20,100+640);
            //            cout << "Some depth values: " << depth_concat.at<float>(200, 320) << " " << depth_concat.at<float>(50, 320) << " "
            //                 << depth[0].at<float>(200, 320) << " " << depth[0].at<float>(50, 320) << " " << depth[1].at<float>(430, 320) << " " << depth[1].at<float>(280, 320) << "\n";
            cv::waitKey(0);
        }

        //						Segment local planes
        //==================================================================
        // #pragma omp parallel num_threads(num_sensors)
        for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++)
        {
            PbMap::pbMapFromPCloud(cloud[sensor_id], v_pbmap[sensor_id]);
            DisplayCloudPbMap::displayAndPause(cloud[sensor_id], v_pbmap[sensor_id]);
            PbMap::displayImagePbMap(cloud[sensor_id], rgb[sensor_id], v_pbmap[sensor_id]);
        }

        //==============================================================================
        //								Get Correspondences
        //==============================================================================
        { boost::mutex::scoped_lock updateLock(visualizationMutex);
            getCorrespondences(); // Of both planes and lines according to s_type strategy
            b_freeze = false;
            updateLock.unlock();
        }

        // Compute the calibration if there are enough measurements

    }

    //========================== Perform calibration ===============================
    bool save_corresp = true;
    calibrate(save_corresp);
}


void KinectRigCalib::viz_cb (pcl::visualization::PCLVisualizer& viz)
{
    //cout << "ExtrinsicRgbdCalibration::viz_cb(...)\n";
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
    //cout << "   ::viz_cb(...)\n";

    viz.removeAllShapes();
    viz.removeAllPointClouds();

    { //mrpt::synch::CCriticalSectionLocker csl(&CS_visualize);
        boost::mutex::scoped_lock updateLock(visualizationMutex);

        char name[1024];
        Eigen::Affine3f Rt;

        sprintf (name, "%zu pts. Params ...", krccloud[1]->size());
        viz.addText (name, 20, 20, "params");

        // Draw camera system
        //Rt.matrix() = initOffset;
        Rt.matrix() = Rt_estimated[1];
        viz.removeCoordinateSystem();
        viz.addCoordinateSystem(0.05, Rt);
        viz.addCoordinateSystem(0.1, Eigen::Affine3f::Identity());

        for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++)
            if (!viz.updatePointCloud (cloud[sensor_id], sensor_labels[sensor_id]))
            {
                viz.addPointCloud (cloud[sensor_id], sensor_labels[sensor_id]);
                Rt.matrix() = Rt_estimated[sensor_id];
                viz.updatePointCloudPose(sensor_labels[sensor_id], Rt);
            }

        // Draw match candidates
        if(b_confirm_visually)
        {
            // Draw a pair of candidate planes to match
            for(size_t i=0; i < 2; i++)
            {
    //            mrpt::pbmap::Plane &plane_i = v_pbmap[sensor_pair[i]].vPlanes[plane_candidate[i]];
                mrpt::pbmap::Plane &plane_i = all_planes.vPlanes[plane_candidate_all[i]];
                sprintf (name, "m_normal_%u", static_cast<unsigned>(i));
                pcl::PointXYZ pt1(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]); // Begin and end points of normal's arrow for visualization
                pcl::PointXYZ pt2(plane_i.v3center[0] + (0.3f * plane_i.v3normal[0]), plane_i.v3center[1] + (0.3f * plane_i.v3normal[1]), plane_i.v3center[2] + (0.3f * plane_i.v3normal[2]));
                viz.addArrow (pt2, pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);
                sprintf (name, "m_plane_contour_%02d", int (sensor_pair[i]));
                viz.addPolygon<PointT> (plane_i.polygonContourPtr, red[0], grn[0], blu[0], name);
                sprintf (name, "m_inliers_%02d", int (sensor_pair[i]));
                pcl::visualization::PointCloudColorHandlerCustom <PointT> color (plane_i.planePointCloudPtr, red[0], grn[0], blu[0]);
                viz.addPointCloud (plane_i.planePointCloudPtr, color, name);
            }

            // Draw a pair of candidate lines to match
            for(size_t i=0; i < 2; i++)
            {
//                if(v_line_has3D[sensor_pair[i]][line_candidate[i]])
                {
                    vector<mrpt::math::TLine3D> & seg3D = v_segments3D[sensor_pair[i]][line_candidate[i]];

                    pcl::PointXYZ pt1(seg3D[0], seg3D[1], seg3D[2]);
                    pcl::PointXYZ pt2(seg3D[3], seg3D[4], seg3D[5]);
                    sprintf (name, "m_line_%lu_%u", sensor_pair[i], static_cast<unsigned>(i));
                    viz.addLine<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], name);
    //                viz.removeShape("line");
    //                viz.addLine<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], "line");

                    viz.removeShape("pt1");
                    viz.addSphere<PointT>(pt1, 0.02, ared[i%10], agrn[i%10], ablu[i%10], "sp1");
                    viz.removeShape("pt2");
                    viz.addSphere<PointT>(pt2, 0.02, ared[i%10], agrn[i%10], ablu[i%10], "sp2");
                }
            }
        }

        // Draw planes
        for(size_t i=0; i < all_planes.vPlanes.size(); i++)
        {
            //            for(map<unsigned, unsigned>::iterator it=plane_corresp.begin(); it!=plane_corresp.end(); it++)
            //              if(it->first == i)
            //              {
            mrpt::pbmap::Plane &plane_i = all_planes.vPlanes[i];
            sprintf (name, "normal_%u", static_cast<unsigned>(i));
            pcl::PointXYZ pt1(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]); // Begin and end points of normal's arrow for visualization
            pcl::PointXYZ pt2(plane_i.v3center[0] + (0.3f * plane_i.v3normal[0]), plane_i.v3center[1] + (0.3f * plane_i.v3normal[1]), plane_i.v3center[2] + (0.3f * plane_i.v3normal[2]));
            //            viz.addArrow (pt2, pt1, ared[5], agrn[5], ablu[5], false, name);
            viz.addArrow (pt2, pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);

            sprintf (name, "plane_contour_%02d", int (i));
            //            viz.addPolygon<PointT> (plane_i.polygonContourPtr, 0.5 * red[5], 0.5 * grn[5], 0.5 * blu[5], name);
            viz.addPolygon<PointT> (plane_i.polygonContourPtr, red[0], grn[0], blu[0], name);

            //            for(map<unsigned, unsigned>::iterator it=plane_corresp.begin(); it!=plane_corresp.end(); it++)
            //              if(it->first == i)
            //              {
            //                mrpt::pbmap::Plane &plane_i = all_planes.vPlanes[i];
            //                sprintf (name, "normal_%u", static_cast<unsigned>(i));
            //                pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
            //                pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
            //                pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.3f * plane_i.v3normal[0]),
            //                                    plane_i.v3center[1] + (0.3f * plane_i.v3normal[1]),
            //                                    plane_i.v3center[2] + (0.3f * plane_i.v3normal[2]));
            //                viz.addArrow (pt2, pt1, ared[5], agrn[5], ablu[5], false, name);
            //
            //                sprintf (name, "plane_contour_%02d", int (i));
            //                viz.addPolygon<PointT> (plane_i.polygonContourPtr, red[0], grn[0], blu[0], name);
            //
            ////                sprintf (name, "inliers_%02d", int (i));
            ////                pcl::visualization::PointCloudColorHandlerCustom <PointT> color (plane_i.planePointCloudPtr, red[0], grn[0], blu[0]);
            ////                viz.addPointCloud (plane_i.planePointCloudPtr, color, name);
            //              }
        }

        // Draw lines
        vector<mrpt::math::TLine3D> &seg3D = v_segments3D[sensor1];
        for(size_t i=0; i < lines.size(); i++)
        {
            pcl::PointXYZ pt1(seg3D[0], seg3D[1], seg3D[2]);
            pcl::PointXYZ pt2(seg3D[3], seg3D[4], seg3D[5]);
            sprintf (name, "line_%lu_%u", sensor1, static_cast<unsigned>(i));
            //viz.addLine<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], name);
            viz.removeShape("line");
            viz.addLine<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], "line");

//                size_t p1 = v_segments2D[sensor1][i][0] + v_segments2D[sensor1][i][1] * cloud[sensor1]->width;
            size_t p1 = line_match1[0] + line_match1[1] * cloud[sensor1]->width;
            if(cloud[sensor1]->points[p1].z > 0.3f && cloud[sensor1]->points[p1].z < 10.f)
            {
                viz.removeShape("sp1");
                viz.addSphere<PointT>(cloud[sensor1]->points[p1], 0.02, ared[i%10], agrn[i%10], ablu[i%10], "sp1");
//                    pcl::ModelCoefficients circle_coeff;
//                    circle_coeff.values.resize(3);    // We need 3 values
//                    circle_coeff.values[0] = x;
//                    circle_coeff.values[1] = y;
//                    circle_coeff.values[2] = radius;
//                    viz.addCircle<pcl::PointXYZ>(pt1, pt2, ared[i%10], agrn[i%10], ablu[i%10], "line");
            }
//                size_t p2 = v_segments2D[sensor1][i][2] + v_segments2D[sensor1][i][3] * cloud[sensor1]->width;
            size_t p2 = line_match1[2] + line_match1[3] * cloud[sensor1]->width;
            if(cloud[sensor1]->points[p2].z > 0.3f && cloud[sensor1]->points[p2].z < 10.f)
            {
                viz.removeShape("sp2");
                viz.addSphere<PointT>(cloud[sensor1]->points[p2], 0.02, ared[i%10], agrn[i%10], ablu[i%10], "sp2");
            }
        }

        size_t sensor2 = 1;
        Rt.matrix() = Rt_estimated[sensor2];
        size_t p1 = line_match2[0] + line_match2[1] * cloud[sensor2]->width;
        if(cloud[sensor2]->points[p1].z > 0.3f && cloud[sensor2]->points[p1].z < 10.f)
        {
            viz.removeShape("l2_sp1");
            viz.addSphere<PointT>(cloud[sensor2]->points[p1], 0.02, ared[1], agrn[1], ablu[1], "l2_sp1");
            viz.updateShapePose("l2_sp1", Rt);
        }
        size_t p2 = line_match2[2] + line_match2[3] * cloud[sensor2]->width;
        if(cloud[sensor2]->points[p2].z > 0.3f && cloud[sensor2]->points[p2].z < 10.f)
        {
            viz.removeShape("l2_sp2");
            viz.addSphere<PointT>(cloud[sensor2]->points[p2], 0.02, ared[1], agrn[1], ablu[1], "l2_sp2");
            viz.updateShapePose("l2_sp2", Rt);
        }

//            //if(plane_corresp.size() > 0)
//            {
//                for(size_t i=0; i < v_pbmap[0].vPlanes.size(); i++)
//                {
//                    //            for(map<unsigned, unsigned>::iterator it=plane_corresp.begin(); it!=plane_corresp.end(); it++)
//                    //              if(it->first == i)
//                    //              {
//                    mrpt::pbmap::Plane &plane_i = v_pbmap[0].vPlanes[i];
//                    sprintf (name, "normal_%u", static_cast<unsigned>(i));
//                    pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
//                    pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
//                    pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.3f * plane_i.v3normal[0]),
//                            plane_i.v3center[1] + (0.3f * plane_i.v3normal[1]),
//                            plane_i.v3center[2] + (0.3f * plane_i.v3normal[2]));
//                    //            viz.addArrow (pt2, pt1, ared[5], agrn[5], ablu[5], false, name);
//                    viz.addArrow (pt2, pt1, ared[0], agrn[0], ablu[0], false, name);

//                    sprintf (name, "plane_contour_%02d", int (i));
//                    //            viz.addPolygon<PointT> (plane_i.polygonContourPtr, 0.5 * red[5], 0.5 * grn[5], 0.5 * blu[5], name);
//                    viz.addPolygon<PointT> (plane_i.polygonContourPtr, red[0], grn[0], blu[0], name);

//                    //            for(map<unsigned, unsigned>::iterator it=plane_corresp.begin(); it!=plane_corresp.end(); it++)
//                    //              if(it->first == i)
//                    //              {
//                    //                mrpt::pbmap::Plane &plane_i = v_pbmap[0].vPlanes[i];
//                    //                sprintf (name, "normal_%u", static_cast<unsigned>(i));
//                    //                pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
//                    //                pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
//                    //                pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.3f * plane_i.v3normal[0]),
//                    //                                    plane_i.v3center[1] + (0.3f * plane_i.v3normal[1]),
//                    //                                    plane_i.v3center[2] + (0.3f * plane_i.v3normal[2]));
//                    //                viz.addArrow (pt2, pt1, ared[5], agrn[5], ablu[5], false, name);
//                    //
//                    //                sprintf (name, "plane_contour_%02d", int (i));
//                    //                viz.addPolygon<PointT> (plane_i.polygonContourPtr, red[0], grn[0], blu[0], name);
//                    //
//                    ////                sprintf (name, "inliers_%02d", int (i));
//                    ////                pcl::visualization::PointCloudColorHandlerCustom <PointT> color (plane_i.planePointCloudPtr, red[0], grn[0], blu[0]);
//                    ////                viz.addPointCloud (plane_i.planePointCloudPtr, color, name);
//                    //              }
//                }
//                for(size_t i=0; i < v_pbmap[1].vPlanes.size(); i++)
//                {
//                    //            for(map<unsigned, unsigned>::iterator it=plane_corresp.begin(); it!=plane_corresp.end(); it++)
//                    //              if(it->second == i)
//                    //              {
//                    //    cout << "   v_pbmap[1] " << i << "\n";

//                    mrpt::pbmap::Plane &plane_i = v_pbmap[1].vPlanes[i];
//                    sprintf (name, "normal_j_%u", static_cast<unsigned>(i));
//                    pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
//                    pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
//                    pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.3f * plane_i.v3normal[0]),
//                            plane_i.v3center[1] + (0.3f * plane_i.v3normal[1]),
//                            plane_i.v3center[2] + (0.3f * plane_i.v3normal[2]));
//                    //            viz.addArrow (pt2, pt1, ared[5], agrn[5], ablu[5], false, name);
//                    viz.addArrow (pt2, pt1, ared[3], agrn[3], ablu[3], false, name);

//                    sprintf (name, "plane_contour_j_%02d", int (i));
//                    //            viz.addPolygon<PointT> (plane_i.polygonContourPtr, 0.5 * red[5], 0.5 * grn[5], 0.5 * blu[5], name);
//                    viz.addPolygon<PointT> (plane_i.polygonContourPtr, red[3], grn[3], blu[3], name);

//                    //            for(map<unsigned, unsigned>::iterator it=plane_corresp.begin(); it!=plane_corresp.end(); it++)
//                    //              if(it->second == i)
//                    //              {
//                    //                mrpt::pbmap::Plane &plane_i = v_pbmap[1].vPlanes[i];
//                    //                sprintf (name, "normal_j_%u", static_cast<unsigned>(i));
//                    //                pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
//                    //                pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
//                    //                pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.3f * plane_i.v3normal[0]),
//                    //                                    plane_i.v3center[1] + (0.3f * plane_i.v3normal[1]),
//                    //                                    plane_i.v3center[2] + (0.3f * plane_i.v3normal[2]));
//                    //                viz.addArrow (pt2, pt1, ared[5], agrn[5], ablu[5], false, name);
//                    //
//                    //                sprintf (name, "plane_contour_j_%02d", int (i));
//                    //                viz.addPolygon<PointT> (plane_i.polygonContourPtr, red[3], grn[3], blu[3], name);
//                    //
//                    ////                sprintf (name, "inliers_j_%02d", int (i));
//                    ////                pcl::visualization::PointCloudColorHandlerCustom <PointT> color (plane_i.planePointCloudPtr, red[3], grn[3], blu[3]);
//                    ////                viz.addPointCloud (plane_i.planePointCloudPtr, color, name);
//                    //              }
//                }
//          }

#if RECORD_VIDEO
        std::string screenshotFile = mrpt::format("im_%04u.png", ++numScreenshot);
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
        if(event.getKeySym () == "e" || event.getKeySym () == "E")
            b_exit = true;
        else if(event.getKeySym () == "k" || event.getKeySym () == "K"){
            confirmed_corresp = 1;
        }
        else if(event.getKeySym () == "l" || event.getKeySym () == "L"){
            confirmed_corresp = -1;
            b_freeze = !b_freeze;
        }
    }

    plane_candidate[0] = i; plane_candidate[1] = j;
    plane_candidate_all[0] = planesIdx_i; plane_candidate_all[1] = planesIdx_j;
    confirmed_corresp = 0;
}

