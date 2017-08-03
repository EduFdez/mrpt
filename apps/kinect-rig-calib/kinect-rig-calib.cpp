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
#include <mrpt/pbmap/PbMapMaker.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

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
    mxmy.resize(num_sensors);
    init_poses = vector<CPose3D>(num_sensors);
    v_approx_trans = vector<double>(num_sensors);
    v_approx_rot = vector<double>(num_sensors);
    rgb.resize(num_sensors);
    depth.resize(num_sensors);
    depth_reg.resize(num_sensors); // Depth image registered to RGB pose
    cloud.resize(num_sensors);
    v_pbmap.resize(num_sensors);
//    v_lines.resize(num_sensors);
//    v_lines3D.resize(num_sensors);

//    std::fill(conditioning, conditioning+8, 9999.9);
//    std::fill(weight_pair, weight_pair+8, 0.0);
//    std::fill(valid_obs, valid_obs+8, 0);
//    std::fill(covariances, covariances+8, Eigen::Matrix<T,3,3>::Zero());

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
        cout << sensor_labels[sensor_id] << " v_approx_trans " << v_approx_trans[sensor_id] << " v_approx_rot " << v_approx_rot[sensor_id] << "\n" << init_poses[sensor_id] << endl;
        cout << Rt_estimated[sensor_id] << endl;

        rgbd_intrinsics[sensor_id].loadFromConfigFile(sensor_labels[sensor_id],cfg);
        mxmy[sensor_id] = rgbd_intrinsics[sensor_id].rightCamera.intrinsicParams(0,0)/rgbd_intrinsics[sensor_id].rightCamera.intrinsicParams(1,1);
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
        ExtrinsicCalibLines<T>::getCorrespondences(rgb);
//    calib_lines.getCorrespondences();
}

void KinectRigCalib::calibrate()
{
    if(s_type == PLANES_AND_LINES || s_type == PLANES)
    {
        //          calib_planes.calcFisherInfMat();
        calib_planes.CalibrateRotationManifold(1);
        calib_planes.CalibrateTranslation(1);
        //          calib_planes.CalibrateRotation();

        //          if(conditioning < 100 & n_plane_corresp > 3)
        //            calib_planes.CalibratePair();

        //          if(conditioning < 50 & n_plane_corresp > 30)
        //            bDoCalibration = true;

        //cout << "run9\n";

        //          while(v3D.b_confirm_visually == true)
        //            boost::this_thread::sleep (boost::posix_time::milliseconds(10));


        // Trim outliers
        //trimOutliersRANSAC(calib_planes.correspondences, conditioningFIM);

        float threshold_conditioning = 800.0;
        if(conditioning < threshold_conditioning)
        {
            cout << "\tSave CorrespMat\n";
            calib_planes.correspondences.saveToTextFile( mrpt::format("%s/correspondences.txt", output_dir.c_str()) );
            conditioningFIM.saveToTextFile( mrpt::format("%s/conditioningFIM.txt", output_dir.c_str()) );

            calib_planes.CalibratePair();

            calib_planes.CalibrateRotationD();

            calib_planes.setInitRt(initOffset);
            calib_planes.CalibrateRotationManifold();
            calib_planes.Rt_estimated.block(0,3,3,1) = calib_planes.CalibrateTranslation();
            cout << "Rt_estimated \n" << calib_planes.Rt_estimated << endl;
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
//    std::vector<pcl::PointCloud<PointT>::Ptr> cloud(num_sensors);
    calib_lines.v_lines.resize(num_sensors);
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
    calib_planes = ExtrinsicCalibPlanes<T>(num_sensors);
    calib_lines = ExtrinsicCalibLines<T>(num_sensors);

    CMatrixDouble conditioningFIM(0,6);
    Eigen::Matrix<T,3,3> FIMrot = Eigen::Matrix<T,3,3>::Zero();
    Eigen::Matrix<T,3,3> FIMtrans = Eigen::Matrix<T,3,3>::Zero();

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
        for(size_t i=0; i < num_sensors; i++)
        {
//            rgb[i] = cv::Mat(obsRGBD[i]->intensityImage.getAs<IplImage>());
//            convertRange_mrpt2cvMat(obsRGBD[i]->rangeImage, depth[i]);
            // With image rectification (to reduce radial distortion)
            cv::Mat src(obsRGBD[i]->intensityImage.getAs<IplImage>());
            undist_rgb[i].undistort(src, rgb[i]);
            cv::Mat raw_depth;
            convertRange_mrpt2cvMat(obsRGBD[i]->rangeImage, raw_depth);
            undist_depth[i].undistort(raw_depth, depth[i]);
            cloud[i] = getPointCloudRegistered(rgb[i], depth[i], rgbd_intrinsics[i], depth_reg[i]);
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
    calibrate();
}
