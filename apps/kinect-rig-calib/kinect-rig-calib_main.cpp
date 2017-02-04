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

// 0 load init file with estimated v_pose and errors. display/verbose parameters
// 1 open rawlog
// 2 identify observation pairs/triples...
// 3 segment planes and lines
// 4 get correspondences
// 5 perform calibration. calibration algorithm in a different file
// 6 visualize and evaluate

//#include <mrpt/math/CMatrixFixedNumeric.h>
//#include <mrpt/utils/CArray.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/maps/CColouredPointsMap.h>
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

#include <omp.h>

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


typedef pcl::PointXYZRGB PointT;

pcl::PointCloud<pcl::Normal>::Ptr computeImgNormal(const pcl::PointCloud<PointT>::Ptr & cloud, const float depth_thres, const float smooth_factor)
{
    //ImgRGBD_3D::fastBilateralFilter(cloud, cloud);

    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
    //ne.setNormalEstimationMethod(ne.SIMPLE_3D_GRADIENT);
    //ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
    //ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(depth_thres); // For VGA: 0.02f, 10.0f
    ne.setNormalSmoothingSize(smooth_factor);
    ne.setDepthDependentSmoothing(true);

    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(cloud);
    ne.compute(*normal_cloud);
    return normal_cloud;
}

size_t segmentPlanes(const pcl::PointCloud<PointT>::Ptr & cloud,
                    vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > & regions, std::vector<pcl::ModelCoefficients> & model_coefficients, std::vector<pcl::PointIndices> & inliers,
                    const float dist_threshold, const float angle_threshold, const size_t min_inliers)
{
//#if _VERBOSE_PROFILING
//    //cout << "ContinuousFeaturesPCL::segmentPlanes... \n";
//    double time_start = rv::utils::getTime();
//    //for(size_t k=0; k<1000; k++)
//    {
//#endif

    pcl::PointCloud<PointT>::Ptr cloud_filtered = cloud;
//    ImgRGBD_3D::fastBilateralFilter(cloud, cloud_filtered, 30, 0.5);
//    computeImgNormal(cloud_filtered);
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud = computeImgNormal(cloud_filtered, 0.02f, 10.0f);

    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers(min_inliers);
    mps.setAngularThreshold(angle_threshold); // (0.017453 * 2.0) // 3 degrees
    mps.setDistanceThreshold(dist_threshold); //2cm
    mps.setInputNormals(normal_cloud);
    mps.setInputCloud(cloud_filtered);

//    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
//    std::vector<pcl::ModelCoefficients> model_coefficients;
//    std::vector<pcl::PointIndices> inliers;
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;
    mps.segmentAndRefine(regions, model_coefficients, inliers, labels, label_indices, boundary_indices);
    size_t n_regions = regions.size();

//#if _VERBOSE_PROFILING
//    }
//    double time_end = rv::utils::getTime();
//    cout << "ContinuousFeaturesPCL::segmentPlanes " << (time_end - time_start)*1000 << " ms. \n";
//#endif

    return n_regions;
}

//template<typename typedata,int m_rows,int m_cols>
//CMatrixFixedNumeric<typedata,m_rows,m_cols> getCMatrix(const Matrix<typedata,m_rows,m_cols> &matrix_eigen)
//{
//    CMatrixFixedNumeric<typedata,m_rows,m_cols> m_CMatrix;
//    for(int r=0; r < matrix_eigen.rows(); r++)
//        for(int c=0; c < matrix_eigen.cols(); c++)
//            m_CMatrix(r,c) = matrix_eigen(r,c);

//    return m_CMatrix;
//}

//// Return a diagonal matrix where the values of the diagonal are assigned from the input vector
//template<typename typedata>
//Matrix<typedata,Dynamic,Dynamic> getDiagonalMatrix(const Matrix<typedata,Dynamic,Dynamic> &matrix_generic)
//{
//    assert(matrix_generic.cols() == matrix_generic.rows());

//    size_t m_size = matrix_generic.cols();
//    Matrix<typedata,Dynamic,Dynamic> m_diag = Matrix<typedata,Dynamic,Dynamic>::Zero(m_size,m_size);
//    for(size_t i=0; i < m_size; i++)
//        m_diag(i,i) = matrix_generic(i,i);

//    return m_diag;
//}


//// Line observation from a 2D LRF.
//// It stores the center of the line segment and a unitary direction vector, together with the covariances of both.
//struct line2D_obs
//{
//    Matrix<double,2,1> center; // The center of the observed line segment
//    Matrix<double,2,2> cov_center;
//    Matrix<double,2,1> dir; // A unitary direction vector
//    Matrix<double,2,2> cov_dir;
//};

//// Line observation from a 2D LRF in 3D coordinates.
//struct line_3D
//{
//    // void line_3D(const line2D_obs &line_2D, const CPose3D &LRF_pose)
//    void get_3D_params(const line2D_obs &line_2D, const CPose3D &LRF_pose)
//    {
//        // center = LRF_pose.getRotationMatrix()*line_2D.center + LRF_pose.m_coords;
//        center_rot = LRF_pose.getRotationMatrix().block(0,0,3,2)*line_2D.center;
//        center = center_rot + LRF_pose.m_coords;
//        cov_center = LRF_pose.getRotationMatrix().block(0,0,3,2)* line_2D.cov_center * LRF_pose.getRotationMatrix().block(0,0,3,2).transpose();
//        dir = LRF_pose.getRotationMatrix().block(0,0,3,2)*line_2D.dir;
//        cov_dir = LRF_pose.getRotationMatrix().block(0,0,3,2) * line_2D.cov_dir * LRF_pose.getRotationMatrix().block(0,0,3,2).transpose();
//    };

//    Matrix<double,3,1> center; // The center of the observed line segment
//    Matrix<double,3,1> center_rot; // The rotated center (translation is not applied)
//    Matrix<double,3,3> cov_center;
//    Matrix<double,3,1> dir; // A unitary direction vector
//    Matrix<double,3,3> cov_dir;
//};

//// Corner Observation by a single LRF
//struct plane_corresp
//{
//    // Two lines observed from the LRF
//    unsigned id_LRF;
//    CArray<line2D_obs,2> lines; // The 2 lines detected by the LRF1
//};

//typedef CArray<CO_1,2> CO; // Corner Observation by 2 LRFs

//typedef CArray<CArray<line_3D,2>,2> CO_3D; // Corner Observation in 3D

////typedef CMatrixTemplateNumeric<double> CO_vector; // Corner Observation by 2 LRFs
////typedef CMatrixFixedNumeric<double,50,1> CO_vector; // Corner Observation by 2 LRFs
//typedef Matrix<double,50,1> CO_vector; // Corner Observation by 2 LRFs

////// The vector form of a CO is needed by RANSAC
////CO_vector CO2vector(const CO &co)
////{
////    CO_vector co_vector;
////    for(size_t LRF_id=0; LRF_id < 2; LRF_id++) // 2 LRFs per CO
////    {
////        co_vector(25*LRF_id,0) = co[LRF_id].id_LRF;
////        for(size_t plane_id=0; plane_id < 2; plane_id++) // 2 line observations per LRF
////        {
////            size_t pos_block = 25*LRF_id + 12*plane_id + 1;
////            co_vector.block(pos_block,0,2,1) = co[LRF_id].lines[plane_id].dir;
////            co_vector.block(pos_block+2,0,2,1) = co[LRF_id].lines[plane_id].cov_dir.block(0,0,2,1);
////            co_vector.block(pos_block+4,0,2,1) = co[LRF_id].lines[plane_id].cov_dir.block(0,1,2,1);
////            //      co_vector.block(pos_block+2,0,4,1) = Map<Matrix<double,4,1>(co[LRF_id].lines[plane_id].cov_center);
////            co_vector.block(pos_block+6,0,2,1) = co[LRF_id].lines[plane_id].center;
////            co_vector.block(pos_block+8,0,2,1) = co[LRF_id].lines[plane_id].cov_center.block(0,0,2,1);
////            co_vector.block(pos_block+10,0,2,1) = co[LRF_id].lines[plane_id].cov_center.block(0,1,2,1);
////        }
////    }

////    return co_vector;
////}

////CO vector2CO(const CO_vector &co_vector)
////{
////    CO co;
////    for(size_t LRF_id=0; LRF_id < 2; LRF_id++) // 2 LRFs per CO
////    {
////        co[LRF_id].id_LRF = co_vector(25*LRF_id,0);
////        for(size_t plane_id=0; plane_id < 2; plane_id++) // 2 line observations per LRF
////        {
////            size_t pos_block = 25*LRF_id + 12*plane_id + 1;
////            co[LRF_id].lines[plane_id].dir = co_vector.block(pos_block,0,2,1);
////            co[LRF_id].lines[plane_id].cov_dir.block(0,0,2,1) = co_vector.block(pos_block+2,0,2,1);
////            co[LRF_id].lines[plane_id].cov_dir.block(0,1,2,1) = co_vector.block(pos_block+4,0,2,1);
////            co[LRF_id].lines[plane_id].center = co_vector.block(pos_block+6,0,2,1);
////            co[LRF_id].lines[plane_id].cov_center.block(0,0,2,1) = co_vector.block(pos_block+8,0,2,1);
////            co[LRF_id].lines[plane_id].cov_center.block(0,1,2,1) = co_vector.block(pos_block+10,0,2,1);
////        }
////    }

////    return co;
////}


void print_help(char ** argv)
{
    cout << "This program computes the extrinsic calibration of several RGB-D sensors (e.g. Kinect) based on plane and line correspondences." << endl
         << "No overlapping required for the sensor FOV, as far as the planes/lines can be observed simultaneously by pairs of sensors." << endl
         << "This program accepts a single argument specifying a configuration file which indicates the rawlog file of sensor observations" << endl
         << "to compute the calibration, the approximated sensor v_pose, and a conservative approximation of their accuracy. The output text" << endl
         << "file with the calibration results, and display/verbose parameters can also be set in the configuration.\n" << endl;

    cout << "Usage: " << argv[0] << " <config_file> \n";
    cout << "    <config_file> configuration file which contains the information of the RGBD sequences and estimated calibration" << endl;
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
        bool showHelp = argc != 2 || !os::_strcmp(argv[1],"-h") || !os::_strcmp(argv[1],"--help");
        if(showHelp)
        {
            print_help(argv);
            mrpt::system::pause();
            return -1;
        }

        //Initial steps. Load configuration file
        //-------------------------------------------
        const string config_file = string( argv[1] );
        ASSERT_FILE_EXISTS_(config_file)
        utils::CConfigFile cfg(config_file);
        string rawlog_file = cfg.read_string("GLOBAL", "rawlog_file", "no file", true);
        string output_dir = cfg.read_string("GLOBAL", "output_dir", "no dir", true);
        size_t decimation = cfg.read_int("GLOBAL", "decimation", 0, true);
        bool display = cfg.read_bool("GLOBAL", "display", 0, true);
        bool verbose = cfg.read_bool("GLOBAL", "verbose", 0, true);
        cout << "dataset: " << rawlog_file << "\noutput: " << rawlog_file << "\ndecimation: " << decimation << endl;
        vector<string> sensor_labels;
        cfg.getAllSections(sensor_labels);
        sensor_labels.erase(sensor_labels.begin()); // Remove the GLOBAL section
        size_t num_sensors = sensor_labels.size();
        cout << "num_sensors: " << num_sensors << endl;
        vector<CPose3D> v_pose(num_sensors);
        vector<float> v_approx_trans(num_sensors);
        vector<float> v_approx_rot(num_sensors);
        for(size_t i=0; i < num_sensors; i++) // Load the approximated v_pose of each sensor and the accuracy of such approximation
        {
            //cfg.read_matrix(sensor_labels[i],"pose",v_pose[i]); // Matlab's matrix format
            v_pose[i] = CPose3D(
                cfg.read_double(sensor_labels[i],"pose_x",0,true),
                cfg.read_double(sensor_labels[i],"pose_y",0,true),
                cfg.read_double(sensor_labels[i],"pose_z",0,true),
                DEG2RAD( cfg.read_double(sensor_labels[i],"pose_yaw",0,true) ),
                DEG2RAD( cfg.read_double(sensor_labels[i],"pose_pitch",0,true) ),
                DEG2RAD( cfg.read_double(sensor_labels[i],"pose_roll",0,true) ) );
            v_approx_trans[i] = cfg.read_float(sensor_labels[i],"approx_translation",0.f,true);
            v_approx_rot[i] = cfg.read_float(sensor_labels[i],"approx_rotation",0.f,true);
            //==============================================================================
            //	TODO		*********				Load stereo calibration!!!
            //==============================================================================
            cout << sensor_labels[i] << " v_approx_trans " << v_approx_trans[i] << " v_approx_rot " << v_approx_rot[i] << "\n" << v_pose[i] << endl;
        }

        //initializeScene();

        //						Open Rawlog File
        //==================================================================
        mrpt::obs::CRawlog dataset;
        if (!dataset.loadFromRawLogFile(rawlog_file))
            throw std::runtime_error("\nCouldn't open rawlog dataset file for input...");

//        // Set external images directory:
//        const string imgsPath = CRawlog::detectImagesDirectory(filename);
//        CImage::IMAGES_PATH_BASE = imgsPath;

        cout << "dataset size " << dataset.size() << "\n";
        //dataset_count = 0;

        vector<mrpt::obs::CObservation3DRangeScanPtr> obsRGBD(num_sensors);  // The RGBD observation
        vector<bool> obs_sensor(num_sensors,false);
        vector<double> obs_sensor_time(num_sensors);
        //CObservation2DRangeScanPtr laserObs;    // Pointer to the laser observation
        size_t n_obs = 0, n_frame_sync = 0;
        CObservationPtr observation;

        //==============================================================================
        //									Main operation
        //==============================================================================

        // Plane segmentation parameters
        const float dist_threshold = 0.04;
        const float angle_threshold = 0.07f;
        const size_t min_inliers = 500;

        while ( n_obs < dataset.size() )
        {
            observation = dataset.getAsObservation(n_obs);
            cout << n_obs << " observation: " << observation->sensorLabel << endl; //<< ". Timestamp " << timestampTotime_t(observation->timestamp) << endl;
            ++n_obs;
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
            //cout << sensor_id << " diff " << timestampTotime_t(observation->timestamp)-obs_sensor_time[sensor_id] << endl;
            obs_sensor_time[sensor_id] = timestampTotime_t(observation->timestamp);

            obsRGBD[sensor_id] = CObservation3DRangeScanPtr(observation);
            obsRGBD[sensor_id]->load();

            bool all_obs = obs_sensor[0];
            for(size_t i=1; i < num_sensors; i++)
                all_obs = all_obs && obs_sensor[i];


            if(all_obs)
            {
                for(size_t i=1; i < num_sensors; i++)
                    cout << i << " time diff to ref " << obs_sensor_time[i]-obs_sensor_time[0] << endl;

                // Check for aproximate synchronization
                bool is_synch = true;
                for(size_t i=0; i < num_sensors; i++)
                {
                    for(size_t j=i; j < num_sensors; j++)
                    {
                        if(fabs(obs_sensor_time[i]-obs_sensor_time[j]) > 0.03) // maximum de-synchronization in seconds
                        {
                            is_synch = false;
                            if( (obs_sensor_time[j]-obs_sensor_time[i]) > 0 )
                                obs_sensor[i] = false;
                            else
                                obs_sensor[j] = false;
                            break;
                        }
                    }
                    if(!is_synch)
                        break;
                }
                if(!is_synch)
                    continue;

                ++n_frame_sync;
                for(size_t i=0; i < num_sensors; i++)
                    obs_sensor[i] = false;

                cout << n_frame_sync << " frames\n";

                // Apply decimation
                if( n_frame_sync % decimation != 0)
                    continue;

                //						Segment local planes
                //==================================================================
                vector<pcl::PointCloud<PointT>::Ptr> v_cloud(num_sensors);
                #pragma omp parallel num_threads(num_sensors)
                {
                    sensor_id = omp_get_thread_num();
                    v_cloud[sensor_id].reset(new pcl::PointCloud<PointT>);
//                    obsRGBD[sensor_id]->project3DPointsFromDepthImageInto(*(v_cloud[sensor_id]), false /* without obs.sensorPose */);

//                    CSimplePointsMap pntsMap;
                    CColouredPointsMap pntsMap;
//                    obsRGBD[sensor_id]->project3DPointsFromDepthImageInto(pntsMap, false /* without obs.sensorPose */);
                    pntsMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
                    pntsMap.loadFromRangeScan(*obsRGBD[sensor_id]);
//                    pntsMap.getPCLPointCloudXYZ(*v_cloud[sensor_id]);
                    pntsMap.getPCLPointCloudXYZRGB(*v_cloud[sensor_id]);
                    cout << sensor_id << " v_cloud " << v_cloud[sensor_id]->height << "x" << v_cloud[sensor_id]->width << endl;

//                    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//                    viewer.showCloud(v_cloud[sensor_id]);
//                    while (!viewer.wasStopped ())
//                        boost::this_thread::sleep (boost::posix_time::milliseconds (10));


//                    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
//                    std::vector<pcl::ModelCoefficients> model_coefficients;
//                    std::vector<pcl::PointIndices> inliers;
//                    size_t n_planes = segmentPlanes(v_cloud[sensor_id], regions, model_coefficients, inliers, dist_threshold, angle_threshold, min_inliers);
//                    cout << sensor_id << "n_planes " << n_planes << endl;

                }



//                cout << "frame360.segmentPlanesLocal() \n";
//                //frame360.segmentPlanesLocal();
//                for(int sensor_id = 0; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
//                    frame360.local_planes_[sensor_id] = frame360.extractPbMap( frame360.getCloudRGBDAsus(sensor_id).getPointCloud(), 0.02f, 0.039812f, 5000 );

//                cout << "Merge planes \n";
//                mrpt::pbmap::PbMap &planes = frame360.planes_;
//                //planes.vPlanes.clear();
//                vector<unsigned> planesSourceIdx(5, 0);
//                for(int sensor_id = 0; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
//                {
//                    planes.MergeWith(frame360.local_planes_[sensor_id], calib.Rt_[sensor_id]);
//                    planesSourceIdx[sensor_id+1] = planesSourceIdx[sensor_id] + frame360.local_planes_[sensor_id].vPlanes.size();
//                    //          cout << planesSourceIdx[sensor_id+1] << " ";
//                }

//                if(display)
                    //Show both images and the plane segmentation
            }
        }

        // Run:
        //calib_LRFs_rawlog_ini(config_file,override_rawlog_file);

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
