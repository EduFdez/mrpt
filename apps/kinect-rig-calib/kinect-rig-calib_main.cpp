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

// 0 load init file with estimated init_poses and errors. display/verbose parameters    -OK
// 1 open rawlog                                                                    -OK
// 2 identify observation pairs/triples...                                          -OK
// 3 segment planes and lines
// 4 get correspondences
// 5 perform calibration. calibration algorithm in a different file
// 6 visualize and evaluate

#include <numeric>
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
#include <mrpt/utils/TStereoCamera.h>
#include <mrpt/utils/CConfigFile.h>
//#include <mrpt/utils/CFileGZInputStream.h>
////#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
//#include <mrpt/system/threads.h>
#include <mrpt/system/filesystem.h>
//#include <mrpt/opengl/CPlanarLaserScan.h>  // This class lives in the lib [mrpt-maps] and must be included by hand
//#include <mrpt/math/ransac_applications.h>
#include <mrpt/vision/chessboard_stereo_camera_calib.h>
#include <mrpt/pbmap/PbMap.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/common/transforms.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <omp.h>

#include "DownsampleRGBD.h"
#include "calib_from_planes3D.h"
#include "extrinsic_calib_pair_xtion.h"

using namespace mrpt;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::pbmap;
using namespace std;
using namespace Eigen;


typedef pcl::PointXYZRGBA PointT;

// Define some colours to draw bolobs, patches, etc.
static const unsigned char red [10] = {255,   0,   0, 255, 255,   0, 255, 204,   0, 255};
static const unsigned char grn [10] = {  0, 255,   0, 255,   0, 255, 160,  51, 128, 222};
static const unsigned char blu [10] = {  0,   0, 255,   0, 255, 255, 0  , 204,   0, 173};

static const double ared [10] = {1.0,   0,   0, 1.0, 1.0,   0, 1.0, 0.8,   0, 1.0};
static const double agrn [10] = {  0, 1.0,   0, 1.0,   0, 1.0, 0.6, 0.2, 0.5, 0.9};
static const double ablu [10] = {  0,   0, 1.0,   0, 1.0, 1.0,   0, 0.8,   0, 0.7};

void convertRange_mrpt2cvMat(const mrpt::math::CMatrix & range_mrpt, cv::Mat & depthImage)
{
    Eigen::MatrixXf range_eigen(range_mrpt);
    cv::eigen2cv(range_eigen, depthImage);
}

Eigen::Matrix4f getPoseEigen(const mrpt::poses::CPose3D & pose)
{
    Eigen::Matrix4f pose_eigen;
    mrpt::math::CMatrixDouble44 pose_mrpt;
    pose.getHomogeneousMatrix(pose_mrpt);
    pose_eigen << pose_mrpt(0,0), pose_mrpt(0,1), pose_mrpt(0,2), pose_mrpt(0,3),
                  pose_mrpt(1,0), pose_mrpt(1,1), pose_mrpt(1,2), pose_mrpt(1,3),
                  pose_mrpt(2,0), pose_mrpt(2,1), pose_mrpt(2,2), pose_mrpt(2,3),
                  pose_mrpt(3,0), pose_mrpt(3,1), pose_mrpt(3,2), pose_mrpt(3,3) ;
    return pose_eigen;
}

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}


// Obtain the rigid transformation from 3 matched planes
CMatrixDouble getAlignment( const CMatrixDouble &matched_planes )
{
  ASSERT_(size(matched_planes,1) == 8 && size(matched_planes,2) == 3);

  //Calculate rotation
  Matrix3f normalCovariances = Matrix3f::Zero();
  normalCovariances(0,0) = 1;
  for(unsigned i=0; i<3; i++)
  {
    Vector3f n_i = Vector3f(matched_planes(0,i), matched_planes(1,i), matched_planes(2,i));
    Vector3f n_ii = Vector3f(matched_planes(4,i), matched_planes(5,i), matched_planes(6,i));
    normalCovariances += n_i * n_ii.transpose();
//    normalCovariances += matched_all_planes.block(i,0,1,3) * matched_all_planes.block(i,4,1,3).transpose();
  }

  JacobiSVD<MatrixXf> svd(normalCovariances, ComputeThinU | ComputeThinV);
  Matrix3f Rotation = svd.matrixV() * svd.matrixU().transpose();

//  float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
//  if(conditioning > 100)
//  {
//    cout << " ConsistencyTest::initPose -> Bad conditioning: " << conditioning << " -> Returning the identity\n";
//    return Eigen::Matrix4f::Identity();
//  }

  double det = Rotation.determinant();
  if(det != 1)
  {
    Eigen::Matrix3f aux;
    aux << 1, 0, 0, 0, 1, 0, 0, 0, det;
    Rotation = svd.matrixV() * aux * svd.matrixU().transpose();
  }


  // Calculate translation
  Vector3f translation;
  Matrix3f hessian = Matrix3f::Zero();
  Vector3f gradient = Vector3f::Zero();
  hessian(0,0) = 1;
  for(unsigned i=0; i<3; i++)
  {
    float trans_error = (matched_planes(3,i) - matched_planes(7,i)); //+n*t
//    hessian += matched_all_planes.block(i,0,1,3) * matched_all_planes.block(i,0,1,3).transpose();
//    gradient += matched_all_planes.block(i,0,1,3) * trans_error;
    Vector3f n_i = Vector3f(matched_planes(0,i), matched_planes(1,i), matched_planes(2,i));
    hessian += n_i * n_i.transpose();
    gradient += n_i * trans_error;
  }
  translation = -hessian.inverse() * gradient;
//cout << "Previous average translation error " << sumError / matched_all_planes.size() << endl;

//  // Form SE3 transformation matrix. This matrix maps the model into the current data reference frame
//  Eigen::Matrix4f rigidTransf;
//  rigidTransf.block(0,0,3,3) = Rotation;
//  rigidTransf.block(0,3,3,1) = translation;
//  rigidTransf.row(3) << 0,0,0,1;

  CMatrixDouble rigidTransf(4,4);
  rigidTransf(0,0) = Rotation(0,0);
  rigidTransf(0,1) = Rotation(0,1);
  rigidTransf(0,2) = Rotation(0,2);
  rigidTransf(1,0) = Rotation(1,0);
  rigidTransf(1,1) = Rotation(1,1);
  rigidTransf(1,2) = Rotation(1,2);
  rigidTransf(2,0) = Rotation(2,0);
  rigidTransf(2,1) = Rotation(2,1);
  rigidTransf(2,2) = Rotation(2,2);
  rigidTransf(0,3) = translation(0);
  rigidTransf(1,3) = translation(1);
  rigidTransf(2,3) = translation(2);
  rigidTransf(3,0) = 0;
  rigidTransf(3,1) = 0;
  rigidTransf(3,2) = 0;
  rigidTransf(3,3) = 1;

  return rigidTransf;
}

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

    return n_regions;
}


// Ransac functions to detect outliers in the plane matching
void ransacPlaneAlignment_fit(
        const CMatrixDouble &planeCorresp,
        const mrpt::vector_size_t  &useIndices,
        vector< CMatrixDouble > &fitModels )
//        vector< Eigen::Matrix4f > &fitModels )
{
  ASSERT_(useIndices.size()==3);

  try
  {
    CMatrixDouble corresp(8,3);

//  cout << "Size planeCorresp: " << endl;
//  cout << "useIndices " << useIndices[0] << " " << useIndices[1]  << " " << useIndices[2] << endl;
    for(unsigned i=0; i<3; i++)
      corresp.col(i) = planeCorresp.col(useIndices[i]);

    fitModels.resize(1);
//    Eigen::Matrix4f &M = fitModels[0];
    CMatrixDouble &M = fitModels[0];
    M = getAlignment(corresp);
  }
  catch(exception &)
  {
    fitModels.clear();
    return;
  }
}

void ransac3Dplane_distance(
        const CMatrixDouble &planeCorresp,
        const vector< CMatrixDouble > & testModels,
        const double distanceThreshold,
        unsigned int & out_bestModelIndex,
        mrpt::vector_size_t & out_inlierIndices )
{
  ASSERT_( testModels.size()==1 )
  out_bestModelIndex = 0;
  const CMatrixDouble &M = testModels[0];

  Eigen::Matrix3f Rotation; Rotation << M(0,0), M(0,1), M(0,2), M(1,0), M(1,1), M(1,2), M(2,0), M(2,1), M(2,2);
  Eigen::Vector3f translation; translation << M(0,3), M(1,3), M(2,3);

    ASSERT_( size(M,1)==4 && size(M,2)==4 )

  const size_t N = size(planeCorresp,2);
  out_inlierIndices.clear();
  out_inlierIndices.reserve(100);
  for (size_t i=0;i<N;i++)
  {
    const Eigen::Vector3f n_i = Eigen::Vector3f(planeCorresp(0,i), planeCorresp(1,i), planeCorresp(2,i));
    const Eigen::Vector3f n_ii = Rotation * Eigen::Vector3f(planeCorresp(4,i), planeCorresp(5,i), planeCorresp(6,i));
    const float d_error = fabs((planeCorresp(7,i) - translation.dot(n_i)) - planeCorresp(3,i));
    const float angle_error = (n_i .cross (n_ii )).norm();

    if (d_error < distanceThreshold)
     if (angle_error < distanceThreshold) // Warning: this threshold has a different dimension
      out_inlierIndices.push_back(i);
  }
}

/** Return "true" if the selected points are a degenerate (invalid) case.
  */
bool ransac3Dplane_degenerate(
        const CMatrixDouble &planeCorresp,
        const mrpt::vector_size_t &useIndices )
{
  ASSERT_( useIndices.size()==3 )

  const Eigen::Vector3f n_1 = Eigen::Vector3f(planeCorresp(0,useIndices[0]), planeCorresp(1,useIndices[0]), planeCorresp(2,useIndices[0]));
  const Eigen::Vector3f n_2 = Eigen::Vector3f(planeCorresp(0,useIndices[1]), planeCorresp(1,useIndices[1]), planeCorresp(2,useIndices[1]));
  const Eigen::Vector3f n_3 = Eigen::Vector3f(planeCorresp(0,useIndices[2]), planeCorresp(1,useIndices[2]), planeCorresp(2,useIndices[2]));
//cout << "degenerate " << useIndices[0] << " " << useIndices[1]  << " " << useIndices[2] << " - " << fabs(n_1. dot( n_2. cross(n_3) ) ) << endl;

  if( fabs(n_1. dot( n_2. cross(n_3) ) ) < 0.9 )
    return true;

  return false;
}


//pcl::PointCloud<PointT>::Ptr pointCloudPtr getPointCloudRegistered(cv::Mat & rgb_img, cv::Mat & depth_img, mrpt::vision::TStereoCalibResults & claib) // mrpt::utils::TStereoCamera
pcl::PointCloud<PointT>::Ptr getPointCloud(cv::Mat & rgb_img, cv::Mat & depth_img, mrpt::utils::TStereoCamera & calib)
{
    const int height = rgb_img.rows;
    const int width = rgb_img.cols;

    const float fx = 525.f;
    const float fy = 525.f;
    const float ox = 319.5f;
    const float oy = 239.5f;
//    const float fx = calib.rightCamera.fx();
//    const float fy = calib.rightCamera.fy();
//    const float ox = calib.rightCamera.cx();
//    const float oy = calib.rightCamera.cy();
    const float inv_fx = 1.f/fx;
    const float inv_fy = 1.f/fy;

    pcl::PointCloud<PointT>::Ptr pointCloudPtr (new pcl::PointCloud<PointT>());
    pointCloudPtr->height = height;
    pointCloudPtr->width = width;
    pointCloudPtr->is_dense = false;
    pointCloudPtr->points.resize(height*width);

    const float minDepth = 0.3;
    const float maxDepth = 10.0;
    if(depth_img.type() == CV_16U) // The image pixels are presented in millimetres
    {
        float minDepth_ = minDepth*1000;
        float maxDepth_ = maxDepth*1000;
//    #if ENABLE_OPENMP_MULTITHREADING_FrameRGBD
//    #pragma omp parallel for
//    #endif
        for( int y = 0; y < height; y++ )
        {
            unsigned short *_depth = depth_img.ptr<unsigned short>(0) + y*width;
            for( int x = 0; x < width; x++ )
            {
                cv::Vec3b& bgr = rgb_img.at<cv::Vec3b>(y,x);
                pointCloudPtr->points[width*y+x].r = bgr[2];
                pointCloudPtr->points[width*y+x].g = bgr[1];
                pointCloudPtr->points[width*y+x].b = bgr[0];

                float z = (*_depth++)*0.001f;
//                    std::cout << "Build " << z << " from " << depth_img.at<unsigned short>(y,x) << std::endl;
                //if(z>0 && z>=minDepth_ && z<=maxDepth_) //If the point has valid depth information assign the 3D point to the point cloud
                if(z>=minDepth_ && z<=maxDepth_) //If the point has valid depth information assign the 3D point to the point cloud
                {
                    pointCloudPtr->points[width*y+x].x = (x - ox) * z * inv_fx;
                    pointCloudPtr->points[width*y+x].y = (y - oy) * z * inv_fy;
                    pointCloudPtr->points[width*y+x].z = z;
                }
                else //else, assign a NAN value
                {
//                        std::cout << z << " ";
//                        std::cout << depth_img.at<unsigned short>(y,x) << " " << std::endl;
                    pointCloudPtr->points[width*y+x].x = std::numeric_limits<float>::quiet_NaN ();
                    pointCloudPtr->points[width*y+x].y = std::numeric_limits<float>::quiet_NaN ();
                    pointCloudPtr->points[width*y+x].z = std::numeric_limits<float>::quiet_NaN ();
                }
            }
        }
    }
    else if(depth_img.type() == CV_32F) // The image pixels are presented in metres
    {
//        #if ENABLE_OPENMP_MULTITHREADING_FrameRGBD
//        #pragma omp parallel for
//        #endif
//            int non_zeros;
        for( int y = 0; y < height; y++ )
        {
            //float *_depth = depth_img.ptr<float>(0) + y*width;
            for( int x = 0; x < width; x++ )
            {
                cv::Vec3b& bgr = rgb_img.at<cv::Vec3b>(y,x);
                pointCloudPtr->points[width*y+x].r = bgr[2];
                pointCloudPtr->points[width*y+x].g = bgr[1];
                pointCloudPtr->points[width*y+x].b = bgr[0];

                //float z = *_depth++;
                float z = depth_img.at<float>(y,x); //convert from milimeters to meters
                //std::cout << "Build " << z << std::endl;
//                    std::cout << "Build " << z << " from " << depth_img.at<float>(y,x) << std::endl;
                //if(z>0 && z>=minDepth && z<=maxDepth) //If the point has valid depth information assign the 3D point to the point cloud

                if(z>=minDepth && z<=maxDepth) //If the point has valid depth information assign the 3D point to the point cloud
                {
                    pointCloudPtr->points[width*y+x].x = (x - ox) * z * inv_fx;
                    pointCloudPtr->points[width*y+x].y = (y - oy) * z * inv_fy;
                    //pointCloudPtr->points[width*y+x].x = -(y - oy) * z * inv_fy;
                    //pointCloudPtr->points[width*y+x].y = (x - ox) * z * inv_fx;
                    pointCloudPtr->points[width*y+x].z = z;
                    //std::cout << y << "x" << x << " pixel pt " << pointCloudPtr->points[width*y+x].x << " " << pointCloudPtr->points[width*y+x].y << " " << pointCloudPtr->points[width*y+x].z << std::endl;
//                        non_zeros++;
                }
                else //else, assign a NAN value
                {
//                        std::cout << z << " ";
                    pointCloudPtr->points[width*y+x].x = std::numeric_limits<float>::quiet_NaN ();
                    pointCloudPtr->points[width*y+x].y = std::numeric_limits<float>::quiet_NaN ();
                    pointCloudPtr->points[width*y+x].z = std::numeric_limits<float>::quiet_NaN ();
                }
            }
        }
//            std::cout << non_zeros << std::endl;
    }

    return pointCloudPtr;
}

pcl::PointCloud<PointT>::Ptr getPointCloudRegistered(cv::Mat & rgb_img, cv::Mat & depth_img, mrpt::utils::TStereoCamera & calib)
{
    const int height = rgb_img.rows;
    const int width = rgb_img.cols;

    const float fx = calib.leftCamera.fx();
    const float fy = calib.leftCamera.fy();
    const float ox = calib.leftCamera.cx();
    const float oy = calib.leftCamera.cy();
    const float inv_fx = 1.f/fx;
    const float inv_fy = 1.f/fy;

    const float fxc = calib.rightCamera.fx();
    const float fyc = calib.rightCamera.fy();
    const float oxc = calib.rightCamera.cx();
    const float oyc = calib.rightCamera.cy();
    pcl::PointCloud<PointT>::Ptr pointCloudPtr (new pcl::PointCloud<PointT>());
    pointCloudPtr->height = height;
    pointCloudPtr->width = width;
    pointCloudPtr->is_dense = false;
    pointCloudPtr->points.resize(height*width);

    const float minDepth = 0.3;
    const float maxDepth = 10.0;
    if(depth_img.type() == CV_16U) // The image pixels are presented in millimetres
    {
        float minDepth_ = minDepth*1000;
        float maxDepth_ = maxDepth*1000;
//    #if ENABLE_OPENMP_MULTITHREADING_FrameRGBD
//    #pragma omp parallel for
//    #endif
        for( int y = 0; y < height; y++ )
        {
            unsigned short *_depth = depth_img.ptr<unsigned short>(0) + y*width;
            for( int x = 0; x < width; x++ )
            {
//                    cv::Vec3b& bgr = rgb_img.at<cv::Vec3b>(y,x);
//                    pointCloudPtr->points[width*y+x].r = bgr[2];
//                    pointCloudPtr->points[width*y+x].g = bgr[1];
//                    pointCloudPtr->points[width*y+x].b = bgr[0];

                float z = (*_depth++)*0.001f;
//                    std::cout << "Build " << z << " from " << depth_img.at<unsigned short>(y,x) << std::endl;
                //if(z>0 && z>=minDepth_ && z<=maxDepth_) //If the point has valid depth information assign the 3D point to the point cloud
                if(z>=minDepth_ && z<=maxDepth_) //If the point has valid depth information assign the 3D point to the point cloud
                {
                    pointCloudPtr->points[width*y+x].x = (x - ox) * z * inv_fx;
                    pointCloudPtr->points[width*y+x].y = (y - oy) * z * inv_fy;
                    pointCloudPtr->points[width*y+x].z = z;
                }
                else //else, assign a NAN value
                {
//                        std::cout << z << " ";
//                        std::cout << depth_img.at<unsigned short>(y,x) << " " << std::endl;
                    pointCloudPtr->points[width*y+x].x = std::numeric_limits<float>::quiet_NaN ();
                    pointCloudPtr->points[width*y+x].y = std::numeric_limits<float>::quiet_NaN ();
                    pointCloudPtr->points[width*y+x].z = std::numeric_limits<float>::quiet_NaN ();
                }
            }
        }
    }
    else if(depth_img.type() == CV_32F) // The image pixels are presented in metres
    {
//        #if ENABLE_OPENMP_MULTITHREADING_FrameRGBD
//        #pragma omp parallel for
//        #endif
//            int non_zeros;
        for( int y = 0; y < height; y++ )
        {
            //float *_depth = depth_img.ptr<float>(0) + y*width;
            for( int x = 0; x < width; x++ )
            {
//                    cv::Vec3b& bgr = rgb_img.at<cv::Vec3b>(y,x);
//                    pointCloudPtr->points[width*y+x].r = bgr[2];
//                    pointCloudPtr->points[width*y+x].g = bgr[1];
//                    pointCloudPtr->points[width*y+x].b = bgr[0];

                //float z = *_depth++;
                float z = depth_img.at<float>(y,x); //convert from milimeters to meters
                //std::cout << "Build " << z << std::endl;
//                    std::cout << "Build " << z << " from " << depth_img.at<float>(y,x) << std::endl;
                //if(z>0 && z>=minDepth && z<=maxDepth) //If the point has valid depth information assign the 3D point to the point cloud

                if(z>=minDepth && z<=maxDepth) //If the point has valid depth information assign the 3D point to the point cloud
                {
                    pointCloudPtr->points[width*y+x].x = (x - ox) * z * inv_fx;
                    pointCloudPtr->points[width*y+x].y = (y - oy) * z * inv_fy;
                    //pointCloudPtr->points[width*y+x].x = -(y - oy) * z * inv_fy;
                    //pointCloudPtr->points[width*y+x].y = (x - ox) * z * inv_fx;
                    pointCloudPtr->points[width*y+x].z = z;
                    //std::cout << y << "x" << x << " pixel pt " << pointCloudPtr->points[width*y+x].x << " " << pointCloudPtr->points[width*y+x].y << " " << pointCloudPtr->points[width*y+x].z << std::endl;
//                        non_zeros++;
                }
                else //else, assign a NAN value
                {
//                        std::cout << z << " ";
                    pointCloudPtr->points[width*y+x].x = std::numeric_limits<float>::quiet_NaN ();
                    pointCloudPtr->points[width*y+x].y = std::numeric_limits<float>::quiet_NaN ();
                    pointCloudPtr->points[width*y+x].z = std::numeric_limits<float>::quiet_NaN ();
                }
            }
        }
//            std::cout << non_zeros << std::endl;
    }

    pcl::PointCloud<PointT>::Ptr pointCloudPtr2 (new pcl::PointCloud<PointT>());
    Eigen::Matrix4f pose_rgb2depth = getPoseEigen(CPose3D(-calib.rightCameraPose));
    pcl::transformPointCloud(*pointCloudPtr, *pointCloudPtr2, pose_rgb2depth);
    //cout << "pose_rgb2depth \n" << pose_rgb2depth << endl;
//        int non_zero = 0;
//        cv::Mat depthImage(depth_img.rows, depth_img.cols,CV_16UC1,cv::Scalar(0));
    cv::Mat depthImage(rgb_img.rows, rgb_img.cols, CV_32FC1, cv::Scalar(0.0));

//        std::set<int> set_x, set_y;
//        std::set<int> set_pt;
    for( int y = 0; y < height; y++ )
    {
        for( int x = 0; x < width; x++ )
        {
            float x_, y_, z_;
            x_ = pointCloudPtr2->points[width*y+x].x;
            y_ = pointCloudPtr2->points[width*y+x].y;
            z_ = pointCloudPtr2->points[width*y+x].z;
//std::cout << "x_: " << x_ << " y_: " << y_ << " z_: " << z_ << std::endl;

            int x_coor = floor(x_*fxc/z_+oxc);
            int y_coor = floor(y_*fyc/z_+oyc);
//std::cout << x << " " << y << " " << x_coor << " " << y_coor << std::endl;
            if ((x_coor>=0)&&(x_coor<depthImage.cols)&&(y_coor>=0)&&(y_coor<depthImage.rows))
            {
                depthImage.at<float>(y_coor,x_coor) = z_;//(ushort) (z_*1000.0);

                cv::Vec3b& bgr = rgb_img.at<cv::Vec3b>(y_coor,x_coor);
                pointCloudPtr2->points[width*y+x].r = bgr[2];
                pointCloudPtr2->points[width*y+x].g = bgr[1];
                pointCloudPtr2->points[width*y+x].b = bgr[0];
            }
        }
    }

    depth_img = depthImage;

    return pointCloudPtr2;
}


/*! This class calibrates the extrinsic parameters of the omnidirectional RGB-D sensor. For that, the sensor is accessed
 *  and big planes are segmented and matched between different single sensors.
*/
class ExtrinsicRgbdCalibration
{
  private:

    boost::mutex visualizationMutex;

    // Sensor parameters
    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > Rt_estimated;
    vector<float> weight_pair;
    size_t num_sensors;
    vector<string> sensor_labels;
    vector<mrpt::utils::TStereoCamera> rgbd_intrinsics;
    vector<CPose3D> init_poses;
    vector<float> v_approx_trans;
    vector<float> v_approx_rot;

    // Observation parameters
    mrpt::pbmap::PbMap all_planes;
    vector<mrpt::pbmap::PbMap> v_pbmap;
    vector<pcl::PointCloud<PointT>::Ptr> cloud;
//    mrpt::pbmap::PbMap planes_i, planes_j;
//    pcl::PointCloud<PointT>::Ptr cloud[0], cloud[1];
    Eigen::Matrix4f initOffset;
    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > initOffsets;

    // Plane segmentation parameters
    const float dist_threshold = 0.02;
    const float angle_threshold = 0.05f;
    const size_t min_inliers = 2e4;
    /*! Maximum curvature to consider the region as planar */
    const float max_curvature_plane = 0.0013;

    bool b_select_manually;
    bool bFreezeFrame;
    bool bExit;

    string rawlog_file;
    string output_dir;
    size_t decimation;
    bool display;
    bool verbose;

    Eigen::Matrix3f covariance;
    float conditioning;
    unsigned valid_obs;

    map<unsigned, unsigned> plane_corresp;

  public:
    ExtrinsicRgbdCalibration() :
              b_select_manually(false),
              bFreezeFrame(false),
              bExit(false)
    {
//      correspondences.resize(8); // 8 pairs of RGBD sensors
//      correspondences_2.resize(8); // 8 pairs of RGBD sensors
//      std::fill(conditioning, conditioning+8, 9999.9);
//      std::fill(weight_pair, weight_pair+8, 0.0);
//      std::fill(valid_obs, valid_obs+8, 0);
//      std::fill(covariances, covariances+8, Eigen::Matrix3f::Zero());
//      calib.loadIntrinsicCalibration();
//      calib.loadExtrinsicCalibration();
      conditioning = 9999.9;
      valid_obs = 0;
      covariance = Eigen::Matrix3f::Zero();
    }

    void loadConfiguration(const string & config_file)
    {
        //Initial steps. Load configuration file
        //-------------------------------------------
        utils::CConfigFile cfg(config_file);
        rawlog_file = cfg.read_string("GLOBAL", "rawlog_file", "no file", true);
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
        num_sensors = sensor_labels.size();

        if(verbose)
        {
            for(size_t i=0; i < num_sensors; i++) // Load the approximated init_poses of each sensor and the accuracy of such approximation
                cout << "sensor: " << sensor_labels[i] << endl;
            cout << "num_sensors: " << num_sensors << endl;
        }

        Rt_estimated = vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >(num_sensors, Eigen::Matrix4f::Identity());
        rgbd_intrinsics = vector<mrpt::utils::TStereoCamera>(num_sensors);
        init_poses = vector<CPose3D>(num_sensors);
        v_approx_trans = vector<float>(num_sensors);
        v_approx_rot = vector<float>(num_sensors);
        cloud.resize(num_sensors);
        v_pbmap.resize(num_sensors);
        for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++) // Load the approximated init_poses of each sensor and the accuracy of such approximation
        {
            //cfg.read_matrix(sensor_labels[sensor_id],"pose",init_poses[sensor_id]); // Matlab's matrix format
            init_poses[sensor_id] = CPose3D(
                        cfg.read_double(sensor_labels[sensor_id],"pose_x",0,true),
                        cfg.read_double(sensor_labels[sensor_id],"pose_y",0,true),
                        cfg.read_double(sensor_labels[sensor_id],"pose_z",0,true),
                        DEG2RAD( cfg.read_double(sensor_labels[sensor_id],"pose_yaw",0,true) ),
                        DEG2RAD( cfg.read_double(sensor_labels[sensor_id],"pose_pitch",0,true) ),
                        DEG2RAD( cfg.read_double(sensor_labels[sensor_id],"pose_roll",0,true) ) );
            Rt_estimated[sensor_id] = getPoseEigen(init_poses[sensor_id]);

            v_approx_trans[sensor_id] = cfg.read_float(sensor_labels[sensor_id],"approx_translation",0.f,true);
            v_approx_rot[sensor_id] = cfg.read_float(sensor_labels[sensor_id],"approx_rotation",0.f,true);
            cout << sensor_labels[sensor_id] << " v_approx_trans " << v_approx_trans[sensor_id] << " v_approx_rot " << v_approx_rot[sensor_id] << "\n" << init_poses[sensor_id] << endl;
            cout << Rt_estimated[sensor_id] << endl;

            rgbd_intrinsics[sensor_id].loadFromConfigFile(sensor_labels[sensor_id],cfg);
            //            string calib_path = mrpt::system::extractFileDirectory(rawlog_file) + "asus_" + std::to_string(sensor_id+1) + "/calib.ini";
            //            mrpt::utils::CConfigFile calib_RGBD(calib_path);
            //            rgbd_intrinsics[sensor_id].loadFromConfigFile("CAMERA_PARAMS", calib_RGBD);
            //            cout << "right2left_camera_pose \n" << rgbd_intrinsics[sensor_id].rightCameraPose << endl;

            //            cout << sensor_labels[sensor_id] << "RGB params: fx=" << rgbd_intrinsics[sensor_id].rightCamera.fx() << " fy=" << rgbd_intrinsics[sensor_id].rightCamera.fy() << " cx=" << rgbd_intrinsics[sensor_id].rightCamera.cx() << " cy=" << rgbd_intrinsics[sensor_id].rightCamera.cy() << endl;
            //            cout << sensor_labels[sensor_id] << "RGB params: fx=" << rgbd_intrinsics2[sensor_id].cam_params.rightCamera.fx() << " fy=" << rgbd_intrinsics2[sensor_id].cam_params.rightCamera.fy() << " cx=" << rgbd_intrinsics2[sensor_id].cam_params.rightCamera.cx() << " cy=" << rgbd_intrinsics2[sensor_id].cam_params.rightCamera.cy() << endl;
        }
        cout << "...loadConfiguration\n";
    }

  /*! This function segments planes from the point cloud */
    void getPlanes(pcl::PointCloud<PointT>::Ptr & point_cloud, mrpt::pbmap::PbMap & pbmap)
    {
      // Downsample and filter point cloud
//      DownsampleRGBD downsampler(2);
//      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampledCloud = downsampler.downsamplePointCloud(point_cloud);
//      cloudImg.setPointCloud(downsampler.downsamplePointCloud(point_cloud));
      pcl::FastBilateralFilter<pcl::PointXYZRGBA> filter;
      filter.setSigmaS (10.0);
      filter.setSigmaR (0.05);
//      filter.setInputCloud(cloudImg.getDownsampledPointCloud(2));
      filter.setInputCloud(point_cloud);
      filter.filter(*point_cloud);

      // Segment planes
      std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
      std::vector<pcl::ModelCoefficients> model_coefficients;
      std::vector<pcl::PointIndices> inliers;
      size_t n_planes = segmentPlanes(point_cloud, regions, model_coefficients, inliers, dist_threshold, angle_threshold, min_inliers);
      cout << " number of planes " << n_planes << " cloud size " << point_cloud->size() << "\n";

      pbmap.vPlanes.clear();

      // Create a vector with the planes detected in this keyframe, and calculate their parameters (normal, center, pointclouds, etc.)
      cout << "cloud_size " << point_cloud->size() << "\n";
      for (size_t i = 0; i < regions.size (); i++)
      {
        mrpt::pbmap::Plane plane;

        plane.v3center = regions[i].getCentroid();
        plane.v3normal = Eigen::Vector3f(model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2]);
        if( plane.v3normal.dot(plane.v3center) > 0)
        {
          plane.v3normal = -plane.v3normal;
  //          plane.d = -plane.d;
        }
        plane.curvature = regions[i].getCurvature ();
  //    cout << i << " getCurvature\n";

  //        if(plane.curvature > max_curvature_plane)
  //          continue;

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        extract.setInputCloud ( point_cloud );
        extract.setIndices ( boost::make_shared<const pcl::PointIndices> (inliers[i]) );
        extract.setNegative (false);
        extract.filter (*plane.planePointCloudPtr);    // Write the planar point cloud
        plane.inliers = inliers[i].indices;

        Eigen::Matrix3f cov_normal = Eigen::Matrix3f::Zero();
        Eigen::Vector3f cov_nd = Eigen::Vector3f::Zero();
        Eigen::Vector3f gravity_center = Eigen::Vector3f::Zero();
        for(size_t j=0; j < inliers[i].indices.size(); j++)
        {
          Eigen::Vector3f pt; pt << plane.planePointCloudPtr->points[j].x, plane.planePointCloudPtr->points[j].y, plane.planePointCloudPtr->points[j].z;
          gravity_center += pt;
        }
        cov_nd = gravity_center;
        gravity_center /= plane.planePointCloudPtr->size();
//      cout << "gravity_center " << gravity_center.transpose() << "   " << plane.v3center.transpose() << endl;
//        Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
        for(size_t j=0; j < inliers[i].indices.size(); j++)
        {
          Eigen::Vector3f pt; pt << plane.planePointCloudPtr->points[j].x, plane.planePointCloudPtr->points[j].y, plane.planePointCloudPtr->points[j].z;
          cov_normal += -pt*pt.transpose();// + (plane.v3normal.dot(pt-gravity_center))*(plane.v3normal.dot(pt))*Eigen::Matrix3f::Identity();
//          Eigen::Vector3f pt_rg = (pt-gravity_center);
//          M += pt_rg * pt_rg.transpose();
        }
//        Eigen::JacobiSVD<Eigen::Matrix3f> svdM(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
//      cout << "normalV " << plane.v3normal.transpose() << " covM \n" << svdM.matrixU() << endl;

        Eigen::Matrix4f fim;//, covariance;
        fim.block(0,0,3,3) = cov_normal;
        fim.block(0,3,3,1) = cov_nd;
        fim.block(3,0,1,3) = cov_nd.transpose();
        fim(3,3) = -plane.planePointCloudPtr->size();
        //fim *= 1 / SIGMA2;
        Eigen::JacobiSVD<Eigen::Matrix4f> svd(fim, Eigen::ComputeFullU | Eigen::ComputeFullV);
        //svd.pinv(plane.information);
        plane.information = pseudoinverse(fim);
//        std::cout << "covariance \n" << plane.information << std::endl;
        plane.information = -fim;
//
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr contourPtr(new pcl::PointCloud<pcl::PointXYZRGBA>);
        contourPtr->points = regions[i].getContour();
//        std::vector<size_t> indices_hull;

//      cout << "Extract contour\n";
        if(contourPtr->size() != 0)
        {
//      cout << "Extract contour2 " << contourPtr->size() << "\n";
          plane.calcConvexHull(contourPtr);
        }
        else
        {
  //        assert(false);
        std::cout << "HULL 000\n" << plane.planePointCloudPtr->size() << std::endl;
          static pcl::VoxelGrid<pcl::PointXYZRGBA> plane_grid;
          plane_grid.setLeafSize(0.05,0.05,0.05);
          plane_grid.setInputCloud (plane.planePointCloudPtr);
          plane_grid.filter (*contourPtr);
          plane.calcConvexHull(contourPtr);
        }

  //        plane.calcConvexHull(contourPtr);
//      cout << "calcConvexHull\n";
        plane.computeMassCenterAndArea();
  //    cout << "Extract convexHull\n";
        // Discard small planes

        plane.d = -plane.v3normal .dot( plane.v3center );

//        // Discard narrow planes
//        plane.calcElongationAndPpalDir();
//        if(plane.elongation > max_elongation_plane)
//          continue;

        bool isSamePlane = false;
        if(plane.curvature < max_curvature_plane)
          for (size_t j = 0; j < pbmap.vPlanes.size(); j++)
            if( pbmap.vPlanes[j].curvature < max_curvature_plane && pbmap.vPlanes[j].isSamePlane(plane, 0.99, 0.05, 0.2) ) // The planes are merged if they are the same
            {
//            cout << "Merge local region\n";
              isSamePlane = true;
  //            double time_start = pcl::getTime();
              pbmap.vPlanes[j].mergePlane2(plane);
  //            double time_end = pcl::getTime();
  //          std::cout << " mergePlane2 took " << double (time_start - time_end) << std::endl;

              break;
            }
        if(!isSamePlane)
        {
//          cout << "New plane\n";
  //          plane.calcMainColor();
          plane.id = pbmap.vPlanes.size();
          pbmap.vPlanes.push_back(plane);
        }
      }
      //double extractPlanes_end = pcl::getTime();
      //std::cout << "getPlanes in " << (extractPlanes_end - extractPlanes_start)*1000 << " ms\n";
    }


//    void trimOutliersRANSAC(mrpt::math::CMatrixDouble &matched_planes, mrpt::math::CMatrixDouble &FIM_values)
//    {
//      cout << "trimOutliersRANSAC... " << endl;

//    //  assert(matched_all_planes.size() >= 3);
//    //  CTicTac tictac;

//      if(matched_all_planes.getRowCount() <= 3)
//      {
//        cout << "Insuficient matched planes " << matched_all_planes.getRowCount() << endl;
////        return Eigen::Matrix4f::Identity();
//        return;
//      }

//      CMatrixDouble planeCorresp(8, matched_all_planes.getRowCount());
//      planeCorresp = matched_all_planes.block(0,0,matched_all_planes.getRowCount(),8).transpose();

//      mrpt::vector_size_t inliers;
//    //  Eigen::Matrix4f best_model;
//      CMatrixDouble best_model;

//      math::RANSAC::execute(planeCorresp,
//                            ransacPlaneAlignment_fit,
//                            ransac3Dplane_distance,
//                            ransac3Dplane_degenerate,
//                            0.05, // threshold
//                            3,  // Minimum set of points
//                            inliers,
//                            best_model,
//                            //false, //verbose
//                            0.51, // probGoodSample
//                            1000 // maxIter
//                            );

//    //  cout << "Computation time: " << tictac.Tac()*1000.0/TIMES << " ms" << endl;

//      cout << "Size planeCorresp: " << size(planeCorresp,2) << endl;
//      cout << "RANSAC finished: " << inliers.size() << " from " << matched_all_planes.getRowCount() << ". \nBest model: \n" << best_model << endl;
//    //        cout << "Best inliers: " << best_inliers << endl;

//      mrpt::math::CMatrixDouble trimMatchedPlanes(inliers.size(), matched_all_planes.getColCount());
//      mrpt::math::CMatrixDouble trimFIM_values(inliers.size(), FIM_values.getColCount());
//      std::vector<double> row;
//      for(unsigned i=0; i < inliers.size(); i++)
//      {
//        trimMatchedPlanes.row(i) = matched_all_planes.row(inliers[i]);
//        trimFIM_values.row(i) = FIM_values.row(inliers[i]);
//      }

//      matched_planes = trimMatchedPlanes;
//      FIM_values = trimFIM_values;
//    }

    void run()
    {
        // Initialize visualizer
        pcl::visualization::CloudViewer viewer("kinect-rig-calib");
        if(display)
        {
            viewer.runOnVisualizationThread (boost::bind(&ExtrinsicRgbdCalibration::viz_cb, this, _1), "viz_cb");
            viewer.registerKeyboardCallback ( &ExtrinsicRgbdCalibration::keyboardEventOccurred, *this );
        }

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

        // Observation parameters
        vector<mrpt::obs::CObservation3DRangeScanPtr> obsRGBD(num_sensors);  // The RGBD observation
        vector<bool> obs_sensor(num_sensors,false);
        vector<double> obs_sensor_time(num_sensors);
        vector<cv::Mat> rgb(num_sensors);
        vector<cv::Mat> depth(num_sensors);

        // Calibration parameters
        float angle_offset = 20;
//        float angle_offset = 180;
        initOffset = Eigen::Matrix4f::Identity();
        initOffset(1,1) = initOffset(2,2) = cos(angle_offset*3.14159/180);
        initOffset(1,2) = -sin(angle_offset*3.14159/180);
        initOffset(2,1) = -initOffset(1,2);
        cout << "initOffset\n" << initOffset << endl;

        CalibratePairRange calibrator;
        // Get the plane correspondences
//        ControlPlanes matches;
//        calibrator.correspondences = mrpt::math::CMatrixDouble(0,10);
        calibrator.correspondences = mrpt::math::CMatrixDouble(0,18);
        calibrator.setInitRt(initOffset);

        CMatrixDouble conditioningFIM(0,6);
        Eigen::Matrix3f FIMrot = Eigen::Matrix3f::Zero();
        Eigen::Matrix3f FIMtrans = Eigen::Matrix3f::Zero();


        //==============================================================================
        //									Main operation
        //==============================================================================
        size_t n_obs = 0, n_frame_sync = 0;
        CObservationPtr observation;
        while ( !bExit && n_obs < dataset.size() )
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
            //cout << sensor_id << " diff " << timestampTotime_t(observation->timestamp)-obs_sensor_time[sensor_id] << endl;
            obs_sensor_time[sensor_id] = timestampTotime_t(observation->timestamp);

            obsRGBD[sensor_id] = CObservation3DRangeScanPtr(observation);
            obsRGBD[sensor_id]->load();

            // Get synchronized observations
            if( !std::accumulate(begin(obs_sensor), end(obs_sensor), true, logical_and<bool>()) )
                continue;

            // Check for aproximate synchronization
            vector<double>::iterator max_time = max_element(obs_sensor_time.begin(), obs_sensor_time.end());
            vector<double>::iterator min_time = min_element(obs_sensor_time.begin(), obs_sensor_time.end());
            if(verbose)
            {
                cout << "max diff " << (*max_time - *min_time)*1e3 << endl;
//                for(size_t i=1; i < num_sensors; i++)
//                    cout << i << " time diff to ref (sensor_id=0) " << (obs_sensor_time[i]-obs_sensor_time[0])*1e3 << " ms." << endl;
            }
            if( (*max_time - *min_time) > 0.005) // maximum de-synchronization in seconds
                continue;

            ++n_frame_sync;                       
            cout << n_frame_sync << " frames sync\n";
            std::fill(obs_sensor.begin(), obs_sensor.end(), false);

            // Apply decimation
            if( n_frame_sync % decimation != 0)
                continue;

            cout << "     use this sync frames \n";

            // Display color and depth images
            if(display)
            {
                for(size_t i=0; i < num_sensors; i++)
                {
                    rgb[i] = cv::Mat(obsRGBD[i]->intensityImage.getAs<IplImage>());
                    convertRange_mrpt2cvMat(obsRGBD[i]->rangeImage, depth[i]);
                }
                int height = rgb[0].cols; // Note that the RGB-D camera is in a vertical configuration
                int width = rgb[0].rows;
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

            vector<size_t> planesSourceIdx(num_sensors+1, 0);
            { boost::mutex::scoped_lock updateLock(visualizationMutex);

            //						Segment local planes
            //==================================================================
//                #pragma omp parallel num_threads(num_sensors)
            for(size_t sensor_id=0; sensor_id < num_sensors; sensor_id++)
            {
//                    sensor_id = omp_get_thread_num();
                cloud[sensor_id] = getPointCloudRegistered(rgb[sensor_id], depth[sensor_id], rgbd_intrinsics[sensor_id]);
                //cloud[sensor_id] = getPointCloud(rgb[sensor_id], depth[sensor_id], rgbd_intrinsics[sensor_id]);
                //obsRGBD[sensor_id]->project3DPointsFromDepthImageInto(*(cloud[sensor_id]), false /* without obs.sensorPose */);
                cout << sensor_id << " cloud " << cloud[sensor_id]->height << "x" << cloud[sensor_id]->width << endl;

//                pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//                viewer.showCloud(cloud[sensor_id]);
//                while (!viewer.wasStopped ())
//                    boost::this_thread::sleep (boost::posix_time::milliseconds (10));

                //cout << "getPlanes\n";
                mrpt::pbmap::PbMap pbmap;
                getPlanes(cloud[sensor_id], v_pbmap[sensor_id]);

//                std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
//                std::vector<pcl::ModelCoefficients> model_coefficients;
//                std::vector<pcl::PointIndices> inliers;
//                size_t n_planes = segmentPlanes(cloud[sensor_id], regions, model_coefficients, inliers, dist_threshold, angle_threshold, min_inliers);
//                cout << sensor_id << " number of planes " << n_planes << endl;


//                cout << "frame360.segmentPlanesLocal() \n";
//                //frame360.segmentPlanesLocal();
//                for(int sensor_id = 0; sensor_id < num_sensors; sensor_id++)
//                    v_pbmap[sensor_id] = frame360.extractPbMap( frame360.getCloudRGBDAsus(sensor_id).getPointCloud(), 0.02f, 0.039812f, 5000 );

//                cout << "Merge planes \n";
//                mrpt::pbmap::PbMap &planes = frame360.planes_;
//                //all_planes.vPlanes.clear();
//                vector<unsigned> planesSourceIdx(5, 0);
//                for(int sensor_id = 0; sensor_id < num_sensors; sensor_id++)
//                {
//                    all_planes.MergeWith(v_pbmap[sensor_id], calib.Rt_[sensor_id]);
//                    planesSourceIdx[sensor_id+1] = planesSourceIdx[sensor_id] + v_pbmap[sensor_id].vPlanes.size();
//                    //          cout << planesSourceIdx[sensor_id+1] << " ";
//                }

            }
            all_planes.vPlanes.clear();
            for(size_t sensor_id = 0; sensor_id < num_sensors; sensor_id++)
            {
                all_planes.MergeWith(v_pbmap[sensor_id], Rt_estimated[sensor_id]);
                planesSourceIdx[sensor_id+1] = planesSourceIdx[sensor_id] + v_pbmap[sensor_id].vPlanes.size();
                //cout << planesSourceIdx[sensor_id+1] << " ";
            }
            bFreezeFrame = false;
            updateLock.unlock();
            }

            if(display)
            {
                // Show registered depth
                int height = depth[0].cols; // Note that the RGB-D camera is in a vertical configuration
                int width = depth[0].rows;
                cv::Mat img_transposed, img_rotated;
                cv::transpose(depth[0], img_transposed);
                cv::flip(img_transposed, img_rotated, 0);
                cv::Mat depth_concat(height, 2*width+20, CV_32FC1, cv::Scalar(0.f));
                cv::Mat tmp = depth_concat(cv::Rect(0, 0, width, height));
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
            //									Data association
            //==============================================================================
            cout << "Data association\n";
            plane_corresp.clear();
            size_t sensor_id1=0;
            size_t sensor_id2=1;
//            for(size_t sensor_id1=0; sensor_id1 < num_sensors; sensor_id1++)
//            {
//    //          matches.mmCorrespondences[sensor_id1] = std::map<unsigned, mrpt::math::CMatrixDouble>();
//              for(size_t sensor_id2=sensor_id1+1; sensor_id2 < num_sensors; sensor_id2++)
//    //            if( sensor_id2 - sensor_id1 == 1 || sensor_id2 - sensor_id1 == 7)
//              {
//    //            cout << " sensor_id1 " << sensor_id1 << " " << v_pbmap_[sensor_id1].vPlanes.size() << " sensor_id2 " << sensor_id2 << " " << v_pbmap_[sensor_id2].vPlanes.size() << endl;
//    //              matches.mmCorrespondences[sensor_id1][sensor_id2] = mrpt::math::CMatrixDouble(0, 10);

                for(size_t i=0; i < v_pbmap[sensor_id1].vPlanes.size(); i++)
                {
                  size_t planesIdx_i = planesSourceIdx[sensor_id1];
                  for(size_t j=0; j < v_pbmap[sensor_id2].vPlanes.size(); j++)
                  {
                    size_t planesIdx_j = planesSourceIdx[sensor_id2];

    //                if(sensor_id1 == 0 && sensor_id2 == 2 && i == 0 && j == 0)
    //                {
    //                  cout << "Inliers " << all_planes.vPlanes[planesIdx_i+i].inliers.size() << " and " << all_planes.vPlanes[planesIdx_j+j].inliers.size() << endl;
    //                  cout << "elongation " << all_planes.vPlanes[planesIdx_i+i].elongation << " and " << all_planes.vPlanes[planesIdx_j+j].elongation << endl;
    //                  cout << "normal " << all_planes.vPlanes[planesIdx_i+i].v3normal.transpose() << " and " << all_planes.vPlanes[planesIdx_j+j].v3normal.transpose() << endl;
    //                  cout << "d " << all_planes.vPlanes[planesIdx_i+i].d << " and " << all_planes.vPlanes[planesIdx_j+j].d << endl;
    //                  cout << "color " << all_planes.vPlanes[planesIdx_i+i].hasSimilarDominantColor(all_planes.vPlanes[planesIdx_j+j],0.06) << endl;
    //                  cout << "nearby " << all_planes.vPlanes[planesIdx_i+i].isPlaneNearby(all_planes.vPlanes[planesIdx_j+j], 0.5) << endl;
    //                }

    //                cout << "  Check planes " << planesIdx_i+i << " and " << planesIdx_j+j << endl;

                    if( all_planes.vPlanes[planesIdx_i+i].inliers.size() > 1000 && all_planes.vPlanes[planesIdx_j+j].inliers.size() > min_inliers &&
                        all_planes.vPlanes[planesIdx_i+i].elongation < 5 && all_planes.vPlanes[planesIdx_j+j].elongation < 5 &&
                        all_planes.vPlanes[planesIdx_i+i].v3normal .dot (all_planes.vPlanes[planesIdx_j+j].v3normal) > 0.99 &&
                        fabs(all_planes.vPlanes[planesIdx_i+i].d - all_planes.vPlanes[planesIdx_j+j].d) < 0.1 )//&&
                        //                    v_pbmap[0].vPlanes[i].hasSimilarDominantColor(v_pbmap[1].vPlanes[j],0.06) &&
                        //                    v_pbmap[0].vPlanes[planes_counter_i+i].isPlaneNearby(v_pbmap[1].vPlanes[planes_counter_j+j], 0.5)
                    {

            //              mrpt::pbmap::PbMap &planes_i = v_pbmap[0];
            //              mrpt::pbmap::PbMap &planes_j = v_pbmap[1];

                        b_select_manually = true;

                        cout << "\tAssociate planes " << endl;
                        Eigen::Vector4f pl1, pl2;
                        pl1.head(3) = v_pbmap[0].vPlanes[i].v3normal; pl1[3] = v_pbmap[0].vPlanes[i].d;
                        pl2.head(3) = v_pbmap[1].vPlanes[j].v3normal; pl2[3] = v_pbmap[1].vPlanes[j].d;
                        //              cout << "Corresp " << v_pbmap[0].vPlanes[i].v3normal.transpose() << " vs " << v_pbmap[1].vPlanes[j].v3normal.transpose() << " = " << v_pbmap[1].vPlanes[j].v3normal.transpose() << endl;
                        ////                        float factorDistInliers = std::min(v_pbmap[0].vPlanes[i].inliers.size(), v_pbmap[1].vPlanes[j].inliers.size()) / std::max(v_pbmap[0].vPlanes[i].v3center.norm(), v_pbmap[1].vPlanes[j].v3center.norm());
                        //                        float factorDistInliers = (v_pbmap[0].vPlanes[i].inliers.size() + v_pbmap[1].vPlanes[j].inliers.size()) / (v_pbmap[0].vPlanes[i].v3center.norm() * v_pbmap[1].vPlanes[j].v3center.norm());
                        //                        weight_pair[couple_id] += factorDistInliers;
                        //                        pl1 *= factorDistInliers;
                        //                        pl2 *= factorDistInliers;
                        //                ++weight_pair[couple_id];

                        //Add constraints
                        //                  correspondences.push_back(pair<Eigen::Vector4f, Eigen::Vector4f>(pl1, pl2));
                        //                        correspondences[couple_id].push_back(pair<Eigen::Vector4f, Eigen::Vector4f>(pl1/v_pbmap[0].vPlanes[i].v3center.norm(), pl2/v_pbmap[1].vPlanes[j].v3center.norm());

                        // Calculate conditioning
                        ++valid_obs;
                        covariance += pl2.head(3) * pl1.head(3).transpose();
                        Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
                        conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
                        cout << "conditioning " << conditioning << endl;

                        //          if(b_select_manually)

                        size_t prevSize = calibrator.correspondences.getRowCount();
                        calibrator.correspondences.setSize(prevSize+1, calibrator.correspondences.getColCount());
                        calibrator.correspondences(prevSize, 0) = pl1[0];
                        calibrator.correspondences(prevSize, 1) = pl1[1];
                        calibrator.correspondences(prevSize, 2) = pl1[2];
                        calibrator.correspondences(prevSize, 3) = pl1[3];
                        calibrator.correspondences(prevSize, 4) = pl2[0];
                        calibrator.correspondences(prevSize, 5) = pl2[1];
                        calibrator.correspondences(prevSize, 6) = pl2[2];
                        calibrator.correspondences(prevSize, 7) = pl2[3];

                        Eigen::Matrix4f informationFusion;
                        Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
                        tf.block(0,0,3,3) = calibrator.Rt_estimated.block(0,0,3,3);
                        tf.block(3,0,1,3) = -calibrator.Rt_estimated.block(0,3,3,1).transpose();
                        informationFusion = v_pbmap[0].vPlanes[i].information;
                        informationFusion += tf * v_pbmap[1].vPlanes[j].information * tf.inverse();
                        Eigen::JacobiSVD<Eigen::Matrix4f> svd_cov(informationFusion, Eigen::ComputeFullU | Eigen::ComputeFullV);
                        Eigen::Vector4f minEigenVector = svd_cov.matrixU().block(0,3,4,1);
                        cout << "minEigenVector " << minEigenVector.transpose() << endl;
                        informationFusion -= svd.singularValues().minCoeff() * minEigenVector * minEigenVector.transpose();
                        cout << "informationFusion \n" << informationFusion << "\n minSV " << svd.singularValues().minCoeff() << endl;

                        calibrator.correspondences(prevSize, 8) = informationFusion(0,0);
                        calibrator.correspondences(prevSize, 9) = informationFusion(0,1);
                        calibrator.correspondences(prevSize, 10) = informationFusion(0,2);
                        calibrator.correspondences(prevSize, 11) = informationFusion(0,3);
                        calibrator.correspondences(prevSize, 12) = informationFusion(1,1);
                        calibrator.correspondences(prevSize, 13) = informationFusion(1,2);
                        calibrator.correspondences(prevSize, 14) = informationFusion(1,3);
                        calibrator.correspondences(prevSize, 15) = informationFusion(2,2);
                        calibrator.correspondences(prevSize, 16) = informationFusion(2,3);
                        calibrator.correspondences(prevSize, 17) = informationFusion(3,3);


                        FIMrot += -skew(v_pbmap[1].vPlanes[j].v3normal) * informationFusion.block(0,0,3,3) * skew(v_pbmap[1].vPlanes[j].v3normal);
                        FIMtrans += v_pbmap[0].vPlanes[i].v3normal * v_pbmap[0].vPlanes[i].v3normal.transpose() * informationFusion(3,3);

                        Eigen::JacobiSVD<Eigen::Matrix3f> svd_rot(FIMrot, Eigen::ComputeFullU | Eigen::ComputeFullV);
                        Eigen::JacobiSVD<Eigen::Matrix3f> svd_trans(FIMtrans, Eigen::ComputeFullU | Eigen::ComputeFullV);
                        //              float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
                        conditioningFIM.setSize(prevSize+1, conditioningFIM.getColCount());
                        conditioningFIM(prevSize, 0) = svd_rot.singularValues()[0];
                        conditioningFIM(prevSize, 1) = svd_rot.singularValues()[1];
                        conditioningFIM(prevSize, 2) = svd_rot.singularValues()[2];
                        conditioningFIM(prevSize, 3) = svd_trans.singularValues()[0];
                        conditioningFIM(prevSize, 4) = svd_trans.singularValues()[1];
                        conditioningFIM(prevSize, 5) = svd_trans.singularValues()[2];

  //              cout << "normalCovariance " << minEigenVector.transpose() << " covM \n" << svd_cov.matrixU() << endl;

  //                calibrator.correspondences(prevSize, 8) = std::min(v_pbmap[0].vPlanes[i].inliers.size(), v_pbmap[1].vPlanes[j].inliers.size());
  //
  //                float dist_center1 = 0, dist_center2 = 0;
  //                for(size_t k=0; k < v_pbmap[0].vPlanes[i].inliers.size(); k++)
  //                  dist_center1 += v_pbmap[0].vPlanes[i].inliers[k] / frameRGBD_[0].getPointCloud()->width + v_pbmap[0].vPlanes[i].inliers[k] % frameRGBD_[0].getPointCloud()->width;
  ////                      dist_center1 += (v_pbmap[0].vPlanes[i].inliers[k] / frame360.sphereCloud->width)*(v_pbmap[0].vPlanes[i].inliers[k] / frame360.sphereCloud->width) + (v_pbmap[0].vPlanes[i].inliers[k] % frame360.sphereCloud->width)+(v_pbmap[0].vPlanes[i].inliers[k] % frame360.sphereCloud->width);
  //                dist_center1 /= v_pbmap[0].vPlanes[i].inliers.size();
  //
  //                for(size_t k=0; k < v_pbmap[1].vPlanes[j].inliers.size(); k++)
  //                  dist_center2 += v_pbmap[1].vPlanes[j].inliers[k] / frameRGBD_[0].getPointCloud()->width + v_pbmap[1].vPlanes[j].inliers[k] % frameRGBD_[0].getPointCloud()->width;
  ////                      dist_center2 += (v_pbmap[1].vPlanes[j].inliers[k] / frame360.sphereCloud->width)*(v_pbmap[1].vPlanes[j].inliers[k] / frame360.sphereCloud->width) + (v_pbmap[1].vPlanes[j].inliers[k] % frame360.sphereCloud->width)+(v_pbmap[1].vPlanes[j].inliers[k] % frame360.sphereCloud->width);
  //                dist_center2 /= v_pbmap[1].vPlanes[j].inliers.size();
  //                calibrator.correspondences(prevSize, 9) = std::max(dist_center1, dist_center2);



//              for(size_t sensor_id1=0; sensor_id1 < num_sensors; sensor_id1++)
//              {
//      //          matches.mmCorrespondences[sensor_id1] = std::map<unsigned, mrpt::math::CMatrixDouble>();
//                for(unsigned sensor_id2=sensor_id1+1; sensor_id2 < num_sensors; sensor_id2++)
//                {
//      //            cout << " sensor_id1 " << sensor_id1 << " " << v_pbmap[sensor_id1].vPlanes.size() << " sensor_id2 " << sensor_id2 << " " << v_pbmap[sensor_id2].vPlanes.size() << endl;
//      //              matches.mmCorrespondences[sensor_id1][sensor_id2] = mrpt::math::CMatrixDouble(0, 10);

//                  for(unsigned i=0; i < v_pbmap[sensor_id1].vPlanes.size(); i++)
//                  {
//                    unsigned planesIdx_i = planesSourceIdx[sensor_id1];
//                    for(unsigned j=0; j < v_pbmap[sensor_id2].vPlanes.size(); j++)
//                    {
//                      unsigned planesIdx_j = planesSourceIdx[sensor_id2];

//      //                if(sensor_id1 == 0 && sensor_id2 == 2 && i == 0 && j == 0)
//      //                {
//      //                  cout << "Inliers " << all_planes.vPlanes[planesIdx_i+i].inliers.size() << " and " << all_planes.vPlanes[planesIdx_j+j].inliers.size() << endl;
//      //                  cout << "elongation " << all_planes.vPlanes[planesIdx_i+i].elongation << " and " << all_planes.vPlanes[planesIdx_j+j].elongation << endl;
//      //                  cout << "normal " << all_planes.vPlanes[planesIdx_i+i].v3normal.transpose() << " and " << all_planes.vPlanes[planesIdx_j+j].v3normal.transpose() << endl;
//      //                  cout << "d " << all_planes.vPlanes[planesIdx_i+i].d << " and " << all_planes.vPlanes[planesIdx_j+j].d << endl;
//      //                  cout << "color " << all_planes.vPlanes[planesIdx_i+i].hasSimilarDominantColor(all_planes.vPlanes[planesIdx_j+j],0.06) << endl;
//      //                  cout << "nearby " << all_planes.vPlanes[planesIdx_i+i].isPlaneNearby(all_planes.vPlanes[planesIdx_j+j], 0.5) << endl;
//      //                }

//      //                cout << "  Check planes " << planesIdx_i+i << " and " << planesIdx_j+j << endl;

//                      if( all_planes.vPlanes[planesIdx_i+i].inliers.size() > 1000 && all_planes.vPlanes[planesIdx_j+j].inliers.size() > 1000 &&
//                          all_planes.vPlanes[planesIdx_i+i].elongation < 5 && all_planes.vPlanes[planesIdx_j+j].elongation < 5 &&
//                          all_planes.vPlanes[planesIdx_i+i].v3normal .dot (all_planes.vPlanes[planesIdx_j+j].v3normal) > 0.99 &&
//                          fabs(all_planes.vPlanes[planesIdx_i+i].d - all_planes.vPlanes[planesIdx_j+j].d) < 0.2 )//&&
//      //                    all_planes.vPlanes[planesIdx_i+i].hasSimilarDominantColor(all_planes.vPlanes[planesIdx_j+j],0.06) &&
//      //                    all_planes.vPlanes[planesIdx_i+i].isPlaneNearby(all_planes.vPlanes[planesIdx_j+j], 0.5) )
//        //                      matches.inliersUpperFringe(all_planes.vPlanes[planesIdx_i+i], 0.2) > 0.2 &&
//        //                      matches.inliersLowerFringe(all_planes.vPlanes[planesIdx_j+j], 0.2) > 0.2 ) // Assign correspondence
//                        {
//      //                  cout << "\t   Associate planes " << planesIdx_i+i << " and " << planesIdx_j+j << endl;

//                        #if VISUALIZE_POINT_CLOUD
//                          // Visualize Control Planes
//                          { boost::mutex::scoped_lock updateLock(visualizationMutex);
//                            match1 = planesIdx_i+i;
//                            match2 = planesIdx_j+j;
//                            drawMatch = true;
//                            keyDown = false;
//                          updateLock.unlock();
//                          }

//      //                    match1 = planesIdx_i+i;
//      //                    match2 = planesIdx_j+j;
//      //                    pcl::visualization::CloudViewer viewer("RGBD360_calib");
//      //                    viewer.runOnVisualizationThread (boost::bind(&GetControlPlanes::viz_cb, this, _1), "viz_cb");
//      //                    viewer.registerKeyboardCallback(&GetControlPlanes::keyboardEventOccurred, *this);

//      //                    cout << " keyDown " << keyDown << endl;
//                          while(!keyDown)
//                            boost::this_thread::sleep (boost::posix_time::milliseconds (10));
//                        #endif

//      //                  cout << "\t   Record corresp " << endl;
//                          unsigned prevSize = calibrator.correspondences.getRowCount();
//                          calibrator.correspondences.setSize(prevSize+1, calibrator.correspondences.getColCount());
//                          calibrator.correspondences(prevSize, 0) = v_pbmap[sensor_id1].vPlanes[i].v3normal[0];
//                          calibrator.correspondences(prevSize, 1) = v_pbmap[sensor_id1].vPlanes[i].v3normal[1];
//                          calibrator.correspondences(prevSize, 2) = v_pbmap[sensor_id1].vPlanes[i].v3normal[2];
//                          calibrator.correspondences(prevSize, 3) = v_pbmap[sensor_id1].vPlanes[i].d;
//                          calibrator.correspondences(prevSize, 4) = v_pbmap[sensor_id2].vPlanes[j].v3normal[0];
//                          calibrator.correspondences(prevSize, 5) = v_pbmap[sensor_id2].vPlanes[j].v3normal[1];
//                          calibrator.correspondences(prevSize, 6) = v_pbmap[sensor_id2].vPlanes[j].v3normal[2];
//                          calibrator.correspondences(prevSize, 7) = v_pbmap[sensor_id2].vPlanes[j].d;
//                          calibrator.correspondences(prevSize, 8) = std::min(v_pbmap[sensor_id1].vPlanes[i].inliers.size(), v_pbmap[sensor_id2].vPlanes[j].inliers.size());

//                          // For several sensors (more than 2)
////                          unsigned prevSize = matches.mmCorrespondences[sensor_id1][sensor_id2].getRowCount();
////                          matches.mmCorrespondences[sensor_id1][sensor_id2].setSize(prevSize+1, matches.mmCorrespondences[sensor_id1][sensor_id2].getColCount());
////                          matches.mmCorrespondences[sensor_id1][sensor_id2](prevSize, 0) = v_pbmap[sensor_id1].vPlanes[i].v3normal[0];
////                          matches.mmCorrespondences[sensor_id1][sensor_id2](prevSize, 1) = v_pbmap[sensor_id1].vPlanes[i].v3normal[1];
////                          matches.mmCorrespondences[sensor_id1][sensor_id2](prevSize, 2) = v_pbmap[sensor_id1].vPlanes[i].v3normal[2];
////                          matches.mmCorrespondences[sensor_id1][sensor_id2](prevSize, 3) = v_pbmap[sensor_id1].vPlanes[i].d;
////                          matches.mmCorrespondences[sensor_id1][sensor_id2](prevSize, 4) = v_pbmap[sensor_id2].vPlanes[j].v3normal[0];
////                          matches.mmCorrespondences[sensor_id1][sensor_id2](prevSize, 5) = v_pbmap[sensor_id2].vPlanes[j].v3normal[1];
////                          matches.mmCorrespondences[sensor_id1][sensor_id2](prevSize, 6) = v_pbmap[sensor_id2].vPlanes[j].v3normal[2];
////                          matches.mmCorrespondences[sensor_id1][sensor_id2](prevSize, 7) = v_pbmap[sensor_id2].vPlanes[j].d;
////                          matches.mmCorrespondences[sensor_id1][sensor_id2](prevSize, 8) = std::min(v_pbmap[sensor_id1].vPlanes[i].inliers.size(), v_pbmap[sensor_id2].vPlanes[j].inliers.size());


////                          float dist_center1 = 0, dist_center2 = 0;
////                          for(unsigned k=0; k < v_pbmap[sensor_id1].vPlanes[i].inliers.size(); k++)
////                            dist_center1 += v_pbmap[sensor_id1].vPlanes[i].inliers[k] / frame360.frameRGBD_[sensor_id1].getPointCloud()->width + v_pbmap[sensor_id1].vPlanes[i].inliers[k] % frame360.frameRGBD_[sensor_id1].getPointCloud()->width;
////      //                      dist_center1 += (v_pbmap[sensor_id1].vPlanes[i].inliers[k] / frame360.sphereCloud->width)*(v_pbmap[sensor_id1].vPlanes[i].inliers[k] / frame360.sphereCloud->width) + (v_pbmap[sensor_id1].vPlanes[i].inliers[k] % frame360.sphereCloud->width)+(v_pbmap[sensor_id1].vPlanes[i].inliers[k] % frame360.sphereCloud->width);
////                          dist_center1 /= v_pbmap[sensor_id1].vPlanes[i].inliers.size();

////                          for(unsigned k=0; k < v_pbmap[sensor_id2].vPlanes[j].inliers.size(); k++)
////                            dist_center2 += v_pbmap[sensor_id2].vPlanes[j].inliers[k] / frame360.frameRGBD_[sensor_id2].getPointCloud()->width + v_pbmap[sensor_id2].vPlanes[j].inliers[k] % frame360.frameRGBD_[sensor_id2].getPointCloud()->width;
////      //                      dist_center2 += (v_pbmap[sensor_id2].vPlanes[j].inliers[k] / frame360.sphereCloud->width)*(v_pbmap[sensor_id2].vPlanes[j].inliers[k] / frame360.sphereCloud->width) + (v_pbmap[sensor_id2].vPlanes[j].inliers[k] % frame360.sphereCloud->width)+(v_pbmap[sensor_id2].vPlanes[j].inliers[k] % frame360.sphereCloud->width);
////                          dist_center2 /= v_pbmap[sensor_id2].vPlanes[j].inliers.size();

////                          matches.mmCorrespondences[sensor_id1][sensor_id2](prevSize, 9) = std::max(dist_center1, dist_center2);
////      //                  cout << "\t Size " << matches.mmCorrespondences[sensor_id1][sensor_id2].getRowCount() << " x " << matches.mmCorrespondences[sensor_id1][sensor_id2].getColCount() << endl;

////                          if( sensor_id2 - sensor_id1 == 1 ) // Calculate conditioning
////                          {
////      //                      updateConditioning(couple_id, correspondences[couple_id].back());
////                            matches.covariances[sensor_id1] += all_planes.vPlanes[planesIdx_i+i].v3normal * all_planes.vPlanes[planesIdx_j+j].v3normal.transpose();
////                            matches.calcAdjacentConditioning(sensor_id1);
////      //                    cout << "Update " << sensor_id1 << endl;

////        //                    // For visualization
////        //                    plane_corresp[couple_id].push_back(pair<mrpt::pbmap::Plane*, mrpt::pbmap::Plane*>(&all_planes.vPlanes[planesIdx_i+i], &all_planes.vPlanes[planes_counter_j+j]));
////                          }
////                          else if(sensor_id2 - sensor_id1 == 7)
////                          {
////      //                      updateConditioning(couple_id, correspondences[couple_id].back());
////                            matches.covariances[sensor_id2] += all_planes.vPlanes[planesIdx_i+i].v3normal * all_planes.vPlanes[planesIdx_j+j].v3normal.transpose();
////                            matches.calcAdjacentConditioning(sensor_id2);
////      //                    cout << "Update " << sensor_id2 << endl;
////                          }

//      //                    break;
//                        }
//                    }
//                  }
//                }
//              }



                { //mrpt::synch::CCriticalSectionLocker csl(&CS_visualize);
                boost::mutex::scoped_lock updateLock(visualizationMutex);

                  // For visualization
                  plane_corresp[i] = j;

  //            pcl::transformPointCloud(*frameRGBD_[1].getPointCloud(), *cloud[1], calibrator.Rt_estimated);
  //            v_pbmap[1] = v_pbmap[1];
  //            for(unsigned k=0; k < v_pbmap[1].vPlanes.size(); k++)
  //              v_pbmap[1].vPlanes[k].transform(calibrator.Rt_estimated);

                updateLock.unlock();
                } // CS_visualize

                break;
              }
            }
          }

        // Compute the calibration if there are enough measurements

//          calibrator.calcFisherInfMat();
          calibrator.CalibrateRotationManifold(1);
          calibrator.CalibrateTranslation(1);
//          calibrator.CalibrateRotation();

//          if(conditioning < 100 & valid_obs > 3)
//            calibrator.CalibratePair();

//          if(conditioning < 50 & valid_obs > 30)
//            bDoCalibration = true;

//cout << "run9\n";

//          while(b_select_manually == true)
//            boost::this_thread::sleep (boost::posix_time::milliseconds(10));
        }


        // Trim outliers
        //trimOutliersRANSAC(calibrator.correspondences, conditioningFIM);


        float threshold_conditioning = 800.0;
        if(conditioning < threshold_conditioning)
        {
        cout << "\tSave CorrespMat\n";
          calibrator.correspondences.saveToTextFile( mrpt::format("%s/correspondences.txt", output_dir.c_str()) );
          conditioningFIM.saveToTextFile( mrpt::format("%s/conditioningFIM.txt", output_dir.c_str()) );

          calibrator.CalibratePair();

          calibrator.CalibrateRotationD();

          calibrator.setInitRt(initOffset);
          calibrator.CalibrateRotationManifold();
          calibrator.Rt_estimated.block(0,3,3,1) = calibrator.CalibrateTranslation();
          cout << "Rt_estimated \n" << calibrator.Rt_estimated << endl;
        }

    }

//    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
//    {
//      if ( event.keyDown () )
//      {
//        if(event.getKeySym () == "s" || event.getKeySym () == "S")
//          bDoCalibration = true;
//        else
//          b_select_manually = true;
//      }
//    }


    void viz_cb (pcl::visualization::PCLVisualizer& viz)
    {
        //cout << "ExtrinsicRgbdCalibration::viz_cb(...)\n";
        if( bFreezeFrame || cloud.empty() || !cloud[0] || cloud[0]->empty())
        {
            boost::this_thread::sleep (boost::posix_time::milliseconds (10));
            return;
        }
        //cout << "   ::viz_cb(...)\n";

        viz.removeAllShapes();
        viz.removeAllPointClouds();

        //      viz.setCameraPosition(0,0,-3,-1,0,0);
        //      viz.setSize(640,480); // Set the window size
        viz.setSize(1280,960); // Set the window size
        //      viz.setSize(800,800); // Set the window size
        //      viz.setCameraPosition(0,0,-5,0,-0.707107,0.707107,1,0,0);

        { //mrpt::synch::CCriticalSectionLocker csl(&CS_visualize);
            boost::mutex::scoped_lock updateLock(visualizationMutex);

            char name[1024];
            Eigen::Affine3f Rt;

            sprintf (name, "%zu pts. Params ...", cloud[1]->size());
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

            // Draw planes
            for(size_t i=0; i < all_planes.vPlanes.size(); i++)
            {
                //            for(map<unsigned, unsigned>::iterator it=plane_corresp.begin(); it!=plane_corresp.end(); it++)
                //              if(it->first == i)
                //              {
                mrpt::pbmap::Plane &plane_i = all_planes.vPlanes[i];
                sprintf (name, "normal_%u", static_cast<unsigned>(i));
                pcl::PointXYZ pt1, pt2; // Begin and end points of normal's arrow for visualization
                pt1 = pcl::PointXYZ(plane_i.v3center[0], plane_i.v3center[1], plane_i.v3center[2]);
                pt2 = pcl::PointXYZ(plane_i.v3center[0] + (0.3f * plane_i.v3normal[0]),
                        plane_i.v3center[1] + (0.3f * plane_i.v3normal[1]),
                        plane_i.v3center[2] + (0.3f * plane_i.v3normal[2]));
                //            viz.addArrow (pt2, pt1, ared[5], agrn[5], ablu[5], false, name);
                viz.addArrow (pt2, pt1, ared[i%10], agrn[i%10], ablu[i%10], false, name);

                sprintf (name, "approx_plane_%02d", int (i));
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
                //                sprintf (name, "approx_plane_%02d", int (i));
                //                viz.addPolygon<PointT> (plane_i.polygonContourPtr, red[0], grn[0], blu[0], name);
                //
                ////                sprintf (name, "inliers_%02d", int (i));
                ////                pcl::visualization::PointCloudColorHandlerCustom <PointT> color (plane_i.planePointCloudPtr, red[0], grn[0], blu[0]);
                ////                viz.addPointCloud (plane_i.planePointCloudPtr, color, name);
                //              }
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

//                    sprintf (name, "approx_plane_%02d", int (i));
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
//                    //                sprintf (name, "approx_plane_%02d", int (i));
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

//                    sprintf (name, "approx_plane_j_%02d", int (i));
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
//                    //                sprintf (name, "approx_plane_j_%02d", int (i));
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
            bFreezeFrame = true;
            updateLock.unlock();
        }
    }

    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
    {
        if ( event.keyDown () )
        {
            if(event.getKeySym () == "e" || event.getKeySym () == "E")
                bExit = true;
            else if(event.getKeySym () == "k" || event.getKeySym () == "K")
                b_select_manually = !b_select_manually;
            else if(event.getKeySym () == "l" || event.getKeySym () == "L"){
                bFreezeFrame = !bFreezeFrame;
                b_select_manually = !b_select_manually;
            }
        }
    }

};



void print_help(char ** argv)
{
    cout << "This program computes the extrinsic calibration of several RGB-D sensors (e.g. Kinect) based on plane and line correspondences." << endl
         << "No overlapping required for the sensor FOV, as far as the planes/lines can be observed simultaneously by pairs of sensors." << endl
         << "This program accepts a single argument specifying a configuration file which indicates the rawlog file of sensor observations" << endl
         << "to compute the calibration, the approximated sensor init_poses, and a conservative approximation of their accuracy. The output text" << endl
         << "file with the calibration results, and display/verbose parameters can also be set in the configuration.\n" << endl;

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
        printf(" kinect-rig-calib - Part of the MRPT\n");
        printf(" MRPT C++ Library: %s - BUILD DATE %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());
        printf("-------------------------------------------------------------------\n");
//        fprintf(stderr, " check\n");

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

        string config_file = find_mrpt_shared_dir() + std::string("config_files/calibration/extrinsic_calib_2rgbd.ini"); // Default config file
        if(argc == 2)
            config_file = argv[1];

        if(!fileExists(config_file))
        {
            cout << "config_file: " << config_file << " does not exist\n\n";
            print_help(argv);
            mrpt::system::pause();
            return -1;
        }

        ExtrinsicRgbdCalibration calibrator;
        calibrator.loadConfiguration(config_file);
        calibrator.run();

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
