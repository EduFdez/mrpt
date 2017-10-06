/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
    APPLICATION: camera-rot-tracker
    FILE: camera-rot-tracker_main.cpp
    AUTHOR: Eduardo Fernández-Moral <efernandezmoral@gmail.com>

    See README.txt for instructions or
          http://www.mrpt.org/Application:camera-rot-tracker
  ---------------------------------------------------------------*/

#include "camera-rot-tracker.h"
#include "../kinect-rig-calib/kinect-rig-calib_misc.h"
#include "DifOdometry_Datasets_RGBD.h"
#include <mrpt/utils/CConfigFile.h>
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
using namespace std;
using namespace Eigen;


void CameraRotTracker::displayObservation()
{
    cout << "CameraRotTracker::displayObservation...\n";

    for(size_t i=1; i < num_sensors; i++)
    {
//        string win_name = format("rgb_%lu",v_rgb[i]);
        cv::imshow(mrpt::format("rgb_%lu",i), v_rgb[i] ); //cv::moveWindow("rgb", 20,20);
    }
    cv::waitKey(0);
}

void CameraRotTracker::loadConfiguration(const string & config_file)
{
    //Initial steps. Load configuration file
    //-------------------------------------------
    utils::CConfigFile cfg(config_file);
    rawlog_file = cfg.read_string("GLOBAL", "rawlog_file", "no file", true);
    output_dir = cfg.read_string("GLOBAL", "output_dir", "no dir", true);

    decimation = cfg.read_int("GLOBAL", "decimation", 0, true);
    display = cfg.read_bool("GLOBAL", "display", 0, true);
    verbose = cfg.read_bool("GLOBAL", "verbose", 0, true);

    line_extraction = cfg.read_int("GLOBAL", "line_extraction", 2, true);
    min_pixels_line = cfg.read_int("GLOBAL", "min_pixels_line", 100, true);
    max_lines       = cfg.read_int("GLOBAL", "max_lines", 20, true);
    min_angle_diff = cfg.read_float("GLOBAL", "min_angle_diff", 1.2, true);
    cout << min_angle_diff << " min_angle_diff\n";

    if(verbose)
        cout << "loadConfiguration -> dataset: " << rawlog_file << "\toutput: " << output_dir << "\tdecimation: " << decimation << endl;

    //string sensor_label = "RGBD";
    intrinsics.loadFromConfigFile("GLOBAL", cfg);
    rgb_undist.setFromCamParams(intrinsics);

    cout << "...CameraRotTracker::loadConfiguration\n";
}

void CameraRotTracker::setNewFrame()
{
    //cout << "CameraRotTracker::setNewFrame..." << num_sensors << endl;
    for(size_t i=1; i < num_sensors; i++)
    {
//        if(obsRGBD[i])
//            obsRGBD[i]->unload();
        obsRGBD[i] = obsRGBD[i-1];
        v_rgb[i] = v_rgb[i-1];
        cv::swap(v_rgb[i], v_rgb[i-1]);
        cv::swap(v_gray[i], v_gray[i-1]);
        cv::swap(v_depth[i], v_depth[i-1]);
        v_cloud[i].swap(v_cloud[i-1]);
        vv_segments2D[i] = vv_segments2D[i-1];
        vv_segmentsDesc[i] = vv_segmentsDesc[i-1];
        vv_length[i] = vv_length[i-1];
        vv_seg_contrast[i] = vv_seg_contrast[i-1];
        vv_segment_n[i] = vv_segment_n[i-1];
        vv_pt_coor[i] = vv_pt_coor[i-1];
        vv_pt_normal[i] = vv_pt_normal[i-1];
        vv_pt_robust[i] = vv_pt_robust[i-1];
    }
}

bool CameraRotTracker::computeRobustNormal(const cv::Mat & depth, const mrpt::utils::TCamera & cam, const int u, const int v, Eigen::Vector3f & normal, const int radius, const float max_angle_cos)
{
    ASSERT_(radius > 0 && depth.type() == CV_32FC1);
    if( u-radius < 0 || u+radius >= depth.cols || v-radius < 0 || v+radius >= depth.rows ) // A robust normal cannot be computed at the limit of the image
        return false;

    float *_depth = reinterpret_cast<float*>(depth.data);
    int row_stride = depth.step / sizeof(float);

    const int patch_size = (radius+1)*(radius+1);
    Eigen::MatrixXf pts(3,patch_size);
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    int num_pts(0);
    for(int r=v-radius; r <= v+radius; r++)
        for(int c=u-radius; c <= u+radius; c++)
        {
            float d = _depth[c+r*row_stride];
            if( d < 0.3f || d > 10.f)
                return false;
            Eigen::Vector3f pt(d*(u-cam.cx())/cam.fx(), d*(v-cam.cy())/cam.fy(), d);
            pts.col(num_pts++) = pt;
            center += pt;
        }
    if(num_pts < 8)
        return false;

    pts = pts.leftCols(num_pts);
    center /= num_pts;
    pts -= center.replicate(1,num_pts);
    Eigen::Matrix3f M = Eigen::Matrix3f::Zero();
    for (int i = 0; i < num_pts; i++)
        M += pts.block<3,1>(0,i) * pts.block<3,1>(0,i).transpose();

    Eigen::EigenSolver<MatrixXf> es(M);
    JacobiSVD<MatrixXf> svd(M, ComputeThinU | ComputeThinV);
    normal = svd.matrixU().col(2);

    cout << "eigenvalues " << es.eigenvalues().transpose() << " eigenvectors\n" << es.eigenvectors() << endl;
    cout << "normal " << normal.transpose() << " eigenvectors\n" << svd.singularValues().transpose() << endl;

    if(svd.singularValues()[2] / svd.singularValues()[0] < 0.01)
        return true;
    else
        return false;
}

bool CameraRotTracker::computeRobustNormal(const cv::Mat & depth, const float fx, const float fy, const int u, const int v, Eigen::Vector3f & normal, const int radius, const float max_angle_cos)
{
    ASSERT_(radius > 0 && depth.type() == CV_32FC1);
    if( u-radius < 0 || u+radius >= depth.cols || v-radius < 0 || v+radius >= depth.rows ) // A robust normal cannot be computed at the limit of the image
        return false;

    float *_depth = reinterpret_cast<float*>(depth.data);
    int row_stride = depth.step / sizeof(float);
    for(int r=v-radius; r <= v+radius; r++)
        for(int c=u-radius; c <= u+radius; c++)
        {
            float d = _depth[c+r*row_stride];
            cout << "d " << d << endl;
            if( d < 0.3f || d > 10.f)
                return false;
        }

    int p = u + v*row_stride;
    float scx = fx / _depth[p]; // Scale the gradient
    float scy = fy / _depth[p]; // Scale the gradient
    std::vector<Eigen::Vector3f> v_normal(radius);
    int r = 1;
    int rv = row_stride;
    for(int i=0; i < radius; i++, r++, rv+=row_stride)
    {
        float dz_dx1 = _depth[p+r] - _depth[p];
        float dz_dx_1 = _depth[p] - _depth[p-r];
        float dz_dx(0.f);
        float prod_dz_dx = dz_dx1*dz_dx_1;
        if(prod_dz_dx >= 0 && (dz_dx1 > 0 || dz_dx_1 > 0) )
            dz_dx = 2*prod_dz_dx / (dz_dx1 + dz_dx_1);
        std::cout << "depth " << i+1 << " " << _depth[p+r] << " " << _depth[p] << " " << _depth[p-r] << std::endl;
        std::cout << i+1 << " dz_dx " << dz_dx << " dz_dx1 " << dz_dx1 << " dz_dx_1 " << dz_dx_1 << std::endl;

        float dz_dy1 = _depth[p+rv] - _depth[p];
        float dz_dy_1 = _depth[p] - _depth[p-rv];
        float dz_dy(0.f);
        float prod_dz_dy = dz_dy1*dz_dy_1;
        if(prod_dz_dy >= 0 && (dz_dy1 > 0 || dz_dy_1 > 0) )
            dz_dy = 2*prod_dz_dy / (dz_dy1 + dz_dy_1);
        std::cout << "depth " << i+1 << " " << _depth[p+rv] << " " << _depth[p] << " " << _depth[p-rv
                     ] << std::endl;
        std::cout << i+1 << " dz_dy " << dz_dy << " dz_dy1 " << dz_dy1 << " dz_dy_1 " << dz_dy_1 << std::endl;

        Eigen::Vector3f n1(-dz_dx1*scx, -dz_dy1*scy, 1.f);
        n1.normalize();
        Eigen::Vector3f n_1(-dz_dx_1*scx, -dz_dy_1*scy, 1.f);
        n_1.normalize();
        if( n1.dot(n_1) < max_angle_cos )
            return false;

        v_normal[i] << -dz_dx*scx, -dz_dy*scy, 1.f;
        v_normal[i].normalize();

        for(int j=0; j < i; j++)
            if( v_normal[i].dot(v_normal[j]) < max_angle_cos )
                return false;

        scx *= 0.5f; // Update scale according to the radius
        scy *= 0.5f; // Update scale according to the radius
    }
    normal.setZero();
    for(int i=0; i < radius; i++)
        normal += v_normal[i];
    normal.normalize();
    return true;
}

vector<cv::Vec2i> CameraRotTracker::getDistributedNormals(cv::Mat & depth, const mrpt::utils::TCamera & cam, vector<Eigen::Vector3f> & v_normal, vector<bool> & v_robust_normal, const int h_divisions, const int v_divisions)
{
    cout << "CameraRotTracker::getDistributedNormals..." << depth.cols << "x" << depth.rows << endl;
    int h_slice = depth.cols / h_divisions;
    int v_slice = depth.rows / v_divisions;
    size_t n_pts = h_divisions*v_divisions;
    vector<cv::Vec2i> pix_coor(n_pts);
    v_normal.resize(n_pts);
    v_robust_normal.resize(n_pts);
    size_t pt_id = 0;
    int u = h_slice/2;
    const int radius = 2;
    const float max_angle_cos = cos(DEG2RAD(1));
    for(int c=0; c < h_divisions; c++, u+=h_slice)
    {
        int v = v_slice/2;
        for(int r=0; r < v_divisions; r++, v+=v_slice, pt_id++)
        {
            cout << "pt_coor " << u << " " << v << endl;
            pix_coor[pt_id][0] = u;
            pix_coor[pt_id][1] = v;
            v_robust_normal[pt_id] = computeRobustNormal(depth, cam, u, v, v_normal[pt_id], radius, max_angle_cos);
        }
    }

    return pix_coor;
}

bool CameraRotTracker::getRobustNormal(pcl::PointCloud<pcl::Normal>::Ptr img_normals, const int u, const int v, Eigen::Vector3f & normal, const int radius, const float max_angle_cos)
{
    ASSERT_(radius > 0 && !img_normals->empty());
    if( u-radius < 0 || u+radius >= int(img_normals->width) || v-radius < 0 || v+radius >= int(img_normals->height) ) // A robust normal cannot be computed at the limit of the image
        return false;

    size_t p = u + v*img_normals->width;
    normal = Map<Vector3f>(img_normals->points[p].normal);
    if( std::isnan(normal(0)) )
        return false;

    Vector3f av_normal = Vector3f::Zero();
    for(int r=v-radius; r <= v+radius; r++)
        for(int c=u-radius; c <= u+radius; c++)
        {
            Map<Vector3f> n2(img_normals->points[c + r*img_normals->width].normal);
//            if( std::isnan(n2(0)) )
//                cout << "\n\n NaN pt_coor " << u << " " << v << endl;
            if( normal.dot(n2) < max_angle_cos ){ //cout << "Inconsistent normal " << normal.transpose() << " av_normal " << n2.transpose() << " angle " << RAD2DEG(acos(normal.dot(n2))) << endl;
                return false;}
//            else
//                cout << "OK normal " << normal.transpose() << " av_normal " << n2.transpose() << " angle " << RAD2DEG(acos(normal.dot(n2))) << endl;

            av_normal += n2;
        }
    av_normal.normalize();

    //cout << "normal " << normal.transpose() << " av_normal " << av_normal.transpose() << " angle " << RAD2DEG(acos(normal.dot(av_normal))) << endl;
    normal = av_normal;

    return true;
}

vector<cv::Vec2i> CameraRotTracker::getDistributedNormals(pcl::PointCloud<pcl::Normal>::Ptr img_normals, vector<Eigen::Vector3f> & v_normal, vector<bool> & v_robust_normal, const int h_divisions, const int v_divisions)
{
    //cout << "CameraRotTracker::getDistributedNormals..." << img_normals->width << "x" << img_normals->height << endl;
    int h_slice = img_normals->width / h_divisions;
    int v_slice = img_normals->height / v_divisions;
    size_t n_pts = h_divisions*v_divisions;
    vector<cv::Vec2i> pix_coor(n_pts);
    v_normal.resize(n_pts);
    v_robust_normal.resize(n_pts);
    size_t pt_id(0);
    size_t valid_pts(0);
    int u = h_slice/2;
    const int radius = 1;
    const float max_angle_cos = cos(DEG2RAD(4));
    for(int c=0; c < h_divisions; c++, u+=h_slice)
    {
        int v = v_slice/2;
        for(int r=0; r < v_divisions; r++, v+=v_slice, pt_id++)
        {
            //cout << "\tpt_coor " << u << " " << v << endl;
            pix_coor[pt_id][0] = u;
            pix_coor[pt_id][1] = v;
            v_robust_normal[pt_id] = getRobustNormal(img_normals, u, v, v_normal[valid_pts], radius, max_angle_cos);
            if(v_robust_normal[pt_id])
                ++valid_pts;
        }
    }
    v_normal.resize(valid_pts);

    return pix_coor;
}
