/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
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

// 0 load init file with estimated poses and errors. display/verbose parameters
// 1 open rawlog
// 2 identify observation pairs/triples...
// 3 segment planes and lines
// 4 get correspondences
// 5 perform calibration. calibration algorithm in a different file
// 6 visualize and evaluate

#include <mrpt/math/CArray.h>
#include <mrpt/slam/CRawlog.h>
#include <mrpt/gui/CDisplayWindow3D.h>
//#include <mrpt/gui/CDisplayWindowPlots.h>
//#include <mrpt/opengl/CGridPlaneXY.h>
//#include <mrpt/opengl/CPointCloud.h>
//#include <mrpt/opengl/stock_objects.h>
//#include <mrpt/opengl/CTexturedPlane.h>
//#include <mrpt/utils/CConfigFile.h>
//#include <mrpt/utils/CFileGZInputStream.h>
////#include <mrpt/utils/CFileGZOutputStream.h>
//#include <mrpt/system/os.h>
//#include <mrpt/system/threads.h>
//#include <mrpt/system/filesystem.h>
//#include <mrpt/opengl/CPlanarLaserScan.h>  // This class lives in the lib [mrpt-maps] and must be included by hand
//#include <mrpt/math/ransac_applications.h>



using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
using namespace mrpt::gui;
//using namespace mrpt::system;
//using namespace mrpt::math;
//using namespace mrpt::poses;
//using namespace mrpt::utils;
using namespace std;
using namespace Eigen;


template<typename typedata,int m_rows,int m_cols>
CMatrixFixedNumeric<typedata,m_rows,m_cols> getCMatrix(const Matrix<typedata,m_rows,m_cols> &matrix_eigen)
{
    CMatrixFixedNumeric<typedata,m_rows,m_cols> m_CMatrix;
    for(int r=0; r < matrix_eigen.rows(); r++)
        for(int c=0; c < matrix_eigen.cols(); c++)
            m_CMatrix(r,c) = matrix_eigen(r,c);

    return m_CMatrix;
}

// Return a diagonal matrix where the values of the diagonal are assigned from the input vector
template<typename typedata>
Matrix<typedata,Dynamic,Dynamic> getDiagonalMatrix(const Matrix<typedata,Dynamic,Dynamic> &matrix_generic)
{
    assert(matrix_generic.cols() == matrix_generic.rows());

    size_t m_size = matrix_generic.cols();
    Matrix<typedata,Dynamic,Dynamic> m_diag = Matrix<typedata,Dynamic,Dynamic>::Zero(m_size,m_size);
    for(size_t i=0; i < m_size; i++)
        m_diag(i,i) = matrix_generic(i,i);

    return m_diag;
}


// Line observation from a 2D LRF.
// It stores the center of the line segment and a unitary direction vector, together with the covariances of both.
struct line2D_obs
{
    Matrix<double,2,1> center; // The center of the observed line segment
    Matrix<double,2,2> cov_center;
    Matrix<double,2,1> dir; // A unitary direction vector
    Matrix<double,2,2> cov_dir;
};

// Line observation from a 2D LRF in 3D coordinates.
struct line_3D
{
    // void line_3D(const line2D_obs &line_2D, const CPose3D &LRF_pose)
    void get_3D_params(const line2D_obs &line_2D, const CPose3D &LRF_pose)
    {
        // center = LRF_pose.getRotationMatrix()*line_2D.center + LRF_pose.m_coords;
        center_rot = LRF_pose.getRotationMatrix().block(0,0,3,2)*line_2D.center;
        center = center_rot + LRF_pose.m_coords;
        cov_center = LRF_pose.getRotationMatrix().block(0,0,3,2)* line_2D.cov_center * LRF_pose.getRotationMatrix().block(0,0,3,2).transpose();
        dir = LRF_pose.getRotationMatrix().block(0,0,3,2)*line_2D.dir;
        cov_dir = LRF_pose.getRotationMatrix().block(0,0,3,2) * line_2D.cov_dir * LRF_pose.getRotationMatrix().block(0,0,3,2).transpose();
    };

    Matrix<double,3,1> center; // The center of the observed line segment
    Matrix<double,3,1> center_rot; // The rotated center (translation is not applied)
    Matrix<double,3,3> cov_center;
    Matrix<double,3,1> dir; // A unitary direction vector
    Matrix<double,3,3> cov_dir;
};

// Corner Observation by a single LRF
struct plane_corresp
{
    // Two lines observed from the LRF
    unsigned id_LRF;
    CArray<line2D_obs,2> lines; // The 2 lines detected by the LRF1
};

typedef CArray<CO_1,2> CO; // Corner Observation by 2 LRFs

typedef CArray<CArray<line_3D,2>,2> CO_3D; // Corner Observation in 3D

//typedef CMatrixTemplateNumeric<double> CO_vector; // Corner Observation by 2 LRFs
//typedef CMatrixFixedNumeric<double,50,1> CO_vector; // Corner Observation by 2 LRFs
typedef Matrix<double,50,1> CO_vector; // Corner Observation by 2 LRFs

//// The vector form of a CO is needed by RANSAC
//CO_vector CO2vector(const CO &co)
//{
//    CO_vector co_vector;
//    for(size_t LRF_id=0; LRF_id < 2; LRF_id++) // 2 LRFs per CO
//    {
//        co_vector(25*LRF_id,0) = co[LRF_id].id_LRF;
//        for(size_t plane_id=0; plane_id < 2; plane_id++) // 2 line observations per LRF
//        {
//            size_t pos_block = 25*LRF_id + 12*plane_id + 1;
//            co_vector.block(pos_block,0,2,1) = co[LRF_id].lines[plane_id].dir;
//            co_vector.block(pos_block+2,0,2,1) = co[LRF_id].lines[plane_id].cov_dir.block(0,0,2,1);
//            co_vector.block(pos_block+4,0,2,1) = co[LRF_id].lines[plane_id].cov_dir.block(0,1,2,1);
//            //      co_vector.block(pos_block+2,0,4,1) = Map<Matrix<double,4,1>(co[LRF_id].lines[plane_id].cov_center);
//            co_vector.block(pos_block+6,0,2,1) = co[LRF_id].lines[plane_id].center;
//            co_vector.block(pos_block+8,0,2,1) = co[LRF_id].lines[plane_id].cov_center.block(0,0,2,1);
//            co_vector.block(pos_block+10,0,2,1) = co[LRF_id].lines[plane_id].cov_center.block(0,1,2,1);
//        }
//    }

//    return co_vector;
//}

//CO vector2CO(const CO_vector &co_vector)
//{
//    CO co;
//    for(size_t LRF_id=0; LRF_id < 2; LRF_id++) // 2 LRFs per CO
//    {
//        co[LRF_id].id_LRF = co_vector(25*LRF_id,0);
//        for(size_t plane_id=0; plane_id < 2; plane_id++) // 2 line observations per LRF
//        {
//            size_t pos_block = 25*LRF_id + 12*plane_id + 1;
//            co[LRF_id].lines[plane_id].dir = co_vector.block(pos_block,0,2,1);
//            co[LRF_id].lines[plane_id].cov_dir.block(0,0,2,1) = co_vector.block(pos_block+2,0,2,1);
//            co[LRF_id].lines[plane_id].cov_dir.block(0,1,2,1) = co_vector.block(pos_block+4,0,2,1);
//            co[LRF_id].lines[plane_id].center = co_vector.block(pos_block+6,0,2,1);
//            co[LRF_id].lines[plane_id].cov_center.block(0,0,2,1) = co_vector.block(pos_block+8,0,2,1);
//            co[LRF_id].lines[plane_id].cov_center.block(0,1,2,1) = co_vector.block(pos_block+10,0,2,1);
//        }
//    }

//    return co;
//}


const char *example_cfg_txt =
        "; ---------------------------------------------------------------\n"
        "; FILE: kinect-rig-calib parameters.txt\n"
        "; ---------------------------------------------------------------\n\n"

        "[INPUT]\n\n"

        ";Absolute path of the rawlog file \n"
        "filename = C:/Users/Mariano/Desktop/rawlog_rgbd_dataset_freiburg1_desk/rgbd_dataset_freiburg1_desk.rawlog \n"

        "[OUTPUT]\n\n"

        ";Path of the rawlog file \n"
        "save_path = /home/efernand/rgbd-rig-calib.txt \n\n"

        ";display: 0 / 1 \n"
        "display = 1 \n\n"

        ";verbose: 0 / 1 \n"
        "verbose = 1 \n\n";

void print_help(char ** argv)
{
    cout << "This program computes the extrinsic calibration of several RGB-D sensors (e.g. Kinect) based on plane and line correspondences." << endl
         << "No overlapping required for the sensor FOV, as far as the planes/lines can be observed simultaneously by pairs of sensors." << endl
         << "This program accepts a single argument specifying a configuration file which indicates the rawlog file of sensor observations" << endl
         << "to compute the calibration, the approximated sensor poses, and a conservative approximation of their accuracy. The output text" << endl
         << "file with the calibration results, and display/verbose parameters can also be set in the configuration.\n" << endl;

    cout << "Usage: " << argv[0] << " <config_file> \n";
    cout << "    <config_file> configuration file which contains the information of the RGBD sequences and estimated calibration" << endl;
    cout << "         " << argv[0] << " -h | --help : shows this help" << endl;
}

void print_help(char ** argv)
{
    cout << "\nThis program performs Visual-Range Odometry (direct registration) on a sequence of RGB-D images.\n\n";

    cout << "Usage: " << argv[0] << " <config_file> \n";
    cout << "    <config_file> configuration file which contains the information of the RGBD sequence, the VO parameters and the groundtruth (if available)." << endl;
    cout << "    <path_results> is the directory where results are saved (v_trajectories, images, etc.)" << endl;
    cout << "         " << argv[0] << " -h | --help : shows this help" << endl;
}

// ------------------------------------------------------
//                         MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
    try
    {
        bool showHelp = argc != 2 || !os::_strcmp(argv[1],"-h") || !os::_strcmp(argv[1],"--help");
        if(showHelp)
        {
            print_help(argv);
            return 0;
        }

        printf(" kinect-rig-calib - Part of the MRPT\n");
        printf(" MRPT C++ Library: %s - BUILD DATE %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());


        printf("-------------------------------------------------------------------\n");

        // Process arguments:
        if (argc<2 || showHelp )
        {
            printf("Usage: %s <config_file.ini> <dataset.rawlog>\n\n",argv[0]);
            if (!showHelp)
            {
                mrpt::system::pause();
                return -1;
            }
            else	return 0;
        }

        const string INI_FILENAME = string( argv[1] );
//        string INI_FILENAME = "/home/edu/Libraries/mrpt_edu/share/mrpt/config_files/kinect-rig-calib/calib-3LRFs-Hokuyo_30LX.ini";
        ASSERT_FILE_EXISTS_(INI_FILENAME)

        string override_rawlog_file;
        if (argc>=3)
            override_rawlog_file = string(argv[2]);

        // Run:
        calib_LRFs_rawlog_ini(INI_FILENAME,override_rawlog_file);

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
