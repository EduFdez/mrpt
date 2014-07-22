/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: LRF-calib
	FILE: LRF-calib_main.cpp
	AUTHOR: Eduardo Fernández-Moral <efernandezmoral@gmail.com>

	See README.txt for instructions or
          http://www.mrpt.org/Application:LRF-calib
  ---------------------------------------------------------------*/

#include <mrpt/math/CArray.h>
#include <mrpt/slam/CRawlog.h>
//#include <mrpt/opengl/COpenGLScene.h>
//#include <mrpt/opengl/CGridPlaneXY.h>
//#include <mrpt/opengl/stock_objects.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
//#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
//#include <mrpt/system/threads.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/opengl/CPlanarLaserScan.h>  // This class lives in the lib [mrpt-maps] and must be included by hand
//#include <mrpt/gui/CDisplayWindow3D.h>

#include <mrpt/math/ransac_applications.h>

//#include <mrpt/base.h>
////#include <mrpt/gui.h>
////#include <mrpt/opengl.h>
//#include <mrpt/slam.h>
//#include <mrpt/utils.h>
//#include <mrpt/obs.h>

#define DEBUG 1
#define NUM_SENSORS 3

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::opengl;
//using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace std;
using namespace Eigen;

//// Forward declaration.
//vector<CPose3D> calibrate_LRFs(const vector<CO> &vCOs, vector<CPose3D> LRF_poses_estim, set<unsigned> idx_estim_LRFs, const size_t id_fixed_LRF=0);

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
struct CO_1
{
    // Two lines observed from the LRF
    unsigned id_LRF;
    CArray<line2D_obs,2> lines; // The 2 lines detected by the LRF1
};

typedef CArray<CO_1,2> CO; // Corner Observation by 2 LRFs

typedef CArray<CArray<line_3D,2>,2> CO_3D; // Corner Observation in 3D

//typedef CMatrixTemplateNumeric<double> CO_vector; // Corner Observation by 2 LRFs
//typedef CMatrixFixedNumeric<double,48,1> CO_vector; // Corner Observation by 2 LRFs
typedef Matrix<double,48,1> CO_vector; // Corner Observation by 2 LRFs

// The vector form of a CO is needed by RANSAC
CO_vector CO2vector(const CO &co)
{
  CO_vector co_vector;
  for(size_t LRF_id=0; LRF_id < 2; LRF_id++) // 2 LRFs per CO
    for(size_t plane_id=0; plane_id < 2; plane_id++) // 2 line observations per LRF
    {
      size_t pos_block = 24*LRF_id + 12*plane_id;
      co_vector.block(pos_block,0,2,1) = co[LRF_id].lines[plane_id].center;
      co_vector.block(pos_block+2,0,2,1) = co[LRF_id].lines[plane_id].cov_center.block(0,0,2,1);
      co_vector.block(pos_block+4,0,2,1) = co[LRF_id].lines[plane_id].cov_center.block(0,1,2,1);
//      co_vector.block(pos_block+2,0,4,1) = Map<Matrix<double,4,1>(co[LRF_id].lines[plane_id].cov_center);
      co_vector.block(pos_block+6,0,2,1) = co[LRF_id].lines[plane_id].dir;
      co_vector.block(pos_block+8,0,2,1) = co[LRF_id].lines[plane_id].cov_dir.block(0,0,2,1);
      co_vector.block(pos_block+10,0,2,1) = co[LRF_id].lines[plane_id].cov_dir.block(0,1,2,1);
    }

  return co_vector;
}

CO vector2CO(const CO_vector &co_vector)
{
  CO co;
  for(size_t LRF_id=0; LRF_id < 2; LRF_id++) // 2 LRFs per CO
  {
    co[LRF_id].id_LRF = LRF_id;

    for(size_t plane_id=0; plane_id < 2; plane_id++) // 2 line observations per LRF
    {
      size_t pos_block = 24*LRF_id + 12*plane_id;
      co[LRF_id].lines[plane_id].center = co_vector.block(pos_block,0,2,1);
      co[LRF_id].lines[plane_id].cov_center.block(0,0,2,1) = co_vector.block(pos_block+2,0,2,1);
      co[LRF_id].lines[plane_id].cov_center.block(0,1,2,1) = co_vector.block(pos_block+4,0,2,1);
      co[LRF_id].lines[plane_id].dir = co_vector.block(pos_block+6,0,2,1);
      co[LRF_id].lines[plane_id].cov_dir.block(0,0,2,1) = co_vector.block(pos_block+8,0,2,1);
      co[LRF_id].lines[plane_id].cov_dir.block(0,1,2,1) = co_vector.block(pos_block+10,0,2,1);
    }
  }

  return co;
}


/**  Compute the error of the vCOs for the given calibration (LRF_poses_estim) and LRF_indices.
  */
double error_COs(const vector<CO> &vCOs, const vector<CPose3D> &LRF_poses_estim, const set<unsigned> &LRF_indices)
{
    double error = 0.0; // Squared error averaged by the number of vCOs

    for(size_t i=0; i < vCOs.size(); i++)
    {
        CO_3D co_i_3d;
        for(size_t LRF_id=0; LRF_id < 2; LRF_id++) // 2 LRFs per CO
        {
            for(size_t plane_id=0; plane_id < 2; plane_id++) // 2 line observations per LRF
            {
                co_i_3d[LRF_id][plane_id].get_3D_params(vCOs[i][LRF_id].lines[plane_id], LRF_poses_estim[vCOs[i][LRF_id].id_LRF]);
            }
        }

        vector<Matrix<double,3,1> > vNormal(2);
        vector<Matrix<double,3,3> > cov_vNormal(2);
        double sigma_planar_constraint[2];
        double error_planarity[2];
        for(size_t plane_id=0; plane_id < 2; plane_id++) // 2 line observations per LRF
        {
            //      crossProduct3D(co_i_3d[0][plane_id].dir, co_i_3d[1][plane_id].dir, vNormal[plane_id]);
            vNormal[plane_id] = co_i_3d[0][plane_id].dir.cross(co_i_3d[1][plane_id].dir);

            cov_vNormal[plane_id] = -skew_symmetric3(co_i_3d[0][plane_id].dir)*co_i_3d[1][plane_id].cov_dir*skew_symmetric3(co_i_3d[0][plane_id].dir)
                                    -skew_symmetric3(co_i_3d[1][plane_id].dir)*co_i_3d[0][plane_id].cov_dir*skew_symmetric3(co_i_3d[1][plane_id].dir);
            sigma_planar_constraint[plane_id] = sqrt(
                    (vNormal[plane_id].transpose()*(co_i_3d[0][plane_id].cov_center+co_i_3d[1][plane_id].cov_center)*vNormal[plane_id])(0,0) +
                    ((co_i_3d[0][plane_id].center-co_i_3d[1][plane_id].center).transpose()*cov_vNormal[plane_id]*(co_i_3d[0][plane_id].center-co_i_3d[1][plane_id].center))(0,0) );

            // Compute the residuals of the planarity constraints
            error_planarity[plane_id] = (vNormal[plane_id].transpose()*(co_i_3d[1][plane_id].center-co_i_3d[0][plane_id].center))(0,0) / sigma_planar_constraint[plane_id];
        }

        Matrix<double,3,3> R_relative = LRF_poses_estim[vCOs[i][0].id_LRF].getRotationMatrix().transpose() * LRF_poses_estim[vCOs[i][1].id_LRF].getRotationMatrix();
        double sigma_orthogonal_constraint = sqrt((vNormal[0].transpose()*R_relative*cov_vNormal[1]*R_relative.transpose()*vNormal[0])(0,0) +
                                                   (vNormal[1].transpose()*R_relative.transpose()*cov_vNormal[0]*R_relative*vNormal[1])(0,0) );

        // Compute the residual of the orthogonality constraint
        double error_orthogonality = vNormal[0].dot(vNormal[1]) / sigma_orthogonal_constraint;

        error += error_orthogonality + error_planarity[0] + error_planarity[1];
    }
    error /= vCOs.size(); // Average CO error

    return error;
}


// ------------------------------------------------------------------------------------------------------------
//				                            Calibrate_LRFs
// Calibrate X LRFs by registering perpendicular plane observations, also called CO (Corner Observations).
// At least two CO are needed, these may be obtained from a single observation of the rig (i.e. a corner with
// 3 perpendicular planes). The problem is solved applying LS to the constraint residuals, using Levenberg-Marquardt
// ------------------------------------------------------------------------------------------------------------
vector<CPose3D> calibrate_LRFs(const vector<CO> &vCOs, vector<CPose3D> LRF_poses_estim, set<unsigned> idx_estim_LRFs = set<unsigned>(), const size_t id_fixed_LRF=0)
{
  if(idx_estim_LRFs.empty())
    for(unsigned j=0; j < LRF_poses_estim.size(); j++)
      idx_estim_LRFs.insert(j);

  assert(idx_estim_LRFs.counts(id_fixed_LRF)); // The LRF chosen as the fixed reference must be one of the LRFs being calibrated

  //vector<CPose3D> LRF_poses_estim = init_poses;
  vector<CPose3D> LRF_poses_estim_temp = LRF_poses_estim;

  size_t system_DoF = 6*(idx_estim_LRFs.size()-1);
//  CMatrixDouble Hessian(system_DoF,system_DoF);
//  Hessian.setZero();
//  CMatrixDouble Gradient(system_DoF,1);
//  Gradient.setZero();
  MatrixXd Hessian = MatrixXd::Zero(system_DoF,system_DoF);
  VectorXd Gradient = VectorXd::Zero(system_DoF);

  // Set Levenberg-Marquardt parameters
  double lambda = 0.001;
  double step = 10; // Update step
  double max_LM_it_lambda = 5;

  // Solve with Levenberg-Marquardt
  size_t iterations = 0;
  double diff_error = 1;
//  CMatrixDouble update(system_DoF,1); // Update poses vector
  VectorXd update(system_DoF); // Update poses vector
  update(0,0) = 1;
  double tol = 10^(-24); // Tolerance
  size_t maxIters = 100;
  double error = error_COs(vCOs, LRF_poses_estim, idx_estim_LRFs);
  while(iterations < maxIters && update.norm() > tol && diff_error > tol )
  {
    for(size_t i=0; i < vCOs.size(); i++)
    {
      CO_3D co_i_3d;
      for(size_t LRF_id=0; LRF_id < 2; LRF_id++) // 2 LRFs per CO
      {
          for(size_t plane_id=0; plane_id < 2; plane_id++) // 2 line observations per LRF
          {
              co_i_3d[LRF_id][plane_id].get_3D_params(vCOs[i][LRF_id].lines[plane_id], LRF_poses_estim[vCOs[i][LRF_id].id_LRF]);
          }
      }

      vector<Matrix<double,3,1> > vNormal(2);
      vector<Matrix<double,3,3> > cov_vNormal(2);

      // Compute the residual and the Jacobian of the orthogonality constraint
      double sigma_planar_constraint[2];
      double error_planarity[2];
      for(size_t plane_id=0; plane_id < 2; plane_id++) // 2 line observations per LRF
      {
        //      crossProduct3D(co_i_3d[0][plane_id].dir, co_i_3d[1][plane_id].dir, vNormal[plane_id]);
        vNormal[plane_id] = co_i_3d[0][plane_id].dir.cross(co_i_3d[1][plane_id].dir);

        cov_vNormal[plane_id] = -skew_symmetric3(co_i_3d[0][plane_id].dir)*co_i_3d[1][plane_id].cov_dir*skew_symmetric3(co_i_3d[0][plane_id].dir)
                                -skew_symmetric3(co_i_3d[1][plane_id].dir)*co_i_3d[0][plane_id].cov_dir*skew_symmetric3(co_i_3d[1][plane_id].dir);
        sigma_planar_constraint[plane_id] = sqrt(
                (vNormal[plane_id].transpose()*(co_i_3d[0][plane_id].cov_center+co_i_3d[1][plane_id].cov_center)*vNormal[plane_id])(0,0) +
                ((co_i_3d[0][plane_id].center-co_i_3d[1][plane_id].center).transpose()*cov_vNormal[plane_id]*(co_i_3d[0][plane_id].center-co_i_3d[1][plane_id].center))(0,0) );

        // Compute the residuals of the co-planarity constraints
        error_planarity[plane_id] = vNormal[plane_id].dot(co_i_3d[0][plane_id].center-co_i_3d[1][plane_id].center) / sigma_planar_constraint[plane_id];

        for(size_t LRF_id=0; LRF_id < 2; LRF_id++) // 2 LRFs per CO
          if(vCOs[i][LRF_id].id_LRF != id_fixed_LRF)
          {
            Matrix<double,1,6> jac_error_planarity;
            jac_error_planarity.block(3,0,1,3) = (-vNormal[plane_id].transpose()*skew_symmetric3(co_i_3d[1][plane_id].center_rot) +
                                                      (co_i_3d[0][plane_id].center-co_i_3d[1][plane_id].center).transpose()*skew_symmetric3(co_i_3d[1][plane_id].dir)*skew_symmetric3(co_i_3d[0][plane_id].dir))
                                                      / sigma_planar_constraint[plane_id];  // Jacobian wrt the rotation
            jac_error_planarity.block(0,0,1,3) = vNormal[plane_id].transpose() / sigma_planar_constraint[plane_id]; // Jacobian wrt the translation

            size_t n_block = (vCOs[i][LRF_id].id_LRF < id_fixed_LRF ) ? 6*(std::distance(idx_estim_LRFs.begin(), idx_estim_LRFs.find(vCOs[i][LRF_id].id_LRF))) :
                                                                         6*(std::distance(idx_estim_LRFs.begin(), idx_estim_LRFs.find(vCOs[i][LRF_id].id_LRF))-1);
            Hessian.block(n_block,n_block,6,6) += jac_error_planarity.transpose() * jac_error_planarity;
            Gradient.block(n_block,0,6,1) += jac_error_planarity.transpose() * error_planarity[plane_id];
          }
      }

      // Compute the residual and the Jacobian of the orthogonality constraint
      Matrix<double,3,3> R_relative = LRF_poses_estim[vCOs[i][0].id_LRF].getRotationMatrix().transpose() * LRF_poses_estim[vCOs[i][1].id_LRF].getRotationMatrix();
      double sigma_orthogonal_constraint = sqrt( (vNormal[0].transpose()*R_relative*cov_vNormal[1]*R_relative.transpose()*vNormal[0])(0,0) +
                                                  (vNormal[1].transpose()*R_relative.transpose()*cov_vNormal[0]*R_relative*vNormal[1])(0,0) );
      double error_orthogonality = vNormal[0].dot(vNormal[1]) / sigma_orthogonal_constraint;

      for(size_t LRF_id=0; LRF_id < 2; LRF_id++) // 2 LRFs per CO
        if(vCOs[i][LRF_id].id_LRF != id_fixed_LRF)
        {
          Matrix<double,1,3> jac_error_orthogonality;
          if(LRF_id==0)
            jac_error_orthogonality = (vNormal[0].transpose()*skew_symmetric3(co_i_3d[1][1].dir)*skew_symmetric3(co_i_3d[0][1].dir) +
                                       vNormal[1].transpose()*skew_symmetric3(co_i_3d[1][0].dir)*skew_symmetric3(co_i_3d[0][0].dir)) / sigma_orthogonal_constraint;
          else
            jac_error_orthogonality =-(vNormal[0].transpose()*skew_symmetric3(co_i_3d[0][1].dir)*skew_symmetric3(co_i_3d[1][1].dir) +
                                       vNormal[1].transpose()*skew_symmetric3(co_i_3d[0][0].dir)*skew_symmetric3(co_i_3d[1][0].dir)) / sigma_orthogonal_constraint;

          size_t n_block = (vCOs[i][LRF_id].id_LRF < id_fixed_LRF ) ? 6*(std::distance(idx_estim_LRFs.begin(), idx_estim_LRFs.find(vCOs[i][LRF_id].id_LRF)))+3 :
                                                                       6*(std::distance(idx_estim_LRFs.begin(), idx_estim_LRFs.find(vCOs[i][LRF_id].id_LRF))-1)+3;
          Hessian.block(n_block,n_block,3,3) += jac_error_orthogonality.transpose() * jac_error_orthogonality;
          Gradient.block(n_block,0,3,1) += jac_error_orthogonality.transpose() * error_orthogonality;
        }
//        error += error_orthogonality + error_planarity[0] + error_planarity[1];
    }

    if(Hessian.rank() < 6*(idx_estim_LRFs.size()-1))
      return LRF_poses_estim;

    // Compute calib update
    update = -(Hessian + lambda*Hessian.diagonal()).inverse() * Gradient;
    set<unsigned>::iterator it_LRF = idx_estim_LRFs.begin();
    for(size_t j=0; j < idx_estim_LRFs.size(); j++, it_LRF++)
      if(*it_LRF != id_fixed_LRF)
      {
        size_t n_block = (*it_LRF < id_fixed_LRF) ? 6*(std::distance(idx_estim_LRFs.begin(),it_LRF)) : 6*(std::distance(idx_estim_LRFs.begin(),it_LRF))-1;
        mrpt::math::CArrayNumeric<double,6> update_pose = mrpt::math::CArrayNumeric<double,6>(update.block(n_block,0,6,1));
        LRF_poses_estim_temp[j] = mrpt::poses::CPose3D::exp(update_pose) + LRF_poses_estim[j]; // Pose composition
      }

    // Compute new error
    double new_error = error_COs(vCOs, LRF_poses_estim_temp, idx_estim_LRFs);
    double diff_error = error - new_error;
    if(diff_error > 0)
    {
      lambda /= step;
      LRF_poses_estim = LRF_poses_estim_temp;
      error = new_error;
      iterations++;
    }
    else
    {
      unsigned LM_it = 0;
      while(LM_it < max_LM_it_lambda && diff_error < 0)
      {
        lambda *= step;
        update = -(Hessian + lambda*Hessian.diagonal()).inverse() * Gradient;
        set<unsigned>::iterator it_LRF = idx_estim_LRFs.begin();
        for(size_t j=0; j < idx_estim_LRFs.size(); j++, it_LRF++)
          if(*it_LRF != id_fixed_LRF)
          {
            size_t n_block = (*it_LRF < id_fixed_LRF ) ? 6*(std::distance(idx_estim_LRFs.begin(),it_LRF)) : 6*(std::distance(idx_estim_LRFs.begin(),it_LRF))-1;
            mrpt::math::CArrayNumeric<double,6> update_pose = mrpt::math::CArrayNumeric<double,6>(update.block(n_block,0,6,1));
            LRF_poses_estim_temp[j] = mrpt::poses::CPose3D::exp(update_pose) + LRF_poses_estim[j]; // Pose composition
          }

        new_error = error_COs(vCOs, LRF_poses_estim_temp, idx_estim_LRFs);
        diff_error = error - new_error;
        if(diff_error > 0)
        {
            LRF_poses_estim = LRF_poses_estim_temp;
            error = new_error;
            iterations++;
        }
        LM_it++;
      }
    }
  }

  return LRF_poses_estim;
}

/*---------------------------------------------------------------
		Aux. functions needed by ransac_LRFcalib
 ---------------------------------------------------------------*/
CPose3D guess_rel_pose12;

void  ransac_LRFcalib_fit(
  const CMatrixTemplateNumeric<double> &allData,
  const vector_size_t &useIndices,
  vector< CMatrixTemplateNumeric<double> > &fitModels )
{
  ASSERT_(useIndices.size()==2); // A minimum of 2 CO is required to compute the calibration

  vector<CPose3D> LRF_poses_estim(2);
  LRF_poses_estim[0] = CPose3D(0,0,0);
  LRF_poses_estim[1] = guess_rel_pose12;

  vector<CO> COs_sample(2);
  for(unsigned i=0; i < useIndices.size(); i++)
    COs_sample[i] = vector2CO(allData.block(0,useIndices[i],48,1));

  try
  {
    vector<CPose3D> LRF_calib = calibrate_LRFs(COs_sample, LRF_poses_estim);
    fitModels.resize(1);
    fitModels[0] = LRF_calib[1].getHomogeneousMatrixVal();
//    fitModels[0] = LRF_calib[1].getHomogeneousMatrix(fitModels[0]);
//    CMatrixTemplateNumeric<double> &M = fitModels[0];
//    M = ;
//    for (size_t i=0;i<3;i++)
//      M(0,i)=line.coefs[i];
  }
  catch(exception &)
  {
    fitModels.clear();
    return;
  }
}


void ransac_LRFcalib_distance(
  const CMatrixTemplateNumeric<double> &allData,
  const vector< CMatrixTemplateNumeric<double> > & testModels,
  const double distanceThreshold,
  unsigned int & out_bestModelIndex,
  vector_size_t & out_inlierIndices )
{
  out_inlierIndices.clear();
  out_bestModelIndex = 0;

  if (testModels.empty()) return; // No model, no inliers.

  ASSERTMSG_( testModels.size()==1, format("Expected testModels.size()=1, but it's = %u",static_cast<unsigned int>(testModels.size()) ) )
  const CMatrixTemplateNumeric<double> &M = testModels[0]; // M stores a 4x4 matrix with the relative pose between a pair of LRFs

  ASSERT_( size(M,1)==4 && size(M,2)==4 )

//  CPose3D relative_pose12(M);
  vector<CPose3D> LRF_poses_estim(2);
  LRF_poses_estim[0] = CPose3D(0,0,0);
  LRF_poses_estim[1] = CPose3D(M);

  const size_t N = size(allData,2);
  out_inlierIndices.reserve(1000);
  for (size_t i=0;i<N;i++)
  {
    CO_3D co_i_3d;
    CO co_i = vector2CO(allData.block(0,i,48,1));
    for(size_t LRF_id=0; LRF_id < 2; LRF_id++) // 2 LRFs per CO
      for(size_t plane_id=0; plane_id < 2; plane_id++) // 2 line observations per LRF
        co_i_3d[LRF_id][plane_id].get_3D_params(co_i[LRF_id].lines[plane_id], LRF_poses_estim[LRF_id]);

    vector<Matrix<double,3,1> > vNormal(2);
    double error_planarity[2];
    for(size_t plane_id=0; plane_id < 2; plane_id++) // 2 line observations per LRF
    {
//      crossProduct3D(co_i_3d[0][plane_id].dir, co_i_3d[1][plane_id].dir, vNormal[plane_id]);
      vNormal[plane_id] = co_i_3d[0][plane_id].dir.cross(co_i_3d[1][plane_id].dir);

      // Compute the residuals of the planarity constraints
      error_planarity[plane_id] = (vNormal[plane_id].transpose()*(co_i_3d[1][plane_id].center-co_i_3d[0][plane_id].center))(0,0);
    }

    // Compute the residual of the orthogonality constraint
    double error_orthogonality = vNormal[0].dot(vNormal[1]);

//  cout << "distance " << d << " " << allData.get_unsafe(0,i) << " " << allData.get_unsafe(1,i) << endl;
    if (error_planarity[0] < distanceThreshold && error_planarity[1] < distanceThreshold && error_orthogonality < distanceThreshold)
      out_inlierIndices.push_back(i);
  }
}

/** Return "true" if the selected points are a degenerate (invalid) case.
  */
bool ransac_LRFcalib_degenerate(
  const CMatrixTemplateNumeric<double> &allData,
  const mrpt::vector_size_t &useIndices )
{
//  ASSERT_( useIndices.size()==2 )
//  The vCOs must be different, and ... TODO
//  if( ..... )
//    return true;
  return false;
}

/*---------------------------------------------------------------
                      ransac_LRFcalib
    Get a set inlier vCOs consistent with a LRF_calibratin
 ---------------------------------------------------------------*/
void ransac_LRFcalib(
                      const vector<CO>      &vCOs,
                      std::vector<size_t>   &inliers,
                      vector<CPose3D>        &LRF_poses_init,
                      const double          threshold,
                      const size_t          min_inliers_for_valid_calib
	)
{
	MRPT_START

	inliers.clear();

//	ASSERT_(vCOs.size() > 2)
	if(vCOs.empty())
		return;

  guess_rel_pose12.inverseComposeFrom(LRF_poses_init[0],LRF_poses_init[1]);

	// The running lists of remaining points after each plane, as a matrix:
	CMatrixTemplateNumeric<double> matCOs( 48, vCOs.size() );
  for(size_t i=0; i < vCOs.size(); i++)
    matCOs.insertCol(i,CO2vector(vCOs[i]));

  CMatrixTemplateNumeric<double> this_best_model;

  math::RANSAC_Template<double>::execute(
    matCOs,
    ransac_LRFcalib_fit,
    ransac_LRFcalib_distance,
    ransac_LRFcalib_degenerate,
    threshold,
    2,  // Minimum set of points
    inliers,
    this_best_model,
    false, // Verbose
    0.99999  // Prob. of good result
    );

//  // Is this calibration good enough?
//  if(inliers.size()>=min_inliers_for_valid_calib)
//  {
//    // Set the return values
//
//  }

	MRPT_END
}

// ------------------------------------------------------------------------------------------------------------
//				                        calib_LRFs_rawlog_ini
// Calibrate the LRFs from their observations in INI_FILENAME. An initialization for the LRFs guessed poses is
// also required. This file also contains information about the indices of the LRFs to be calibrated.
// ------------------------------------------------------------------------------------------------------------
void calib_LRFs_rawlog_ini(const string &INI_FILENAME, const string &override_rawlog_file)
{
	MRPT_START

	CConfigFile				iniFile(INI_FILENAME);

	// ------------------------------------------
	//			Load config from file:
	// ------------------------------------------
	const string RAWLOG_FILE            = !override_rawlog_file.empty() ? override_rawlog_file : iniFile.read_string("calib-LRFs","rawlog_file","",  /*Force existence:*/ true);
//	const unsigned int rawlog_offset  = iniFile.read_int("calib-LRFs","rawlog_offset",0,  /*Force existence:*/ true);
	const string OUT_DIR_STD			      = iniFile.read_string("calib-LRFs","logOutput_dir","log_out",  /*Force existence:*/ true);
	const int M_num_LRFs		            = iniFile.read_int("calib-LRFs","M_num_LRFs",3,  /*Force existence:*/ true);
//	const int resolution		            = iniFile.read_int("calib-LRFs","resolution",1081,  /*Force existence:*/ true);
	const int decimation		            = iniFile.read_int("calib-LRFs","decimation",10,  /*Force existence:*/ true);
	const double threshold_line		    = iniFile.read_double("calib-LRFs","threshold_line",0.03,  /*Force existence:*/ true);
	const size_t min_inliers_line		  = static_cast<size_t>(iniFile.read_int("calib-LRFs","min_inliers_line",100,  /*Force existence:*/ true));
//	const int LOG_FREQUENCY		    = iniFile.read_int("calib-LRFs","LOG_FREQUENCY",5,  /*Force existence:*/ true);
//	const bool  SAVE_POSE_LOG		    = iniFile.read_bool("calib-LRFs","SAVE_POSE_LOG", false,  /*Force existence:*/ true);
//	const bool  SAVE_3D_SCENE          = iniFile.read_bool("calib-LRFs","SAVE_3D_SCENE", false,  /*Force existence:*/ true);
//	const bool  CAMERA_3DSCENE_FOLLOWS_ROBOT = iniFile.read_bool("calib-LRFs","CAMERA_3DSCENE_FOLLOWS_ROBOT", true,  /*Force existence:*/ true);

  set<unsigned> idx_estim_LRFs;
  vector<CPose3D> initial_Poses(M_num_LRFs);
  vector<string> LRF_labels(M_num_LRFs);
  for(int j=0; j < M_num_LRFs; j++)
  {
    string LRF_id = mrpt::format("LRF%i",j+1);

    LRF_labels[j] = iniFile.read_string("LRF_id","sensorLabel","",  /*Force existence:*/ true);

    idx_estim_LRFs.insert(j);

    initial_Poses[j].setFromValues(
      iniFile.read_float(LRF_id,"pose_x",0),
      iniFile.read_float(LRF_id,"pose_y",0),
      iniFile.read_float(LRF_id,"pose_z",0),
      DEG2RAD( iniFile.read_float(LRF_id,"pose_yaw",0) ),
      DEG2RAD( iniFile.read_float(LRF_id,"pose_pitch",0) ),
      DEG2RAD( iniFile.read_float(LRF_id,"pose_roll",0) )
      );
  }

//	bool 	SHOW_PROGRESS_3D_REAL_TIME = false;
//	int		SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS = 0;
//	bool 	SHOW_LASER_SCANS_3D = true;

//	const char* OUT_DIR = OUT_DIR_STD.c_str();

    #if DEBUG
        // Print params:
        printf("Running with the following parameters:\n");
        printf(" RAWLOG file:'%s'\n", RAWLOG_FILE.c_str());
//        printf(" Output directory:\t\t\t'%s'\n",OUT_DIR);
//        printf(" Log record freq:\t\t\t%u\n",LOG_FREQUENCY);
//        printf("  SAVE_3D_SCENE:\t\t\t%c\n", SAVE_3D_SCENE ? 'Y':'N');
//        printf("  SAVE_POSE_LOG:\t\t\t%c\n", SAVE_POSE_LOG ? 'Y':'N');
//        printf("  CAMERA_3DSCENE_FOLLOWS_ROBOT:\t%c\n",CAMERA_3DSCENE_FOLLOWS_ROBOT ? 'Y':'N');
        printf("\n");
    #endif

    // Checks:
    ASSERT_(RAWLOG_FILE.size()>0)
    ASSERT_FILE_EXISTS_(RAWLOG_FILE)

//    mrpt::math::CMatrixDouble matObsLaser1(0,1081);
//    mrpt::math::CMatrixDouble matObsLaser2(0,1081);
//    mrpt::math::CMatrixDouble matObsLaser3(0,1081);

    CFileGZInputStream   rawlogFile(RAWLOG_FILE);   // "file.rawlog"
    CActionCollectionPtr action;
    CSensoryFramePtr     observations;
    CObservationPtr      observation;
    size_t              rawlogEntry=0;

//    CObservation2DRangeScanPtr obsLRFs[M_num_LRFs];
    vector<CObservation2DRangeScanPtr> obsLRFs(M_num_LRFs);
    vector<bool> scan_available(M_num_LRFs,false);
//    std::fill(scan_available, scan_available+M_num_LRFs, false);

    int num_observations = 0, count_valid_obs = 0;
    int num_RangeObs = 0;
    vector<CO> vCOs; // The Corner Observations required for calibration
    while ( CRawlog::getActionObservationPairOrObservation(
                                                 rawlogFile,      // Input file
                                                 action,            // Possible out var: action of a pair action/obs
                                                 observations,  // Possible out var: obs's of a pair action/obs
                                                 observation,    // Possible out var: a single obs.
                                                 rawlogEntry    // Just an I/O counter
                                                 ) )
    {
      // Process observations
      if (observation)
      {
        assert(IS_CLASS(observation, CObservation2DRangeScan));

        cout << "Observation " << num_RangeObs++ << " timestamp " << observation->timestamp << endl;
  //      cout << (CObservation2DRangeScanPtr(observation))->aperture;

  //      cout << "Read observation\n";
        for(int j=0; j < M_num_LRFs; j++)
          if(observation->sensorLabel == LRF_labels[j])
          {
            obsLRFs[j] = CObservation2DRangeScanPtr(observation);
            scan_available[j] = true;
            break;
          }

        bool all_scans = true;
        for(int j=0; j < M_num_LRFs; j++)
          all_scans = all_scans && scan_available[j];

        if(!all_scans)
          continue;

        // Reset the counter of simultaneous observations
        for(int j=0; j < M_num_LRFs; j++)
          scan_available[j] = false;

        // Apply decimation
        count_valid_obs++;
        if(count_valid_obs%decimation != 0)
          continue;

        num_observations++;
//        matObsLaser1.setSize(2*num_observations,matObsLaser1.getColCount());
//        matObsLaser2.setSize(2*num_observations,matObsLaser2.getColCount());
//        matObsLaser3.setSize(2*num_observations,matObsLaser3.getColCount());

        // Segment lines from the LRFs scans
        std::vector<std::vector<std::pair<size_t,TLine2D> > > detected_lines;
        for(int j=0; j < M_num_LRFs; j++)
        {
          mrpt::slam::CSimplePointsMap m_cache_points;
          m_cache_points.clear();
          m_cache_points.insertionOptions.minDistBetweenLaserPoints = 0;
          m_cache_points.insertionOptions.isPlanarMap=false;
          m_cache_points.insertObservation( &(*obsLRFs[j]) );
          size_t n;
          const float	*x,*y,*z;
          m_cache_points.getPointsBuffer(n,x,y,z);

          Matrix<float,Dynamic,1> x_eigen(n), y_eigen(n);
//          Map<Matrix<float,Dynamic,1> > x_eigen(x,n);
          for(size_t i=0; i < n; i++)
          {
            x_eigen(i) = x[i];
            y_eigen(i) = y[i];
    //        cout << i << " scan " << obsLRFs->scan[i] << " x " << x[i] << " y " << y[i] << " z " << z[i] << endl;
          }

          std::vector<std::pair<size_t,TLine2D> > detected_lines_LRF;
          mrpt::math::ransac_detect_2D_lines(x_eigen,y_eigen,detected_lines_LRF,threshold_line,min_inliers_line);
          detected_lines[j] = detected_lines_LRF;
  //      cout << "scan1 size " << obsLRFs[j]->scan.size() << " n " << n << endl;
        }

        // Generate vCOs.
        // All the line combinations are used, despite many of them are not real vCOs. A RANSAC procedure is
        // applied later to discard such outliers
        for(int j=0; j < M_num_LRFs; j++)
          if(detected_lines[j].size() > 1 && detected_lines[(j+1)%3].size() > 1)
            for(size_t a=0; a < detected_lines[j].size(); a++)
              for(size_t aa=a+1; aa < detected_lines[j].size(); aa++)
                for(size_t b=0; b < detected_lines[(j+1)%3].size(); b++)
                  for(size_t bb=b+1; bb < detected_lines[(j+1)%3].size(); bb++)
                  {
                    CO CO_guess;

                    CO_guess[0].id_LRF = j;
                    CO_guess[0].lines[0].center = Vector2d(1/detected_lines[j][a].second.coefs[0], (-detected_lines[j][a].second.coefs[2]-1)/detected_lines[j][a].second.coefs[1]);
                    CO_guess[0].lines[0].cov_center = Matrix2d::Identity();
                    CO_guess[0].lines[0].dir = Vector2d(-detected_lines[j][a].second.coefs[1], detected_lines[j][a].second.coefs[0]);
                    CO_guess[0].lines[0].cov_dir = Matrix2d::Identity();

                    CO_guess[0].lines[1].center = Vector2d(1/detected_lines[j][aa].second.coefs[0], (-detected_lines[j][aa].second.coefs[2]-1)/detected_lines[j][aa].second.coefs[1]);
                    CO_guess[0].lines[1].cov_center = Matrix2d::Identity();
                    CO_guess[0].lines[1].dir = Vector2d(-detected_lines[j][aa].second.coefs[1], detected_lines[j][aa].second.coefs[0]);
                    CO_guess[0].lines[1].cov_dir = Matrix2d::Identity();

                    CO_guess[1].id_LRF = (j+1)%3;
                    CO_guess[1].lines[0].center = Vector2d(1/detected_lines[(j+1)%3][b].second.coefs[0], (-detected_lines[(j+1)%3][b].second.coefs[2]-1)/detected_lines[(j+1)%3][b].second.coefs[1]);
                    CO_guess[1].lines[0].cov_center = Matrix2d::Identity();
                    CO_guess[1].lines[0].dir = Vector2d(-detected_lines[(j+1)%3][b].second.coefs[1], detected_lines[(j+1)%3][b].second.coefs[0]);
                    CO_guess[1].lines[0].cov_dir = Matrix2d::Identity();

                    CO_guess[1].lines[1].center = Vector2d(1/detected_lines[(j+1)%3][bb].second.coefs[0], (-detected_lines[(j+1)%3][bb].second.coefs[2]-1)/detected_lines[(j+1)%3][bb].second.coefs[1]);
                    CO_guess[1].lines[1].cov_center = Matrix2d::Identity();
                    CO_guess[1].lines[1].dir = Vector2d(-detected_lines[(j+1)%3][bb].second.coefs[1], detected_lines[(j+1)%3][bb].second.coefs[0]);
                    CO_guess[1].lines[1].cov_dir = Matrix2d::Identity();

                    vCOs.push_back(CO_guess);
                  }
      }
    }

    // RANSAC Outlier rejection
//     [COs_ransac{j} Rt_estim_sample{j}] = RANSAC_clean_outlierCOs(vCOs{j}, Rt(:,:,[j, rem(j,3)+1]), 10^3, 0.01, 0.01, 0.05);

    vector<CO> vCOs_ransac;
    set<unsigned>::iterator it_LRF1 = idx_estim_LRFs.begin(), it_LRF2;
    for(int j=0; j < M_num_LRFs; j++, it_LRF1++)
    {
      it_LRF2 = it_LRF1; it_LRF2++;
      if(it_LRF2==idx_estim_LRFs.end())
        it_LRF2 = idx_estim_LRFs.begin();

      unsigned LRF1 = *it_LRF1;
      unsigned LRF2 = *it_LRF2;

      vector<CO> vCOs_12;
      for(size_t i=0; i < vCOs.size(); i++)
        if( (vCOs[i][0].id_LRF == LRF1 && vCOs[i][1].id_LRF == LRF2) || (vCOs[i][1].id_LRF == LRF1 && vCOs[i][0].id_LRF == LRF2) )
          vCOs_12.push_back(vCOs[i]);

      vector<CPose3D> LRF_poses_init(2);
      LRF_poses_init[0] = initial_Poses[j];

      std::vector<size_t> inliers;
      double threshold_CO = 0.01;
      size_t min_inliers = 4;
      ransac_LRFcalib(vCOs_12, inliers, LRF_poses_init, threshold_CO, min_inliers);

      // Calibrate pair of LRFs
      set<unsigned> idx_pair_LRFs;
      idx_pair_LRFs.insert(LRF1);
      idx_pair_LRFs.insert(LRF2);
      vector<CPose3D> calib12_ransac = calibrate_LRFs(vCOs_12, LRF_poses_init, idx_pair_LRFs);

      // Get all the vCOs that pass the RANSAC test
      vCOs_ransac.insert(vCOs_ransac.begin(), vCOs_12.begin(), vCOs_12.end());
    }

    // Calibration
    vector<CPose3D> calib = calibrate_LRFs(vCOs, initial_Poses, idx_estim_LRFs);


    // Visualization
//	// Create 3D window if requested:
//	CDisplayWindow3DPtr	win3D;
//#if MRPT_HAS_WXWIDGETS
//	if (SHOW_PROGRESS_3D_REAL_TIME)
//	{
//		win3D = CDisplayWindow3D::Create("ICP-SLAM @ MRPT C++ Library", 600, 500);
//		win3D->setCameraZoom(20);
//		win3D->setCameraAzimuthDeg(-45);
//	}
//#endif

//	if (win3D)
//		win3D->waitForKey();

	MRPT_END
}


// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{
		bool showHelp    = argc>1 && !os::_strcmp(argv[1],"--help");
		bool showVersion = argc>1 && !os::_strcmp(argv[1],"--version");

		printf(" LRF-calib - Part of the MRPT\n");
		printf(" MRPT C++ Library: %s - BUILD DATE %s\n", MRPT_getVersion().c_str(), MRPT_getCompilationDate().c_str());

		if (showVersion)
			return 0;	// Program end

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
		ASSERT_FILE_EXISTS_(INI_FILENAME)

		string override_rawlog_file;
		if (argc>=3)
			override_rawlog_file = string(argv[2]);

		// Run:
    calib_LRFs_rawlog_ini(INI_FILENAME,override_rawlog_file);

		//pause();
		return 0;
	}
	catch (exception &e)
	{
		setConsoleColor(CONCOL_RED,true);
		cerr << "Program finished for an exception!!" << endl;
		setConsoleColor(CONCOL_NORMAL,true);

		cerr << e.what() << endl;

		mrpt::system::pause();
		return -1;
	}
	catch (...)
	{
		setConsoleColor(CONCOL_RED,true);
		cerr << "Program finished for an untyped exception!!" << endl;
		setConsoleColor(CONCOL_NORMAL,true);

		mrpt::system::pause();
		return -1;
	}
}
