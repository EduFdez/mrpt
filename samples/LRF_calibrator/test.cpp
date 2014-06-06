/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>
//#include <mrpt/gui.h>
//#include <mrpt/opengl.h>
#include <mrpt/slam.h>
#include <mrpt/utils.h>
#include <mrpt/obs.h>

#define USE_DEBUG_SEQUENCE 0
#define NUM_SENSORS 2

using namespace std;
using namespace Eigen;
using namespace mrpt;
//using namespace mrpt::gui;
//using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace mrpt::poses;

#define SIGMA2 0.01
#define RECORD_VIDEO 0
int numScreenshot = 0;

typedef Eigen::Matrix<float,12,1> line_CL;

/*---------------------------------------------------------------
		Aux. functions needed by ransac_detect_2D_lines
 ---------------------------------------------------------------*/
void  ransac2Dline_fit_(
  const CMatrixTemplateNumeric<float> &allData,
  const vector_size_t &useIndices,
  vector< CMatrixTemplateNumeric<float> > &fitModels )
{
  ASSERT_(useIndices.size()==2);

  TPoint2D p1( allData(0,useIndices[0]),allData(1,useIndices[0]) );
  TPoint2D p2( allData(0,useIndices[1]),allData(1,useIndices[1]) );

  try
  {
    TLine2D  line(p1,p2);
    fitModels.resize(1);
    CMatrixTemplateNumeric<float> &M = fitModels[0];

    M.setSize(1,3);
    for (size_t i=0;i<3;i++)
      M(0,i)=line.coefs[i];
//  cout << "Line model " << allData(0,useIndices[0]) << " " << allData(1,useIndices[0]) << " " << allData(0,useIndices[1]) << " " << allData(1,useIndices[1]) << " M " << M << endl;
  }
  catch(exception &)
  {
    fitModels.clear();
    return;
  }
}


void ransac2Dline_distance_(
  const CMatrixTemplateNumeric<float> &allData,
  const vector< CMatrixTemplateNumeric<float> > & testModels,
  const float distanceThreshold,
  unsigned int & out_bestModelIndex,
  vector_size_t & out_inlierIndices )
{
  out_inlierIndices.clear();
  out_bestModelIndex = 0;

  if (testModels.empty()) return; // No model, no inliers.

  ASSERTMSG_( testModels.size()==1, format("Expected testModels.size()=1, but it's = %u",static_cast<unsigned int>(testModels.size()) ) )
  const CMatrixTemplateNumeric<float> &M = testModels[0];

  ASSERT_( size(M,1)==1 && size(M,2)==3 )

  TLine2D  line;
  line.coefs[0] = M(0,0);
  line.coefs[1] = M(0,1);
  line.coefs[2] = M(0,2);

  const size_t N = size(allData,2);
  out_inlierIndices.reserve(100);
  for (size_t i=0;i<N;i++)
  {
    const double d = line.distance( TPoint2D( allData.get_unsafe(0,i),allData.get_unsafe(1,i) ) );
//  cout << "distance " << d << " " << allData.get_unsafe(0,i) << " " << allData.get_unsafe(1,i) << endl;
    if (d<distanceThreshold)
      out_inlierIndices.push_back(i);
  }
}

/** Return "true" if the selected points are a degenerate (invalid) case.
  */
bool ransac2Dline_degenerate_(
  const CMatrixTemplateNumeric<float> &allData,
  const mrpt::vector_size_t &useIndices )
{
//  ASSERT_( useIndices.size()==2 )
//
//  const Eigen::Vector2d origin = Eigen::Vector2d(allData(0,useIndices[0]), allData(1,useIndices[0]));
//  const Eigen::Vector2d end = Eigen::Vector2d(allData(0,useIndices[1]), allData(1,useIndices[1]));
//
//  if( (end-origin).norm() < 0.01 )
//    return true;
  return false;
}

/*---------------------------------------------------------------
				ransac_detect_3D_lines
 ---------------------------------------------------------------*/
template <typename NUMTYPE>
void mrpt::math::ransac_detect_2D_lines(
	const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &x,
	const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &y,
	std::vector<std::pair<size_t,TLine2D> >   &out_detected_lines,
	const double           threshold,
	const size_t           min_inliers_for_valid_line
	)
{
	MRPT_START

	ASSERT_(x.size()==y.size())

	out_detected_lines.clear();

	if (x.empty())
		return;

	// The running lists of remaining points after each plane, as a matrix:
	CMatrixTemplateNumeric<NUMTYPE> remainingPoints( 2, x.size() );
	remainingPoints.insertRow(0,x);
	remainingPoints.insertRow(1,y);

	// ---------------------------------------------
	// For each line:
	// ---------------------------------------------
	while (size(remainingPoints,2)>=2)
	{
		mrpt::vector_size_t				this_best_inliers;
		CMatrixTemplateNumeric<NUMTYPE> this_best_model;

		math::RANSAC_Template<NUMTYPE>::execute(
			remainingPoints,
			ransac2Dline_fit_,
			ransac2Dline_distance_,
			ransac2Dline_degenerate_,
			threshold,
			2,  // Minimum set of points
			this_best_inliers,
			this_best_model,
			false, // Verbose
			0.99999  // Prob. of good result
			);

		// Is this plane good enough?
		if (this_best_inliers.size()>=min_inliers_for_valid_line)
		{
//		  line_CL this_line_CL;
//		  this_best_model = this_best_model / sqrt(this_best_model(0,0)*this_best_model(0,0) + this_best_model(0,1)*this_best_model(0,1));
//		  this_line_CL[0] = this_best_model(0,1);
//		  this_line_CL[1] = -this_best_model(0,0);
//
//		  vector<2,float> line_center(2,0.0);
//		  for(int i=0; i < this_best_inliers.size(); i++)
//		  {
//        line_center(0) += x[this_best_inliers(i)];
//        line_center(1) += y[this_best_inliers(i)];
//		  }
//		  line_center = line_center / this_best_inliers.size();

			// Add this plane to the output list:
			out_detected_lines.push_back(
				std::make_pair<size_t,TLine2D>(
					this_best_inliers.size(),
					TLine2D(this_best_model(0,0), this_best_model(0,1),this_best_model(0,2) )
					) );

			out_detected_lines.rbegin()->second.unitarize();

			// Discard the selected points so they are not used again for finding subsequent planes:
			remainingPoints.removeColumns(this_best_inliers);
		}
		else
		{
			break; // Do not search for more planes.
		}
	}

	MRPT_END
}



///*! This class contains the functionality to calibrate the extrinsic parameters of a pair Laser-Range camera (e.g. Kinect, ToF, etc).
// *  This extrinsic calibration is obtained by matching planes and lines that are observed by both types of sensors at the same time.
// */
//class CalibPairLaserKinect
//{
//  public:
//
//    /*! The extrinsic matrix estimated by this calibration method */
//    Eigen::Matrix4f Rt_estimated;
//
//    Eigen::Matrix3f rotation;
//
//    Eigen::Vector3f translation;
//
//    /*! The plane-line correspondences */
//    mrpt::math::CMatrixDouble correspondences;
//
//    /*! Load an initial estimation of Rt between the pair of Asus sensors from file */
//    void setInitRt(const std::string Rt_file)
//    {
//      assert( fexists(Rt_file.c_str()) );
//
//      Rt_estimated.loadFromTextFile(Rt_file);
//    }
//
//    /*! Load an initial estimation of Rt between the pair of Asus sensors from file */
//    void setInitRt(Eigen::Matrix4f initRt)
//    {
//      Rt_estimated = initRt;
////      Rt_estimated = Eigen::Matrix4f::Identity();
////      Rt_estimated(1,1) = Rt_estimated(2,2) = cos(45*PI/180);
////      Rt_estimated(1,2) = -sin(45*PI/180);
////      Rt_estimated(2,1) = -Rt_estimated(1,2);
//    }
//
//    /*! Get the sum of squared rotational errors for the input extrinsic matrices. TODO: the input argument of this function is unsafe -> fix it */
//    float calcCorrespRotError(Eigen::Matrix4f &Rt_)
//    {
//      Eigen::Matrix3f R = Rt_.block(0,0,3,3);
//      return calcCorrespRotError(R);
//    }
//
//    float calcCorrespRotError()
//    {
//      Eigen::Matrix3f R = Rt_estimated.block(0,0,3,3);
//      return calcCorrespRotError(R);
//    }
//
//    float calcCorrespRotError(Eigen::Matrix3f &Rot_)
//    {
//      float accum_error2 = 0.0;
////      float accum_error_deg = 0.0;
//      for(unsigned i=0; i < correspondences.getRowCount(); i++)
//      {
////        float weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));
////        float weight = 1.0;
//        Eigen::Vector3f n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
//        Eigen::Vector3f l_obs_ii; l_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
////        Eigen::Vector3f n_ii = Rot_ * n_obs_ii;
////        Eigen::Vector3f rot_error = (n_obs_i - n_ii);
//        accum_error2 += pow(n_obs_i.transpose() * Rot_ * l_obs_ii, 2);
////        accum_error2 += weight * fabs(rot_error.dot(rot_error));
////        accum_error_deg += acos(fabs(rot_error.dot(rot_error)));
//      }
//
////      std::cout << "AvError deg " << accum_error_deg/correspondences.getRowCount() << std::endl;
//      return accum_error2/correspondences.getRowCount();
//    }
//
//    float calcCorrespTransError(Eigen::Matrix4f &Rt_)
//    {
//      Eigen::Matrix3f R = Rt_.block(0,0,3,3);
//      Eigen::Vector3f t = Rt_.block(0,3,3,1);
//
//      float accum_error2 = 0.0;
//      for(unsigned i=0; i < correspondences.getRowCount(); i++)
//      {
////        float weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));
//        float weight = 1.0;
//        Eigen::Vector3f n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
//        float d_obs_i = correspondences(i,3);
//        Eigen::Vector3f c_obs_ii; c_obs_ii << correspondences(i,7), correspondences(i,8), correspondences(i,9);
//
//        float trans_error = (d_obs_i + n_obs_i.dot(t - R * c_obs_ii));
//        accum_error2 += weight * trans_error * trans_error;
//      }
//      return accum_error2/correspondences.getRowCount();
//    }
//
//    /*! Get the rotation of the laser wrt the range camera */
//    Eigen::Matrix3f CalibrateRotation(int weightedLS = 0)
//    {
//    cout << "CalibrateRotation Plane-Line...\n";
//      Eigen::Matrix<float,3,3> hessian;
//      Eigen::Matrix<float,3,1> gradient;
//      Eigen::Matrix<float,3,1> update_vector;
//      Eigen::Matrix<float,1,3> jacobian_rot_ii; // Jacobians of the rotation
//      float accum_error2;
//      float av_angle_error;
//      unsigned numControlPlanes;
//
//      Eigen::Matrix4f Rt_estimatedTemp;
//      Rt_estimatedTemp = Rt_estimated;
//
//      // Parameters of the Least-Squares optimization
//      unsigned _max_iterations = 10;
//      float _epsilon_transf = 0.00001;
//      float _convergence_error = 0.000001;
//
//      float increment = 1000, diff_error = 1000;
//      int it = 0;
//      while(it < _max_iterations && increment > _epsilon_transf && diff_error > _convergence_error)
//      {
//        // Calculate the hessian and the gradient
//        hessian = Eigen::Matrix<float,3,3>::Zero(); // Hessian of the rotation of the decoupled system
//        gradient = Eigen::Matrix<float,3,1>::Zero(); // Gradient of the rotation of the decoupled system
//        accum_error2 = 0.0;
//        av_angle_error = 0.0;
//        numControlPlanes = 0;
//
////        for(int sensor_id = 0; sensor_id < NUM_ASUS_SENSORS-1; sensor_id++)
//        {
////          assert( correspondences.getRowCount() >= 3 );
//
//          for(unsigned i=0; i < correspondences.getRowCount(); i++)
//          {
////          float weight = (inliers / correspondences(i,3)) / correspondences.getRowCount()
//            Eigen::Vector3f n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
////            float d_obs_i = correspondences(i,3);
//            Eigen::Vector3f l_obs_ii; l_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
////            Eigen::Vector3f c_obs_ii; c_obs_ii << correspondences(i,7), correspondences(i,8), correspondences(i,9);
////            jacobian_rot_i = skew(-n_i);
//            jacobian_rot_ii = -(n_obs_i.transpose() * skew(Rt_estimated.block(0,0,3,3) * l_obs_ii));
//            float rot_error = n_obs_i.transpose() * Rt_estimated.block(0,0,3,3) * l_obs_ii;
//            accum_error2 += pow(rot_error,2);
//            av_angle_error += PI/2 - fabs(acos(rot_error));
//            numControlPlanes++;
////          cout << "rotation error_i " << rot_error.transpose() << endl;
////            if(weightedLS == 1 && correspondences.getColCount() == 10)
////            {
////              // The weight takes into account the number of inliers of the patch, the distance of the patch's center to the image center and the distance of the plane to the sensor
//////              float weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));// / correspondences.getRowCount();
//////              hessian += weight * (jacobian_rot_ii.transpose() * jacobian_rot_ii);
//////              gradient += weight * (jacobian_rot_ii.transpose() * rot_error);
////              Eigen::Matrix3f information;
////              information << correspondences(i,8), correspondences(i,9), correspondences(i,10), correspondences(i,11),
////                            correspondences(i,9), correspondences(i,12), correspondences(i,13), correspondences(i,14),
////                            correspondences(i,10), correspondences(i,13), correspondences(i,15), correspondences(i,16),
////                            correspondences(i,11), correspondences(i,14), correspondences(i,16), correspondences(i,17);
////              hessian += jacobian_rot_ii.transpose() * information.block(0,0,3,3) * jacobian_rot_ii;
////              gradient += jacobian_rot_ii.transpose() * information.block(0,0,3,3) * rot_error;
////            }
////            else
//            {
//              hessian += jacobian_rot_ii.transpose() * jacobian_rot_ii;
//              gradient += jacobian_rot_ii.transpose() * rot_error;
//            }
//
//            Eigen::JacobiSVD<Eigen::Matrix3f> svd(hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
//            float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
////            Eigen::Matrix3f cov = hessian.inverse();
////            Eigen::JacobiSVD<Eigen::Matrix3f> svd2(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
//
////            float minFIM_rot = std::min(hessian(0,0), std::min(hessian(1,1), hessian(2,2)));
////            std::cout << " det " << hessian.determinant() << " minFIM_rot " << minFIM_rot << " conditioningX " << conditioning << std::endl;
//////            std::cout << hessian(0,0) << " " << hessian(1,1) << " " << hessian(2,2) << endl;
//////            std::cout << "COV " << svd2.singularValues().transpose() << endl;
////            std::cout << "FIM rotation " << svd.singularValues().transpose() << endl;
//          }
//          accum_error2 /= numControlPlanes;
//          av_angle_error /= numControlPlanes;
//        }
//
//        Eigen::JacobiSVD<Eigen::Matrix3f> svd(hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
//        float conditioning = svd.singularValues().maxCoeff()/svd.singularValues().minCoeff();
////        Eigen::Matrix3f cov;
////        svd.pinv(cov);
////        std::cout << "hessian \n" << hessian << "inv\n" << hessian.inverse() << "\ncov \n" << cov << std::endl;
//
////        std::cout << "conditioning " << conditioning << std::endl;
////        if(conditioning > 100)
////          return Eigen::Matrix3f::Identity();
//
//        // Solve the rotation
//        update_vector = -hessian.inverse() * gradient;
////      cout << "update_vector " << update_vector.transpose() << endl;
//
//        // Update rotation of the poses
////        for(int sensor_id = 1; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
//        {
//          mrpt::poses::CPose3D pose;
//          mrpt::math::CArrayNumeric< double, 3 > rot_manifold;
//          rot_manifold[0] = update_vector(0,0);
//          rot_manifold[1] = update_vector(1,0);
//          rot_manifold[2] = update_vector(2,0);
////          rot_manifold[2] = update_vector(3*sensor_id-3,0) / 4; // Limit the turn around the Z (depth) axis
////          rot_manifold[2] = 0; // Limit the turn around the Z (depth) axis
//          mrpt::math::CMatrixDouble33 update_rot = pose.exp_rotation(rot_manifold);
//  //      cout << "update_rot\n" << update_rot << endl;
//          Eigen::Matrix3f update_rot_eig;
//          update_rot_eig << update_rot(0,0), update_rot(0,1), update_rot(0,2),
//                            update_rot(1,0), update_rot(1,1), update_rot(1,2),
//                            update_rot(2,0), update_rot(2,1), update_rot(2,2);
//          Rt_estimatedTemp = Rt_estimated;
//          Rt_estimatedTemp.block(0,0,3,3) = update_rot_eig * Rt_estimated.block(0,0,3,3);
//  //      cout << "old rotation" << sensor_id << "\n" << Rt_estimated.block(0,0,3,3) << endl;
//  //      cout << "new rotation\n" << Rt_estimatedTemp.block(0,0,3,3) << endl;
//        }
//
//        accum_error2 = calcCorrespRotError(Rt_estimated);
////        float new_accum_error2 = calcCorrespRotErrorWeight(Rt_estimatedTemp);
//        float new_accum_error2 = calcCorrespRotError(Rt_estimatedTemp);
//
//        cout << "New rotation error " << new_accum_error2 << " previous " << accum_error2 << endl;
//  //    cout << "Closing loop? \n" << Rt_estimated[0].inverse() * Rt_estimated[7] * Rt_78;
//
//        // Assign new rotations
//        if(new_accum_error2 < accum_error2)
////          for(int sensor_id = 1; sensor_id < NUM_ASUS_SENSORS; sensor_id++)
//            Rt_estimated = Rt_estimatedTemp;
////            Rt_estimated.block(0,0,3,3) = Rt_estimatedTemp.block(0,0,3,3);
//
//        increment = update_vector .dot (update_vector);
//        diff_error = accum_error2 - new_accum_error2;
//        ++it;
////      cout << "Iteration " << it << " increment " << increment << " diff_error " << diff_error << endl;
//      }
//
//      std::cout << "ErrorCalibRotation " << accum_error2 << " " << av_angle_error << std::endl;
//      std::cout << "Rotation \n"<< Rt_estimated.block(0,0,3,3) << std::endl;
//    }
//
//    Eigen::Vector3f CalibrateTranslation(const int weightedLS = 0)
//    {
//    cout << "CalibrateTranslation Laser-Kinect\n";
//      // Calibration system
//      Eigen::Matrix3f translationHessian = Eigen::Matrix3f::Zero();
//      Eigen::Vector3f translationGradient = Eigen::Vector3f::Zero();
//
////      Eigen::Vector3f translation2 = Eigen::Vector3f::Zero();
//
////              translationHessian += v3normal1 * v3normal1.transpose();
////  //            double error = d2 - d1;
////              translationGradient += v3normal1 * (d2 - d1);
//      for(unsigned i=0; i < correspondences.getRowCount(); i++)
//      {
//        Eigen::Vector3f n_obs_i; n_obs_i << correspondences(i,0), correspondences(i,1), correspondences(i,2);
//        float d_obs_i = correspondences(i,3);
////        Eigen::Vector3f l_obs_ii; l_obs_ii << correspondences(i,4), correspondences(i,5), correspondences(i,6);
//        Eigen::Vector3f c_obs_ii; c_obs_ii << correspondences(i,7), correspondences(i,8), correspondences(i,9);
//        float trans_error = (d_obs_i - n_obs_i.dot(Rt_estimated.block(0,0,3,3) * c_obs_ii));
//
////        if(weightedLS == 1 && correspondences.getColCount() == 18)
////        {
////          // The weight takes into account the number of inliers of the patch, the distance of the patch's center to the image center and the distance of the plane to the sensor
//////          float weight = (correspondences(i,8) / (correspondences(i,3) * correspondences(i,9)));// / correspondences.getRowCount();
////          float weight = correspondences(i,17);
////          translationHessian += weight * (n_obs_i * n_obs_i.transpose() );
////          translationGradient += weight * (n_obs_i * trans_error);
////        }
////        else
//        {
//          translationHessian += (n_obs_i * n_obs_i.transpose() );
//          translationGradient += (n_obs_i * trans_error);
//        }
//      }
//
//      Eigen::JacobiSVD<Eigen::Matrix3f> svd(translationHessian, Eigen::ComputeFullU | Eigen::ComputeFullV);
//      std::cout << "FIM translation " << svd.singularValues().transpose() << endl;
//
////      cout << "translationHessian \n" << translationHessian << "\n HessianInv \n" << translationHessian.inverse() << endl;
////      calcFisherInfMat();
//
//      translation = translationHessian.inverse() * translationGradient;
//    std::cout << "translation " << translation.transpose() << std::endl;
//
//      return translation;
//    }
//
//    void CalibratePair()
//    {
////      calibrated_Rt = Eigen::Matrix4f::Identity();
//      Rt_estimated.block(0,0,3,3) = CalibrateRotation();
//      Rt_estimated.block(0,3,3,1) = CalibrateTranslation();
////      std::cout << "Rt_estimated\n" << Rt_estimated << std::endl;
//
//      std::cout << "Errors av rot " << calcCorrespRotError(Rt_estimated) << " av trans " << calcCorrespTransError(Rt_estimated) << std::endl;
//    }
//};

//int main ()
//{
//	CTicTac  tictac;
//
//  CObservation2DRangeScanPtr obsLaser1, obsLaser2;
//  const float useful_aperture = 3.14159; // Limit the field of view of the laser to 60 deg
//	int idPreviousLaserScan = -1;
//  mrpt::math::CMatrixDouble COs(48,0);
//  mrpt::math::CMatrixDouble matObsLaser1(0,1081);
//  mrpt::math::CMatrixDouble matObsLaser2(0,1081);
//
//  string save_path = "/home/edu";
////  CFileGZInputStream   rawlogFile("/media/Datos/Dropbox/2LRFs__2014-06-04_17h55m30s.rawlog");   // "file.rawlog"
////  CFileGZInputStream   rawlogFile("/home/edu/bin/mrpt_edu/bin/2LRFs__2014-06-06_10h14m23s.rawlog");   // "file.rawlog"
//  CFileGZInputStream   rawlogFile("/home/edu/bin/mrpt_edu/bin/2LRFs__2014-06-06_11h44m59s.rawlog");   // "file.rawlog"
//  CActionCollectionPtr action;
//  CSensoryFramePtr     observations;
//  CObservationPtr      observation;
//  size_t               rawlogEntry=0;
//
//  const int decimation = 10;
//  int num_observations = 0, count_valid_obs = 0;
//  int num_CObs = 0;
//  while ( CRawlog::getActionObservationPairOrObservation(
//                                               rawlogFile,      // Input file
//                                               action,            // Possible out var: action of a pair action/obs
//                                               observations,  // Possible out var: obs's of a pair action/obs
//                                               observation,    // Possible out var: a single obs.
//                                               rawlogEntry    // Just an I/O counter
//                                               ) )
//  {
//    // Process observations
//    if (observation)
//    {
//      assert(IS_CLASS(observation, CObservation2DRangeScan));
//
////      cout << "Observation " << num_CObs++ << " timestamp " << observation->timestamp << endl;
////      cout << (CObservation2DRangeScanPtr(observation))->aperture;
//
////      cout << "Read observation\n";
//      if(observation->sensorLabel == "HOKUYO1")
//      {
////        if(obsLaser1)
////          obsLaser1.clear();
//        obsLaser1 = CObservation2DRangeScanPtr(observation);
//
//        if(idPreviousLaserScan == 1 || idPreviousLaserScan == -1)
//        {
//          idPreviousLaserScan = 1;
//          continue;
//        }
//        idPreviousLaserScan = 1;
//
////        cout << "Laser timestamp " << obsLaser->timestamp << endl;
////        cout << "Scan width " << obsLaser1->scan.size() << endl;
//      }
//      else if(observation->sensorLabel == "HOKUYO2")
//      {
//        obsLaser2 = CObservation2DRangeScanPtr(observation);
//
//        if(idPreviousLaserScan == 2 || idPreviousLaserScan == -1)
//        {
//          idPreviousLaserScan = 2;
//          continue;
//        }
//        idPreviousLaserScan = 2;
//      }
//
//      idPreviousLaserScan = -1;
//
//      // Apply decimation
//      count_valid_obs++;
//      if(count_valid_obs%decimation != 0)
//        continue;
//
//      num_observations++;
//      matObsLaser1.setSize(2*num_observations,matObsLaser1.getColCount());
//      matObsLaser2.setSize(2*num_observations,matObsLaser2.getColCount());
//
//      mrpt::slam::CSimplePointsMap m_cache_points;
//      m_cache_points.clear();
//      m_cache_points.insertionOptions.minDistBetweenLaserPoints = 0;
//      m_cache_points.insertionOptions.isPlanarMap=false;
//      m_cache_points.insertObservation( &(*obsLaser1) );
//      size_t n;
//      const float	*x,*y,*z;
//      m_cache_points.getPointsBuffer(n,x,y,z);
////    cout << "scan1 size " << obsLaser1->scan.size() << " n " << n << endl;
//
//      int x_row = 2*num_observations-2;
//      int y_row = 2*num_observations-1;
//      for(size_t i=0; i < n; i++)
//      {
//        matObsLaser1(x_row,i) = x[i];
//        matObsLaser1(y_row,i) = y[i];
////        cout << i << " scan " << obsLaser->scan[i] << " x " << x[i] << " y " << y[i] << " z " << z[i] << endl;
//      }
//      if(n < obsLaser1->scan.size())
//        matObsLaser1(x_row,n) = pow(10,9);
//
////      int discard_side = n * (1 - useful_aperture/obsLaser1->aperture)/2;
////      int n_useful_aperture = n - 2*discard_side;
////
////      vector_float x_1(n_useful_aperture), y_1(n_useful_aperture);
////      for(size_t i=0; i < n_useful_aperture; i++)
////      {
////        int meas_id = i+discard_side;
////        x_1[i] = x[meas_id];
////        y_1[i] = y[meas_id];
////      }
//
//
//      m_cache_points.clear();
//      m_cache_points.insertObservation( &(*obsLaser2) );
//      m_cache_points.getPointsBuffer(n,x,y,z);
////    cout << "scan2 size " << obsLaser2->scan.size() << " n " << n << endl;
//
//      for(size_t i=0; i < n; i++)
//      {
//        matObsLaser2(x_row,i) = x[i];
//        matObsLaser2(y_row,i) = y[i];
////        cout << i << " scan " << obsLaser->scan[i] << " x " << x[i] << " y " << y[i] << " z " << z[i] << endl;
//      }
//      if(n < obsLaser2->scan.size())
//        matObsLaser2(x_row,n) = pow(10,9);
//
////    cout << "ObservationPair " << num_observations << endl;
//
////      obsLaser1.clear();
////      obsLaser2.clear();
//
////      int discard_side = n * (1 - useful_aperture/obsLaser2->aperture)/2;
////      int n_useful_aperture = n - 2*discard_side;
////
//////          Eigen::Matrix<float,Eigen::Dynamic,1> x_(241);
////      vector_float x_2(n_useful_aperture), y_2(n_useful_aperture);
////      for(size_t i=0; i < n_useful_aperture; i++)
////      {
////        int meas_id = i+discard_side;
////        x_2[i] = x[meas_id];
////        y_2[i] = y[meas_id];
////      }
//
//
////      // Run RANSAC
////      // ------------------------------------
////      const double DIST_THRESHOLD = 0.1;
////      const int MIN_INLIERS_LINE = 40;
////      vector<pair<size_t,TLine2D > > detectedLines1, detectedLines2;
////      ransac_detect_2D_lines(x_1, y_1, detectedLines1, DIST_THRESHOLD, MIN_INLIERS_LINE);
////      ransac_detect_2D_lines(x_2, y_2, detectedLines2, DIST_THRESHOLD, MIN_INLIERS_LINE);
////
////    cout << detectedLines1.size() << " and " << detectedLines2.size() << " detected lines " << endl;
////
////
////      // Stablish the correspondences
////      for(unsigned i=0; i < detectedLines1.size(); i++)
////        for(unsigned ii=i+1; ii < detectedLines1.size(); ii++)
////        {
////          for(unsigned j=0; j < detectedLines2.size(); j++)
////          for(unsigned jj=j+1; jj < detectedLines2.size(); jj++)
////          {
////            unsigned prevSize = COs.getColCount();
////            COs.setSize(COs.getRowCount(), prevSize+1);
////            COs.block(0,prevSize, 12, 1) = detectedLines1[i].first;
////            COs.block(12,prevSize, 12, 1) = detectedLines1[ii].first;
////            COs.block(24,prevSize, 12, 1) = detectedLines2[j].first;
////            COs.block(36,prevSize, 12, 1) = detectedLines2[jj].first;
////          }
////        }
//
//    }
//  }
//
////  cout << "\tSave COs\n";
////  COs.saveToTextFile( mrpt::format("%s/COs.txt", PROJECT_SOURCE_PATH) );
//
//  cout << "\tSave LRF Observations " << matObsLaser1.getRowCount() << "\n";
//  matObsLaser1.saveToTextFile( mrpt::format("%s/matObsLaser1.txt", save_path.c_str() ) );
//  matObsLaser2.saveToTextFile( mrpt::format("%s/matObsLaser2.txt", save_path.c_str() ) );
//
//	return (0);
//}

int main ()
{
	CTicTac  tictac;

  CObservation2DRangeScanPtr obsLaser1, obsLaser2, obsLaser3;
//  const float useful_aperture = 3.14159; // Limit the field of view of the laser to 60 deg
	bool scan1 = false, scan2 = false, scan3 = false;
  mrpt::math::CMatrixDouble matObsLaser1(0,1081);
  mrpt::math::CMatrixDouble matObsLaser2(0,1081);
  mrpt::math::CMatrixDouble matObsLaser3(0,1081);

  string save_path = "/home/edu";
  CFileGZInputStream   rawlogFile("/home/edu/bin/mrpt_edu/bin/2LRFs__2014-06-06_13h51m36s.rawlog");   // "file.rawlog"
  CActionCollectionPtr action;
  CSensoryFramePtr     observations;
  CObservationPtr      observation;
  size_t               rawlogEntry=0;

  const int decimation = 10;
  int num_observations = 0, count_valid_obs = 0;
  int num_CObs = 0;
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

      cout << "Observation " << num_CObs++ << " timestamp " << observation->timestamp << endl;
//      cout << (CObservation2DRangeScanPtr(observation))->aperture;

//      cout << "Read observation\n";
      if(observation->sensorLabel == "HOKUYO1")
      {
//        if(obsLaser1)
//          obsLaser1.clear();
        obsLaser1 = CObservation2DRangeScanPtr(observation);
        scan1 = true;

//        cout << "Laser timestamp " << obsLaser->timestamp << endl;
//        cout << "Scan width " << obsLaser1->scan.size() << endl;
      }
      else if(observation->sensorLabel == "HOKUYO2")
      {
        obsLaser2 = CObservation2DRangeScanPtr(observation);
        scan2 = true;
      }
        else if(observation->sensorLabel == "HOKUYO3")
        {
          obsLaser3 = CObservation2DRangeScanPtr(observation);
          scan3 = true;
        }

      if(!(scan1 && scan2 && scan3))
        continue;

      scan1 = scan2 = scan3 = false; // Reset the counter of simultaneous observations

      // Apply decimation
      count_valid_obs++;
      if(count_valid_obs%decimation != 0)
        continue;

      num_observations++;
      matObsLaser1.setSize(2*num_observations,matObsLaser1.getColCount());
      matObsLaser2.setSize(2*num_observations,matObsLaser2.getColCount());
      matObsLaser3.setSize(2*num_observations,matObsLaser3.getColCount());

      mrpt::slam::CSimplePointsMap m_cache_points;
      m_cache_points.clear();
      m_cache_points.insertionOptions.minDistBetweenLaserPoints = 0;
      m_cache_points.insertionOptions.isPlanarMap=false;
      m_cache_points.insertObservation( &(*obsLaser1) );
      size_t n;
      const float	*x,*y,*z;
      m_cache_points.getPointsBuffer(n,x,y,z);
    cout << "scan1 size " << obsLaser1->scan.size() << " n " << n << endl;

      int x_row = 2*num_observations-2;
      int y_row = 2*num_observations-1;
      for(size_t i=0; i < n; i++)
      {
        matObsLaser1(x_row,i) = x[i];
        matObsLaser1(y_row,i) = y[i];
//        cout << i << " scan " << obsLaser->scan[i] << " x " << x[i] << " y " << y[i] << " z " << z[i] << endl;
      }
      if(n < obsLaser1->scan.size())
        matObsLaser1(x_row,n) = pow(10,9);

//      int discard_side = n * (1 - useful_aperture/obsLaser1->aperture)/2;
//      int n_useful_aperture = n - 2*discard_side;
//
//      vector_float x_1(n_useful_aperture), y_1(n_useful_aperture);
//      for(size_t i=0; i < n_useful_aperture; i++)
//      {
//        int meas_id = i+discard_side;
//        x_1[i] = x[meas_id];
//        y_1[i] = y[meas_id];
//      }


      m_cache_points.clear();
      m_cache_points.insertObservation( &(*obsLaser2) );
      m_cache_points.getPointsBuffer(n,x,y,z);
    cout << "scan2 size " << obsLaser2->scan.size() << " n " << n << endl;

      for(size_t i=0; i < n; i++)
      {
        matObsLaser2(x_row,i) = x[i];
        matObsLaser2(y_row,i) = y[i];
//        cout << i << " scan " << obsLaser->scan[i] << " x " << x[i] << " y " << y[i] << " z " << z[i] << endl;
      }
      if(n < obsLaser2->scan.size())
        matObsLaser2(x_row,n) = pow(10,9);


      m_cache_points.clear();
      m_cache_points.insertObservation( &(*obsLaser3) );
      m_cache_points.getPointsBuffer(n,x,y,z);
    cout << "scan3 size " << obsLaser3->scan.size() << " n " << n << endl;

      for(size_t i=0; i < n; i++)
      {
        matObsLaser3(x_row,i) = x[i];
        matObsLaser3(y_row,i) = y[i];
//        cout << i << " scan " << obsLaser->scan[i] << " x " << x[i] << " y " << y[i] << " z " << z[i] << endl;
      }
      if(n < obsLaser3->scan.size())
        matObsLaser3(x_row,n) = pow(10,9);

    }
  }

//  cout << "\tSave COs\n";
//  COs.saveToTextFile( mrpt::format("%s/COs.txt", PROJECT_SOURCE_PATH) );

  cout << "\tSave LRF Observations " << matObsLaser1.getRowCount() << "\n";
  matObsLaser1.saveToTextFile( mrpt::format("%s/matObsLaser1.txt", save_path.c_str() ) );
  matObsLaser2.saveToTextFile( mrpt::format("%s/matObsLaser2.txt", save_path.c_str() ) );
  matObsLaser3.saveToTextFile( mrpt::format("%s/matObsLaser3.txt", save_path.c_str() ) );

	return (0);
}
