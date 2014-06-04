///* +---------------------------------------------------------------------------+
//   |                     Mobile Robot Programming Toolkit (MRPT)               |
//   |                          http://www.mrpt.org/                             |
//   |                                                                           |
//   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
//   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
//   | Released under BSD License. See details in http://www.mrpt.org/License    |
//   +---------------------------------------------------------------------------+ */
//
//#include <mrpt/base.h>
//#include <mrpt/gui.h>
//#include <mrpt/opengl.h>
//#include <mrpt/slam.h>
//#include <mrpt/utils.h>
//#include <mrpt/obs.h>
//
//#include <pcl/filters/extract_indices.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
//
//#include <pcl/segmentation/organized_multi_plane_segmentation.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/common/common.h>
//#include <pcl/common/transforms.h>
//
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//
//#include <Frame360.h>
//#include <Frame360_Visualizer.h>
//#include <RegisterRGBD360.h>
//#include <Calibrator.h>
//
//#include <RGBDGrabber_OpenNI2.h>
//#include <SerializeFrameRGBD.h> // For time-stamp conversion
//
//#include <pcl/console/parse.h>
//
////#include "opencv2/highgui/highgui.hpp"
////#include "opencv2/imgproc/imgproc.hpp"
//
//#include <stdio.h>
//#include <signal.h>
//
//#define USE_DEBUG_SEQUENCE 0
//#define NUM_SENSORS 2
////const int NUM_SENSORS = 2;
//
//using namespace std;
//using namespace Eigen;
//using namespace mrpt;
//using namespace mrpt::gui;
//using namespace mrpt::opengl;
//using namespace mrpt::math;
//using namespace mrpt::utils;
//using namespace mrpt::slam;
//using namespace mrpt::poses;
//
//#define SIGMA2 0.01
//#define RECORD_VIDEO 0
//int numScreenshot = 0;
//
//RGBDGrabber_OpenNI2 *grabber[NUM_SENSORS]; // This object is declared as global to be able to use it after capturing Ctrl-C interruptions
//
//const int min_inliers = 1000;
//
///*! Catch interruptions like Ctrl-C */
//void INThandler(int sig)
//{
//  char c;
//
//  signal(sig, SIG_IGN);
//  printf("\n  Do you really want to quit? [y/n] ");
//  c = getchar();
//  if (c == 'y' || c == 'Y')
//  {
//    for(unsigned sensor_id = 0; sensor_id < NUM_SENSORS; sensor_id++)
//    {
//      delete grabber[sensor_id]; // Turn off each Asus XPL sensor before exiting the program
//    }
//    exit(0);
//  }
//}
//
///*---------------------------------------------------------------
//		Aux. functions needed by ransac_detect_2D_lines
// ---------------------------------------------------------------*/
//void  ransac2Dline_fit_(
//  const CMatrixTemplateNumeric<float> &allData,
//  const vector_size_t &useIndices,
//  vector< CMatrixTemplateNumeric<float> > &fitModels )
//{
//  ASSERT_(useIndices.size()==2);
//
//  TPoint2D p1( allData(0,useIndices[0]),allData(1,useIndices[0]) );
//  TPoint2D p2( allData(0,useIndices[1]),allData(1,useIndices[1]) );
//
//  try
//  {
//    TLine2D  line(p1,p2);
//    fitModels.resize(1);
//    CMatrixTemplateNumeric<float> &M = fitModels[0];
//
//    M.setSize(1,3);
//    for (size_t i=0;i<3;i++)
//      M(0,i)=line.coefs[i];
////  cout << "Line model " << allData(0,useIndices[0]) << " " << allData(1,useIndices[0]) << " " << allData(0,useIndices[1]) << " " << allData(1,useIndices[1]) << " M " << M << endl;
//  }
//  catch(exception &)
//  {
//    fitModels.clear();
//    return;
//  }
//}
//
//
//void ransac2Dline_distance_(
//  const CMatrixTemplateNumeric<float> &allData,
//  const vector< CMatrixTemplateNumeric<float> > & testModels,
//  const float distanceThreshold,
//  unsigned int & out_bestModelIndex,
//  vector_size_t & out_inlierIndices )
//{
//  out_inlierIndices.clear();
//  out_bestModelIndex = 0;
//
//  if (testModels.empty()) return; // No model, no inliers.
//
//  ASSERTMSG_( testModels.size()==1, format("Expected testModels.size()=1, but it's = %u",static_cast<unsigned int>(testModels.size()) ) )
//  const CMatrixTemplateNumeric<float> &M = testModels[0];
//
//  ASSERT_( size(M,1)==1 && size(M,2)==3 )
//
//  TLine2D  line;
//  line.coefs[0] = M(0,0);
//  line.coefs[1] = M(0,1);
//  line.coefs[2] = M(0,2);
//
//  const size_t N = size(allData,2);
//  out_inlierIndices.reserve(100);
//  for (size_t i=0;i<N;i++)
//  {
//    const double d = line.distance( TPoint2D( allData.get_unsafe(0,i),allData.get_unsafe(1,i) ) );
////  cout << "distance " << d << " " << allData.get_unsafe(0,i) << " " << allData.get_unsafe(1,i) << endl;
//    if (d<distanceThreshold)
//      out_inlierIndices.push_back(i);
//  }
//}
//
///** Return "true" if the selected points are a degenerate (invalid) case.
//  */
//bool ransac2Dline_degenerate_(
//  const CMatrixTemplateNumeric<float> &allData,
//  const mrpt::vector_size_t &useIndices )
//{
////  ASSERT_( useIndices.size()==2 )
////
////  const Eigen::Vector2d origin = Eigen::Vector2d(allData(0,useIndices[0]), allData(1,useIndices[0]));
////  const Eigen::Vector2d end = Eigen::Vector2d(allData(0,useIndices[1]), allData(1,useIndices[1]));
////
////  if( (end-origin).norm() < 0.01 )
////    return true;
//  return false;
//}
//
///*---------------------------------------------------------------
//				ransac_detect_3D_lines
// ---------------------------------------------------------------*/
//void ransac_detect_3D_lines(
//	const pcl::PointCloud<PointT>::Ptr &scan,
//	Eigen::Matrix<float,Eigen::Dynamic,6> &lines,
////	CMatrixTemplateNumeric<float,Eigen::Dynamic,6> &lines,
//	const double           threshold,
//	const size_t           min_inliers_for_valid_line
//	)
//{
//	ASSERT_(scan->size() )
////cout << "ransac_detect_2D_lines \n";
//
//	if(scan->empty())
//		return;
//
//	// The running lists of remaining points after each plane, as a matrix:
//	CMatrixTemplateNumeric<float> remainingPoints( 2, scan->size() );
//	for(unsigned i=0; i < scan->size(); i++)
//	{
//    remainingPoints(0,i) = scan->points[i].y;
//    remainingPoints(1,i) = scan->points[i].z;
//	}
//
////cout << "Size remaining pts " << size(remainingPoints,1) << " " << size(remainingPoints,2) << endl;
//
//	// ---------------------------------------------
//	// For each line:
//	// ---------------------------------------------
//	std::vector<std::pair<size_t,TLine2D> > out_detected_lines;
////	while (size(remainingPoints,2)>=2)
//	{
//		mrpt::vector_size_t				this_best_inliers;
//		CMatrixTemplateNumeric<float> this_best_model;
//
//		math::RANSAC_Template<float>::execute(
//			remainingPoints,
//			ransac2Dline_fit_,
//			ransac2Dline_distance_,
//			ransac2Dline_degenerate_,
//			threshold,
//			2,  // Minimum set of points
//			this_best_inliers,
//			this_best_model,
//			false, // Verbose
//			0.99  // Prob. of good result
//			);
////cout << "Size this_best_inliers " << this_best_inliers.size() << endl;
//
//		// Is this plane good enough?
//		if (this_best_inliers.size()>=min_inliers_for_valid_line)
//		{
//			// Add this plane to the output list:
//			out_detected_lines.push_back(
//				std::make_pair<size_t,TLine2D>(
//					this_best_inliers.size(),
//					TLine2D(this_best_model(0,0), this_best_model(0,1),this_best_model(0,2) )
//					) );
//
//			out_detected_lines.rbegin()->second.unitarize();
//
//			int prev_size = size(lines,1);
////    cout << "prevSize lines " << prev_size << endl;
//			lines.setSize(prev_size+1,6);
//			float mod_dir = sqrt(1+pow(this_best_model(0,0)/this_best_model(0,1),2));
//			lines(prev_size,0) = 0; // The reference system for the laser is aligned in the horizontal axis
//			lines(prev_size,1) = 1/mod_dir;
//			lines(prev_size,2) = -(this_best_model(0,0)/this_best_model(0,1))/mod_dir;
//			lines(prev_size,3) = 0;
//			lines(prev_size,4) = scan->points[this_best_inliers[0]].y;
////			lines(prev_size,4) = scan->points[this_best_inliers[0]].x;
//			lines(prev_size,5) = scan->points[this_best_inliers[0]].z;
//			// Discard the selected points so they are not used again for finding subsequent planes:
//			remainingPoints.removeColumns(this_best_inliers);
//		}
////		else
////		{
////			break; // Do not search for more planes.
////		}
//	}
//}
//
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
//
//int main ()
//{
//	CTicTac  tictac;
//
//  CObservation3DRangeScanPtr obsKinect;
////  CObservation3DRangeScanPtr obsToF;
//  CObservation2DRangeScanPtr obsLaser;
//  const size_t fieldOfView = 241; // Limit the field of view of the laser to 60 deg
//  const size_t offset60deg = (1081-241)/2; // Limit the field of view of the laser to 60 deg
//	bool bPrevLaserScan = false;
//  mrpt::math::CMatrixDouble correspPlaneLine(0,10);
//
//  CFileGZInputStream   rawlogFile("/media/Data/Datasets360/Laser+Kinect/dataset_2014-02-12_17h06m40s.rawlog");   // "file.rawlog"
//  CActionCollectionPtr action;
//  CSensoryFramePtr     observations;
//  CObservationPtr         observation;
//  size_t               rawlogEntry=0;
//  bool        end = false;
//
//  while ( CRawlog::getActionObservationPairOrObservation(
//         rawlogFile,      // Input file
//         action,            // Possible out var: action of a pair action/obs
//         observations,  // Possible out var: obs's of a pair action/obs
//         observation,    // Possible out var: a single obs.
//         rawlogEntry    // Just an I/O counter
//         ) )
//  {
//    // Process observations
//    if (observation)
//    {
////      cout << "Read observation\n";
//      if(IS_CLASS(observation, CObservation2DRangeScan))
//      {
//        assert(observation->sensorLabel == "HOKUYO_UTM");
//
//        obsLaser = CObservation2DRangeScanPtr(observation);
//        bPrevLaserScan = true;
////        cout << "Laser timestamp " << obsLaser->timestamp << endl;
////        cout << "Scan width " << obsLaser->scan.size() << endl;
//      }
//      else if(IS_CLASS(observation, CObservation3DRangeScan))
//      {
//        assert(observation->sensorLabel == "KINECT");
//
//        obsKinect = CObservation3DRangeScanPtr(observation);
////        cout << "Kinect timestamp " << obsKinect->timestamp << endl;
//
//        if(bPrevLaserScan && (obsKinect->timestamp - obsLaser->timestamp) < 250000)
//        {
//          mrpt::slam::CSimplePointsMap m_cache_points;
//          m_cache_points.clear();
//          m_cache_points.insertionOptions.minDistBetweenLaserPoints = 0;
//          m_cache_points.insertionOptions.isPlanarMap=false;
//          m_cache_points.insertObservation( &(*obsLaser) );
//          size_t n;
//          const float	*x,*y,*z;
//          m_cache_points.getPointsBuffer(n,x,y,z);
////          for(size_t i=0; i < obsLaser->scan.size(); i++)
////            cout << i << " scan " << obsLaser->scan[i] << " x " << x[i] << " y " << y[i] << " z " << z[i] << endl;
//
////          Eigen::Matrix<float,Eigen::Dynamic,1> x_(241);
//          vector_float x_(1081), y_(1081);
//          for(size_t i=0; i < 1081; i++)
////          vector_float x_(fieldOfView), y_(fieldOfView);
////          for(size_t i=offset60deg; i < fieldOfView; i++)
//          {
//            x_[i] = x[i+offset60deg];
//            y_[i] = y[i+offset60deg];
//          }
//
//          // Run RANSAC
//          // ------------------------------------
//          vector<pair<size_t,TLine2D > > detectedLines;
//          const double DIST_THRESHOLD = 0.1;
//          ransac_detect_2D_lines(x_, y_, detectedLines, DIST_THRESHOLD, 20);
//        cout << detectedLines.size() << " detected lines " << endl;
//
//          //Copy to the CColouredPointsMap
//          CColouredPointsMap m_pntsMap;
//    //      CPointsMap m_pntsMap;
//
//    //  		m_pntsMap.clear();
//          m_pntsMap.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
//          m_pntsMap.insertionOptions.minDistBetweenLaserPoints = 0; // don't drop any point
//          m_pntsMap.insertionOptions.disableDeletion = true;
//          m_pntsMap.insertionOptions.fuseWithExisting = false;
//          m_pntsMap.insertionOptions.insertInvalidPoints = true;
//          m_pntsMap.insertObservation(obsKinect.pointer());
//          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudKinect(new pcl::PointCloud<pcl::PointXYZRGBA>);
//          m_pntsMap.getPCLPointCloud(*cloudKinect);
//
//          cout << "Cloud-Kinect pts " << cloudKinect->points.size() << endl;
//
//          CloudRGBD_Ext cloud_;
//          DownsampleRGBD downsampler(2);
//          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampledCloud = downsampler.downsamplePointCloud(cloudKinect);
//          cloud_.setPointCloud(downsampledCloud);
//          mrpt::pbmap::PbMap planes_;
////          getPlanesInFrame(cloud_, planes_);
//
////          //Extract a plane with RANSAC
////          Eigen::VectorXf modelcoeff_Plane(4);
////          vector<int> inliers;
////          pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (cloudKinect));
////          pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
////          ransac.setDistanceThreshold (.03);
////          tictac.Tic();
////          ransac.computeModel();
////          ransac.getModelCoefficients(modelcoeff_Plane);
////          if(modelcoeff_Plane[3] < 0) modelcoeff_Plane *= -1;
////    //      modelcoeff_Plane *= (modelcoeff_Plane[3]/fabs(modelcoeff_Plane[3]));
////        cout << "RANSAC (pcl) computation time: " << tictac.Tac()*1000.0 << " ms " << modelcoeff_Plane.transpose() << endl;
////          ransac.getInliers(inliers);
//
////          // Stablish the correspondences
////          for(unsigned i=0; i < planes_.vPlanes.size(); i++)
////          {
////            for(unsigned j=0; j < detectedLines.size(); j++)
////            {
////              if( planes_i.vPlanes[i].inliers.size() > min_inliers && planes_j.vPlanes[j].inliers.size() > min_inliers &&
////                  planes_i.vPlanes[i].elongation < 5 && planes_j.vPlanes[j].elongation < 5 &&
////                  planes_i.vPlanes[i].v3normal .dot (planes_j.vPlanes[j].v3normal) > 0.99 &&
////                  fabs(planes_i.vPlanes[i].d - planes_j.vPlanes[j].d) < 0.1 //&&
//////                    planes_i.vPlanes[i].hasSimilarDominantColor(planes_j.vPlanes[j],0.06) &&
//////                    planes_i.vPlanes[planes_counter_i+i].isPlaneNearby(planes_j.vPlanes[planes_counter_j+j], 0.5)
////                )
////              {
////                unsigned prevSize = correspPlaneLine.getRowCount();
////                correspPlaneLine.setSize(prevSize+1, correspPlaneLine.getColCount());
////                correspPlaneLine(prevSize, 0) = modelcoeff_Plane[0];
////                correspPlaneLine(prevSize, 1) = modelcoeff_Plane[1];
////                correspPlaneLine(prevSize, 2) = modelcoeff_Plane[2];
////                correspPlaneLine(prevSize, 3) = modelcoeff_Plane[3];
////                correspPlaneLine(prevSize, 4) = modelcoeff_Line[0];
////                correspPlaneLine(prevSize, 5) = modelcoeff_Line[1];
////                correspPlaneLine(prevSize, 6) = modelcoeff_Line[2];
////                correspPlaneLine(prevSize, 7) = modelcoeff_Line[3];
////                correspPlaneLine(prevSize, 8) = modelcoeff_Line[4];
////                correspPlaneLine(prevSize, 9) = modelcoeff_Line[5];
////              }
////            }
////          }
//        }
//      }
//    }
//  }
//
//  cout << "\tSave correspPlaneLine\n";
//  correspPlaneLine.saveToTextFile( mrpt::format("%s/correspPlaneLine.txt", PROJECT_SOURCE_PATH) );
//
//	return (0);
//}
