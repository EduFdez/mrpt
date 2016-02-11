/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CStereoGrabber_Bumblebee_H
#define CStereoGrabber_Bumblebee_H

#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/hwdrivers/link_pragmas.h>
#include <mrpt/utils/CUncopiable.h>

#ifndef MRPT_OS_WINDOWS
	#include <mrpt/hwdrivers/CImageGrabber_dc1394.h>
#endif

#include <mrpt/config.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** Options used when creating a bumblebee camera capture object
		  * \ingroup mrpt_hwdrivers_grp
		  */
		struct HWDRIVERS_IMPEXP TCaptureOptions_bumblebee
		{
			TCaptureOptions_bumblebee();

			int	frame_width, frame_height;	//!< Capture resolution (Default: 640x480)
			bool color;						//!< Indicates if the Bumblebee camera must capture color images (Default: false -> grayscale)
			bool getRectified;				//!< Indicates if the Bumblebee camera must capture rectified images (Default: true -> rectified)
			double framerate;				//!< Bumblebee camera frame rate (Default: 15 fps)
		};

		/** A class for grabing stereo images from a "Bumblebee" or "Bumblebee2" camera
		  * NOTE:
		  *		- Windows:
		  *			- This class is only available when compiling MRPT with "MRPT_HAS_BUMBLEBEE".
		  *			- You will need the "include" and "lib" directories of the vendor's proprietary software to be included in VC++ includes path.
		  *		- Linux:
		  *			- This class is only available when compiling MRPT with "MRPT_HAS_LIBDC1394_2".
		  *			- Capture will be made in color, full resolution and "raw" (not rectified) only.
		  *
		  * Once connected to a camera, you can call "getStereoObservation" to retrieve the stereo images.
		  *
		  * \sa You'll probably want to use instead the most generic camera grabber in MRPT: mrpt::hwdrivers::CCameraSensor
		  * \ingroup mrpt_hwdrivers_grp
		  */
		class HWDRIVERS_IMPEXP  CStereoGrabber_Bumblebee : public mrpt::utils::CUncopiable
		{
		protected:
#ifdef MRPT_OS_WINDOWS
			void			*m_triclops;					//!< The Triclops context (TriclopsContext)
			void			*m_flycapture;					//!< The Flycapture context (FlyCaptureContext).
			vector_byte		m_imgBuff;						//!< A buffer to store an image
#else
			mrpt::hwdrivers::CImageGrabber_dc1394	*m_firewire_capture;  //!< The actual capture object used in Linux / Mac.
#endif

			bool			m_bInitialized;					//!< If this has been correctly initiated
			unsigned int	m_resolutionX, m_resolutionY;	//!< The desired resolution

			float			m_baseline;						//!< Camera baseline
			float			m_focalLength;					//!< Camera focal length
			float			m_centerCol, m_centerRow;		//!< Camera center coordinates


		private:

#ifdef MRPT_OS_WINDOWS
			void scaleImage( void* 	image, unsigned char	ucMinOut,  unsigned char	ucMaxOut );
			void convertTriclopsImageTo8BitsIplImage( void *src, void* dst );

			/** Splits a TriclopsImage (grayscale) into two separate IplImages (from the left and right cameras) (for internal use only)
			  * triclopsImage [input]. The Triclops image to split
			  * dstL [output]. The Left CImage.
			  * dstR [output]. The Right CImage.
			*/
			static void convertTriclopsImagesToIplImages(
				void* triclopsImage,
				void* dstL,
				void* dstR );

#endif
			/** Splits a Flycapture image into two separate IplImages (from the left and right cameras) (for internal use only)
			  * triclopsImage [input]. The FlyCapture image to split
			  * dstL [output]. The Left CImage.
			  * dstR [output]. The Right CImage.
			*/
			static void convertFlyCaptureImagesToIplImages( void* flycapImage, void* dstL, void* dstR );

		public:

			TCaptureOptions_bumblebee	m_options;			//!< Bumblebee camera frame rate (Default: 15 fps)

			/** Constructor: */
			CStereoGrabber_Bumblebee( int cameraIndex = 0, const TCaptureOptions_bumblebee &options = TCaptureOptions_bumblebee() );

			/** Destructor */
			virtual ~CStereoGrabber_Bumblebee(void);

			/** Grab stereo images, and return the pair of rectified images.
			 * \param out_observation The object to be filled with sensed data.
			 *
			 *  NOTICE: (1) That the member "CObservationStereoImages::refCameraPose" must be
			 *                set on the return of this method, since we don't know here the robot physical structure.
			 *          (2) The images are already rectified.
			 *
			 * \return false on any error, true if all go fine.
			*/
			bool  getStereoObservation( mrpt::obs::CObservationStereoImages &out_observation );


		};	// End of class

	} // End of NS
} // End of NS


#endif
