/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h" // Precompiled header

#include <mrpt/hwdrivers/COpenNI2Generic.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/system/threads.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

#if MRPT_HAS_OPENNI2
#	include <OpenNI.h>
#	include <PS1080.h>
#endif

using namespace mrpt::hwdrivers;
using namespace mrpt::system;
using namespace mrpt::slam;
using namespace mrpt::synch;
using namespace std;

// #define DEVICE_LIST_PTR (reinterpret_cast<openni::Array<openni::DeviceInfo>* >(deviceListPtr))
// #define DEVICE_ID_PTR (reinterpret_cast<openni::Device*>(vp_devices[sensor_id]))
// #define DEPTH_STREAM_ID_PTR (reinterpret_cast<openni::VideoStream*>(vp_depth_stream[sensor_id]))
// #define RGB_STREAM_ID_PTR (reinterpret_cast<openni::VideoStream*>(vp_rgb_stream[sensor_id]))
// #define DEPTH_FRAME_ID_PTR (reinterpret_cast<openni::VideoFrameRef*>(vp_frame_depth[sensor_id]))
// #define RGB_FRAME_ID_PTR (reinterpret_cast<openni::VideoFrameRef*>(vp_frame_rgb[sensor_id]))

// Initialize static member
std::vector<stlplus::smart_ptr<COpenNI2Generic::CDevice> > COpenNI2Generic::vDevices = std::vector<stlplus::smart_ptr<COpenNI2Generic::CDevice> >();
int COpenNI2Generic::numInstances = 0;

/*
void openni::VideoMode::setResolution()
Setter function for the resolution of this VideoMode. Application use of this function is not recommended.
Instead, use SensorInfo::getSupportedVideoModes() to obtain a list of valid video modes

-- cited from OpenNI2 help. setResolution() is not recommended.
*/
#if 0
#if MRPT_HAS_OPENNI2
bool setONI2StreamMode(openni::VideoStream& stream, int w, int h, int fps, openni::PixelFormat format)
{
//std::cout << "Ask mode: " << w << "x" << h << " " << fps << " fps. format " << format << std::endl;
	bool found = false;
	const openni::Array<openni::VideoMode>& modes = stream.getSensorInfo().getSupportedVideoModes();
	for(int i = 0, i_end = modes.getSize();i < i_end;++i){
//	  std::cout << "Mode: " << modes[i].getResolutionX() << "x" << modes[i].getResolutionY() << " " << modes[i].getFps() << " fps. format " << modes[i].getPixelFormat() << std::endl;
		if(modes[i].getResolutionX() != w){
			continue;
		}
		if(modes[i].getResolutionY() != h){
			continue;
		}
		if(modes[i].getFps() != fps){
			continue;
		}
		if(modes[i].getPixelFormat() != format){
			continue;
		}
		openni::Status rc = stream.setVideoMode(modes[i]);
		if(rc != openni::STATUS_OK){
			printf("%s:Couldn't find RGB stream:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
			return false;
		}
		return true;
	}
	return false;
}
#endif
#endif

bool        setONI2StreamMode(openni::VideoStream& stream, int w, int h, int fps, openni::PixelFormat format);
std::string oni2DevInfoStr(const openni::DeviceInfo& info, int tab = 0);
bool        cmpONI2Device(const openni::DeviceInfo& i1, const openni::DeviceInfo& i2);

class COpenNI2Generic::CDevice{
public:
	typedef stlplus::smart_ptr<CDevice> Ptr;
	enum{
		COLOR_STREAM, DEPTH_STREAM, STREAM_TYPE_SIZE
	};
private:
	class CStream{
	public:
		typedef stlplus::smart_ptr<CStream> Ptr;
	private:
		std::ostream&       m_log;
		openni::Device&     m_device;
		std::string         m_strName;
		openni::SensorType  m_type;
		openni::VideoStream m_stream;
		openni::PixelFormat m_format;
	public:
		CStream(openni::Device& device, openni::SensorType type, openni::PixelFormat format, std::ostream& log);
		virtual ~CStream();
		const std::string& getName()const{ return m_strName; }
		bool               isValid()const;
		bool               isMirrorSupported()const;
		bool               setMirror(bool flag);
		void               setCloseRange(int& value);
		virtual bool       open(int w, int h, int fps);
		virtual bool       start();
		virtual void       destroy();
		virtual bool       getFrame(openni::VideoFrameRef& frame, uint64_t &timestamp, bool &there_is_obs, bool &hardware_error);

		int                getFrameWidth() const{ return m_stream.getVideoMode().getResolutionX(); }
		int                getFrameHeight()const{ return m_stream.getVideoMode().getResolutionY(); } 
		double             getHFov()const{ return m_stream.getHorizontalFieldOfView(); }
		double             getFx()  const{ return getFrameWidth()  / (2.0 * tan(getHFov() / 2.0)); }
		double             getVFov()const{ return m_stream.getVerticalFieldOfView(); }
		double             getFy()  const{ return getFrameHeight() / (2.0 * tan(getVFov() / 2.0)); }
		double             getCx()  const{ return (getFrameWidth()  - 1) * 0.5; }
		double             getCy()  const{ return (getFrameHeight() - 1) * 0.5; }

		void getCameraParam(mrpt::utils::TCamera & param)const{
			param.ncols = getFrameWidth();
			param.nrows = getFrameHeight();
			param.fx(getFx());
			param.fy(getFy());
			param.cx(getCx());
			param.cy(getCy());
		}

		static  Ptr        create(openni::Device& device, openni::SensorType type, openni::PixelFormat format, std::ostream& log);
	};
	openni::DeviceInfo  m_info; 
	openni::Device      m_device;
	CStream::Ptr        m_streams[STREAM_TYPE_SIZE];
	bool                m_mirror;
	std::stringstream   m_log;

	bool synchMirrorMode();
	bool startStreams();

	inline void resize(mrpt::utils::CImage& rgb  , int w, int h){ rgb.resize(w, h, CH_RGB, true); }
	inline void resize(mrpt::math::CMatrix& depth, int w, int h){ depth.resize(h, w); }
	inline void resize(mrpt::slam::CObservation3DRangeScan& obs, int w, int h){
		resize(obs.intensityImage, w, h);
		obs.rangeImage_setSize(h, w);
	}

	inline void setPixel(const openni::RGB888Pixel& src, mrpt::utils::CImage& rgb  , int x, int y){ rgb.setPixel(x, y, (src.r << 16) + (src.g << 8) + src.b); }
	inline void setPixel(const openni::DepthPixel& src , mrpt::math::CMatrix& depth, int x, int y){
		static const double rate = 1.0 / 1000;
		depth(y, x) = src * rate;
	}

	template <class NI_PIXEL, class MRPT_DATA>
	void copyRow(const char* src, MRPT_DATA& rgb, int w, const int y){
		const NI_PIXEL* s = (const NI_PIXEL*)src;
		for (int xc = 0; xc < w; ++xc, ++s){
			int x = xc;
			if(isMirrorMode()){
				x = w - xc - 1;
			}
			setPixel(*s, rgb, x, y);
		}
	}

	template <class NI_PIXEL, class MRPT_DATA>
	void copyFrame(openni::VideoFrameRef& frame, MRPT_DATA& dst){
		const char*  data    = (const char*)frame.getData();
		const int    stride  = frame.getStrideInBytes();
		const int    width   = frame.getWidth();
		const int    height  = frame.getHeight();
		resize(dst, width, height);
		for (int y = 0; y < height; ++y, data+=stride){
			copyRow<NI_PIXEL, MRPT_DATA>(data, dst, width, y);
		}
	}
public:
	CDevice(const openni::DeviceInfo& info, openni::PixelFormat rgb, openni::PixelFormat depth);
	virtual ~CDevice();

	const openni::DeviceInfo& getInfo()const{ return m_info; }
	std::string getLog()const{ return m_log.str(); }

	void clearLog(){ m_log.str(""); m_log.clear(); }
	bool isMirrorMode()const{ return m_mirror; }
	void setMirrorMode(bool mode){ m_mirror = mode; }
	bool hasColor()const{ return m_streams[COLOR_STREAM]->isValid(); }
	bool hasDepth()const{ return m_streams[DEPTH_STREAM]->isValid(); }

	bool isOpen()const;
	void close();
	bool open(int w, int h, int fps);

	bool getNextFrameRGB(mrpt::utils::CImage &img, uint64_t &timestamp, bool &there_is_obs, bool &hardware_error);
	bool getNextFrameD  (mrpt::math::CMatrix &img, uint64_t &timestamp, bool &there_is_obs, bool &hardware_error);
	bool getNextFrameRGBD(mrpt::slam::CObservation3DRangeScan &obs, bool &there_is_obs, bool &hardware_error);

	bool getCameraParam(int streamType, mrpt::utils::TCamera& param)const{
		if(streamType < 0 || streamType >= STREAM_TYPE_SIZE){
			return false;
		}
		if(m_streams[streamType]->isValid() == false){ return false; }
		m_streams[streamType]->getCameraParam(param);
		return true;
	}
	static Ptr create(const openni::DeviceInfo& info, openni::PixelFormat rgb, openni::PixelFormat depth);
};


/*-------------------------------------------------------------
ctor
-------------------------------------------------------------*/
COpenNI2Generic::COpenNI2Generic() :
	m_width(640),
	m_height(480),
	m_fps(30),
  m_verbose(false),
	m_rgb_format(openni::PIXEL_FORMAT_RGB888),
	m_depth_format(openni::PIXEL_FORMAT_DEPTH_1_MM),
  m_grab_image(true),
	m_grab_depth(true),
	m_grab_3D_points(true)
{
	if(numInstances == 0){
		if(openni::OpenNI::initialize() != openni::STATUS_OK){
			THROW_EXCEPTION(mrpt::format("After initialization:\n %s\n", openni::OpenNI::getExtendedError()))
		}else{
		  std::cerr << "[" << __FUNCTION__ << "]" << std::endl << " Initialized OpenNI2." << std::endl;
		}
	}
	numInstances++;
}

/*-------------------------------------------------------------
dtor
-------------------------------------------------------------*/
COpenNI2Generic::~COpenNI2Generic()
{
	numInstances--;
	if(numInstances == 0){
		kill();
	}
}

int COpenNI2Generic::getNumDevices()const{
	return vDevices.size();
}

void COpenNI2Generic::setVerbose(bool verbose){
  m_verbose = verbose;
}

bool COpenNI2Generic::isVerbose()const{
  return m_verbose;
}

void  COpenNI2Generic::showLog(const std::string& message)const{
  if(isVerbose() == false){
    return;
  }
  std::cerr << message;
}
/** This method can or cannot be implemented in the derived class, depending on the need for it.
*  \exception This method must throw an exception with a descriptive message if some critical error is found.
*/
int COpenNI2Generic::getConnectedDevices()
{
#if MRPT_HAS_OPENNI2
	// Get devices list
	openni::Array<openni::DeviceInfo> oni2InfoArray;
	openni::OpenNI::enumerateDevices(&oni2InfoArray);

	const size_t numDevices = oni2InfoArray.getSize();
	showLog(mrpt::format("[%s]\n", __FUNCTION__));
	showLog(mrpt::format(" Get device list. %d devices connected.\n", (int)numDevices));

	// Search new devices.
	std::set<int> newDevices;
	for(unsigned i=0; i < numDevices; i++){
		const openni::DeviceInfo& info = oni2InfoArray[i];
		showLog(mrpt::format("  Device[%d]\n", i));
		showLog(oni2DevInfoStr(info, 3) + "\n");
		bool isExist = false;
		for(unsigned int j = 0, j_end = vDevices.size();j < j_end && isExist == false;++j){
			if(cmpONI2Device(info, vDevices[j]->getInfo())){
				isExist = true;
			}
		}
		if(isExist == false){
			newDevices.insert(i);
		}
	}
	// Add new devices to device list(static member).
	for(std::set<int>::const_iterator it = newDevices.begin(), it_end = newDevices.end();it != it_end;++it){
		const openni::DeviceInfo& info = oni2InfoArray[*it];
		CDevice::Ptr device = CDevice::create(info, (openni::PixelFormat)m_rgb_format, (openni::PixelFormat)m_depth_format);
		vDevices.push_back(device);
	}

	if(getNumDevices() == 0){
	  showLog(" No devices connected -> EXIT\n");
	}else{
	  showLog(mrpt::format(" %d devices were found.\n", getNumDevices()));
	}
	return getNumDevices();
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

void COpenNI2Generic::kill()
{
#if MRPT_HAS_OPENNI2
	vDevices.clear();
	openni::OpenNI::shutdown();
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

bool COpenNI2Generic::isOpen(const unsigned sensor_id) const
{
  if((int)sensor_id >= getNumDevices()){
		return false;
	}
	return vDevices[sensor_id]->isOpen();
}

#if 0
bool COpenNI2Generic::initONI2RGBStream(unsigned sensor_id, int w, int h, int fps, void *pFormat)
{
	openni::Status rc = openni::STATUS_OK;
	rc = RGB_STREAM_ID_PTR->create(*DEVICE_ID_PTR, openni::SENSOR_COLOR);
	if(rc != openni::STATUS_OK){
		printf("%s: Couldn't find RGB stream:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
		return false;
	}
	rc = RGB_STREAM_ID_PTR->setMirroringEnabled(false);
	if (rc != openni::STATUS_OK){
		printf("%s: setMirroringEnabled(false) failed:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
		return false;
	}
	openni::VideoMode options = RGB_STREAM_ID_PTR->getVideoMode();
	printf("Initial resolution RGB (%d, %d) FPS %d Format %d\n", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat());
	if(setONI2StreamMode(*RGB_STREAM_ID_PTR, w, h, fps, *(reinterpret_cast<openni::PixelFormat*>(pFormat))) == false){
		printf("%s: Can't find desired rgb mode\n", __FUNCTION__ );
		return false;
	}
	options = RGB_STREAM_ID_PTR->getVideoMode();
	printf("  -> (%d, %d) FPS %d Format %d\n", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat());
	rc = RGB_STREAM_ID_PTR->start();
	if (rc != openni::STATUS_OK){
		printf("%s: Couldn't start RGB stream:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
		RGB_STREAM_ID_PTR->destroy();
		return false;
	}
	return true;
}

bool COpenNI2Generic::initONI2DepthStream(unsigned sensor_id, int w, int h, int fps, void *pFormat)
{
	openni::Status rc = DEPTH_STREAM_ID_PTR->create(*DEVICE_ID_PTR, openni::SENSOR_DEPTH);
	if (rc != openni::STATUS_OK){
		printf("%s: Couldn't find depth stream:\n%s\n", __FUNCTION__, openni::OpenNI::getExtendedError());
		return false;
	}
	openni::VideoMode options = DEPTH_STREAM_ID_PTR->getVideoMode();
	printf("Initial resolution Depth(%d, %d) FPS %d Format %d\n", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat());
	if(setONI2StreamMode(*DEPTH_STREAM_ID_PTR, w, h, fps, *(reinterpret_cast<openni::PixelFormat*>(pFormat))) == false){
		printf("%s: Can't find desired depth mode\n", __FUNCTION__ );
		return false;
	}
	options = DEPTH_STREAM_ID_PTR->getVideoMode();
	printf("  -> (%d, %d) FPS %d Format %d\n", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat());
	DEPTH_STREAM_ID_PTR->setMirroringEnabled(false);
	rc = DEPTH_STREAM_ID_PTR->start();
	if (rc != openni::STATUS_OK){
		printf(" Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
		DEPTH_STREAM_ID_PTR->destroy();
		return false;
	}
	return true;
}
#endif

void COpenNI2Generic::open(unsigned sensor_id)
{
#if MRPT_HAS_OPENNI2
	// Sensor index validation.
	if (!getNumDevices()){
		THROW_EXCEPTION("No OpenNI2 devices found.")
	}
	if ((int)sensor_id >= getNumDevices()){
		THROW_EXCEPTION("Sensor index is higher than the number of connected devices.")
	}
	showLog(mrpt::format("[%s]\n", __FUNCTION__));
	showLog(mrpt::format(" open[%d] ...\n", sensor_id));

	if(isOpen(sensor_id)){
	  showLog(mrpt::format(" The sensor [%d] is already opened\n", sensor_id));
		return;
	}
	vDevices[sensor_id]->open(m_width, m_height, m_fps);
	showLog(vDevices[sensor_id]->getLog() + "\n");
	showLog(mrpt::format(" Device [%d] ", sensor_id));
	if(vDevices[sensor_id]->isOpen()){
	  showLog(" open successfully.\n");
	}else{
	  showLog(" open failed.\n");
	}
	mrpt::system::sleep(2000); // Sleep 2s
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

void COpenNI2Generic::close(unsigned sensor_id)
{
#if MRPT_HAS_OPENNI2
	// Sensor index validation.
	if (!getNumDevices()){
		THROW_EXCEPTION("No OpenNI2 devices found.")
	}
	if ((int)sensor_id >= getNumDevices()){
		THROW_EXCEPTION("Sensor index is higher than the number of connected devices.")
	}
	vDevices[sensor_id]->close();
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

/** The main data retrieving function, to be called after calling loadConfig() and initialize().
*  \param out_obs The output retrieved observation (only if there_is_obs=true).
*  \param timestamp The timestamp of the capture (only if there_is_obs=true).
*  \param there_is_obs If set to false, there was no new observation.
*  \param hardware_error True on hardware/comms error.
*  \param sensor_id The index of the sensor accessed.
*
*/
void COpenNI2Generic::getNextFrameRGB(
	mrpt::utils::CImage &rgb_img,
	uint64_t &timestamp,
	bool &there_is_obs,
	bool &hardware_error,
	unsigned sensor_id )
{
#if MRPT_HAS_OPENNI2
	// Sensor index validation.
	if (!getNumDevices()){
		THROW_EXCEPTION("No OpenNI2 devices found.")
	}
	if ((int)sensor_id >= getNumDevices()){
		THROW_EXCEPTION("Sensor index is higher than the number of connected devices.")
	}
	if(vDevices[sensor_id]->getNextFrameRGB(rgb_img, timestamp, there_is_obs, hardware_error) == false){
	  showLog(mrpt::format("[%s]\n", __FUNCTION__ ));
	  showLog(mrpt::format(" Error [%d]th Sensor.\n", sensor_id));
	  showLog(std::string(" ") + vDevices[sensor_id]->getLog() + "\n");
	}
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

/** The main data retrieving function, to be called after calling loadConfig() and initialize().
*  \param depth_img The output retrieved depth image (only if there_is_obs=true).
*  \param timestamp The timestamp of the capture (only if there_is_obs=true).
*  \param there_is_obs If set to false, there was no new observation.
*  \param hardware_error True on hardware/comms error.
*  \param sensor_id The index of the sensor accessed.
*
*/
void COpenNI2Generic::getNextFrameD(
	mrpt::math::CMatrix &depth_img,
    uint64_t &timestamp,
	bool &there_is_obs,
	bool &hardware_error,
	unsigned sensor_id )
{ 
#if MRPT_HAS_OPENNI2
	// Sensor index validation.
	if (getNumDevices() == 0){
		THROW_EXCEPTION("No OpenNI2 devices found.")
	}
	if ((int)sensor_id >= getNumDevices()){
		THROW_EXCEPTION("Sensor index is higher than the number of connected devices.")
	}
	if(vDevices[sensor_id]->getNextFrameD(depth_img, timestamp, there_is_obs, hardware_error) == false){
	  showLog(mrpt::format("[%s]\n", __FUNCTION__));
	  showLog(mrpt::format(" Error [%d]th Sensor.\n", sensor_id));
	  showLog(std::string(" ") + vDevices[sensor_id]->getLog() + "\n");
	}
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

/** The main data retrieving function, to be called after calling loadConfig() and initialize().
*  \param out_obs The output retrieved observation (only if there_is_obs=true).
*  \param there_is_obs If set to false, there was no new observation.
*  \param hardware_error True on hardware/comms error.
*  \param sensor_id The index of the sensor accessed.
*
*/
void COpenNI2Generic::getNextFrameRGBD(
	mrpt::slam::CObservation3DRangeScan &out_obs,
	bool &there_is_obs,
	bool &hardware_error,
	unsigned sensor_id )
{
#if MRPT_HAS_OPENNI2
//	cout << "COpenNI2Generic::getNextFrameRGBD \n";
#if 0
	there_is_obs=false;
	hardware_error = false;

	// Read a frame (depth + rgb)
	DEPTH_STREAM_ID_PTR->readFrame(DEPTH_FRAME_ID_PTR);

	RGB_STREAM_ID_PTR->readFrame(RGB_FRAME_ID_PTR);

	if ((DEPTH_FRAME_ID_PTR->getWidth() != RGB_FRAME_ID_PTR->getWidth()) || (DEPTH_FRAME_ID_PTR->getHeight() != RGB_FRAME_ID_PTR->getHeight()))
	{
		cout << "\nBoth frames don't have the same size.";
	}
	else
	{
		there_is_obs=true;

		CObservation3DRangeScan  newObs;
		newObs.hasConfidenceImage = false;

		// Set intensity image ----------------------
		if (m_grab_image)
		{
      if(!m_has_color)
        THROW_EXCEPTION("This OpenNI2 device does not support color imaging")
			newObs.hasIntensityImage  = true;
		}

		// Set range image --------------------------
		if (m_grab_depth || m_grab_3D_points)
		{
			newObs.hasRangeImage = true;
			newObs.range_is_depth = true;
		}

		newObs.timestamp = mrpt::system::getCurrentTime();
		newObs.rangeImage_setSize(height,width);

		// Read one frame
		const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)DEPTH_FRAME_ID_PTR->getData();
		const openni::RGB888Pixel* pRgbRow = (const openni::RGB888Pixel*)RGB_FRAME_ID_PTR->getData();
		int rowSize = DEPTH_FRAME_ID_PTR->getStrideInBytes() / sizeof(openni::DepthPixel);

		utils::CImage iimage(width,height,CH_RGB);
		for (int yc = 0; yc < DEPTH_FRAME_ID_PTR->getHeight(); ++yc)
		{
			const openni::DepthPixel* pDepth = pDepthRow;
			const openni::RGB888Pixel* pRgb = pRgbRow;
			for (int xc = 0; xc < DEPTH_FRAME_ID_PTR->getWidth(); ++xc, ++pDepth, ++pRgb)
			{
				newObs.rangeImage(yc,xc) = (*pDepth)*1.0/1000;
				iimage.setPixel(xc,yc,(pRgb->r<<16)+(pRgb->g<<8)+pRgb->b);

				//newObs.intensityImage.setPixel(xc,yc,(*pRgb));
			}

			pDepthRow += rowSize;
			pRgbRow += rowSize;
		}
		newObs.intensityImage = iimage;

		// Save the observation to the user's object:
		out_obs.swap(newObs);

		// Set common data into observation:
		// --------------------------------------
		out_obs.timestamp = mrpt::system::now();
	}
#endif
	// Sensor index validation.
	if (!getNumDevices()){
		THROW_EXCEPTION("No OpenNI2 devices found.")
	}
	if ((int)sensor_id >= getNumDevices()){
		THROW_EXCEPTION("Sensor index is higher than the number of connected devices.")
	}
	if(vDevices[sensor_id]->getNextFrameRGBD(out_obs, there_is_obs, hardware_error) == false){
	  showLog(mrpt::format("[%s]\n", __FUNCTION__));
	  showLog(mrpt::format(" Error [%d]th Sensor.\n", sensor_id));
	  showLog(std::string(" ") + vDevices[sensor_id]->getLog() + "\n");
	}
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}

bool  COpenNI2Generic::getColorSensorParam(mrpt::utils::TCamera& param, unsigned sensor_id)const{
	if(isOpen(sensor_id) == false){
		return false;
	}
	return vDevices[sensor_id]->getCameraParam(CDevice::COLOR_STREAM, param);
}

bool  COpenNI2Generic::getDepthSensorParam(mrpt::utils::TCamera& param, unsigned sensor_id)const{
	if(isOpen(sensor_id) == false){
		return false;
	}
	return vDevices[sensor_id]->getCameraParam(CDevice::DEPTH_STREAM, param);
}
/*
void openni::VideoMode::setResolution()
Setter function for the resolution of this VideoMode. Application use of this function is not recommended.
Instead, use SensorInfo::getSupportedVideoModes() to obtain a list of valid video modes

-- cited from OpenNI2 help. setResolution() is not recommended.
*/
bool setONI2StreamMode(openni::VideoStream& stream, int w, int h, int fps, openni::PixelFormat format){
	//std::cout << "Ask mode: " << w << "x" << h << " " << fps << " fps. format " << format << std::endl;
	bool found = false;
	const openni::Array<openni::VideoMode>& modes = stream.getSensorInfo().getSupportedVideoModes();
	for(int i = 0, i_end = modes.getSize();i < i_end;++i){
		// std::cout << "Mode: " << modes[i].getResolutionX() << "x" << modes[i].getResolutionY() << " " << modes[i].getFps() << " fps. format " << modes[i].getPixelFormat() << std::endl;
		if(modes[i].getResolutionX() != w){
			continue;
		}
		if(modes[i].getResolutionY() != h){
			continue;
		}
		if(modes[i].getFps() != fps){
			continue;
		}
		if(modes[i].getPixelFormat() != format){
			continue;
		}
		openni::Status rc = stream.setVideoMode(modes[i]);
		if(rc != openni::STATUS_OK){
			return false;
		}
		return true;
	}
	return false;
}

std::string oni2DevInfoStr(const openni::DeviceInfo& info, int tab){
        std::stringstream sst;
	std::string space;
	for(int i = 0;i < tab;++i){
	  space += " ";
	}
	sst << space << "name="    << info.getName()        << std::endl;
	sst << space << "uri="     << info.getUri()         << std::endl;
	sst << space << "vendor="  << info.getVendor()      << std::endl;
	sst << space << "product=" << info.getUsbProductId();
	return sst.str();
}

bool cmpONI2Device(const openni::DeviceInfo& i1, const openni::DeviceInfo& i2){
	return (strcmp(i1.getUri(), i2.getUri()) == 0);
}
//
COpenNI2Generic::CDevice::CDevice(const openni::DeviceInfo& info, openni::PixelFormat rgb, openni::PixelFormat depth)
 :m_info(info), m_mirror(true)
{
	m_streams[COLOR_STREAM] = CStream::create(m_device, openni::SENSOR_COLOR, rgb  , m_log);
	m_streams[DEPTH_STREAM] = CStream::create(m_device, openni::SENSOR_DEPTH, depth, m_log);
}

COpenNI2Generic::CDevice::~CDevice(){
	close();
}

bool COpenNI2Generic::CDevice::synchMirrorMode(){
	m_mirror = false;
	// Check whether both stream support mirroring.
	for(int i = 0;i < STREAM_TYPE_SIZE;++i){
		bool mirror_support;
		try{
			mirror_support = m_streams[i]->isMirrorSupported();
		}catch(std::logic_error& e){
			throw(e);
		}
		if(mirror_support == false){
		  m_log << "[" << __FUNCTION__ << "]" << std::endl;
			m_log << " openni::STREAM_PROPERTY_MIRRORING is not supported on " << m_streams[i]->getName() << "." << std::endl;
			m_log << " We assume this is MS Kinect and taken images are inverted to right and left." << std::endl;
			// In this case, getMirroringEnabled() method always returns false. So we cannot confirm whether the images are inverted or not.
			m_mirror  = true;
			break;
		}
	}
	// Set both stream to same mirror mode.
	for(int i = 0;i < STREAM_TYPE_SIZE;++i){
		if(m_streams[i]->isMirrorSupported() == false){
			break;
		}
		if(m_streams[i]->setMirror(m_mirror) == false){
			return false;
		}
	}
	return true;
}

bool COpenNI2Generic::CDevice::startStreams(){
	for(int i = 0;i < STREAM_TYPE_SIZE;++i){
		if(m_streams[i]->start() == false){
			return false;
		}
	}
	return true;
}

bool COpenNI2Generic::CDevice::isOpen()const{
	return m_streams[COLOR_STREAM]->isValid() || m_streams[DEPTH_STREAM]->isValid();
}

void COpenNI2Generic::CDevice::close(){
	for(int i = 0;i < STREAM_TYPE_SIZE;++i){
		m_streams[i]->destroy();
	}
	m_device.close();
}

bool COpenNI2Generic::CDevice::open(int w, int h, int fps){
	clearLog();
	close();
	m_log << "[" << __FUNCTION__ << "]" << std::endl;
	openni::Status rc = m_device.open(getInfo().getUri());
	if(rc != openni::STATUS_OK){
	  m_log << "[" <<  __FUNCTION__ << "]" << std::endl << " Failed to open device " << getInfo().getUri() << " " << openni::OpenNI::getExtendedError() << std::endl;
		return false;
	}
	for(int i = 0;i < STREAM_TYPE_SIZE;++i){
		if(m_streams[i]->open(w, h, fps) == false){
			return false;
		}
	}
	if(synchMirrorMode() == false){
		close();
		return false;
	}
	int CloseRange;
	m_streams[DEPTH_STREAM]->setCloseRange(CloseRange);
	m_log << " Close range: " <<  (CloseRange? "On" : "Off") << std::endl;

	if(m_device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)){
		if(m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR) != openni::STATUS_OK){
			m_log << " setImageRegistrationMode() Failed:" << openni::OpenNI::getExtendedError() << endl;
		}else{
			m_log << " setImageRegistrationMode() Success" << endl;
		}
	}else{
		m_log << "  Device doesn't do image registration!" << endl;
	}

	if(startStreams() == false){
		close();
		return false;
	}
	return true;
}

bool COpenNI2Generic::CDevice::getNextFrameRGB(mrpt::utils::CImage &img, uint64_t &timestamp, bool &there_is_obs, bool &hardware_error){ 
#if MRPT_HAS_OPENNI2
	if(!hasColor()){
		THROW_EXCEPTION("This OpenNI2 device does not support color imaging")
	}
	openni::VideoFrameRef frame;
	if(m_streams[COLOR_STREAM]->getFrame(frame, timestamp, there_is_obs, hardware_error) == false){
		return false;
	}
	copyFrame<openni::RGB888Pixel, mrpt::utils::CImage>(frame, img);
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
		return true;
}

bool COpenNI2Generic::CDevice::getNextFrameD(mrpt::math::CMatrix &img, uint64_t &timestamp, bool &there_is_obs, bool &hardware_error){
#if MRPT_HAS_OPENNI2
	if(!hasDepth()){
		THROW_EXCEPTION("This OpenNI2 device does not support depth imaging")
	}
	openni::VideoFrameRef frame;
	if(m_streams[DEPTH_STREAM]->getFrame(frame, timestamp, there_is_obs, hardware_error) == false){
		return false;
	}
	copyFrame<openni::DepthPixel, mrpt::math::CMatrix>(frame, img);
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
		return true;
}

bool COpenNI2Generic::CDevice::getNextFrameRGBD(mrpt::slam::CObservation3DRangeScan &obs, bool &there_is_obs, bool &hardware_error){ 
#if MRPT_HAS_OPENNI2
	clearLog();
	there_is_obs   = false;
	hardware_error = false;

	if(!hasColor()){
		THROW_EXCEPTION("This OpenNI2 device does not support color imaging")
	}
	if(!hasDepth()){
		THROW_EXCEPTION("This OpenNI2 device does not support depth imaging")
	}
	// Read a frame (depth + rgb)
	mrpt::system::TTimeStamp tm;
	openni::VideoFrameRef    frame[STREAM_TYPE_SIZE];
	for(int i = 0;i < STREAM_TYPE_SIZE;++i){
		if(m_streams[i]->getFrame(frame[i], tm, there_is_obs, hardware_error) == false){
			return false;
		}
		if(there_is_obs == false || hardware_error == true){
			return false;
		}
	}

	const int width  = frame[COLOR_STREAM].getWidth();
	const int height = frame[COLOR_STREAM].getHeight();
	if((frame[DEPTH_STREAM].getWidth() != width) || (frame[DEPTH_STREAM].getHeight() != height)){
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " Both frames don't have the same size." << std::endl;
		return false;
	}
	there_is_obs = true;
	obs.hasConfidenceImage = false;
	obs.hasIntensityImage  = true;
	obs.hasRangeImage      = true;
	obs.range_is_depth     = true;
	obs.hasPoints3D        = false;
	obs.timestamp          = mrpt::system::getCurrentTime();
	resize(obs, width, height);

	const char* data[STREAM_TYPE_SIZE] = 
	{
		(const char*)frame[COLOR_STREAM].getData(),
		(const char*)frame[DEPTH_STREAM].getData()
	};
	const int   step[STREAM_TYPE_SIZE] = 
	{
		frame[COLOR_STREAM].getStrideInBytes(),
		frame[DEPTH_STREAM].getStrideInBytes()
	};

	for (int yc = 0; yc < height; ++yc){
		const openni::RGB888Pixel* pRgb   = (const openni::RGB888Pixel*)data[COLOR_STREAM];
		const openni::DepthPixel * pDepth = (const openni::DepthPixel *)data[DEPTH_STREAM];
		for (int xc = 0; xc < width; ++xc, ++pDepth, ++pRgb){
			int x = xc;
			if(isMirrorMode()){
				x = width - x - 1;
			}
			setPixel(*pRgb  , obs.intensityImage, x, yc);
			setPixel(*pDepth, obs.rangeImage    , x, yc);
		}
		data[COLOR_STREAM] += step[COLOR_STREAM];
		data[DEPTH_STREAM] += step[DEPTH_STREAM];
	}
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
		return true;
}

COpenNI2Generic::CDevice::Ptr COpenNI2Generic::CDevice::create(const openni::DeviceInfo& info, openni::PixelFormat rgb, openni::PixelFormat depth){ 
	return Ptr(new CDevice(info, rgb, depth)); 
}
//
COpenNI2Generic::CDevice::CStream::CStream(openni::Device& device, openni::SensorType type, openni::PixelFormat format, std::ostream& log)
	:m_log(log), m_device(device), m_strName("Unknown"), m_type(type), m_format(format)
{
	if(m_type == openni::SENSOR_COLOR){
		m_strName = "openni::SENSOR_COLOR";
	}else if(m_type == openni::SENSOR_DEPTH){
		m_strName = "openni::SENSOR_DEPTH";
	}else{
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " Unknown SensorType -> " << m_type << std::endl;
	}
}

COpenNI2Generic::CDevice::CStream::~CStream(){
	destroy();
}

bool COpenNI2Generic::CDevice::CStream::isMirrorSupported()const{
	if(isValid() == false){
		THROW_EXCEPTION(getName() + " is not opened.");
	}
	return m_stream.isPropertySupported(openni::STREAM_PROPERTY_MIRRORING);
}

bool COpenNI2Generic::CDevice::CStream::setMirror(bool flag){
	if(isValid() == false){
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " " << getName() << " is not opened." << std::endl;
		return false;
	}
	if(m_stream.isPropertySupported(openni::STREAM_PROPERTY_MIRRORING) == false){
		return false;
	}
	if(m_stream.setMirroringEnabled(flag) != openni::STATUS_OK){
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " setMirroringEnabled() failed: " << openni::OpenNI::getExtendedError() << std::endl;
		return false;
	}
	return true;
}

bool COpenNI2Generic::CDevice::CStream::isValid()const{ 
	return m_stream.isValid(); 
}

void COpenNI2Generic::CDevice::CStream::destroy(){
	m_stream.destroy();
}

void COpenNI2Generic::CDevice::CStream::setCloseRange(int& value){
	m_stream.setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, value);
	m_stream.getProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, &value);
}

bool COpenNI2Generic::CDevice::CStream::open(int w, int h, int fps){
	destroy();
	if(m_type != openni::SENSOR_COLOR && m_type != openni::SENSOR_DEPTH){
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " Unknown SensorType -> " << m_type << std::endl;
		return false;
	}
	openni::Status rc = openni::STATUS_OK;
	rc = m_stream.create(m_device, m_type);
	if(rc != openni::STATUS_OK){
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " Couldn't find sensor " << m_strName << ":" << openni::OpenNI::getExtendedError() << std::endl;
		return false;
	}
	openni::VideoMode options = m_stream.getVideoMode();
	m_log << "[" << __FUNCTION__ << "]" << std::endl;
	m_log << " " << m_strName << std::endl;
	m_log << " " << mrpt::format("Initial resolution (%d, %d) FPS %d Format %d", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat()) << std::endl;
	if(setONI2StreamMode(m_stream, w, h, fps, m_format) == false){
		m_log << " Can't find desired mode in the " << getName() << std::endl;
		destroy();
		return false;
	}
	options = m_stream.getVideoMode();
	m_log << " " << mrpt::format("-> (%d, %d) FPS %d Format %d", options.getResolutionX(), options.getResolutionY(), options.getFps(), options.getPixelFormat()) << std::endl;
	return true;
}

bool COpenNI2Generic::CDevice::CStream::start(){
	if(isValid() == false){
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " " << getName() << " is not opened." << std::endl;
		return false; 
	}
	if(m_stream.start() != openni::STATUS_OK){
	  m_log << "[" << __FUNCTION__ << "]" << std::endl << " Couldn't start " << getName() << " stream:" << openni::OpenNI::getExtendedError() << std::endl;
	}
	return true;
}

COpenNI2Generic::CDevice::CStream::Ptr COpenNI2Generic::CDevice::CStream::create(openni::Device& device, openni::SensorType type, openni::PixelFormat format, std::ostream& log){ 
	return Ptr(new CStream(device, type, format, log));
}

bool COpenNI2Generic::CDevice::CStream::getFrame(openni::VideoFrameRef& frame, uint64_t &timestamp, bool &there_is_obs, bool &hardware_error)
{ 
#if MRPT_HAS_OPENNI2
	there_is_obs   = false;
	hardware_error = false;
	if(isValid() == false){
		return false;
	}
	openni::Status rc = m_stream.readFrame(&frame);
	if(rc != openni::STATUS_OK){
		hardware_error = true;
		std::string message = mrpt::format("Failed to grab frame from %s", getName().c_str());
		THROW_EXCEPTION(message);
	}
	there_is_obs = true;
	timestamp    = mrpt::system::getCurrentTime();
	return true;
#else
	THROW_EXCEPTION("MRPT was built without OpenNI2 support")
#endif // MRPT_HAS_OPENNI2
}
