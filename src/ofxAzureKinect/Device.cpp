#include "Device.h"

#include "ofLog.h"


namespace ofxAzureKinect
{
	static constexpr int32_t TIMEOUT_IN_MS = 1000;
	static constexpr int64_t WAIT_FOR_SYNCHRONIZED_CAPTURE_TIMEOUT = 60000;

	static void log_lagging_time(const char *lagger, k4a::capture &master, k4a::capture &sub)
	{
		int32_t diff = master.get_color_image().get_device_timestamp().count() - sub.get_color_image().get_device_timestamp().count();
		std::cout << std::setw(6) << lagger << " lagging: mc:" << std::setw(6)
			<< master.get_color_image().get_device_timestamp().count() << "us sc:" << std::setw(6)
			<< sub.get_color_image().get_device_timestamp().count() << "us diff:" << diff << endl;
	}

	static void log_synced_image_time(k4a::capture &master, k4a::capture &sub)
	{
		int32_t diff = master.get_color_image().get_device_timestamp().count() - sub.get_color_image().get_device_timestamp().count();
		std::cout << "Sync'd capture: mc:" << std::setw(6) << master.get_color_image().get_device_timestamp().count()
			<< "us sc:" << std::setw(6) << sub.get_color_image().get_device_timestamp().count() << "us diff:" << diff << endl;
	}

	DeviceSettings::DeviceSettings(int idx)
		: depthMode(K4A_DEPTH_MODE_WFOV_2X2BINNED)
		, colorResolution(K4A_COLOR_RESOLUTION_2160P)
		, colorFormat(K4A_IMAGE_FORMAT_COLOR_BGRA32)
		, cameraFps(K4A_FRAMES_PER_SECOND_30)
		, wiredSyncMode(K4A_WIRED_SYNC_MODE_STANDALONE)
		, depthDelayUsec(0)
		, subordinateDelayUsec(0)
		, updateColor(true)
		, updateIr(true)
		, updateWorld(true)
		, updateVbo(true)
		, syncImages(true)
		, enableIMU(false)
	{}

	BodyTrackingSettings::BodyTrackingSettings()
		: sensorOrientation(K4ABT_SENSOR_ORIENTATION_DEFAULT)
		, processingMode(K4ABT_TRACKER_PROCESSING_MODE_GPU)
		, gpuDeviceID(0)
		, updateBodies(false)
	{}

	void Device::Frame::swapFrame(Frame & f)
	{
		this->depthPix.swap(f.depthPix);
		std::swap(this->depthPixDeviceTime, f.depthPixDeviceTime);

		if (this->bColorPixUpdated) {
			this->colorPix.swap(f.colorPix);
			std::swap(this->colorPixDeviceTime, f.colorPixDeviceTime);
			std::swap(this->bColorPixUpdated, f.bColorPixUpdated);
		}

		this->irPix.swap(f.irPix);
		this->depthInColorPix.swap(f.depthInColorPix);
		this->colorInDepthPix.swap(f.colorInDepthPix);
		this->bodyIndexPix.swap(f.bodyIndexPix);
		std::swap(this->bodySkeletons, f.bodySkeletons);
		std::swap(this->bodyIDs, f.bodyIDs);

		std::swap(this->positionCache, f.positionCache);
		std::swap(this->uvCache, f.uvCache);
		std::swap(this->numPoints, f.numPoints);
	}

	void Device::Frame::reset()
	{
		*this = Frame();
	}

	Device::JpegDecodeThread::JpegDecodeThread() : jpegDecompressor(tjInitDecompress())
	{
	}

	Device::JpegDecodeThread::~JpegDecodeThread()
	{
		toProcess.close();
		processed.close();
		tjDestroy(jpegDecompressor);
	}

	bool Device::JpegDecodeThread::pushTaskIfEmpty(JpegTask & b)
	{
		if (toProcess.empty()) {
			toProcess.send(b);
			return true;
		}
		return false;
	}

	bool Device::JpegDecodeThread::update(ofPixels & outPix, std::chrono::microseconds & outTime)
	{
		DecodedPix ret;
		bool onceReceived = false;
		while (processed.tryReceive(ret)) {
			onceReceived = true;
		}

		if (onceReceived) {
			outPix = std::move(ret.colorPix);
			outTime = ret.colorPixDeviceTime;
		}
		return onceReceived;
	}

	void Device::JpegDecodeThread::threadedFunction()
	{
		while (isThreadRunning()) {
			JpegTask b;
			if (toProcess.tryReceive(b)) {
				DecodedPix pix;
				int width, height, jpegSubsamp, jpegColorspace;
				const int header = tjDecompressHeader3(this->jpegDecompressor,
					(const unsigned char*)b.colorPixBuf.getData(),
					static_cast<unsigned long>(b.colorPixBuf.size()),
					&width, &height, &jpegSubsamp, &jpegColorspace);
				pix.colorPix.allocate(width, height, OF_PIXELS_BGRA);
				const int decompressStatus = tjDecompress2(this->jpegDecompressor,
					(const unsigned char*)b.colorPixBuf.getData(),
					static_cast<unsigned long>(b.colorPixBuf.size()),
					pix.colorPix.getData(),
					width, 0, height, TJPF_BGRA, TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE);
				pix.colorPixDeviceTime = b.colorPixDeviceTime;
				processed.send(pix);
			}
			std::this_thread::sleep_for(std::chrono::microseconds(100));
		}
	}

	int Device::getInstalledCount()
	{
		return k4a_device_get_installed_count();
	}

	Device::Device()
		: index(-1)
		, pixFrameNum(0)
		, texFrameNum(0)
		, bOpen(false)
		, bStreaming(false)
		, bNewFrame(false)
		, bNewBuffer(false)
		, bUpdateColor(false)
		, bUpdateIr(false)
		, bUpdateBodies(false)
		, bUpdateWorld(false)
		, bUpdateVbo(false)
		, bodyTracker(nullptr)
		, jpegDecompressor(tjInitDecompress())
		, bPlayback(false)
		, bEnableIMU(false)
		, bMultiDeviceSyncCapture(false)
		, bRecording(false)
		, bAsyncJpegDecode(false)
	{}

	Device::~Device()
	{
		this->close();

		tjDestroy(jpegDecompressor);
	}

	bool Device::load(string filename)
	{
		bPlayback = true;
		playback = new Playback();

		if (playback->load(filename))
		{
			k4a_record_configuration_t playback_config = playback->getDeviceSettings();

			this->config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
			this->config.depth_mode = playback_config.depth_mode;
			this->config.color_format = playback_config.color_format;
			this->config.color_resolution = playback_config.color_resolution;
			this->config.camera_fps = playback_config.camera_fps;
			this->bEnableIMU = playback_config.imu_track_enabled;

			this->config.wired_sync_mode = playback_config.wired_sync_mode;
			this->config.depth_delay_off_color_usec = playback_config.depth_delay_off_color_usec;
			this->config.subordinate_delay_off_master_usec = playback_config.subordinate_delay_off_master_usec;

			this->serialNumber = playback->getSerialNumber();
			this->bUpdateColor = playback_config.color_track_enabled;
			this->bUpdateIr = playback_config.ir_track_enabled;
			this->bUpdateWorld = playback_config.depth_track_enabled;
			this->bUpdateVbo = playback_config.depth_track_enabled;

			// Add Playback Listeners
			this->eventListeners.push(this->play.newListener([this](bool) {
				listener_playback_play(this->play);
			}));
			this->eventListeners.push(this->pause.newListener([this](bool) {
				listener_playback_pause(this->pause);
			}));
			this->eventListeners.push(this->stop.newListener([this](bool) {
				listener_playback_stop(this->stop);
			}));
			this->eventListeners.push(this->seek.newListener([this](bool) {
				listener_playback_seek(this->seek);
			}));

			ofLogNotice(__FUNCTION__) << "Successfully opened device " << this->index << " with serial number " << this->serialNumber << ".";

			bOpen = true;
			return true;
		}
		return false;
	}

	bool Device::open(uint32_t idx)
	{
		if (this->bOpen)
		{
			ofLogWarning(__FUNCTION__) << "Device " << this->index << " / " << this->serialNumber << " already open!";
			return false;
		}

		// Load the device at the requested index.
		try
		{
			// Open connection to the device.
			this->device = k4a::device::open(idx);

			// Get the device index and serial number.
			this->index = idx;
			this->serialNumber = this->device.get_serialnum();
		}
		catch (const k4a::error& e)
		{
			ofLogError(__FUNCTION__) << e.what();

			this->device.close();

			return false;
		}

		ofLogNotice(__FUNCTION__) << "Successfully opened device " << this->index << " / " << this->serialNumber << ".";
		this->bOpen = true;

		return true;
		//return this->open(DeviceSettings(idx), BodyTrackingSettings());
	}

	bool Device::open(const std::string & serialNumber)
	{
		if (this->bOpen)
		{
			ofLogWarning(__FUNCTION__) << "Device " << this->index << " / " << this->serialNumber << " already open!";
			return false;
		}

		// Loop through devices and find the one with the requested serial.
		bool deviceFound = false;
		int numConnected = Device::getInstalledCount();
		for (int i = 0; i < numConnected; ++i)
		{
			try
			{
				// Open connection to the device.
				this->device = k4a::device::open(static_cast<uint32_t>(i));

				// Get the device serial number and check it.
				this->serialNumber = this->device.get_serialnum();
				if (this->serialNumber == serialNumber)
				{
					deviceFound = true;
					this->index = i;
					break;
				}
				else
				{
					this->device.close();
				}
			}
			catch (const k4a::error& e)
			{
				// Don't worry about it; we just might be trying to access an already open device.
				continue;
			}
		}

		if (!deviceFound)
		{
			ofLogError(__FUNCTION__) << "No device found with serial number " << serialNumber;
			return false;
		}

		ofLogNotice(__FUNCTION__) << "Successfully opened device " << this->index << " / " << this->serialNumber << ".";
		this->bOpen = true;

		return true;
	}

	bool Device::close()
	{
		if (!this->bOpen)
			return false;

		if (bPlayback)
		{
			this->stopCameras();
		}
		else
		{
			// Stop IMU if cameras are enabled
			if (this->bEnableIMU)
			{
				k4a_device_stop_imu(device.handle());
			}

			this->stopCameras();

			this->device.close();
		}

		this->eventListeners.unsubscribeAll();

		this->index = -1;
		this->bOpen = false;
		this->serialNumber = "";

		return true;
	}

	bool Device::startCameras(DeviceSettings deviceSettings, BodyTrackingSettings bodyTrackingSettings)
	{
		if (!this->bOpen)
		{
			ofLogError(__FUNCTION__) << "Open device before starting cameras!";
			return false;
		}

		if (!bPlayback) {
			// Generate device config.
			this->config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
			this->config.depth_mode = deviceSettings.depthMode;
			this->config.color_format = deviceSettings.colorFormat;
			this->config.color_resolution = deviceSettings.colorResolution;
			this->config.camera_fps = deviceSettings.cameraFps;
			this->config.synchronized_images_only = deviceSettings.syncImages;
			this->bEnableIMU = deviceSettings.enableIMU;

			this->config.wired_sync_mode = deviceSettings.wiredSyncMode;
			this->config.depth_delay_off_color_usec = deviceSettings.depthDelayUsec;
			this->config.subordinate_delay_off_master_usec = deviceSettings.subordinateDelayUsec;

			// Generate tracker config.
			this->trackerConfig.sensor_orientation = bodyTrackingSettings.sensorOrientation;
			this->trackerConfig.gpu_device_id = bodyTrackingSettings.gpuDeviceID;

			// Set update flags.
			this->bUpdateColor = deviceSettings.updateColor;
			this->bUpdateIr = deviceSettings.updateIr;
			this->bUpdateWorld = deviceSettings.updateWorld;
			this->bUpdateVbo = deviceSettings.updateWorld && deviceSettings.updateVbo;
		}
		this->bUpdateBodies = bodyTrackingSettings.updateBodies;

		// Get calibration.
		if (bPlayback)
		{
			auto calibration_handle = playback->getCalibration();
			this->calibration.depth_camera_calibration = calibration_handle.depth_camera_calibration;
			this->calibration.color_camera_calibration = calibration_handle.color_camera_calibration;
			this->calibration.depth_mode = calibration_handle.depth_mode;
			this->calibration.color_resolution = calibration_handle.color_resolution;

			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
					this->calibration.extrinsics[i][j] = calibration_handle.extrinsics[i][j];
			}
		}
		else
		{
			try
			{
				this->calibration = this->device.get_calibration(this->config.depth_mode, this->config.color_resolution);
			}
			catch (const k4a::error &e)
			{
				ofLogError(__FUNCTION__) << e.what();
				return false;
			}
		}

		if (this->bUpdateColor)
		{
			// Create transformation.
			this->transformation = k4a::transformation(this->calibration);
		}

		if (this->bUpdateBodies)
		{
			// Create Body Tracker
			tracker = BodyTracker(calibration, trackerConfig);
		}

		if (this->bUpdateWorld)
		{
			// Load depth to world LUT.
			this->setupDepthToWorldTable();

			if (this->bUpdateColor)
			{
				// Load color to world LUT.
				this->setupColorToWorldTable();
			}
		}

		// Check compatible sync mode and connection.
		if (this->config.wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER && !this->isSyncOutConnected())
		{
			ofLogWarning(__FUNCTION__) << "Wired sync mode set to Master but Sync Out not connected! Reverting to Standalone.";
			this->config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
		}
		else if (this->config.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE && !this->isSyncInConnected())
		{
			ofLogWarning(__FUNCTION__) << "Wired sync mode set to Subordinate but Sync In not connected! Reverting to Standalone.";
			this->config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
		}

		if (this->config.wired_sync_mode != K4A_WIRED_SYNC_MODE_SUBORDINATE)
		{
			this->config.subordinate_delay_off_master_usec = 0;
		}

		// Start cameras.
		if (bPlayback)
		{
			playback->play();
		}
		else
		{
			try
			{
				this->device.start_cameras(&this->config);
			}
			catch (const k4a::error &e)
			{
				ofLogError(__FUNCTION__) << e.what();
				return false;
			}

			// Can only start the IMU if cameras are enabled
			if (this->bEnableIMU)
			{
				k4a_device_start_imu(device.handle());
			}
		}

		if (!bMultiDeviceSyncCapture) {
			this->startThread();
		}
		else {
			this->waitForThread();
		}

		if (this->config.color_format == K4A_IMAGE_FORMAT_COLOR_MJPG) {
			this->decodeThread.startThread();
		} 
		else {
			this->decodeThread.waitForThread();
		}

		ofAddListener(ofEvents().update, this, &Device::update);

		this->bStreaming = true;

		return true;
	}

	bool Device::stopCameras()
	{
		if (!this->bStreaming)
			return false;

		std::unique_lock<std::mutex> lock(this->mutex);
		if (this->decodeThread.isThreadRunning()) {
			this->decodeThread.waitForThread();
		}
		this->stopThread();
		this->condition.notify_all();

		ofRemoveListener(ofEvents().update, this, &Device::update);

		this->depthToWorldImg.reset();
		this->colorToWorldImg.reset();
		this->transformation.destroy();

		if (this->bUpdateBodies)
		{
			k4abt_tracker_shutdown(this->bodyTracker);
			k4abt_tracker_destroy(this->bodyTracker);
			this->bodyTracker = nullptr;
		}

		if (bPlayback)
		{
			playback->stop();
			playback->close();
		}
		else
		{
			this->device.stop_cameras();
		}

		this->frameBack.reset();
		this->frameSwap.reset();
		this->frameFront.reset();
		this->depthToWorldPix.clear();
		this->colorToWorldPix.clear();
		this->depthTex.clear();
		this->colorTex.clear();
		this->irTex.clear();
		this->depthToWorldTex.clear();
		this->colorToWorldTex.clear();
		this->depthInColorTex.clear();
		this->colorInDepthTex.clear();
		this->bodyIndexTex.clear();
		this->pointCloudVbo.clear();

		this->bStreaming = false;

		return true;
	}

	bool Device::isSyncInConnected() const
	{
		return this->device.is_sync_in_connected();
	}

	bool Device::isSyncOutConnected() const
	{
		return this->device.is_sync_out_connected();
	}

	void Device::threadedFunction()
	{
		while (this->isThreadRunning())
		{
			// During recording, do not wait for render thread, not to drop frames.
			if (!this->bRecording) {
				std::unique_lock<std::mutex> lock(this->mutex);
				while (this->isThreadRunning() && this->texFrameNum != this->pixFrameNum)
				{
					this->condition.wait(lock);
				}
			}
			this->updatePixels();

			if (this->lock()) {
				this->frameBack.swapFrame(this->frameSwap);
				this->bNewBuffer = true;
				this->unlock();
			}

			std::this_thread::sleep_for(std::chrono::microseconds(100));
		}
	}

	void Device::update(ofEventArgs &args)
	{
		this->bNewFrame = false;

		if (this->bNewBuffer)
		{
			if (this->decodeThread.isThreadRunning()) {
				auto ret = this->decodeThread.update(this->frameSwap.colorPix, this->frameSwap.colorPixDeviceTime);
				if (ret) {
					this->frameSwap.bColorPixUpdated = true;
				}
			}
			if (this->lock()) {
				this->frameSwap.swapFrame(this->frameFront);
				this->frameFront.bColorPixUpdated = false;
				this->bNewBuffer = false;
				this->unlock();
			}
			this->updateTextures();
			this->condition.notify_all();
		}
	}

	void Device::updatePixels()
	{
		// Get a capture.
		if (this->bPlayback)
		{
			if (playback->isPlaying())
			{
				capture = k4a::capture(playback->getNextCapture());
				if (bEnableIMU)
				{
					imu_sample = playback->getNextImuSample();
					// printf(" | Accelerometer temperature:%.2f x:%.4f y:%.4f z: %.4f\n",
					// 	   imu_sample.temperature,
					// 	   imu_sample.acc_sample.xyz.x,
					// 	   imu_sample.acc_sample.xyz.y,
					// 	   imu_sample.acc_sample.xyz.z);
				}
			}
			else if (playback->isPaused())
			{
				playback->seek();
				capture = k4a::capture(playback->getNextCapture());
			}
			else
			{
				// if we are stopped, just return
				return;
			}
		}
		else if (this->bMultiDeviceSyncCapture && master_device_capture != nullptr) {
		}
		else
		{
			try
			{
				if (!this->device.get_capture(&this->capture, std::chrono::milliseconds(TIMEOUT_IN_MS)))
				{
					ofLogWarning(__FUNCTION__) << "Timed out waiting for a capture for device " << this->index << "::" << this->serialNumber << ".";
					return;
				}
			}
			catch (const k4a::error &e)
			{
				ofLogError(__FUNCTION__) << e.what();
				return;
			}
		}

		// Probe for a depth16 image.
		auto& f = this->frameBack;
		auto depthImg = this->capture.get_depth_image();
		if (depthImg)
		{
			const auto depthDims = glm::ivec2(depthImg.get_width_pixels(), depthImg.get_height_pixels());
			if (!f.depthPix.isAllocated())
			{
				f.depthPix.allocate(depthDims.x, depthDims.y, 1);
			}

			const auto depthData = reinterpret_cast<uint16_t *>(depthImg.get_buffer());
			f.depthPix.setFromPixels(depthData, depthDims.x, depthDims.y, 1);
			f.depthPixDeviceTime = depthImg.get_device_timestamp();

			ofLogVerbose(__FUNCTION__) << "Capture Depth16 " << depthDims.x << "x" << depthDims.y << " stride: " << depthImg.get_stride_bytes() << ".";
		}
		else
		{
			ofLogWarning(__FUNCTION__) << "No Depth16 capture found!";
		}

		k4a::image colorImg;
		if (this->bUpdateColor)
		{
			// Probe for a color image.
			colorImg = this->capture.get_color_image();
			if (colorImg)
			{
				const auto colorDims = glm::ivec2(colorImg.get_width_pixels(), colorImg.get_height_pixels());
				if (!f.colorPix.isAllocated())
				{
					f.colorPix.allocate(colorDims.x, colorDims.y, OF_PIXELS_BGRA);
				}

				if (this->config.color_format == K4A_IMAGE_FORMAT_COLOR_MJPG)
				{
					// during recording, jpeg decode task is dispatched to another thread, not to drop recording frames.
					if (this->bRecording || this->bAsyncJpegDecode) {
						JpegTask task;
						task.colorPixBuf.set((const char*)colorImg.get_buffer(), colorImg.get_size());
						task.colorPixDeviceTime = colorImg.get_device_timestamp();
						this->decodeThread.pushTaskIfEmpty(task);
						f.bColorPixUpdated = false;
					}
					else {
						const int decompressStatus = tjDecompress2(this->jpegDecompressor,
							colorImg.get_buffer(),
							static_cast<unsigned long>(colorImg.get_size()),
							f.colorPix.getData(),
							colorDims.x,
							0, // pitch
							colorDims.y,
							TJPF_BGRA,
							TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE);
						f.bColorPixUpdated = true;
					}
				}
				else
				{
					const auto colorData = reinterpret_cast<uint8_t *>(colorImg.get_buffer());
					f.colorPix.setFromPixels(colorData, colorDims.x, colorDims.y, 4);
					f.bColorPixUpdated = true;
				}

				if (f.bColorPixUpdated) {
					f.colorPixDeviceTime = colorImg.get_device_timestamp();
				}

				ofLogVerbose(__FUNCTION__) << "Capture Color " << colorDims.x << "x" << colorDims.y << " stride: " << colorImg.get_stride_bytes() << ".";
			}
			else
			{
				ofLogWarning(__FUNCTION__) << "No Color capture found!";
			}
		}

		k4a::image irImg;
		if (this->bUpdateIr)
		{
			// Probe for a IR16 image.
			irImg = this->capture.get_ir_image();
			if (irImg)
			{
				const auto irSize = glm::ivec2(irImg.get_width_pixels(), irImg.get_height_pixels());
				if (!f.irPix.isAllocated())
				{
					f.irPix.allocate(irSize.x, irSize.y, 1);
				}

				const auto irData = reinterpret_cast<uint16_t *>(irImg.get_buffer());
				f.irPix.setFromPixels(irData, irSize.x, irSize.y, 1);

				ofLogVerbose(__FUNCTION__) << "Capture Ir16 " << irSize.x << "x" << irSize.y << " stride: " << irImg.get_stride_bytes() << ".";
			}
			else
			{
				ofLogWarning(__FUNCTION__) << "No Ir16 capture found!";
			}
		}

		if (this->bUpdateBodies)
		{
			tracker.update(capture.handle());
		}

		if (this->bUpdateVbo)
		{
			if (this->bUpdateColor)
			{
				this->updatePointsCache(depthImg, depthToWorldImg); //(colorImg, this->colorToWorldImg);
			}
			else
			{
				this->updatePointsCache(depthImg, depthToWorldImg);
			}
		}

		if (colorImg && this->bUpdateColor && this->config.color_format == K4A_IMAGE_FORMAT_COLOR_BGRA32)
		{
			// TODO: Fix this for non-BGRA formats, maybe always keep a BGRA k4a::image around.
			this->updateDepthInColorFrame(depthImg, colorImg);
			this->updateColorInDepthFrame(depthImg, colorImg);
		}

		// Do any recording before releasing the capture
		if (this->bRecording)
		{
			k4a_capture_t capture_handle = capture.handle();
			recording->record(&capture_handle);
		}

		// Release images.
		depthImg.reset();
		colorImg.reset();
		irImg.reset();

		// Release capture.
		this->capture.reset();

		// Update frame number.
		this->pixFrameNum = ofGetFrameNum();
	}

	void Device::updateTextures()
	{
		auto& f = this->frameFront;
		if (f.depthPix.isAllocated())
		{
			// Update the depth texture.
			if (!this->depthTex.isAllocated())
			{
				this->depthTex.allocate(f.depthPix);
				this->depthTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
			}

			this->depthTex.loadData(f.depthPix);
			ofLogVerbose(__FUNCTION__) << "Update Depth16 " << this->depthTex.getWidth() << "x" << this->depthTex.getHeight() << ".";
		}

		if (this->bUpdateColor && f.colorPix.isAllocated())
		{
			// Update the color texture.
			if (!this->colorTex.isAllocated())
			{
				this->colorTex.allocate(f.colorPix);
				this->colorTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);

				if (this->config.color_format == K4A_IMAGE_FORMAT_COLOR_BGRA32)
				{
					this->colorTex.bind();
					{
						glTexParameteri(this->colorTex.texData.textureTarget, GL_TEXTURE_SWIZZLE_R, GL_BLUE);
						glTexParameteri(this->colorTex.texData.textureTarget, GL_TEXTURE_SWIZZLE_B, GL_RED);
					}
					this->colorTex.unbind();
				}
			}

			this->colorTex.loadData(f.colorPix);
			ofLogVerbose(__FUNCTION__) << "Update Color " << this->colorTex.getWidth() << "x" << this->colorTex.getHeight() << ".";
		}

		if (this->bUpdateIr && f.irPix.isAllocated())
		{
			// Update the IR16 image.
			if (!this->irTex.isAllocated())
			{
				this->irTex.allocate(f.irPix);
				this->irTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
				this->irTex.setRGToRGBASwizzles(true);
			}

			this->irTex.loadData(f.irPix);
			ofLogVerbose(__FUNCTION__) << "Update Ir16 " << this->irTex.getWidth() << "x" << this->irTex.getHeight() << ".";
		}

		if (this->bUpdateBodies && f.bodyIndexPix.isAllocated())
		{
			if (!this->bodyIndexTex.isAllocated())
			{
				this->bodyIndexTex.allocate(f.bodyIndexPix);
				this->bodyIndexTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
			}

			this->bodyIndexTex.loadData(f.bodyIndexPix);
		}

		if (this->bUpdateVbo)
		{
			this->pointCloudVbo.setVertexData(f.positionCache.data(), f.numPoints, GL_STREAM_DRAW);
			this->pointCloudVbo.setTexCoordData(f.uvCache.data(), f.numPoints, GL_STREAM_DRAW);
		}

		if (this->bUpdateColor && this->config.color_format == K4A_IMAGE_FORMAT_COLOR_BGRA32)
		{
			if (f.depthInColorPix.isAllocated())
			{
				if (!this->depthInColorTex.isAllocated())
				{
					this->depthInColorTex.allocate(f.depthInColorPix);
					this->depthInColorTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
				}

				this->depthInColorTex.loadData(f.depthInColorPix);
			}

			if (f.colorInDepthPix.isAllocated())
			{
				if (!this->colorInDepthTex.isAllocated())
				{
					this->colorInDepthTex.allocate(f.colorInDepthPix);
					this->colorInDepthTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
					this->colorInDepthTex.bind();
					{
						glTexParameteri(this->colorInDepthTex.texData.textureTarget, GL_TEXTURE_SWIZZLE_R, GL_BLUE);
						glTexParameteri(this->colorInDepthTex.texData.textureTarget, GL_TEXTURE_SWIZZLE_B, GL_RED);
					}
					this->colorInDepthTex.unbind();
				}

				this->colorInDepthTex.loadData(f.colorInDepthPix);
			}
		}

		// Update frame number.
		this->texFrameNum = this->pixFrameNum;
		this->bNewFrame = true;
	}

	bool Device::setupDepthToWorldTable()
	{
		if (this->setupImageToWorldTable(K4A_CALIBRATION_TYPE_DEPTH, this->depthToWorldImg))
		{
			const int width = this->depthToWorldImg.get_width_pixels();
			const int height = this->depthToWorldImg.get_height_pixels();

			const auto data = reinterpret_cast<float *>(this->depthToWorldImg.get_buffer());

			if (!this->depthToWorldPix.isAllocated())
			{
				this->depthToWorldPix.allocate(width, height, 2);
				this->depthToWorldTex.allocate(width, height, GL_RG32F);
				this->depthToWorldTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
			}

			this->depthToWorldPix.setFromPixels(data, width, height, 2);
			this->depthToWorldTex.loadData(this->depthToWorldPix);

			return true;
		}

		return false;
	}

	bool Device::setupColorToWorldTable()
	{
		if (this->setupImageToWorldTable(K4A_CALIBRATION_TYPE_COLOR, this->colorToWorldImg))
		{
			const int width = this->colorToWorldImg.get_width_pixels();
			const int height = this->colorToWorldImg.get_height_pixels();

			const auto data = reinterpret_cast<float *>(this->colorToWorldImg.get_buffer());

			if (!this->colorToWorldPix.isAllocated())
			{
				this->colorToWorldPix.allocate(width, height, 2);
				this->colorToWorldTex.allocate(width, height, GL_RG32F);
				this->colorToWorldTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
			}

			this->colorToWorldPix.setFromPixels(data, width, height, 2);
			this->colorToWorldTex.loadData(this->colorToWorldPix);

			return true;
		}

		return false;
	}

	bool Device::setupImageToWorldTable(k4a_calibration_type_t type, k4a::image &img)
	{
		const k4a_calibration_camera_t &calibrationCamera = (type == K4A_CALIBRATION_TYPE_DEPTH) ? this->calibration.depth_camera_calibration : this->calibration.color_camera_calibration;

		const auto dims = glm::ivec2(
			calibrationCamera.resolution_width,
			calibrationCamera.resolution_height);

		try
		{
			img = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
									 dims.x, dims.y,
									 dims.x * static_cast<int>(sizeof(k4a_float2_t)));
		}
		catch (const k4a::error &e)
		{
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		auto imgData = reinterpret_cast<k4a_float2_t *>(img.get_buffer());

		k4a_float2_t p;
		k4a_float3_t ray;
		int idx = 0;
		for (int y = 0; y < dims.y; ++y)
		{
			p.xy.y = static_cast<float>(y);

			for (int x = 0; x < dims.x; ++x)
			{
				p.xy.x = static_cast<float>(x);

				if (this->calibration.convert_2d_to_3d(p, 1.f, type, type, &ray))
				{
					imgData[idx].xy.x = ray.xyz.x;
					imgData[idx].xy.y = ray.xyz.y;
				}
				else
				{
					// The pixel is invalid.
					//ofLogNotice(__FUNCTION__) << "Pixel " << depthToWorldData[idx].xy.x << ", " << depthToWorldData[idx].xy.y << " is invalid";
					imgData[idx].xy.x = 0;
					imgData[idx].xy.y = 0;
				}

				++idx;
			}
		}

		return true;
	}

	// Kinect Thread function.
	bool Device::updatePointsCache(k4a::image &frameImg, k4a::image &tableImg)
	{
		const auto frameDims = glm::ivec2(frameImg.get_width_pixels(), frameImg.get_height_pixels());
		const auto tableDims = glm::ivec2(tableImg.get_width_pixels(), tableImg.get_height_pixels());
		if (frameDims != tableDims)
		{
			ofLogError(__FUNCTION__) << "Image dims mismatch! " << frameDims << " vs " << tableDims;
			return false;
		}

		const auto frameData = reinterpret_cast<uint16_t *>(frameImg.get_buffer());
		const auto tableData = reinterpret_cast<k4a_float2_t *>(tableImg.get_buffer());

		auto& f = this->frameBack;
		f.positionCache.resize(frameDims.x * frameDims.y);
		f.uvCache.resize(frameDims.x * frameDims.y);

		int count = 0;
		for (int y = 0; y < frameDims.y; ++y)
		{
			for (int x = 0; x < frameDims.x; ++x)
			{
				int idx = y * frameDims.x + x;
				if (frameData[idx] != 0 &&
					tableData[idx].xy.x != 0 && tableData[idx].xy.y != 0)
				{
					float depthVal = static_cast<float>(frameData[idx]);
					f.positionCache[count] = glm::vec3(
						tableData[idx].xy.x * depthVal,
						tableData[idx].xy.y * depthVal,
						depthVal);

					f.uvCache[count] = glm::vec2(x, y);

					++count;
				}
			}
		}

		f.numPoints = count;

		return true;
	}

	// Kinect Thread function.
	bool Device::updateDepthInColorFrame(const k4a::image &depthImg, const k4a::image &colorImg)
	{
		const auto colorDims = glm::ivec2(colorImg.get_width_pixels(), colorImg.get_height_pixels());

		k4a::image transformedDepthImg;
		try
		{
			transformedDepthImg = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
													 colorDims.x, colorDims.y,
													 colorDims.x * static_cast<int>(sizeof(uint16_t)));

			this->transformation.depth_image_to_color_camera(depthImg, &transformedDepthImg);
		}
		catch (const k4a::error &e)
		{
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		const auto transformedColorData = reinterpret_cast<uint16_t *>(transformedDepthImg.get_buffer());

		auto& f = this->frameBack;
		if (!f.depthInColorPix.isAllocated())
		{
			f.depthInColorPix.allocate(colorDims.x, colorDims.y, 1);
		}

		f.depthInColorPix.setFromPixels(transformedColorData, colorDims.x, colorDims.y, 1);

		ofLogVerbose(__FUNCTION__) << "Depth in Color " << colorDims.x << "x" << colorDims.y << " stride: " << transformedDepthImg.get_stride_bytes() << ".";

		transformedDepthImg.reset();

		return true;
	}

	// Kinect Thread function.
	bool Device::updateColorInDepthFrame(const k4a::image &depthImg, const k4a::image &colorImg)
	{
		const auto depthDims = glm::ivec2(depthImg.get_width_pixels(), depthImg.get_height_pixels());

		k4a::image transformedColorImg;
		try
		{
			transformedColorImg = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
													 depthDims.x, depthDims.y,
													 depthDims.x * 4 * static_cast<int>(sizeof(uint8_t)));

			this->transformation.color_image_to_depth_camera(depthImg, colorImg, &transformedColorImg);
		}
		catch (const k4a::error &e)
		{
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		const auto transformedColorData = reinterpret_cast<uint8_t *>(transformedColorImg.get_buffer());

		auto& f = this->frameBack;
		if (!f.colorInDepthPix.isAllocated())
		{
			f.colorInDepthPix.allocate(depthDims.x, depthDims.y, OF_PIXELS_BGRA);
		}

		f.colorInDepthPix.setFromPixels(transformedColorData, depthDims.x, depthDims.y, 4);

		ofLogVerbose(__FUNCTION__) << "Color in Depth " << depthDims.x << "x" << depthDims.y << " stride: " << transformedColorImg.get_stride_bytes() << ".";

		transformedColorImg.reset();

		return true;
	}

	bool Device::isOpen() const
	{
		return this->bOpen;
	}

	bool Device::isStreaming() const
	{
		return this->bStreaming;
	}

	bool Device::isFrameNew() const
	{
		return this->bNewFrame;
	}

	const std::string &Device::getSerialNumber() const
	{
		return this->serialNumber;
	}

	const ofShortPixels &Device::getDepthPix() const
	{
		return this->frameFront.depthPix;
	}

	const ofTexture &Device::getDepthTex() const
	{
		return this->depthTex;
	}

	const ofPixels &Device::getColorPix() const
	{
		return this->frameFront.colorPix;
	}

	const ofTexture &Device::getColorTex() const
	{
		return this->colorTex;
	}

	const ofShortPixels &Device::getIrPix() const
	{
		return this->frameFront.irPix;
	}

	const ofTexture &Device::getIrTex() const
	{
		return this->irTex;
	}

	const ofFloatPixels &Device::getDepthToWorldPix() const
	{
		return this->depthToWorldPix;
	}

	const ofTexture &Device::getDepthToWorldTex() const
	{
		return this->depthToWorldTex;
	}

	const ofFloatPixels &Device::getColorToWorldPix() const
	{
		return this->colorToWorldPix;
	}

	const ofTexture &Device::getColorToWorldTex() const
	{
		return this->colorToWorldTex;
	}

	const ofShortPixels &Device::getDepthInColorPix() const
	{
		return this->frameFront.depthInColorPix;
	}

	const ofTexture &Device::getDepthInColorTex() const
	{
		return this->depthInColorTex;
	}

	const ofPixels &Device::getColorInDepthPix() const
	{
		return this->frameFront.colorInDepthPix;
	}

	const ofTexture &Device::getColorInDepthTex() const
	{
		return this->colorInDepthTex;
	}

	const ofVbo &Device::getPointCloudVbo() const
	{
		return this->pointCloudVbo;
	}

	int32_t Device::getColorCameraControlValue(k4a_color_control_command_t command) const
	{
		k4a_color_control_mode_t mode;
		int32_t ret;
		this->device.get_color_control(command, &mode, &ret);
		if (mode == K4A_COLOR_CONTROL_MODE_AUTO) {
			return INT_MIN;
		}
		else {
			return ret;
		}
	}

	void Device::setColorCameraControlValue(k4a_color_control_command_t command, int32_t value)
	{
		this->device.set_color_control(command, K4A_COLOR_CONTROL_MODE_MANUAL, value);
	}

	int32_t Device::getExposureTimeAbsolute() const
	{
		return getColorCameraControlValue(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE);
	}

	void Device::setExposureTimeAbsolute(int32_t exposure_usec)
	{
		setColorCameraControlValue(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, exposure_usec);
	}

	void Device::startRecording(std::string filename, float delay)
	{
		if (isRecording()) {
			return;
		}

		recording = new Record();
		recording->setup(device.handle(), this->config, bEnableIMU, delay, filename);
		recording->start();

		bRecording = true;
	}

	void Device::stopRecording()
	{
		if (recording) {
			recording->stop();
			delete recording;
			recording = nullptr;
		}
		bRecording = false;
	}

	bool Device::isRecording() const
	{
		return bRecording;
	}

	float Device::getRecordingTimerDelay()
	{
		if (recording != nullptr)
			return recording->getTimerDelay();
		else
			return -1;
	}

	void Device::listener_playback_play(bool val)
	{
		playback->play();
	}
	void Device::listener_playback_pause(bool val)
	{
		playback->pause();
	}
	void Device::listener_playback_stop(bool val)
	{
		playback->stop();
	}
	void Device::listener_playback_seek(float val)
	{
		playback->seek(val);
	}


	void MultiDeviceSyncCapture::setMasterDevice(Device * p)
	{
		master_device = p;
		master_device->bMultiDeviceSyncCapture = true;
		master_device->master_device_capture = this;
	}
	void MultiDeviceSyncCapture::addSubordinateDevice(Device * p)
	{
		p->bMultiDeviceSyncCapture = true;
		p->master_device_capture = this;
		subordinate_devices.push_back(p);
	}

	void MultiDeviceSyncCapture::start()
	{
		if (master_device == nullptr || subordinate_devices.empty()) {
			cerr << "MultiDeviceSyncCapture::setMasterDevice, addSubordinateDevice must be called before start." << endl;
			return;
		}
		startThread();
	}

	void MultiDeviceSyncCapture::stop()
	{
		waitForThread();
	}

	void MultiDeviceSyncCapture::setMaxAllowableTimeOffsetUsec(uint32_t usec)
	{
		max_allowable_time_offset_error_for_image_timestamp = std::chrono::microseconds(usec);
	}

	void MultiDeviceSyncCapture::threadedFunction()
	{
		while (isThreadRunning()) {
			// Dealing with the synchronized cameras is complex. The Azure Kinect DK:
			//      (a) does not guarantee exactly equal timestamps between depth and color or between cameras (delays can
			//      be configured but timestamps will only be approximately the same)
			//      (b) does not guarantee that, if the two most recent images were synchronized, that calling get_capture
			//      just once on each camera will still be synchronized.
			// There are several reasons for all of this. Internally, devices keep a queue of a few of the captured images
			// and serve those images as requested by get_capture(). However, images can also be dropped at any moment, and
			// one device may have more images ready than another device at a given moment, et cetera.
			//
			// Also, the process of synchronizing is complex. The cameras are not guaranteed to exactly match in all of
			// their timestamps when synchronized (though they should be very close). All delays are relative to the master
			// camera's color camera. To deal with these complexities, we employ a fairly straightforward algorithm. Start
			// by reading in two captures, then if the camera images were not taken at roughly the same time read a new one
			// from the device that had the older capture until the timestamps roughly match.

			// The captures used in the loop are outside of it so that they can persist across loop iterations. This is
			// necessary because each time this loop runs we'll only update the older capture.
			// The captures are stored in a vector where the first element of the vector is the master capture and
			// subsequent elements are subordinate captures
			//std::vector<k4a::capture> captures(subordinate_devices.size() + 1); // add 1 for the master
			auto devices = subordinate_devices;
			devices.push_back(master_device);

			size_t current_index = 0;
			master_device->device.get_capture(&master_device->capture, std::chrono::milliseconds{ K4A_WAIT_INFINITE });
			++current_index;
			for (auto &d : subordinate_devices)
			{
				d->device.get_capture(&d->capture, std::chrono::milliseconds{ K4A_WAIT_INFINITE });
				++current_index;
			}

			bool have_synced_images = false;
			std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
			while (!have_synced_images)
			{
				// Timeout if this is taking too long
				int64_t duration_ms =
					std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count();
				if (duration_ms > WAIT_FOR_SYNCHRONIZED_CAPTURE_TIMEOUT)
				{
					cerr << "ERROR: Timedout waiting for synchronized captures\n";
				}

				k4a::image master_color_image = master_device->capture.get_color_image();
				std::chrono::microseconds master_color_image_time = master_color_image.get_device_timestamp();

				for (size_t i = 0; i < subordinate_devices.size(); ++i)
				{
					k4a::image sub_image;
					if (compare_sub_depth_instead_of_color)
					{
						sub_image = subordinate_devices[i]->capture.get_depth_image();
					}
					else
					{
						sub_image = subordinate_devices[i]->capture.get_color_image();
					}

					if (master_color_image && sub_image)
					{
						std::chrono::microseconds sub_image_time = sub_image.get_device_timestamp();
						// The subordinate's color image timestamp, ideally, is the master's color image timestamp plus the
						// delay we configured between the master device color camera and subordinate device color camera
						std::chrono::microseconds expected_sub_image_time =
							master_color_image_time +
							std::chrono::microseconds{ subordinate_devices[i]->config.subordinate_delay_off_master_usec } +
							std::chrono::microseconds{ subordinate_devices[i]->config.depth_delay_off_color_usec };
						std::chrono::microseconds sub_image_time_error = sub_image_time - expected_sub_image_time;
						// The time error's absolute value must be within the permissible range. So, for example, if
						// MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP is 2, offsets of -2, -1, 0, 1, and -2 are
						// permitted
						if (sub_image_time_error < -max_allowable_time_offset_error_for_image_timestamp)
						{
							// Example, where MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP is 1
							// time                    t=1  t=2  t=3
							// actual timestamp        x    .    .
							// expected timestamp      .    .    x
							// error: 1 - 3 = -2, which is less than the worst-case-allowable offset of -1
							// the subordinate camera image timestamp was earlier than it is allowed to be. This means the
							// subordinate is lagging and we need to update the subordinate to get the subordinate caught up
							log_lagging_time("sub", master_device->capture, subordinate_devices[i]->capture);
							subordinate_devices[i]->device.get_capture(&subordinate_devices[i]->capture,
								std::chrono::milliseconds{ K4A_WAIT_INFINITE });
							break;
						}
						else if (sub_image_time_error > max_allowable_time_offset_error_for_image_timestamp)
						{
							// Example, where MAX_ALLOWABLE_TIME_OFFSET_ERROR_FOR_IMAGE_TIMESTAMP is 1
							// time                    t=1  t=2  t=3
							// actual timestamp        .    .    x
							// expected timestamp      x    .    .
							// error: 3 - 1 = 2, which is more than the worst-case-allowable offset of 1
							// the subordinate camera image timestamp was later than it is allowed to be. This means the
							// subordinate is ahead and we need to update the master to get the master caught up
							log_lagging_time("master", master_device->capture, subordinate_devices[i]->capture);
							master_device->device.get_capture(&master_device->capture, std::chrono::milliseconds{ K4A_WAIT_INFINITE });
							break;
						}
						else
						{
							// These captures are sufficiently synchronized. If we've gotten to the end, then all are
							// synchronized.
							if (i == subordinate_devices.size() - 1)
							{
								log_synced_image_time(master_device->capture, subordinate_devices[i]->capture);
								have_synced_images = true; // now we'll finish the for loop and then exit the while loop
							}
						}
					}
					else if (!master_color_image)
					{
						std::cout << "Master image was bad!\n";
						master_device->device.get_capture(&master_device->capture, std::chrono::milliseconds{ K4A_WAIT_INFINITE });
						break;
					}
					else if (!sub_image)
					{
						std::cout << "Subordinate image was bad!" << endl;
						subordinate_devices[i]->device.get_capture(&subordinate_devices[i]->capture,
							std::chrono::milliseconds{ K4A_WAIT_INFINITE });
						break;
					}
				}
			}
			// if we've made it to here, it means that we have synchronized captures.
			for (auto& device : devices) {
				device->updatePixels();
			}
			for (auto& device : devices) {
				if (device->lock()) {
					device->frameBack.swapFrame(device->frameSwap);
					device->bNewBuffer = true;
					device->unlock();
				}
			}

			std::this_thread::sleep_for(std::chrono::microseconds(100));
		}
	}
} // namespace ofxAzureKinect
