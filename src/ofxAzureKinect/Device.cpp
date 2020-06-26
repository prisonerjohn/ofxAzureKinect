#include "Device.h"

#include "ofLog.h"

const int32_t TIMEOUT_IN_MS = 1000;

namespace ofxAzureKinect
{
	DeviceSettings::DeviceSettings(int idx)
		: deviceIndex(idx), deviceSerial(""), depthMode(K4A_DEPTH_MODE_WFOV_2X2BINNED), colorResolution(K4A_COLOR_RESOLUTION_2160P), colorFormat(K4A_IMAGE_FORMAT_COLOR_BGRA32), cameraFps(K4A_FRAMES_PER_SECOND_30), wiredSyncMode(K4A_WIRED_SYNC_MODE_STANDALONE), subordinateDelayUsec(0), updateColor(true), updateIr(true), updateWorld(true), updateVbo(true), syncImages(true), enableIMU(false)
	{
	}

	BodyTrackingSettings::BodyTrackingSettings()
		: sensorOrientation(K4ABT_SENSOR_ORIENTATION_DEFAULT), processingMode(K4ABT_TRACKER_PROCESSING_MODE_GPU), gpuDeviceID(0), updateBodies(false)
	{
	}

	int Device::getInstalledCount()
	{
		return k4a_device_get_installed_count();
	}

	Device::Device()
		: index(-1), pixFrameNum(0), texFrameNum(0), bOpen(false), bStreaming(false), bUpdateColor(false), bUpdateIr(false), bUpdateBodies(false), bUpdateWorld(false), bUpdateVbo(false), bodyTracker(nullptr), jpegDecompressor(tjInitDecompress())
	{
	}

	Device::~Device()
	{
		this->close();

		tjDestroy(jpegDecompressor);
	}

	bool Device::open(string filename)
	{
		bPlayback = true;
		playback = new Playback();

		if (playback->load_file(filename))
		{
			k4a_record_configuration_t playback_config = playback->get_device_settings();

			this->config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
			this->config.depth_mode = playback_config.depth_mode;
			this->config.color_format = playback_config.color_format;
			this->config.color_resolution = playback_config.color_resolution;
			this->config.camera_fps = playback_config.camera_fps;
			this->enableIMU = playback_config.imu_track_enabled;

			this->serialNumber = playback->get_serial_number();
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

	bool Device::open(string filename, BodyTrackingSettings bodyTrackingSettings)
	{
		bool result = open(filename);
		
		if (result)
		{
			this->bUpdateBodies = bodyTrackingSettings.updateBodies;

			// Setup Body Tracking Config
			this->trackerConfig.sensor_orientation = bodyTrackingSettings.sensorOrientation;
			this->trackerConfig.gpu_device_id = bodyTrackingSettings.gpuDeviceID;

			// Setup Device Calibration
			auto calibration_handle = playback->get_calibration();
			this->calibration.depth_camera_calibration = calibration_handle.depth_camera_calibration;
			this->calibration.color_camera_calibration = calibration_handle.color_camera_calibration;
			this->calibration.depth_mode = calibration_handle.depth_mode;
			this->calibration.color_resolution = calibration_handle.color_resolution;
			for (int i = 0; i < 4; i++)
			{
				for (int j = 0; j < 4; j++)
					this->calibration.extrinsics[i][j] = calibration_handle.extrinsics[i][j];
			}

			// Create Body Tracker
			tracker = BodyTracker(calibration, trackerConfig);

		}

		return result;
	}

	bool Device::open(int idx)
	{
		return this->open(DeviceSettings(idx), BodyTrackingSettings());
	}

	bool Device::open(DeviceSettings deviceSettings)
	{
		return this->open(deviceSettings, BodyTrackingSettings());
	}

	bool Device::open(DeviceSettings deviceSettings, BodyTrackingSettings bodyTrackingSettings)
	{
		this->config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		this->config.depth_mode = deviceSettings.depthMode;
		this->config.color_format = deviceSettings.colorFormat;
		this->config.color_resolution = deviceSettings.colorResolution;
		this->config.camera_fps = deviceSettings.cameraFps;
		this->config.synchronized_images_only = deviceSettings.syncImages;
		this->enableIMU = deviceSettings.enableIMU;

		this->config.wired_sync_mode = deviceSettings.wiredSyncMode;
		this->config.subordinate_delay_off_master_usec = deviceSettings.subordinateDelayUsec;

		this->trackerConfig.sensor_orientation = bodyTrackingSettings.sensorOrientation;
		this->trackerConfig.gpu_device_id = bodyTrackingSettings.gpuDeviceID;

		if (this->bOpen)
		{
			ofLogWarning(__FUNCTION__) << "Device " << this->index << " already open!";
			return false;
		}

		if (deviceSettings.deviceSerial.empty())
		{
			// Simply load the device at the requested index.
			try
			{
				// Open connection to the device.
				this->device = k4a::device::open(static_cast<uint32_t>(deviceSettings.deviceIndex));

				// Get the device serial number.
				this->serialNumber = this->device.get_serialnum();
			}
			catch (const k4a::error &e)
			{
				ofLogError(__FUNCTION__) << e.what();

				this->device.close();

				return false;
			}
		}
		else
		{
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
					if (this->serialNumber == deviceSettings.deviceSerial)
					{
						deviceFound = true;
						deviceSettings.deviceIndex = i;
						break;
					}
					else
					{
						this->device.close();
					}
				}
				catch (const k4a::error &e)
				{
					// Don't worry about it; we just might be trying to access an already open device.
					continue;
				}
			}

			if (!deviceFound)
			{
				ofLogError(__FUNCTION__) << "No device found with serial number " << deviceSettings.deviceSerial;
				return false;
			}
		}

		this->index = deviceSettings.deviceIndex;
		this->bOpen = true;

		this->bUpdateColor = deviceSettings.updateColor;
		this->bUpdateIr = deviceSettings.updateIr;
		this->bUpdateWorld = deviceSettings.updateWorld;
		this->bUpdateVbo = deviceSettings.updateWorld && deviceSettings.updateVbo;

		this->bUpdateBodies = bodyTrackingSettings.updateBodies;

		if (bUpdateBodies){

			try
			{
				this->calibration = this->device.get_calibration(this->config.depth_mode, this->config.color_resolution);
			}
			catch (const k4a::error &e)
			{
				ofLogError(__FUNCTION__) << e.what();
				return false;
			}

			// Create Body Tracker
			tracker = BodyTracker(calibration, trackerConfig);
		}

		// Add Recording Listener
		this->eventListeners.push(this->bRecord.newListener([this](bool) {
			handle_recording(this->bRecord);
		}));

		ofLogNotice(__FUNCTION__) << "Successfully opened device " << this->index << " with serial number " << this->serialNumber << ".";

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
			if (this->enableIMU)
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

	bool Device::startCameras()
	{
		if (!this->bOpen)
		{
			ofLogError(__FUNCTION__) << "Open device before starting cameras!";
			return false;
		}

		// Get calibration.
		if (bPlayback)
		{
			auto calibration_handle = playback->get_calibration();
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
			// Create tracker.
			k4abt_tracker_create(&this->calibration, this->trackerConfig, &this->bodyTracker);
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
			if (this->enableIMU)
			{
				k4a_device_start_imu(device.handle());
			}
		}

		this->startThread();
		ofAddListener(ofEvents().update, this, &Device::update);

		this->bStreaming = true;

		return true;
	}

	bool Device::stopCameras()
	{
		if (!this->bStreaming)
			return false;

		std::unique_lock<std::mutex> lock(this->mutex);
		this->stopThread();
		this->condition.notify_all();

		ofRemoveListener(ofEvents().update, this, &Device::update);

		this->depthToWorldImg.reset();
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
			std::unique_lock<std::mutex> lock(this->mutex);

			while (this->isThreadRunning() && this->texFrameNum != this->pixFrameNum)
			{
				this->condition.wait(lock);
			}

			this->updatePixels();
		}
	}

	void Device::update(ofEventArgs &args)
	{
		this->bNewFrame = false;

		if (this->texFrameNum != this->pixFrameNum)
		{
			std::unique_lock<std::mutex> lock(this->mutex);

			this->updateTextures();

			this->condition.notify_all();
		}
	}

	void Device::updatePixels()
	{
		// Get a capture.
		if (bPlayback)
		{
			if (playback->is_playing())
			{
				capture = k4a::capture(playback->get_next_capture());
				if (enableIMU)
				{
					imu_sample = playback->get_next_imu_sample();
					// printf(" | Accelerometer temperature:%.2f x:%.4f y:%.4f z: %.4f\n",
					// 	   imu_sample.temperature,
					// 	   imu_sample.acc_sample.xyz.x,
					// 	   imu_sample.acc_sample.xyz.y,
					// 	   imu_sample.acc_sample.xyz.z);
				}
			}
			else if (playback->is_paused())
			{
				playback->seek();
				capture = k4a::capture(playback->get_next_capture());
			}
			else
			{
				// if we are stopped, just return
				return;
			}
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
		auto depthImg = this->capture.get_depth_image();
		if (depthImg)
		{
			const auto depthDims = glm::ivec2(depthImg.get_width_pixels(), depthImg.get_height_pixels());
			if (!depthPix.isAllocated())
			{
				this->depthPix.allocate(depthDims.x, depthDims.y, 1);
			}

			const auto depthData = reinterpret_cast<uint16_t *>(depthImg.get_buffer());
			this->depthPix.setFromPixels(depthData, depthDims.x, depthDims.y, 1);

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
				if (!colorPix.isAllocated())
				{
					this->colorPix.allocate(colorDims.x, colorDims.y, OF_PIXELS_BGRA);
				}

				if (this->config.color_format == K4A_IMAGE_FORMAT_COLOR_MJPG)
				{
					const int decompressStatus = tjDecompress2(this->jpegDecompressor,
															   colorImg.get_buffer(),
															   static_cast<unsigned long>(colorImg.get_size()),
															   this->colorPix.getData(),
															   colorDims.x,
															   0, // pitch
															   colorDims.y,
															   TJPF_BGRA,
															   TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE);
				}
				else
				{
					const auto colorData = reinterpret_cast<uint8_t *>(colorImg.get_buffer());
					this->colorPix.setFromPixels(colorData, colorDims.x, colorDims.y, 4);
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
				if (!this->irPix.isAllocated())
				{
					this->irPix.allocate(irSize.x, irSize.y, 1);
				}

				const auto irData = reinterpret_cast<uint16_t *>(irImg.get_buffer());
				this->irPix.setFromPixels(irData, irSize.x, irSize.y, 1);

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
				this->updatePointsCache(depthImg, this->depthToWorldImg); //(colorImg, this->colorToWorldImg);
			}
			else
			{
				this->updatePointsCache(depthImg, this->depthToWorldImg);
			}
		}

		if (colorImg && this->bUpdateColor && this->config.color_format == K4A_IMAGE_FORMAT_COLOR_BGRA32)
		{
			// TODO: Fix this for non-BGRA formats, maybe always keep a BGRA k4a::image around.
			this->updateDepthInColorFrame(depthImg, colorImg);
			this->updateColorInDepthFrame(depthImg, colorImg);
		}

		// Do any recording before releasing the capture
		if (bRecord)
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
		// Update the depth texture.
		if (!this->depthTex.isAllocated())
		{
			this->depthTex.allocate(this->depthPix);
			this->depthTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
		}

		this->depthTex.loadData(this->depthPix);
		ofLogVerbose(__FUNCTION__) << "Update Depth16 " << this->depthTex.getWidth() << "x" << this->depthTex.getHeight() << ".";

		if (this->bUpdateColor)
		{
			// Update the color texture.
			if (!this->colorTex.isAllocated())
			{
				this->colorTex.allocate(this->colorPix);
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

			this->colorTex.loadData(this->colorPix);

			ofLogVerbose(__FUNCTION__) << "Update Color " << this->colorTex.getWidth() << "x" << this->colorTex.getHeight() << ".";
		}

		if (this->bUpdateIr)
		{
			// Update the IR16 image.
			if (!this->irTex.isAllocated())
			{
				this->irTex.allocate(this->irPix);
				this->irTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
				this->irTex.setRGToRGBASwizzles(true);
			}

			this->irTex.loadData(this->irPix);

			ofLogVerbose(__FUNCTION__) << "Update Ir16 " << this->irTex.getWidth() << "x" << this->irTex.getHeight() << ".";
		}

		if (this->bUpdateBodies)
		{
			tracker.update_texture();
		}

		if (this->bUpdateVbo)
		{
			this->pointCloudVbo.setVertexData(this->positionCache.data(), this->numPoints, GL_STREAM_DRAW);
			this->pointCloudVbo.setTexCoordData(this->uvCache.data(), this->numPoints, GL_STREAM_DRAW);
		}

		if (this->bUpdateColor && this->config.color_format == K4A_IMAGE_FORMAT_COLOR_BGRA32)
		{
			if (this->depthInColorPix.isAllocated() && !this->depthInColorTex.isAllocated())
			{
				this->depthInColorTex.allocate(this->depthInColorPix);
				this->depthInColorTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
			}

			this->depthInColorTex.loadData(this->depthInColorPix);

			if (this->colorInDepthPix.isAllocated() && !this->colorInDepthTex.isAllocated())
			{
				this->colorInDepthTex.allocate(this->colorInDepthPix);
				this->colorInDepthTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
				this->colorInDepthTex.bind();
				{
					glTexParameteri(this->colorInDepthTex.texData.textureTarget, GL_TEXTURE_SWIZZLE_R, GL_BLUE);
					glTexParameteri(this->colorInDepthTex.texData.textureTarget, GL_TEXTURE_SWIZZLE_B, GL_RED);
				}
				this->colorInDepthTex.unbind();
			}

			this->colorInDepthTex.loadData(this->colorInDepthPix);
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

		this->positionCache.resize(frameDims.x * frameDims.y);
		this->uvCache.resize(frameDims.x * frameDims.y);

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
					this->positionCache[count] = glm::vec3(
						tableData[idx].xy.x * depthVal,
						tableData[idx].xy.y * depthVal,
						depthVal);

					this->uvCache[count] = glm::vec2(x, y);

					++count;
				}
			}
		}

		this->numPoints = count;

		return true;
	}

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

		if (!this->depthInColorPix.isAllocated())
		{
			this->depthInColorPix.allocate(colorDims.x, colorDims.y, 1);
		}

		this->depthInColorPix.setFromPixels(transformedColorData, colorDims.x, colorDims.y, 1);

		ofLogVerbose(__FUNCTION__) << "Depth in Color " << colorDims.x << "x" << colorDims.y << " stride: " << transformedDepthImg.get_stride_bytes() << ".";

		transformedDepthImg.reset();

		return true;
	}

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

		if (!this->colorInDepthPix.isAllocated())
		{
			this->colorInDepthPix.allocate(depthDims.x, depthDims.y, OF_PIXELS_BGRA);
		}

		this->colorInDepthPix.setFromPixels(transformedColorData, depthDims.x, depthDims.y, 4);

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
		return this->depthPix;
	}

	const ofTexture &Device::getDepthTex() const
	{
		return this->depthTex;
	}

	const ofPixels &Device::getColorPix() const
	{
		return this->colorPix;
	}

	const ofTexture &Device::getColorTex() const
	{
		return this->colorTex;
	}

	const ofShortPixels &Device::getIrPix() const
	{
		return this->irPix;
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
		return this->depthInColorPix;
	}

	const ofTexture &Device::getDepthInColorTex() const
	{
		return this->depthInColorTex;
	}

	const ofPixels &Device::getColorInDepthPix() const
	{
		return this->colorInDepthPix;
	}

	const ofTexture &Device::getColorInDepthTex() const
	{
		return this->colorInDepthTex;
	}

	const ofVbo &Device::getPointCloudVbo() const
	{
		return this->pointCloudVbo;
	}

	void Device::handle_recording(bool val)
	{
		if (val)
		{
			recording = new Record();
			recording->setup(device.handle(), this->config, enableIMU);
			recording->start();
		}
		else
		{
			recording->stop();
		}
	}

	float Device::get_recording_timer_delay()
	{
		if (recording != nullptr)
			return recording->get_timer_delay();
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

} // namespace ofxAzureKinect
