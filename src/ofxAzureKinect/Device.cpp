#include "Device.h"

#include "ofLog.h"

const int32_t TIMEOUT_IN_MS = 1000;

namespace ofxAzureKinect
{
	DeviceSettings::DeviceSettings(int idx)
		: deviceIndex(idx)
		, depthMode(K4A_DEPTH_MODE_WFOV_2X2BINNED)
		, colorResolution(K4A_COLOR_RESOLUTION_2160P)
		, colorFormat(K4A_IMAGE_FORMAT_COLOR_BGRA32)
		, cameraFps(K4A_FRAMES_PER_SECOND_30)
		, sensorOrientation(K4ABT_SENSOR_ORIENTATION_DEFAULT)
		, updateColor(true)
		, updateIr(true)
		, updateBodies(false)
		, updateWorld(true)
		, updateVbo(true)
		, synchronized(true)
	{}

	int Device::getInstalledCount()
	{
		return k4a_device_get_installed_count();
	}

	Device::Device()
		: index(-1)
		, bOpen(false)
		, bStreaming(false)
		, bUpdateColor(false)
		, bUpdateIr(false)
		, bUpdateBodies(false)
		, bUpdateWorld(false)
		, bUpdateVbo(false)
		, bodyTracker(nullptr)
		, jpegDecompressor(tjInitDecompress())
	{}

	Device::~Device()
	{
		close();

		tjDestroy(jpegDecompressor);
	}

	bool Device::open(int idx)
	{
		return open(DeviceSettings(idx));
	}

	bool Device::open(DeviceSettings settings)
	{
		this->config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		this->config.depth_mode = settings.depthMode;
		this->config.color_format = settings.colorFormat;
		this->config.color_resolution = settings.colorResolution;
		this->config.camera_fps = settings.cameraFps;
		this->config.synchronized_images_only = settings.synchronized;

		this->trackerConfig.sensor_orientation = settings.sensorOrientation;

		if (this->bOpen)
		{
			ofLogWarning(__FUNCTION__) << "Device " << this->index << " already open!";
			return false;
		}

		try
		{
			// Open connection to the device.
			this->device = k4a::device::open(static_cast<uint32_t>(settings.deviceIndex));

			// Get the device serial number.
			this->serialNumber = this->device.get_serialnum();


		}
		catch (const k4a::error& e)
		{
			ofLogError(__FUNCTION__) << e.what();
			
			this->device.close();

			return false;
		}

		this->index = settings.deviceIndex;
		this->bOpen = true;

		this->bUpdateColor = settings.updateColor;
		this->bUpdateIr = settings.updateIr;
		this->bUpdateBodies = settings.updateBodies;
		this->bUpdateWorld = settings.updateWorld;
		this->bUpdateVbo = settings.updateWorld && settings.updateVbo;

		ofLogNotice(__FUNCTION__) << "Successfully opened device " << this->index << " with serial number " << this->serialNumber << ".";

		return true;
	}

	bool Device::close()
	{
		if (!this->bOpen) return false;

		this->stopCameras();

		this->device.close();

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
		try
		{
			this->calibration = this->device.get_calibration(this->config.depth_mode, this->config.color_resolution);
		}
		catch (const k4a::error& e)
		{
			ofLogError(__FUNCTION__) << e.what();
			return false;
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

		// Start cameras.
		try
		{
			this->device.start_cameras(&this->config);
		}
		catch (const k4a::error& e)
		{
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		ofAddListener(ofEvents().update, this, &Device::updateCameras);

		this->bStreaming = true;

		return true;
	}

	bool Device::stopCameras()
	{
		if (!this->bStreaming) return false;

		ofRemoveListener(ofEvents().update, this, &Device::updateCameras);

		this->depthToWorldImg.reset();
		this->transformation.destroy();

		if (this->bUpdateBodies)
		{
			k4abt_tracker_shutdown(this->bodyTracker);
			k4abt_tracker_destroy(this->bodyTracker);
			this->bodyTracker = nullptr;
		}

		this->device.stop_cameras();

		this->bStreaming = false;

		return true;
	}

	void Device::updateCameras(ofEventArgs& args)
	{
		// Get a capture.
		try
		{ 
			if (!this->device.get_capture(&this->capture, std::chrono::milliseconds(TIMEOUT_IN_MS)))
			{
				ofLogWarning(__FUNCTION__) << "Timed out waiting for a capture for device " << this->index << ".";
				return;
			}
		}
		catch (const k4a::error& e)
		{
			ofLogError(__FUNCTION__) << e.what();
			return;
		}

		// Probe for a depth16 image.
		auto depthImg = this->capture.get_depth_image();
		if (depthImg)
		{
			const auto depthDims = glm::ivec2(depthImg.get_width_pixels(), depthImg.get_height_pixels());
			if (!depthPix.isAllocated())
			{
				this->depthPix.allocate(depthDims.x, depthDims.y, 1);
				this->depthTex.allocate(depthDims.x, depthDims.y, GL_R16);
				this->depthTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
			}

			const auto depthData = reinterpret_cast<uint16_t*>(depthImg.get_buffer());
			this->depthPix.setFromPixels(depthData, depthDims.x, depthDims.y, 1);
			this->depthTex.loadData(this->depthPix);

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
					this->colorTex.allocate(colorDims.x, colorDims.y, GL_RGBA8, ofGetUsingArbTex(), GL_BGRA, GL_UNSIGNED_BYTE);
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
					const auto colorData = reinterpret_cast<uint8_t*>(colorImg.get_buffer());
					this->colorPix.setFromPixels(colorData, colorDims.x, colorDims.y, 4);
				}

				this->colorTex.loadData(this->colorPix);

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
					this->irTex.allocate(irSize.x, irSize.y, GL_R16);
					this->irTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
					this->irTex.setRGToRGBASwizzles(true);
				}

				const auto irData = reinterpret_cast<uint16_t*>(irImg.get_buffer());
				this->irPix.setFromPixels(irData, irSize.x, irSize.y, 1);
				this->irTex.loadData(this->irPix);

				ofLogVerbose(__FUNCTION__) << "Capture Ir16 " << irSize.x << "x" << irSize.y << " stride: " << irImg.get_stride_bytes() << ".";
			}
			else
			{
				ofLogWarning(__FUNCTION__) << "No Ir16 capture found!";
			}
		}

		if (this->bUpdateBodies)
		{
			k4a_wait_result_t enqueueResult = k4abt_tracker_enqueue_capture(this->bodyTracker, this->capture.handle(), K4A_WAIT_INFINITE);
			if (enqueueResult == K4A_WAIT_RESULT_FAILED)
			{
				ofLogError(__FUNCTION__) << "Failed adding capture to tracker process queue!";
			}
			else
			{
				k4abt_frame_t bodyFrame = nullptr;
				k4a_wait_result_t popResult = k4abt_tracker_pop_result(this->bodyTracker, &bodyFrame, K4A_WAIT_INFINITE);
				if (popResult == K4A_WAIT_RESULT_SUCCEEDED)
				{
					// Probe for a body index map image.
					k4a::image bodyIndexImg = k4abt_frame_get_body_index_map(bodyFrame);
					const auto bodyIndexSize = glm::ivec2(bodyIndexImg.get_width_pixels(), bodyIndexImg.get_height_pixels());
					if (!this->bodyIndexPix.isAllocated())
					{
						this->bodyIndexPix.allocate(bodyIndexSize.x, bodyIndexSize.y, 1);
						this->bodyIndexPix.allocate(bodyIndexSize.x, bodyIndexSize.y, GL_R);
						this->bodyIndexTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
						this->bodyIndexTex.setRGToRGBASwizzles(true);
					}

					const auto bodyIndexData = reinterpret_cast<uint8_t*>(bodyIndexImg.get_buffer());
					this->bodyIndexPix.setFromPixels(bodyIndexData, bodyIndexSize.x, bodyIndexSize.y, 1);
					this->bodyIndexTex.loadData(this->bodyIndexPix);

					ofLogVerbose(__FUNCTION__) << "Capture BodyIndex " << bodyIndexSize.x << "x" << bodyIndexSize.y << " stride: " << bodyIndexImg.get_stride_bytes() << ".";
					bodyIndexImg.reset();

					size_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
					ofLogVerbose(__FUNCTION__) << numBodies << " bodies found!";

					this->bodySkeletons.resize(numBodies);
					this->bodyIDs.resize(numBodies);
					for (size_t i = 0; i < numBodies; i++)
					{
						k4abt_skeleton_t skeleton;
						k4abt_frame_get_body_skeleton(bodyFrame, i, &skeleton);
						this->bodySkeletons[i] = skeleton;
						uint32_t id = k4abt_frame_get_body_id(bodyFrame, i);
						this->bodyIDs[i] = id;
					}

					// Release body frame once we're finished.
					k4abt_frame_release(bodyFrame);
				}
			}
		}

		if (this->bUpdateVbo)
		{
			if (this->bUpdateColor)
			{
				this->updateWorldVbo(depthImg, this->colorToWorldImg);
			}
			else
			{
				this->updateWorldVbo(depthImg, this->depthToWorldImg);
			}
		}

		if (colorImg && this->bUpdateColor && this->config.color_format == K4A_IMAGE_FORMAT_COLOR_BGRA32)
		{
			// TODO: Fix this for non-BGRA formats, maybe always keep a BGRA k4a::image around.
			this->updateDepthInColorFrame(depthImg, colorImg);
			this->updateColorInDepthFrame(depthImg, colorImg);
		}

		// Release images.
		depthImg.reset();
		colorImg.reset();
		irImg.reset();

		// Release capture.
		this->capture.reset();
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
		if (this->setupImageToWorldTable(K4A_CALIBRATION_TYPE_DEPTH, this->colorToWorldImg))
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

	bool Device::setupImageToWorldTable(k4a_calibration_type_t type, k4a::image& img)
	{
		const k4a_calibration_camera_t& calibrationCamera = (type == K4A_CALIBRATION_TYPE_DEPTH) ? this->calibration.depth_camera_calibration : this->calibration.color_camera_calibration;

		const auto dims = glm::ivec2(
			calibrationCamera.resolution_width,
			calibrationCamera.resolution_height);

		try
		{
			img = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
				dims.x, dims.y,
				dims.x * static_cast<int>(sizeof(k4a_float2_t)));
		}
		catch (const k4a::error& e)
		{
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		auto imgData = reinterpret_cast<k4a_float2_t*>(img.get_buffer());

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

	bool Device::updateWorldVbo(k4a::image& frameImg, k4a::image& tableImg)
	{
		const auto frameDims = glm::ivec2(frameImg.get_width_pixels(), frameImg.get_height_pixels());
		const auto tableDims = glm::ivec2(tableImg.get_width_pixels(), tableImg.get_height_pixels());
		if (frameDims != tableDims)
		{
			ofLogError(__FUNCTION__) << "Image dims mismatch! " << frameDims << " vs " << tableDims;
			return false;
		}

		const auto frameData = reinterpret_cast<uint16_t*>(frameImg.get_buffer());
		const auto tableData = reinterpret_cast<k4a_float2_t*>(tableImg.get_buffer());

		this->positionCache.resize(frameDims.x * frameDims.y);
		this->uvCache.resize(frameDims.x * frameDims.y);

		int numPoints = 0;
		for (int y = 0; y < frameDims.y; ++y)
		{
			for (int x = 0; x < frameDims.x; ++x)
			{
				int idx = y * frameDims.x + x;
				if (frameData[idx] != 0 &&
					tableData[idx].xy.x != 0 && tableData[idx].xy.y != 0)
				{
					uint16_t depthVal = (frameData[idx]);
					this->positionCache[numPoints] = glm::vec3(
						tableData[idx].xy.x * depthVal,
						tableData[idx].xy.y * depthVal,
						depthVal
					);

					this->uvCache[numPoints] = glm::vec2(x, y);

					++numPoints;
				}
			}
		}

		this->pointCloudVbo.setVertexData(this->positionCache.data(), numPoints, GL_STREAM_DRAW);
		this->pointCloudVbo.setTexCoordData(this->uvCache.data(), numPoints, GL_STREAM_DRAW);

		return true;
	}

	bool Device::updateDepthInColorFrame(const k4a::image& depthImg, const k4a::image& colorImg)
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
		catch (const k4a::error& e)
		{
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		const auto transformedColorData = reinterpret_cast<uint16_t*>(transformedDepthImg.get_buffer());

		if (!this->depthInColorPix.isAllocated())
		{
			this->depthInColorPix.allocate(colorDims.x, colorDims.y, 1);
			this->depthInColorTex.allocate(colorDims.x, colorDims.y, GL_R16);
			this->depthInColorTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
		}

		this->depthInColorPix.setFromPixels(transformedColorData, colorDims.x, colorDims.y, 1);
		this->depthInColorTex.loadData(this->depthInColorPix);

		ofLogVerbose(__FUNCTION__) << "Depth in Color " << colorDims.x << "x" << colorDims.y << " stride: " << transformedDepthImg.get_stride_bytes() << ".";

		transformedDepthImg.reset();

		return true;
	}

	bool Device::updateColorInDepthFrame(const k4a::image& depthImg, const k4a::image& colorImg)
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
		catch (const k4a::error& e)
		{
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		const auto transformedColorData = reinterpret_cast<uint8_t*>(transformedColorImg.get_buffer());

		if (!this->colorInDepthPix.isAllocated())
		{
			this->colorInDepthPix.allocate(depthDims.x, depthDims.y, OF_PIXELS_BGRA);
			this->colorInDepthTex.allocate(depthDims.x, depthDims.y, GL_RGBA8, ofGetUsingArbTex(), GL_BGRA, GL_UNSIGNED_BYTE);
			this->colorInDepthTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
			this->colorInDepthTex.bind();
			{
				glTexParameteri(this->colorInDepthTex.texData.textureTarget, GL_TEXTURE_SWIZZLE_R, GL_BLUE);
				glTexParameteri(this->colorInDepthTex.texData.textureTarget, GL_TEXTURE_SWIZZLE_B, GL_RED);
			}
			this->colorInDepthTex.unbind();
		}

		this->colorInDepthPix.setFromPixels(transformedColorData, depthDims.x, depthDims.y, 4);
		this->colorInDepthTex.loadData(this->colorInDepthPix);

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

	const ofShortPixels& Device::getDepthPix() const
	{
		return this->depthPix;
	}

	const ofTexture& Device::getDepthTex() const
	{
		return this->depthTex;
	}

	const ofPixels& Device::getColorPix() const
	{
		return this->colorPix;
	}

	const ofTexture& Device::getColorTex() const
	{
		return this->colorTex;
	}

	const ofShortPixels& Device::getIrPix() const
	{
		return this->irPix;
	}

	const ofTexture& Device::getIrTex() const
	{
		return this->irTex;
	}

	const ofFloatPixels& Device::getDepthToWorldPix() const
	{
		return this->depthToWorldPix;
	}

	const ofTexture& Device::getDepthToWorldTex() const
	{
		return this->depthToWorldTex;
	}

	const ofFloatPixels& Device::getColorToWorldPix() const
	{
		return this->colorToWorldPix;
	}

	const ofTexture& Device::getColorToWorldTex() const
	{
		return this->colorToWorldTex;
	}

	const ofShortPixels& Device::getDepthInColorPix() const
	{
		return this->depthInColorPix;
	}

	const ofTexture& Device::getDepthInColorTex() const
	{
		return this->depthInColorTex;
	}

	const ofPixels& Device::getColorInDepthPix() const
	{
		return this->colorInDepthPix;
	}

	const ofTexture& Device::getColorInDepthTex() const
	{
		return this->colorInDepthTex;
	}

	const ofPixels& Device::getBodyIndexPix() const
	{
		return this->bodyIndexPix;
	}

	const ofTexture& Device::getBodyIndexTex() const
	{
		return this->bodyIndexTex;
	}

	size_t Device::getNumBodies() const
	{
		return this->bodySkeletons.size();
	}

	const std::vector<k4abt_skeleton_t>& Device::getBodySkeletons() const
	{
		return this->bodySkeletons;
	}

	const std::vector<uint32_t>& Device::getBodyIDs() const
	{
		return this->bodyIDs;
	}

	const ofVbo& Device::getPointCloudVbo() const
	{
		return this->pointCloudVbo;
	}
}
