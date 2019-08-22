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
		, updateColor(true)
		, updateIr(true)
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
		, bUpdateWorld(false)
		, bUpdateVbo(false)
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

		if (this->bUpdateWorld)
		{
			// Load depth to world LUT.
			this->setupDepthToWorldFrame();
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

		this->device.stop_cameras();

		this->bStreaming = false;

		return true;
	}

	void Device::updateCameras(ofEventArgs& args)
	{
		// Get a depth frame.
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

		if (this->bUpdateVbo)
		{
			this->updateDepthToWorldVbo(depthImg);
		}

		if (colorImg && this->bUpdateColor && this->config.color_format == K4A_IMAGE_FORMAT_COLOR_BGRA32)
		{
			// TODO: Fix this for non-BGRA formats, maybe always keep a BGRA k4a::image around.
			this->updateColorInDepthFrame(depthImg, colorImg);
		}

		// Release images.
		depthImg.reset();
		colorImg.reset();
		irImg.reset();

		// Release capture.
		this->capture.reset();
	}

	bool Device::setupDepthToWorldFrame()
	{
		const k4a_calibration_camera_t& calibrationCamera = this->calibration.depth_camera_calibration;
		
		const auto depthDims = glm::ivec2(
			calibrationCamera.resolution_width,
			calibrationCamera.resolution_height);

		try
		{
			this->depthToWorldImg = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
				depthDims.x, depthDims.y,
				depthDims.x * static_cast<int>(sizeof(k4a_float2_t)));
		}
		catch (const k4a::error& e)
		{
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		auto depthToWorldData = reinterpret_cast<k4a_float2_t*>(this->depthToWorldImg.get_buffer());
		
		k4a_float2_t p;
		k4a_float3_t ray;
		int idx = 0;
		for (int y = 0; y < depthDims.y; ++y)
		{
			p.xy.y = static_cast<float>(y);

			for (int x = 0; x < depthDims.x; ++x)
			{
				p.xy.x = static_cast<float>(x);

				if (this->calibration.convert_2d_to_3d(p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray))
				{
					depthToWorldData[idx].xy.x = ray.xyz.x;
					depthToWorldData[idx].xy.y = ray.xyz.y;
				}
				else
				{
					// The pixel is invalid.
					depthToWorldData[idx].xy.x = 0;
					depthToWorldData[idx].xy.y = 0;
				}

				++idx;
			}
		}

		if (!this->depthToWorldPix.isAllocated())
		{
			this->depthToWorldPix.allocate(depthDims.x, depthDims.y, 2);
			this->depthToWorldTex.allocate(depthDims.x, depthDims.y, GL_RG32F);
			this->depthToWorldTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
		}

		this->depthToWorldPix.setFromPixels((float *)depthToWorldData, depthDims.x, depthDims.y, 2);
		this->depthToWorldTex.loadData(this->depthToWorldPix);

		return true;
	}

	bool Device::updateDepthToWorldVbo(k4a::image& depthImg)
	{
		const auto depthDims = glm::ivec2(depthImg.get_width_pixels(), depthImg.get_height_pixels());

		const auto depthData = reinterpret_cast<uint16_t*>(depthImg.get_buffer());
		const auto depthToWorldData = reinterpret_cast<k4a_float2_t*>(this->depthToWorldImg.get_buffer());

		this->positionCache.resize(depthDims.x * depthDims.y);
		this->uvCache.resize(depthDims.x * depthDims.y);

		int numPoints = 0;
		for (int y = 0; y < depthDims.y; ++y)
		{
			for (int x = 0; x < depthDims.x; ++x)
			{
				int idx = y * depthDims.x + x;
				if (depthData[idx] != 0 && 
					depthToWorldData[idx].xy.x != 0 && depthToWorldData[idx].xy.y != 0)
				{
					this->positionCache[numPoints] = glm::vec3(
						depthToWorldData[idx].xy.x * static_cast<float>(depthData[idx]),
						depthToWorldData[idx].xy.y * static_cast<float>(depthData[idx]),
						static_cast<float>(depthData[idx])
					);

					this->uvCache[numPoints] = glm::vec2(x, y);

					++numPoints;
				}
			}
		}

		this->pointCloudVbo.setVertexData(this->positionCache.data(), numPoints, GL_DYNAMIC_DRAW);
		this->pointCloudVbo.setTexCoordData(this->uvCache.data(), numPoints, GL_DYNAMIC_DRAW);

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
				depthDims.x * 4 * (int)sizeof(uint8_t));

			this->transformation.color_image_to_depth_camera(depthImg, colorImg, &transformedColorImg);
		}
		catch (const k4a::error& e)
		{
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		const auto transformedColorData = reinterpret_cast<uint8_t*>(transformedColorImg.get_buffer());
		const auto transformedColorDims = glm::ivec2(transformedColorImg.get_width_pixels(), transformedColorImg.get_height_pixels());

		if (!this->colorInDepthPix.isAllocated())
		{
			this->colorInDepthPix.allocate(transformedColorDims.x, transformedColorDims.y, OF_PIXELS_BGRA);
			this->colorInDepthTex.allocate(transformedColorDims.x, transformedColorDims.y, GL_RGBA8, ofGetUsingArbTex(), GL_BGRA, GL_UNSIGNED_BYTE);
			this->colorInDepthTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
			this->colorInDepthTex.bind();
			{
				glTexParameteri(this->colorInDepthTex.texData.textureTarget, GL_TEXTURE_SWIZZLE_R, GL_BLUE);
				glTexParameteri(this->colorInDepthTex.texData.textureTarget, GL_TEXTURE_SWIZZLE_B, GL_RED);
			}
			this->colorInDepthTex.unbind();
		}

		this->colorInDepthPix.setFromPixels(transformedColorData, transformedColorDims.x, transformedColorDims.y, 4);
		this->colorInDepthTex.loadData(this->colorInDepthPix);

		ofLogVerbose(__FUNCTION__) << "Color in Depth " << transformedColorDims.x << "x" << transformedColorDims.y << " stride: " << transformedColorImg.get_stride_bytes() << ".";

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

	const ofPixels& Device::getColorInDepthPix() const
	{
		return this->colorInDepthPix;
	}

	const ofTexture& Device::getColorInDepthTex() const
	{
		return this->colorInDepthTex;
	}

	const ofVbo& Device::getPointCloudVbo() const
	{
		return this->pointCloudVbo;
	}
}
