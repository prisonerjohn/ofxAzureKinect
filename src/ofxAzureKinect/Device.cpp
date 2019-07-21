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
		, updatePointCloud(true)
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
		, bUpdatePointCloud(false)
	{}

	Device::~Device()
	{
		close();
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

		// Open connection to the device.
		if (K4A_RESULT_SUCCEEDED != 
			k4a_device_open(settings.deviceIndex, &this->device))
		{
			ofLogError(__FUNCTION__) << "Failed to open device " << settings.deviceIndex << "!";
			return false;
		}

		// Get the device serial number.
		char * serialNumberBuffer = nullptr;
		size_t serialNumberLength = 0;

		if (K4A_BUFFER_RESULT_TOO_SMALL != 
			k4a_device_get_serialnum(this->device, NULL, &serialNumberLength))
		{
			ofLogError(__FUNCTION__) << "Failed to get device " << settings.deviceIndex << " serial number length!";

			k4a_device_close(this->device);
			this->device = nullptr;

			return false;
		}

		serialNumberBuffer = new char[serialNumberLength];
		if (serialNumberBuffer == nullptr)
		{
			ofLogError(__FUNCTION__) << "Failed to allocate serial number memory (" << serialNumberLength <<") for device " << settings.deviceIndex << "!";

			k4a_device_close(this->device);
			this->device = nullptr;

			return false;
		}

		if (K4A_BUFFER_RESULT_SUCCEEDED != 
			k4a_device_get_serialnum(this->device, serialNumberBuffer, &serialNumberLength))
		{
			ofLogError(__FUNCTION__) << "Failed to get serial number for device " << settings.deviceIndex << "!";

			delete[] serialNumberBuffer;
			serialNumberBuffer = nullptr;

			k4a_device_close(this->device);
			this->device = nullptr;

			return false;
		}

		this->serialNumber = serialNumberBuffer;	
		delete[] serialNumberBuffer;

		this->index = settings.deviceIndex;
		this->bOpen = true;

		this->bUpdateColor = settings.updateColor;
		this->bUpdateIr = settings.updateIr;
		this->bUpdatePointCloud = settings.updatePointCloud;

		ofLogNotice(__FUNCTION__) << "Successfully opened device " << this->index << " with serial number " << this->serialNumber << ".";

		return true;
	}

	bool Device::close()
	{
		if (!this->bOpen) return false;

		this->stopCameras();

		k4a_device_close(this->device);

		this->device = nullptr;

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
		if (K4A_RESULT_SUCCEEDED != 
			k4a_device_get_calibration(this->device, this->config.depth_mode, this->config.color_resolution, &this->calibration))
		{
			ofLogError(__FUNCTION__) << "Failed to get calibration for device " << this->index;
			return false;
		}

		if (this->bUpdateColor)
		{
			// Create transformation.
			this->transformation = k4a_transformation_create(&this->calibration);
		}

		if (this->bUpdatePointCloud)
		{
			// Load depth to world LUT.
			this->setupDepthToWorldFrame(
				this->calibration.depth_camera_calibration.resolution_width,
				this->calibration.depth_camera_calibration.resolution_height);
		}

		// Start cameras.
		if (K4A_RESULT_SUCCEEDED != 
			k4a_device_start_cameras(this->device, &config))
		{
			ofLogError(__FUNCTION__) << "Failed to start cameras for device " << this->index;
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

		k4a_image_release(this->depthToWorldImg);
		k4a_transformation_destroy(this->transformation);
		
		k4a_device_stop_cameras(this->device);

		this->bStreaming = false;

		return true;
	}

	void Device::updateCameras(ofEventArgs& args)
	{
		// Get a depth frame.
		const auto waitResult = k4a_device_get_capture(this->device, &this->capture, TIMEOUT_IN_MS);
		if (K4A_WAIT_RESULT_TIMEOUT == waitResult)
		{
			ofLogWarning(__FUNCTION__) << "Timed out waiting for a capture for device " << this->index << ".";
			return;
		}
		if (K4A_WAIT_RESULT_FAILED == waitResult)
		{
			ofLogWarning(__FUNCTION__) << "Failed to read a capture for device " << this->index << ".";
			return;
		}

		// Probe for a depth16 image.
		auto depthImage = k4a_capture_get_depth_image(this->capture);
		if (depthImage)
		{
			const auto depthData = (uint16_t*)(void*)k4a_image_get_buffer(depthImage);
			const auto depthSize = glm::ivec2(
				k4a_image_get_width_pixels(depthImage),
				k4a_image_get_height_pixels(depthImage));

			if (!depthPix.isAllocated())
			{
				this->depthPix.allocate(depthSize.x, depthSize.y, 1);
				this->depthTex.allocate(depthSize.x, depthSize.y, GL_R16);
			}

			this->depthPix.setFromPixels(depthData, depthSize.x, depthSize.y, 1);
			this->depthTex.loadData(this->depthPix);

			ofLogVerbose(__FUNCTION__) << "Capture Depth16 " << depthSize.x << "x" << depthSize.y << " stride: " << k4a_image_get_stride_bytes(depthImage) << ".";
		}
		else
		{
			ofLogWarning(__FUNCTION__) << "No Depth16 capture found!";
		}

		k4a_image_t colorImage;
		if (this->bUpdateColor)
		{
			// Probe for a color image.
			colorImage = k4a_capture_get_color_image(this->capture);
			if (colorImage)
			{
				const auto colorData = (uint8_t*)(void*)k4a_image_get_buffer(colorImage);
				const auto colorSize = glm::ivec2(
					k4a_image_get_width_pixels(colorImage),
					k4a_image_get_height_pixels(colorImage));

				if (!colorPix.isAllocated())
				{
					this->colorPix.allocate(colorSize.x, colorSize.y, OF_PIXELS_BGRA);
					this->colorTex.allocate(colorSize.x, colorSize.y, GL_RGBA8, ofGetUsingArbTex(), GL_BGRA, GL_UNSIGNED_BYTE);
					// TODO: Fix rendering of BGRA texture.
					//glTexParameteri(this->colorTex.texData.textureTarget, GL_TEXTURE_SWIZZLE_R, GL_BLUE);
					//glTexParameteri(this->colorTex.texData.textureTarget, GL_TEXTURE_SWIZZLE_B, GL_RED);
				}

				this->colorPix.setFromPixels(colorData, colorSize.x, colorSize.y, 4);
				this->colorTex.loadData(this->colorPix);

				ofLogVerbose(__FUNCTION__) << "Capture Color " << colorSize.x << "x" << colorSize.y << " stride: " << k4a_image_get_stride_bytes(colorImage) << ".";
			}
			else
			{
				ofLogWarning(__FUNCTION__) << "No Color capture found!";
			}
		}

		k4a_image_t irImage;
		if (this->bUpdateIr)
		{
			// Probe for a IR16 image.
			irImage = k4a_capture_get_ir_image(this->capture);
			if (irImage)
			{
				const auto irData = (uint16_t*)(void*)k4a_image_get_buffer(irImage);
				const auto irSize = glm::ivec2(
					k4a_image_get_width_pixels(irImage),
					k4a_image_get_height_pixels(irImage));

				if (!this->irPix.isAllocated())
				{
					this->irPix.allocate(irSize.x, irSize.y, 1);
					this->irTex.allocate(irSize.x, irSize.y, GL_R16);
					this->irTex.setRGToRGBASwizzles(true);
				}

				this->irPix.setFromPixels(irData, irSize.x, irSize.y, 1);
				this->irTex.loadData(this->irPix);

				ofLogVerbose(__FUNCTION__) << "Capture Ir16 " << irSize.x << "x" << irSize.y << " stride: " << k4a_image_get_stride_bytes(irImage) << ".";
			}
			else
			{
				ofLogWarning(__FUNCTION__) << "No Ir16 capture found!";
			}
		}

		if (this->bUpdatePointCloud)
		{
			this->updateDepthToWorldFrame(depthImage);
		}

		if (colorImage && this->bUpdateColor)
		{
			this->updateColorInDepthFrame(depthImage, colorImage);
		}

		// Release images.
		if (colorImage)
		{
			k4a_image_release(colorImage);
		}
		if (depthImage)
		{
			k4a_image_release(depthImage);
		}
		if (irImage)
		{
			k4a_image_release(irImage);
		}

		// Release capture.
		k4a_capture_release(this->capture);
	}

	bool Device::setupDepthToWorldFrame(int width, int height)
	{
		// Create depth to world LUT.
		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
			width, height,
			width * (int)sizeof(k4a_float2_t),
			&this->depthToWorldImg))
		{
			ofLogError(__FUNCTION__) << "Failed to create depth to world image!";
			return false;
		}

		auto imageData = (k4a_float2_t *)(void *)k4a_image_get_buffer(this->depthToWorldImg);
		const auto imageSize = glm::ivec2(
			k4a_image_get_width_pixels(this->depthToWorldImg),
			k4a_image_get_height_pixels(this->depthToWorldImg));

		k4a_float2_t p;
		k4a_float3_t ray;
		int valid;
		int idx = 0;
		for (int y = 0; y < imageSize.y; ++y)
		{
			p.xy.y = (float)y;

			for (int x = 0; x < imageSize.x; ++x)
			{
				p.xy.x = (float)x;

				k4a_calibration_2d_to_3d(&this->calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

				if (valid)
				{
					imageData[idx].xy.x = ray.xyz.x;
					imageData[idx].xy.y = ray.xyz.y;
				}
				else
				{
					imageData[idx].xy.x = nanf("");
					imageData[idx].xy.y = nanf("");
				}

				++idx;
			}
		}

		if (!this->depthToWorldPix.isAllocated())
		{
			this->depthToWorldPix.allocate(imageSize.x, imageSize.y, 2);
			this->depthToWorldTex.allocate(imageSize.x, imageSize.y, GL_RG32F);
		}

		this->depthToWorldPix.setFromPixels((float *)imageData, imageSize.x, imageSize.y, 2);
		this->depthToWorldTex.loadData(this->depthToWorldPix);

		return true;
	}

	bool Device::updateDepthToWorldFrame(const k4a_image_t depthImage)
	{
		const auto depthSize = glm::ivec2(
			k4a_image_get_width_pixels(depthImage),
			k4a_image_get_height_pixels(depthImage));

		const auto depthData = (uint16_t*)(void*)k4a_image_get_buffer(depthImage);
		const auto depthToWorldData = (k4a_float2_t *)(void *)k4a_image_get_buffer(this->depthToWorldImg);
		
		this->positionCache.resize(depthSize.x * depthSize.y);
		this->uvCache.resize(depthSize.x * depthSize.y);

		int numPoints = 0;
		for (int y = 0; y < depthSize.y; ++y)
		{
			for (int x = 0; x < depthSize.x; ++x)
			{
				int idx = y * depthSize.x + x;
				if (depthData[idx] != 0 && 
					!isnan(depthToWorldData[idx].xy.x) && !isnan(depthToWorldData[idx].xy.y))
				{
					this->positionCache[numPoints] = glm::vec3(
						depthToWorldData[idx].xy.x * (float)depthData[idx],
						depthToWorldData[idx].xy.y * (float)depthData[idx],
						(float)depthData[idx]
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

	bool Device::updateColorInDepthFrame(const k4a_image_t depthImage, const k4a_image_t colorImage)
	{
		const auto depthSize = glm::ivec2(
			k4a_image_get_width_pixels(depthImage),
			k4a_image_get_height_pixels(depthImage));

		k4a_image_t transformedColorImage = NULL;
		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
			depthSize.x, depthSize.y,
			depthSize.x * 4 * (int)sizeof(uint8_t),
			&transformedColorImage))
		{
			ofLogError(__FUNCTION__) << "Failed to create transformed color image!";
			return false;
		}

		if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(this->transformation,
			depthImage,
			colorImage,
			transformedColorImage))
		{
			ofLogError(__FUNCTION__) << "Failed to compute transformed color image!";
			return false;
		}

		auto transformedColorData = (uint8_t*)(void*)k4a_image_get_buffer(transformedColorImage);
		const auto transformedColorSize = glm::ivec2(
			k4a_image_get_width_pixels(transformedColorImage), 
			k4a_image_get_height_pixels(transformedColorImage));

		if (!this->colorInDepthPix.isAllocated())
		{
			this->colorInDepthPix.allocate(transformedColorSize.x, transformedColorSize.y, OF_PIXELS_BGRA);
			this->colorInDepthTex.allocate(transformedColorSize.x, transformedColorSize.y, GL_RGBA8, ofGetUsingArbTex(), GL_BGRA, GL_UNSIGNED_BYTE);
		}

		this->colorInDepthPix.setFromPixels(transformedColorData, transformedColorSize.x, transformedColorSize.y, 4);
		this->colorInDepthTex.loadData(this->colorInDepthPix);

		ofLogVerbose(__FUNCTION__) << "Color in Depth " << transformedColorSize.x << "x" << transformedColorSize.y << " stride: " << k4a_image_get_stride_bytes(transformedColorImage) << ".";

		k4a_image_release(transformedColorImage);
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
