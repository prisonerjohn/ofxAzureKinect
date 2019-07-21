#include "Device.h"

#include "ofLog.h"

const int32_t TIMEOUT_IN_MS = 1000;

namespace ofxAzureKinect
{
	int Device::getInstalledCount()
	{
		return k4a_device_get_installed_count();
	}

	Device::Device()
		: index(-1)
		, bOpen(false)
		, bStreaming(false)
	{
		// TODO: Add a method to set these options.
		this->config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		this->config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
		this->config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
		this->config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
		this->config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	}

	Device::~Device()
	{
		close();
	}

	bool Device::open(int deviceIdx)
	{
		if (this->bOpen)
		{
			ofLogWarning(__FUNCTION__) << "Device " << this->index << " already open!";
			return false;
		}

		// Open connection to the device.
		if (K4A_RESULT_SUCCEEDED != 
			k4a_device_open(deviceIdx, &this->device))
		{
			ofLogError(__FUNCTION__) << "Failed to open device " << deviceIdx << "!";
			return false;
		}

		// Get the device serial number.
		char * serialNumberBuffer = nullptr;
		size_t serialNumberLength = 0;

		if (K4A_BUFFER_RESULT_TOO_SMALL != 
			k4a_device_get_serialnum(this->device, NULL, &serialNumberLength))
		{
			ofLogError(__FUNCTION__) << "Failed to get device " << deviceIdx << " serial number length!";

			k4a_device_close(this->device);
			this->device = nullptr;

			return false;
		}

		serialNumberBuffer = new char[serialNumberLength];
		if (serialNumberBuffer == nullptr)
		{
			ofLogError(__FUNCTION__) << "Failed to allocate serial number memory (" << serialNumberLength <<") for device " << deviceIdx << "!";

			k4a_device_close(this->device);
			this->device = nullptr;

			return false;
		}

		if (K4A_BUFFER_RESULT_SUCCEEDED != 
			k4a_device_get_serialnum(this->device, serialNumberBuffer, &serialNumberLength))
		{
			ofLogError(__FUNCTION__) << "Failed to get serial number for device " << deviceIdx << "!";

			delete[] serialNumberBuffer;
			serialNumberBuffer = nullptr;

			k4a_device_close(this->device);
			this->device = nullptr;

			return false;
		}

		this->serialNumber = serialNumberBuffer;
		
		delete[] serialNumberBuffer;

		this->index = deviceIdx;
		this->bOpen = true;

		ofLogNotice(__FUNCTION__) << "Successfully opened device " << this->index << " with serial number " << this->serialNumber << ".";

		return true;
	}

	bool Device::close(int deviceIdx)
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

		// Create depth to world LUT.
		k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
			this->calibration.depth_camera_calibration.resolution_width,
			this->calibration.depth_camera_calibration.resolution_height,
			this->calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
			&this->depthToWorldTable);

		k4a_float2_t *tableData = (k4a_float2_t *)(void *)k4a_image_get_buffer(this->depthToWorldTable);

		this->depthToWorldSize = glm::ivec2(
			this->calibration.depth_camera_calibration.resolution_width,
			this->calibration.depth_camera_calibration.resolution_height
		);

		k4a_float2_t p;
		k4a_float3_t ray;
		int valid;
		int idx = 0;
		for (int y = 0; y < this->depthToWorldSize.y; ++y)
		{
			p.xy.y = (float)y;

			for (int x = 0; x < this->depthToWorldSize.x; ++x)
			{
				p.xy.x = (float)x;

				k4a_calibration_2d_to_3d(&this->calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

				if (valid)
				{
					tableData[idx].xy.x = ray.xyz.x;
					tableData[idx].xy.y = ray.xyz.y;
				}
				else
				{
					tableData[idx].xy.x = nanf("");
					tableData[idx].xy.y = nanf("");
				}

				++idx;
			}
		}

		this->depthToWorldPix.allocate(this->depthToWorldSize.x, this->depthToWorldSize.y, 2);
		this->depthToWorldPix.setFromPixels((float *)tableData, this->depthToWorldSize.x, this->depthToWorldSize.y, 2);
		this->depthToWorldTex.allocate(this->depthToWorldSize.x, this->depthToWorldSize.y, GL_RG32F);
		this->depthToWorldTex.loadData(this->depthToWorldPix);

		// Create point cloud table.
		k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
			this->calibration.depth_camera_calibration.resolution_width,
			this->calibration.depth_camera_calibration.resolution_height,
			this->calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
			&this->pointCloudTable);

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

		k4a_image_t image;

		// Probe for a color image.
		image = k4a_capture_get_color_image(this->capture);
		if (image)
		{
			auto colorData = (uint8_t*)(void*)k4a_image_get_buffer(image);

			if (!colorPix.isAllocated())
			{
				this->colorSize = glm::ivec2(k4a_image_get_width_pixels(image), k4a_image_get_height_pixels(image));

				this->colorPix.allocate(this->colorSize.x, this->colorSize.y, OF_PIXELS_BGRA);
				this->colorTex.allocate(this->colorSize.x, this->colorSize.y, GL_RGBA8, ofGetUsingArbTex(), GL_BGRA, GL_UNSIGNED_BYTE);
				// TODO: Fix rendering of BGRA texture.
				//glTexParameteri(this->colorTex.texData.textureTarget, GL_TEXTURE_SWIZZLE_R, GL_BLUE);
				//glTexParameteri(this->colorTex.texData.textureTarget, GL_TEXTURE_SWIZZLE_B, GL_RED);
			}

			this->colorPix.setFromPixels(colorData, this->colorSize.x, this->colorSize.y, 4);
			this->colorTex.loadData(this->colorPix);

			ofLogVerbose(__FUNCTION__) << "Capture Color " << this->colorSize.x << "x" << this->colorSize.y << " stride: " << k4a_image_get_stride_bytes(image) << ".";

			k4a_image_release(image);
		}
		else
		{
			ofLogWarning(__FUNCTION__) << "No Color capture found!";
		}

		// Probe for a IR16 image.
		image = k4a_capture_get_ir_image(this->capture);
		if (image)
		{
			auto irData = (uint16_t*)(void*)k4a_image_get_buffer(image);

			if (!irPix.isAllocated())
			{
				this->irSize = glm::ivec2(k4a_image_get_width_pixels(image), k4a_image_get_height_pixels(image));

				this->irPix.allocate(this->irSize.x, this->irSize.y, 1);
				this->irTex.allocate(this->irSize.x, this->irSize.y, GL_R16);
				this->irTex.setRGToRGBASwizzles(true);
			}

			this->irPix.setFromPixels(irData, this->irSize.x, this->irSize.y, 1);
			this->irTex.loadData(this->irPix);

			ofLogVerbose(__FUNCTION__) << "Capture Ir16 " << this->irSize.x << "x" << this->irSize.y << " stride: " << k4a_image_get_stride_bytes(image) << ".";

			k4a_image_release(image);
		}
		else
		{
			ofLogWarning(__FUNCTION__) << "No Ir16 capture found!";
		}

		// Probe for a depth16 image.
		image = k4a_capture_get_depth_image(this->capture);
		if (image)
		{
			auto depthData = (uint16_t*)(void*)k4a_image_get_buffer(image);

			if (!depthPix.isAllocated())
			{
				this->depthSize = glm::ivec2(k4a_image_get_width_pixels(image), k4a_image_get_height_pixels(image));
			
				this->depthPix.allocate(this->depthSize.x, this->depthSize.y, 1);
				this->depthTex.allocate(this->depthSize.x, this->depthSize.y, GL_R16);
			}

			this->depthPix.setFromPixels(depthData, this->depthSize.x, this->depthSize.y, 1);
			this->depthTex.loadData(this->depthPix);

			ofLogVerbose(__FUNCTION__) << "Capture Depth16 " << this->depthSize.x << "x" << this->depthSize.y << " stride: " << k4a_image_get_stride_bytes(image) << ".";

			// Probe for a point cloud frame.
			k4a_float2_t *tableData = (k4a_float2_t *)(void *)k4a_image_get_buffer(this->depthToWorldTable);
			k4a_float3_t *pointCloudData = (k4a_float3_t *)(void *)k4a_image_get_buffer(this->pointCloudTable);

			this->pointCloudMesh.clear();
			this->pointCloudMesh.setMode(OF_PRIMITIVE_POINTS);

			int numPoints = 0;
			for (int i = 0; i < this->depthSize.x * this->depthSize.y; i++)
			{
				if (depthData[i] != 0 && !isnan(tableData[i].xy.x) && !isnan(tableData[i].xy.y))
				{
					pointCloudData[i].xyz.x = tableData[i].xy.x * (float)depthData[i];
					pointCloudData[i].xyz.y = tableData[i].xy.y * (float)depthData[i];
					pointCloudData[i].xyz.z = (float)depthData[i];

					this->pointCloudMesh.addVertex(glm::vec3(tableData[i].xy.x * (float)depthData[i], tableData[i].xy.y * (float)depthData[i], (float)depthData[i]));

					//if (numPoints < 100) ofLogNotice(__FUNCTION__) << "Add point " << numPoints << ": " << pointCloudData[i].xyz.x << ", " << pointCloudData[i].xyz.y << ", " << pointCloudData[i].xyz.z;
					++numPoints;
				}
				else
				{
					pointCloudData[i].xyz.x = nanf("");
					pointCloudData[i].xyz.y = nanf("");
					pointCloudData[i].xyz.z = nanf("");
				}
			}

			this->pointCloudMesh.addVertices((glm::vec3 *)pointCloudData, numPoints);

			k4a_image_release(image);
		}
		else
		{
			ofLogWarning(__FUNCTION__) << "No Depth16 capture found!";
		}

		// Release capture.
		k4a_capture_release(this->capture);
	}

	bool Device::isOpen() const
	{
		return this->bOpen;
	}

	bool Device::isStreaming() const
	{
		return this->bStreaming;
	}

	const glm::ivec2& Device::getDepthSize() const
	{
		return this->depthSize;
	}

	const ofShortPixels& Device::getDepthPix() const
	{
		return this->depthPix;
	}

	const ofTexture& Device::getDepthTex() const
	{
		return this->depthTex;
	}

	const glm::ivec2& Device::getColorSize() const
	{
		return this->colorSize;
	}

	const ofPixels& Device::getColorPix() const
	{
		return this->colorPix;
	}

	const ofTexture& Device::getColorTex() const
	{
		return this->colorTex;
	}

	const glm::ivec2& Device::getIrSize() const
	{
		return this->irSize;
	}

	const ofShortPixels& Device::getIrPix() const
	{
		return this->irPix;
	}

	const ofTexture& Device::getIrTex() const
	{
		return this->irTex;
	}

	const glm::ivec2& Device::getDepthToWorldSize() const
	{
		return this->depthToWorldSize;
	}

	const ofFloatPixels& Device::getDepthToWorldPix() const
	{
		return this->depthToWorldPix;
	}

	const ofTexture& Device::getDepthToWorldTex() const
	{
		return this->depthToWorldTex;
	}

	const ofVboMesh& Device::getPointCloudMesh() const
	{
		return this->pointCloudMesh;
	}
}