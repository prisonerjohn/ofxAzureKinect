#include "Device.h"

#include "ofLog.h"

const int32_t TIMEOUT_IN_MS = 1000;

namespace ofxKinectForAzure
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
		if (K4A_RESULT_SUCCEEDED != k4a_device_open(deviceIdx, &this->device))
		{
			ofLogError(__FUNCTION__) << "Failed to open device " << deviceIdx << "!";
			return false;
		}

		// Get the device serial number.
		char * serialNumberBuffer = nullptr;
		size_t serialNumberLength = 0;

		if (K4A_BUFFER_RESULT_TOO_SMALL != k4a_device_get_serialnum(this->device, NULL, &serialNumberLength))
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

		if (K4A_BUFFER_RESULT_SUCCEEDED != k4a_device_get_serialnum(this->device, serialNumberBuffer, &serialNumberLength))
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

		k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
		config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
		config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		config.camera_fps = K4A_FRAMES_PER_SECOND_30;

		if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(this->device, &config))
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
				this->colorTex.allocate(this->colorSize.x, this->colorSize.y, GL_RGBA);
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

		// probe for a IR16 image
		image = k4a_capture_get_ir_image(this->capture);
		if (image != NULL)
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
		if (image != NULL)
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
;
	}
}