#include "Stream.h"

namespace ofxAzureKinect
{
	Stream::Stream()
		: bOpen(false)
		, bStreaming(false)
		, bNewFrame(false)
		, serialNumber("")
		, pixFrameNum(0)
		, texFrameNum(0)
		, bUpdateColor(false)
		, bUpdateIr(false)
		, bUpdateWorld(false)
		, bUpdateVbo(false)
		, jpegDecompressor(tjInitDecompress())
	{}

	Stream::~Stream()
	{
		tjDestroy(jpegDecompressor);
	}

	bool Stream::setupDepthToWorldTable()
	{
		if (this->setupImageToWorldTable(K4A_CALIBRATION_TYPE_DEPTH, this->depthToWorldImg))
		{
			const int width = this->depthToWorldImg.get_width_pixels();
			const int height = this->depthToWorldImg.get_height_pixels();

			const auto data = reinterpret_cast<float*>(this->depthToWorldImg.get_buffer());

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

	bool Stream::setupColorToWorldTable()
	{
		if (this->setupImageToWorldTable(K4A_CALIBRATION_TYPE_COLOR, this->colorToWorldImg))
		{
			const int width = this->colorToWorldImg.get_width_pixels();
			const int height = this->colorToWorldImg.get_height_pixels();

			const auto data = reinterpret_cast<float*>(this->colorToWorldImg.get_buffer());

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

	bool Stream::setupImageToWorldTable(k4a_calibration_type_t type, k4a::image& img)
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

	bool Stream::startStreaming()
	{
		if (this->bStreaming) return false;

		this->startThread();
		ofAddListener(ofEvents().update, this, &Stream::update);

		this->bStreaming = true;

		return true;
	}

	bool Stream::stopStreaming()
	{
		if (!this->bStreaming) return false;

		std::unique_lock<std::mutex> lock(this->mutex);
		this->stopThread();
		this->condition.notify_all();

		ofRemoveListener(ofEvents().update, this, &Stream::update);

		this->bStreaming = false;

		return true;
	}

	void Stream::threadedFunction()
	{
		while (this->isThreadRunning())
		{
			std::unique_lock<std::mutex> lock(this->mutex);

			while (this->isThreadRunning() && this->texFrameNum != this->pixFrameNum)
			{
				this->condition.wait(lock);
			}

			if (this->isThreadRunning() && this->updateCapture())
			{
				this->updatePixels();

				this->releaseCapture();
			}
		}
	}

	void Stream::update(ofEventArgs& args)
	{
		this->bNewFrame = false;

		if (this->texFrameNum != this->pixFrameNum)
		{
			std::unique_lock<std::mutex> lock(this->mutex);

			this->updateTextures();

			this->condition.notify_all();
		}
	}

	void Stream::releaseCapture()
	{
		this->capture.reset();
	}

	void Stream::updatePixels()
	{
		// Probe for a depth16 image.
		auto depthImg = this->capture.get_depth_image();
		if (depthImg)
		{
			const auto depthDims = glm::ivec2(depthImg.get_width_pixels(), depthImg.get_height_pixels());
			if (!depthPix.isAllocated())
			{
				this->depthPix.allocate(depthDims.x, depthDims.y, 1);
			}

			const auto depthData = reinterpret_cast<uint16_t*>(depthImg.get_buffer());
			this->depthPix.setFromPixels(depthData, depthDims.x, depthDims.y, 1);

			ofLogVerbose(__FUNCTION__) << "Capture Depth16 " << depthDims.x << "x" << depthDims.y << " stride: " << depthImg.get_stride_bytes() << ".";
		}
		else
		{
			ofLogWarning(__FUNCTION__) << "No Depth16 capture found (" << ofGetFrameNum() << ")!";
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

				if (this->getColorFormat() == K4A_IMAGE_FORMAT_COLOR_MJPG)
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

				ofLogVerbose(__FUNCTION__) << "Capture Color " << colorDims.x << "x" << colorDims.y << " stride: " << colorImg.get_stride_bytes() << ".";
			}
			else
			{
				ofLogWarning(__FUNCTION__) << "No Color capture found (" << ofGetFrameNum() << ")!";
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

				const auto irData = reinterpret_cast<uint16_t*>(irImg.get_buffer());
				this->irPix.setFromPixels(irData, irSize.x, irSize.y, 1);

				ofLogVerbose(__FUNCTION__) << "Capture Ir16 " << irSize.x << "x" << irSize.y << " stride: " << irImg.get_stride_bytes() << ".";
			}
			else
			{
				ofLogWarning(__FUNCTION__) << "No Ir16 capture found (" << ofGetFrameNum() << ")!";
			}
		}

		if (this->bUpdateVbo)
		{
			if (this->bUpdateColor)
			{
				this->updatePointsCache(colorImg, this->colorToWorldImg);
			}
			else
			{
				this->updatePointsCache(depthImg, this->depthToWorldImg);
			}
		}

		if (colorImg && this->bUpdateColor && this->getColorFormat() == K4A_IMAGE_FORMAT_COLOR_BGRA32)
		{
			// TODO: Fix this for non-BGRA formats, maybe always keep a BGRA k4a::image around.
			this->updateDepthInColorFrame(depthImg, colorImg);
			this->updateColorInDepthFrame(depthImg, colorImg);
		}

		// Release images.
		depthImg.reset();
		colorImg.reset();
		irImg.reset();

		// Update frame number.
		this->pixFrameNum = ofGetFrameNum();
	}

	void Stream::updateTextures()
	{
		if (this->depthPix.isAllocated())
		{
			// Update the depth texture.
			if (!this->depthTex.isAllocated())
			{
				this->depthTex.allocate(this->depthPix);
				this->depthTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
			}

			this->depthTex.loadData(this->depthPix);
			ofLogVerbose(__FUNCTION__) << "Update Depth16 " << this->depthTex.getWidth() << "x" << this->depthTex.getHeight() << ".";
		}

		if (this->bUpdateColor && this->colorPix.isAllocated())
		{
			// Update the color texture.
			if (!this->colorTex.isAllocated())
			{
				this->colorTex.allocate(this->colorPix);
				this->colorTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);

				if (this->getColorFormat() == K4A_IMAGE_FORMAT_COLOR_BGRA32)
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

		if (this->bUpdateIr && this->irPix.isAllocated())
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

		if (this->bUpdateVbo)
		{
			this->pointCloudVbo.setVertexData(this->positionCache.data(), this->numPoints, GL_STREAM_DRAW);
			this->pointCloudVbo.setTexCoordData(this->uvCache.data(), this->numPoints, GL_STREAM_DRAW);
		}

		if (this->bUpdateColor && this->getColorFormat() == K4A_IMAGE_FORMAT_COLOR_BGRA32)
		{
			if (this->depthInColorPix.isAllocated())
			{
				if (!this->depthInColorTex.isAllocated())
				{
					this->depthInColorTex.allocate(this->depthInColorPix);
					this->depthInColorTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
				}

				this->depthInColorTex.loadData(this->depthInColorPix);
			}

			if (this->colorInDepthPix.isAllocated())
			{
				if (!this->colorInDepthTex.isAllocated())
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
		}

		// Update frame number.
		this->texFrameNum = this->pixFrameNum;
		this->bNewFrame = true;
	}

	bool Stream::updatePointsCache(k4a::image& frameImg, k4a::image& tableImg)
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
						depthVal
					);

					this->uvCache[count] = glm::vec2(x, y);

					++count;
				}
			}
		}

		this->numPoints = count;

		return true;
	}

	bool Stream::updateDepthInColorFrame(const k4a::image& depthImg, const k4a::image& colorImg)
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
		}

		this->depthInColorPix.setFromPixels(transformedColorData, colorDims.x, colorDims.y, 1);

		ofLogVerbose(__FUNCTION__) << "Depth in Color " << colorDims.x << "x" << colorDims.y << " stride: " << transformedDepthImg.get_stride_bytes() << ".";

		transformedDepthImg.reset();

		return true;
	}

	bool Stream::updateColorInDepthFrame(const k4a::image& depthImg, const k4a::image& colorImg)
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
		}

		this->colorInDepthPix.setFromPixels(transformedColorData, depthDims.x, depthDims.y, 4);

		ofLogVerbose(__FUNCTION__) << "Color in Depth " << depthDims.x << "x" << depthDims.y << " stride: " << transformedColorImg.get_stride_bytes() << ".";

		transformedColorImg.reset();

		return true;
	}

	bool Stream::isOpen() const
	{
		return this->bOpen;
	}

	bool Stream::isStreaming() const
	{
		return this->bStreaming;
	}

	bool Stream::isFrameNew() const
	{
		return this->bNewFrame;
	}

	const std::string& Stream::getSerialNumber() const
	{
		return this->serialNumber;
	}

	uint32_t Stream::getFramerate() const
	{
		switch (this->getCameraFps())
		{
		case FramesPerSecond::K4A_FRAMES_PER_SECOND_5:
			return 5;

		case FramesPerSecond::K4A_FRAMES_PER_SECOND_15:
			return 15;
			
		case FramesPerSecond::K4A_FRAMES_PER_SECOND_30:
		default:
			return 30;
		}
	}

	const ofShortPixels& Stream::getDepthPix() const
	{
		return this->depthPix;
	}

	const ofTexture& Stream::getDepthTex() const
	{
		return this->depthTex;
	}

	const ofPixels& Stream::getColorPix() const
	{
		return this->colorPix;
	}

	const ofTexture& Stream::getColorTex() const
	{
		return this->colorTex;
	}

	const ofShortPixels& Stream::getIrPix() const
	{
		return this->irPix;
	}

	const ofTexture& Stream::getIrTex() const
	{
		return this->irTex;
	}

	const ofFloatPixels& Stream::getDepthToWorldPix() const
	{
		return this->depthToWorldPix;
	}

	const ofTexture& Stream::getDepthToWorldTex() const
	{
		return this->depthToWorldTex;
	}

	const ofFloatPixels& Stream::getColorToWorldPix() const
	{
		return this->colorToWorldPix;
	}

	const ofTexture& Stream::getColorToWorldTex() const
	{
		return this->colorToWorldTex;
	}

	const ofShortPixels& Stream::getDepthInColorPix() const
	{
		return this->depthInColorPix;
	}

	const ofTexture& Stream::getDepthInColorTex() const
	{
		return this->depthInColorTex;
	}

	const ofPixels& Stream::getColorInDepthPix() const
	{
		return this->colorInDepthPix;
	}

	const ofTexture& Stream::getColorInDepthTex() const
	{
		return this->colorInDepthTex;
	}

	const ofVbo& Stream::getPointCloudVbo() const
	{
		return this->pointCloudVbo;
	}
}
