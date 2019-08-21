#pragma once

#include <string>
#include <k4a/k4a.hpp>
#include "turbojpeg.h"

#include "ofBufferObject.h"
#include "ofEvents.h"
#include "ofPixels.h"
#include "ofTexture.h"
#include "ofVboMesh.h"
#include "ofVectorMath.h"

namespace ofxAzureKinect
{
	typedef k4a_depth_mode_t DepthMode;
	typedef k4a_color_resolution_t ColorResolution;
	typedef k4a_image_format_t ImageFormat;
	typedef k4a_fps_t FramesPerSecond;

	struct DeviceSettings
	{
		int deviceIndex;

		DepthMode depthMode;
		ColorResolution colorResolution;
		ImageFormat colorFormat;
		FramesPerSecond cameraFps;
		
		bool updateColor;
		bool updateIr;
		bool updateWorld;
		bool updateVbo;

		bool synchronized;

		DeviceSettings(int idx = 0);
	};

	class Device
	{
	public:
		static int getInstalledCount();

	public:
		Device();
		~Device();

		bool open(int idx = 0);
		bool open(DeviceSettings settings);
		bool close();

		bool startCameras();
		bool stopCameras();

		bool isOpen() const;
		bool isStreaming() const;

		const ofShortPixels& getDepthPix() const;
		const ofTexture& getDepthTex() const;

		const ofPixels& getColorPix() const;
		const ofTexture& getColorTex() const;

		const ofShortPixels& getIrPix() const;
		const ofTexture& getIrTex() const;

		const ofFloatPixels& getDepthToWorldPix() const;
		const ofTexture& getDepthToWorldTex() const;

		const ofPixels& getColorInDepthPix() const;
		const ofTexture& getColorInDepthTex() const;

		const ofVbo& getPointCloudVbo() const;

	private:
		void updateCameras(ofEventArgs& args);

		bool setupDepthToWorldFrame();
		bool updateDepthToWorldVbo(k4a::image& depthImg);

		bool updateColorInDepthFrame(const k4a::image& depthImg, const k4a::image& colorImg);

	private:
		int index;
		bool bOpen;
		bool bStreaming;

		bool bUpdateColor;
		bool bUpdateIr;
		bool bUpdateWorld;
		bool bUpdateVbo;

		std::string serialNumber;

		k4a_device_configuration_t config;
		k4a::calibration calibration;
		k4a::transformation transformation;
		k4a::device device;
		k4a::capture capture;

		tjhandle jpegDecompressor;

		ofShortPixels depthPix;
		ofTexture depthTex;

		ofPixels colorPix;
		ofTexture colorTex;

		ofShortPixels irPix;
		ofTexture irTex;

		k4a::image depthToWorldImg;
		ofFloatPixels depthToWorldPix;
		ofTexture depthToWorldTex;

		ofPixels colorInDepthPix;
		ofTexture colorInDepthTex;

		std::vector<glm::vec3> positionCache;
		std::vector<glm::vec2> uvCache;
		ofVbo pointCloudVbo;
	};
}