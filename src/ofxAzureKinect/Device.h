#pragma once

#include <string>
#include <k4a/k4a.h>

#include "ofEvents.h"
#include "ofPixels.h"
#include "ofTexture.h"
#include "ofVectorMath.h"

namespace ofxAzureKinect
{
	class Device
	{
	public:
		static int getInstalledCount();

	public:
		Device();
		~Device();

		bool open(int deviceIdx = 0);
		bool close(int deviceIdx = 0);

		bool startCameras();
		bool stopCameras();

		bool isOpen() const;
		bool isStreaming() const;

		const glm::ivec2& getDepthSize() const;
		const ofShortPixels& getDepthPix() const;
		const ofTexture& getDepthTex() const;

		const glm::ivec2& getColorSize() const;
		const ofPixels& getColorPix() const;
		const ofTexture& getColorTex() const;

		const glm::ivec2& getIrSize() const;
		const ofShortPixels& getIrPix() const;
		const ofTexture& getIrTex() const;

	private:
		void updateCameras(ofEventArgs& args);

	private:
		k4a_device_t device = nullptr;
		k4a_capture_t capture = nullptr;

		int index;
		bool bOpen;
		bool bStreaming;

		std::string serialNumber;

		glm::ivec2 depthSize;
		glm::ivec2 colorSize;
		glm::ivec2 irSize;

		ofShortPixels depthPix;
		ofPixels colorPix;
		ofShortPixels irPix;

		ofTexture depthTex;
		ofTexture colorTex;
		ofTexture irTex;
	};
}