#pragma once

#include <string>
#include <k4a/k4a.h>

#include "ofEvents.h"
#include "ofPixels.h"
#include "ofTexture.h"
#include "ofVboMesh.h"
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

		bool setupDepthToWorldFrame(int width, int height);
		bool updateDepthToWorldFrame(const k4a_image_t depthImage);

		bool updateColorInDepthFrame(const k4a_image_t depthImage, const k4a_image_t colorImage);

	private:
		k4a_device_configuration_t config;
		k4a_calibration_t calibration;
		k4a_transformation_t transformation;
		k4a_device_t device = nullptr;
		k4a_capture_t capture = nullptr;

		int index;
		bool bOpen;
		bool bStreaming;

		std::string serialNumber;

		ofShortPixels depthPix;
		ofTexture depthTex;

		ofPixels colorPix;
		ofTexture colorTex;

		ofShortPixels irPix;
		ofTexture irTex;

		k4a_image_t depthToWorldImg;
		ofFloatPixels depthToWorldPix;
		ofTexture depthToWorldTex;

		ofPixels colorInDepthPix;
		ofTexture colorInDepthTex;

		std::vector<glm::vec3> positionCache;
		std::vector<glm::vec2> uvCache;
		ofVbo pointCloudVbo;
	};
}