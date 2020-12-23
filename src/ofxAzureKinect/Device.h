#pragma once

#include <string>

#include <k4a/k4a.hpp>
#include <k4abt.h>

#include "ofParameter.h"
#include "ofPixels.h"
#include "ofTexture.h"

#include "Recorder.h"
#include "Stream.h"
#include "Types.h"

namespace ofxAzureKinect
{
	struct DeviceSettings
	{
		DepthMode depthMode;
		ColorResolution colorResolution;
		ImageFormat colorFormat;
		FramesPerSecond cameraFps;

		WiredSyncMode wiredSyncMode;
		uint32_t depthDelayUsec;
		uint32_t subordinateDelayUsec;

		bool updateColor;
		bool updateIr;
		bool updateWorld;
		bool updateVbo;

		bool syncImages;

		DeviceSettings();
	};

	struct BodyTrackingSettings
	{
		SensorOrientation sensorOrientation;
		ProcessingMode processingMode;
		int32_t gpuDeviceID;

		bool updateBodies;

		BodyTrackingSettings();
	};

	class Device 
		: public Stream
	{
	public:
		static int getInstalledCount();

	public:
		Device();
		~Device();

		bool open(uint32_t idx = 0);
		bool open(const std::string& serialNumber);
		bool close();

		bool startCameras(DeviceSettings deviceSettings = DeviceSettings(), BodyTrackingSettings bodyTrackingSettings = BodyTrackingSettings());
		bool stopCameras();

		bool startRecording(std::string filepath = "");
		bool stopRecording();

		bool isSyncInConnected() const;
		bool isSyncOutConnected() const;

		bool isRecording() const;

		DepthMode getDepthMode() const override;
		ImageFormat getColorFormat() const override;
		ColorResolution getColorResolution() const override;
		FramesPerSecond getCameraFps() const override;

		WiredSyncMode getWiredSyncMode() const override;
		uint32_t getDepthDelayUsec() const override;
		uint32_t getSubordinateDelayUsec() const override;

		bool getSyncImages() const;

		const ofPixels& getBodyIndexPix() const;
		const ofTexture& getBodyIndexTex() const;

		size_t getNumBodies() const;
		const std::vector<k4abt_skeleton_t>& getBodySkeletons() const;
		const std::vector<uint32_t>& getBodyIDs() const;

	public:
		ofParameter<float> jointSmoothing{ "Joint Smoothing", 0.0f, 0.0f, 1.0f };

	protected:
		bool updateCapture() override;

		void updatePixels() override;
		void updateTextures() override;

	private:
		int index;
	
		bool bRecording;

		bool bUpdateBodies;
		
		k4a_device_configuration_t config;
		k4a::device device;
		
		k4abt_tracker_configuration_t trackerConfig;
		k4abt_tracker_t bodyTracker;

		Recorder recorder;

		ofPixels bodyIndexPix;
		ofTexture bodyIndexTex;
		std::vector<k4abt_skeleton_t> bodySkeletons;
		std::vector<uint32_t> bodyIDs;

		ofEventListeners eventListeners;
	};
}