#pragma once

#include <k4a/k4a.hpp>
#include <k4abt.h>
#include <turbojpeg.h>

#include "ofBufferObject.h"
#include "ofEvents.h"
#include "ofFpsCounter.h"
#include "ofParameter.h"
#include "ofPixels.h"
#include "ofTexture.h"
#include "ofThread.h"
#include "ofVboMesh.h"
#include "ofVectorMath.h"

#include "Types.h"
#include "Record.h"
#include "Playback.h"
#include "BodyTracker.h"

namespace ofxAzureKinect
{
	struct DeviceSettings
	{
		int deviceIndex;
		std::string deviceSerial;

		DepthMode depthMode;
		ColorResolution colorResolution;
		ImageFormat colorFormat;
		FramesPerSecond cameraFps;

		WiredSyncMode wiredSyncMode;
		uint32_t subordinateDelayUsec;

		bool updateColor;
		bool updateIr;
		bool updateWorld;
		bool updateVbo;

		bool syncImages;

		bool enableIMU;

		DeviceSettings(int idx = 0);
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
		: ofThread
	{
	public:
		static int getInstalledCount();

	public:
		Device();
		~Device();

		bool open(string filename);
		bool open(string filename, BodyTrackingSettings bodyTrackingSettings);
		bool open(int idx = 0);
		bool open(DeviceSettings settings);
		bool open(DeviceSettings settings, BodyTrackingSettings bodyTrackingSettings);
		bool close();

		bool startCameras();
		bool stopCameras();

		bool isSyncInConnected() const;
		bool isSyncOutConnected() const;

		bool isOpen() const;
		bool isStreaming() const;
		bool isFrameNew() const;

		const std::string &getSerialNumber() const;

		const ofShortPixels &getDepthPix() const;
		const ofTexture &getDepthTex() const;

		const ofPixels &getColorPix() const;
		const ofTexture &getColorTex() const;

		const ofShortPixels &getIrPix() const;
		const ofTexture &getIrTex() const;

		const ofFloatPixels &getDepthToWorldPix() const;
		const ofTexture &getDepthToWorldTex() const;

		const ofFloatPixels &getColorToWorldPix() const;
		const ofTexture &getColorToWorldTex() const;

		const ofShortPixels &getDepthInColorPix() const;
		const ofTexture &getDepthInColorTex() const;

		const ofPixels &getColorInDepthPix() const;
		const ofTexture &getColorInDepthTex() const;

		const ofVbo &getPointCloudVbo() const;

		BodyTracker *get_body_tracker() { return &tracker; }

	public:
		ofParameter<bool> bRecord{"bRecord", false};
		float get_recording_timer_delay();
		ofParameter<bool> play{"play", false};
		ofParameter<bool> pause{"pause", false};
		ofParameter<bool> stop{"stop", false};
		ofParameter<float> seek{"Seek", 0.0f, 0.0f, 1.0f};

	protected:
		void threadedFunction() override;

	private:
		void updatePixels();
		void updateTextures();

		void update(ofEventArgs &args);

		bool setupDepthToWorldTable();
		bool setupColorToWorldTable();
		bool setupImageToWorldTable(k4a_calibration_type_t type, k4a::image &img);

		bool updatePointsCache(k4a::image &frameImg, k4a::image &tableImg);

		bool updateDepthInColorFrame(const k4a::image &depthImg, const k4a::image &colorImg);
		bool updateColorInDepthFrame(const k4a::image &depthImg, const k4a::image &colorImg);

	private:
		int index;
		bool bOpen;
		bool bStreaming;
		bool bPlayback;

		bool bUpdateColor;
		bool bUpdateIr;
		bool bUpdateBodies;
		bool bUpdateWorld;
		bool bUpdateVbo;

		std::condition_variable condition;
		uint64_t pixFrameNum;
		uint64_t texFrameNum;

		bool bNewFrame;

		std::string serialNumber;

		k4a_device_configuration_t config;
		k4a::calibration calibration;
		k4a::transformation transformation;
		k4a::device device;
		k4a::capture capture;

		k4abt_tracker_configuration_t trackerConfig;
		k4abt_tracker_t bodyTracker;

		tjhandle jpegDecompressor;

		k4a_imu_sample_t imu_sample;
		bool enableIMU = false;
		void startIMU();

		ofShortPixels depthPix;
		ofTexture depthTex;

		ofPixels colorPix;
		ofTexture colorTex;

		ofShortPixels irPix;
		ofTexture irTex;

		k4a::image depthToWorldImg;
		ofFloatPixels depthToWorldPix;
		ofTexture depthToWorldTex;

		k4a::image colorToWorldImg;
		ofFloatPixels colorToWorldPix;
		ofTexture colorToWorldTex;

		ofShortPixels depthInColorPix;
		ofTexture depthInColorTex;

		ofPixels colorInDepthPix;
		ofTexture colorInDepthTex;

		ofPixels bodyIndexPix;
		ofTexture bodyIndexTex;
		std::vector<k4abt_skeleton_t> bodySkeletons;
		std::vector<uint32_t> bodyIDs;

		std::vector<glm::vec3> positionCache;
		std::vector<glm::vec2> uvCache;
		size_t numPoints;
		ofVbo pointCloudVbo;

		ofEventListeners eventListeners;

		BodyTracker tracker;

		Record *recording;
		void handle_recording(bool val);

		Playback *playback;
		void listener_playback_play(bool val);
		void listener_playback_pause(bool val);
		void listener_playback_stop(bool val);
		void listener_playback_seek(float val);
	};
} // namespace ofxAzureKinect