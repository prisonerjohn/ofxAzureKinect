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

	class MultiDeviceSyncCapture;

	class Device
		: public ofThread
	{
	protected:
		struct Frame
		{
			ofShortPixels depthPix;
			std::chrono::microseconds depthPixDeviceTime;

			ofPixels colorPix;
			bool bColorPixUpdated = false;
			std::chrono::microseconds colorPixDeviceTime;

			ofShortPixels irPix;
			ofShortPixels depthInColorPix;
			ofPixels colorInDepthPix;

			ofPixels bodyIndexPix;
			std::vector<k4abt_skeleton_t> bodySkeletons;
			std::vector<uint32_t> bodyIDs;

			std::vector<glm::vec3> positionCache;
			std::vector<glm::vec2> uvCache;
			size_t numPoints;

			void swapFrame(Frame& f);
			void reset();
		};

		struct JpegTask
		{
			ofBuffer colorPixBuf;
			std::chrono::microseconds colorPixDeviceTime;
		};

		struct DecodedPix
		{
			ofPixels colorPix;
			std::chrono::microseconds colorPixDeviceTime;
		};

		struct JpegDecodeThread : public ofThread
		{
		protected:
			tjhandle jpegDecompressor;
			ofThreadChannel<JpegTask> toProcess;
			ofThreadChannel<DecodedPix> processed;
			void threadedFunction() override;

		public:

			JpegDecodeThread();
			~JpegDecodeThread();

			void start();
			void stop();

			bool pushTaskIfEmpty(JpegTask& b);
			bool update(ofPixels& outPix, std::chrono::microseconds& outTime);
		} decodeThread;
	public:
		friend class MultiDeviceSyncCapture;

		static int getInstalledCount();

	public:
		Device();
		~Device();

		bool open(uint32_t idx = 0);
		bool open(const std::string& serialNumber);

		bool load(string filename);
		bool close();

		bool startCameras(DeviceSettings deviceSettings = DeviceSettings(), BodyTrackingSettings bodyTrackingSettings = BodyTrackingSettings());
		bool stopCameras();

		virtual void update();

		bool isSyncInConnected() const;
		bool isSyncOutConnected() const;

		bool isOpen() const;
		bool isStreaming() const;
		bool isFrameNew() const;

		const std::string &getSerialNumber() const;

		const ofShortPixels &getDepthPix() const;
		const ofTexture &getDepthTex() const;
		const std::chrono::microseconds& getDepthTexDeviceTime() const {
			return this->frameFront.depthPixDeviceTime;
		}

		const ofPixels &getColorPix() const;
		const ofTexture &getColorTex() const;
		const std::chrono::microseconds& getColorTexDeviceTime() const {
			return this->frameFront.colorPixDeviceTime;
		}

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

		BodyTracker *getBodyTracker() { return &tracker; }

		int32_t getColorCameraControlValue(k4a_color_control_command_t command) const;
		void setColorCameraControlValue(k4a_color_control_command_t command, int32_t value);

		int32_t getExposureTimeAbsolute() const;
		void setExposureTimeAbsolute(int32_t exposure_usec);

		void startRecording(std::string filename = "", float delay = 0.0f);
		void stopRecording();
		bool isRecording() const;

		bool isAsyncJpegDecode() const { return this->bAsyncJpegDecode; }
		void setAsyncJpegDecode(bool b) { this->bAsyncJpegDecode = b; }

		bool isEnableAutoUpdate() const { return this->bEnableAutoUpdate; }
		void setEnableAutoUpdate(bool b) { this->bEnableAutoUpdate = b; }

		bool isEnableThread() const { return this->bEnableThread; }
		void setEnableThread(bool b) { this->bEnableThread = b; }

		k4a_imu_sample_t getIMUSample() const { return imu_sample; }
	public:
		float getRecordingTimerDelay();
		ofParameter<bool> play{"play", false};
		ofParameter<bool> pause{"pause", false};
		ofParameter<bool> stop{"stop", false};
		ofParameter<float> seek{"Seek", 0.0f, 0.0f, 1.0f};

		Playback* getPlayback() {return this->playback; }
		Record* getRecord() { return this->recording; }

	protected:
		virtual void threadedFunction() override;

		virtual void updatePixels();
		virtual void updateTextures();

		virtual void update(ofEventArgs &args);

		bool setupDepthToWorldTable();
		bool setupColorToWorldTable();
		bool setupImageToWorldTable(k4a_calibration_type_t type, k4a::image &img);

		bool updatePointsCache(k4a::image &frameImg, k4a::image &tableImg);

		bool updateDepthInColorFrame(const k4a::image &depthImg, const k4a::image &colorImg);
		bool updateColorInDepthFrame(const k4a::image &depthImg, const k4a::image &colorImg);

	protected:
		int index;
		bool bOpen;
		bool bStreaming;
		bool bPlayback;
		bool bRecording;
		bool bEnableIMU;

		bool bUpdateColor;
		bool bUpdateIr;
		bool bUpdateBodies;
		bool bUpdateWorld;
		bool bUpdateVbo;

		bool bMultiDeviceSyncCapture;
		bool bAsyncJpegDecode;
		bool bEnableAutoUpdate;
		bool bEnableThread;

		std::condition_variable condition;
		uint64_t pixFrameNum;
		uint64_t texFrameNum;

		bool bNewBuffer;
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

		// triple buffer
		Frame frameBack;
		Frame frameSwap;
		Frame frameFront;

		// these are thread safe
		k4a::image depthToWorldImg;
		ofFloatPixels depthToWorldPix;
		k4a::image colorToWorldImg;
		ofFloatPixels colorToWorldPix;

		ofTexture depthTex;
		ofTexture colorTex;
		ofTexture irTex;
		ofTexture depthToWorldTex;
		ofTexture colorToWorldTex;
		ofTexture depthInColorTex;
		ofTexture colorInDepthTex;
		ofTexture bodyIndexTex;
		ofVbo pointCloudVbo;

		ofEventListeners eventListeners;

		BodyTracker tracker;

		Record *recording;
		Playback *playback;
		void listener_playback_play(bool val);
		void listener_playback_pause(bool val);
		void listener_playback_stop(bool val);
		void listener_playback_seek(float val);

		MultiDeviceSyncCapture* master_device_capture = nullptr;
	};

	// reference implementation
	// https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/green_screen/MultiDeviceCapturer.h
	//
	// Note  for use this class.
	//   device firmware version must be matched and latest.
	//   set manual exposure time to shorter, for acquiring correct sync.
	//   more info on : https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1261
	//
	// Not sophisticated implementation.
	class MultiDeviceSyncCapture : public ofThread
	{
	protected:
		friend class Device;
		Device* master_device = nullptr;
		std::vector<Device*> subordinate_devices;
		bool compare_sub_depth_instead_of_color = false;
		std::chrono::microseconds max_allowable_time_offset_error_for_image_timestamp = std::chrono::microseconds(1000);
	public:

		// these funcs must be called before Device::startCameras();
		void setMasterDevice(Device* p);
		void addSubordinateDevice(Device* p);

		void removeAllDevices();

		void start();
		void stop();

		void setMaxAllowableTimeOffsetUsec(uint32_t usec);

	protected:
		void threadedFunction() override;
	};
} // namespace ofxAzureKinect