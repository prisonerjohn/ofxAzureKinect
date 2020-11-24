#pragma once

#include <stdint.h>

#include <k4a/k4a.hpp>
#include <k4arecord/playback.h>

#include "ofMain.h"


namespace ofxAzureKinect
{
	class Playback
	{

	public:
		Playback(){};
		~Playback(){};

		bool load(string _filename);
		k4a_record_configuration_t getDeviceSettings();
		string getSerialNumber();
		k4a_calibration_t getCalibration();

		k4a_capture_t getNextCapture();
		k4a_imu_sample_t getNextImuSample();

		void seek();
		void seek(float t);
		void seekByDeviceTime(uint32_t device_usec);

		void play();
		void pause();
		void stop();
		void close();

		bool isPlaying() { return status == PLAYING; }
		bool isPaused()  { return status == PAUSED; }
		bool isStopped() { return status == STOPPED; }

		uint32_t getStartTimeOffsetUsec() const {
			return config.start_timestamp_offset_usec;
		}
		uint32_t getRecordingLengthUsec() const {
			return recording_length;
		}

	private:
		k4a_playback_t playback;
		k4a_capture_t capture;
		k4a_imu_sample_t imu_sample;
		k4a_record_configuration_t config;

		char *filename;
		bool k4a_failed = false;

		int recording_length = 0;
		float seek_head = 0;

		float in = 0, out = 0;

		string get_tag(string tag_name);
		string serial_number;

		enum Status
		{
			STOPPED,
			PAUSED,
			PLAYING
		};
		Status status = STOPPED;

		bool b_seek_by_device_time = false;
		uint32_t last_seek_device_usec = 0;
	};
} // namespace ofxAzureKinect