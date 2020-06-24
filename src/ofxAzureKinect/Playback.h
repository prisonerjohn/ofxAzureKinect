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

		bool load_file(string _filename);
		k4a_record_configuration_t get_device_settings();
		string get_serial_number();
		k4a_calibration_t get_calibration();

		k4a_capture_t get_next_capture();
		k4a_imu_sample_t get_next_imu_sample();

		void seek();
		void seek(float t);

		void play();
		void pause();
		void stop();
		void close();

		bool is_playing() { return status == PLAYING; }
		bool is_paused()  { return status == PAUSED; }
		bool is_stopped() { return status == STOPPED; }

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
	};
} // namespace ofxAzureKinect