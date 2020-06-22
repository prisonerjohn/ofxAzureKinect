#pragma once

#include <k4a/k4a.hpp>
#include <k4arecord/record.h>

#include "ofMain.h"

// BUG: Playback is too fast ... maybe frames are dropping and not being recorded?
// Duration seems to be correct, but actual playback is 2X speed (10 sec recording playsback in 5sec)
// syncImages doesn't seem to have an effect

namespace ofxAzureKinect
{
	class Record
	{

	public:
		Record(){};
		~Record();

		void setup(k4a_device_t device, k4a_device_configuration_t config, bool recording_imu_enabled = true, float delay = 3, string filename = "");
		void start();
		void stop();
		void record(k4a_capture_t *capture);

		// Set a recording delay (in seconds)
		void set_delay(float _delay) { this->delay = _delay; }
		float get_timer_delay();


	private:
		k4a_device_t device;
		k4a_record_t recording;

		char *filename;
		bool recording_imu_enabled = true;
		
		void record_imu();

		bool k4a_failed = false;

		float delay = 0;
		float delay_start = 0;	
	};
} // namespace ofxAzureKinect