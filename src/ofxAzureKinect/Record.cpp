#include "Record.h"

namespace ofxAzureKinect
{

	void Record::setup(k4a_device_t device, k4a_device_configuration_t config, bool recording_imu_enabled, float delay, string filename)
	{
		this->device = device;
		this->recording_imu_enabled = recording_imu_enabled;
		this->delay = delay;

		// Set a timestamped or user defined filename
		if (filename == "")
			filename = ofToDataPath("output_" + ofGetTimestampString("%Y%m%d_%H-%M-%S") + ".mkv");
		else
		{
			filename = ofToDataPath(filename);
		}

		// Convert filename from string to char*
		char *temp = new char[filename.size() + 1];
		copy(filename.begin(), filename.end(), temp);
		temp[filename.size()] = '\0';

		// Set the filename for recording output
		this->filename = temp;

		// Set up the default recording tracks
		if (K4A_FAILED(k4a_record_create(this->filename, device, config, &recording)))
		{
			printf("Unable to create recording file: %s\n", this->filename);
			k4a_failed = true;
			return;
		}

		// Add any custom tracks and tags you want to record
		// See example code: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/k4arecord_custom_track/main.c
		// ...

		// Add IMU track
		if (this->recording_imu_enabled)
		{
			k4a_record_add_imu_track(recording);
		}

		// Write the recording header after all the track metadata is set up.
		k4a_record_write_header(recording);

		cout << "Recording Setup to file: " << this->filename << endl;
	}

	void Record::start()
	{
		delay_start = ofGetElapsedTimef();
		if (delay != 0)
		{
			cout << "Recording Will Begin In " << delay << " seconds!" << endl;
		}
	}

	void Record::stop()
	{
		if (k4a_failed)
		{
			ofLogError(__FUNCTION__) << "Recording Failed ... see error above.";
		}
		else
		{
			cout << "\nSaving Recording to: " << filename << endl;
			k4a_record_flush(recording);
			k4a_record_close(recording);
			cout << "Done." << endl;
		}
	}

	void Record::record(k4a_capture_t *capture)
	{
		if (k4a_failed)
		{
			ofLogError(__FUNCTION__) << "Recording Failed ... see error above.";
		}
		else
		{
			// Wait for any recording delay
			if (ofGetElapsedTimef() - delay_start > delay)
			{

				// Write the capture to any built-in tracks
				k4a_record_write_capture(recording, *capture);

				// Write the capture for any other custom tracks (not the IMU; do that after releasing the capture)
				// ...

				if (recording_imu_enabled)
				{
					// Record IMU IMU data
					record_imu(); //device, recording);
				}

				// Indicate that we are recording
				cout << ".";
				cout.flush();
			}
			else
			{
				// Print the countdown
				// cout << (delay - (ofGetElapsedTimef() - delay_start)) << endl;
			}
		}
	}

	void Record::record_imu()
	{
		// Code from k4arecorder (Line 213)
		// https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/master/tools/k4arecorder/recorder.cpp
		// and https://docs.microsoft.com/en-us/azure/kinect-dk/retrieve-imu-samples

		k4a_wait_result_t result;

		// Loop to get the queued IMU samples after every capture.
		// We kick out of the loop when result returns K4A_WAIT_RESULT_TIMEOUT
		do
		{
			k4a_imu_sample_t sample;
			result = k4a_device_get_imu_sample(device, &sample, 0);
			if (result == K4A_WAIT_RESULT_SUCCEEDED)
			{
				// Write the IMU data to file
				k4a_result_t write_result = k4a_record_write_imu_sample(recording, sample);
				if (K4A_FAILED(write_result))
				{
					ofLogError(__FUNCTION__) << "Runtime error: k4a_record_write_imu_sample() returned " << write_result;
					break;
				}
			}
			else if (result == K4A_WAIT_RESULT_TIMEOUT)
			{
				// Indicates that there are no queued samples and none have arrived in the timeout specified.
				break;
			}
			else
			{
				ofLogError(__FUNCTION__) << "Runtime error: k4a_device_get_imu_sample() returned " << result;
				break;
			}

			// printf(" | Accelerometer temperature:%.2f x:%.4f y:%.4f z: %.4f\n",
			// 	   sample.temperature,
			// 	   sample.acc_sample.xyz.x,
			// 	   sample.acc_sample.xyz.y,
			// 	   sample.acc_sample.xyz.z);

		} while (result != K4A_WAIT_RESULT_FAILED);
	}

	float Record::get_timer_delay()
	{
		return MAX(delay - (ofGetElapsedTimef() - delay_start), 0);
	}

} // namespace ofxAzureKinect