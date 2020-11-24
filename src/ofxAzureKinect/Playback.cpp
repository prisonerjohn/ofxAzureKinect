#include "Playback.h"

namespace ofxAzureKinect
{

	bool Playback::load(string _filename)
	{
		// Convert filename from string to char*
		char *temp = new char[_filename.size() + 1];
		copy(_filename.begin(), _filename.end(), temp);
		temp[_filename.size()] = '\0';

		// Set the filename for recording output
		this->filename = temp;

		if (k4a_playback_open(this->filename, &playback) != K4A_RESULT_SUCCEEDED)
		{
			ofLogError(__FUNCTION__) << "Failed to open recording: " << filename;
			k4a_failed = true;
			return false;
		}

		recording_length = k4a_playback_get_recording_length_usec(playback);
		printf("Recording is %lld seconds long\n", recording_length / 1000000);

		return true;
	}

	void Playback::play()
	{
		status = PLAYING;
		cout << "play" << endl;
	}

	void Playback::pause()
	{
		status = PAUSED;
		cout << "pause" << endl;
	}

	void Playback::stop()
	{
		seek(0);
		status = STOPPED;
		cout << "stop" << endl;
	}

	k4a_capture_t Playback::getNextCapture()
	{
		k4a_stream_result_t result = k4a_playback_get_next_capture(playback, &capture);
		if (result == K4A_STREAM_RESULT_SUCCEEDED)
		{
			return capture;
		}
		else if (result == K4A_STREAM_RESULT_EOF)
		{
			// End of file reached
			// ofLog() << "End of file reached." << endl;
			seek(0);
			return getNextCapture();
		}
		else if (result == K4A_STREAM_RESULT_FAILED)
		{
			ofLogError(__FUNCTION__) << "Failed to read entire recording." << endl;
			return nullptr;
		}
		return nullptr;
	}

	k4a_imu_sample_t Playback::getNextImuSample()
	{
		// k4a_imu_sample_t imu_sample;
		// checking for the IMU tracking isn't working ... may have to add a special tag when recording?
		// cout << "IMU TRACK: " << k4a_playback_check_track_exists (playback, "K4A_IMU_TRACK") << endl;
		if (config.imu_track_enabled)
			k4a_playback_get_next_imu_sample(playback, &imu_sample);
		return imu_sample;
	}

	void Playback::seek(float amt)
	{
		b_seek_by_device_time = false;
		seek_head = amt;
		int play_head = int(ofMap(seek_head, 0, 1, 0, recording_length, true));

		// Seek to 10 seconds from the start
		if (k4a_playback_seek_timestamp(playback, play_head, K4A_PLAYBACK_SEEK_BEGIN) != K4A_RESULT_SUCCEEDED)
		{
			ofLogError(__FUNCTION__) << "K4A_PLAYBACK_SEEK FAILED.";
			return;
		}
	}

	void Playback::seekByDeviceTime(uint32_t device_usec)
	{
		b_seek_by_device_time = true;
		last_seek_device_usec = device_usec;

		if (k4a_playback_seek_timestamp(playback, last_seek_device_usec, K4A_PLAYBACK_SEEK_DEVICE_TIME) != K4A_RESULT_SUCCEEDED)
		{
			ofLogError(__FUNCTION__) << "K4A_PLAYBACK_SEEK FAILED.";
			return;
		}
	}

	void Playback::seek()
	{
		if (b_seek_by_device_time) {
			this->seekByDeviceTime(last_seek_device_usec);
		}
		else
		{
			this->seek(this->seek_head);
		}
	}

	void Playback::close()
	{
		status = STOPPED;
		k4a_playback_close(playback);
	}

	k4a_record_configuration_t Playback::getDeviceSettings()
	{
		k4a_playback_get_record_configuration(playback, &config);
		return config;
	}

	k4a_calibration_t Playback::getCalibration()
	{
		k4a_calibration_t calibration;
		if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration))
		{
			ofLogError(__FUNCTION__) << "Failed to get calibration data from recording.";
		}
		return calibration;
	}

	string Playback::getSerialNumber()
	{
		return get_tag("K4A_DEVICE_SERIAL_NUMBER");
	}

	string Playback::get_tag(string tag_name)
	{
		char result_buffer[256];
		size_t result_size = 256;

		k4a_buffer_result_t result = k4a_playback_get_tag(
			playback, tag_name.c_str(), result_buffer, &result_size);
		if (K4A_BUFFER_RESULT_SUCCEEDED == result)
		{
			return result_buffer;
		}
		else if (K4A_BUFFER_RESULT_TOO_SMALL == result)
		{
			ofLogError(__FUNCTION__) << "Tag's {" << tag_name << "} has content that is too long.";
		}
		else
		{
			ofLogError(__FUNCTION__) << "Tag {" << tag_name << "} does not exist.";
		}
		return "";
	}

} // namespace ofxAzureKinect