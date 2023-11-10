#include "Playback.h"

namespace ofxAzureKinect
{
	PlaybackSettings::PlaybackSettings()
		: updateColor(true)
		, updateIr(true)
		, updateWorld(true)
		, updateVbo(true)
		, forceVboToDepthSize(false)
		, autoloop(true)
	{}

	Playback::Playback()
		: Stream()
		, bUpdateDepth(true)
		, bLoops(true)
		, bPaused(false)
		, lastFrameSecs(0)
		, duration(0)
	{

	}

	Playback::~Playback()
	{
		this->close();
	}

	bool Playback::open(std::string filepath)
	{
		if (this->bOpen) return false;

		if (filepath.empty())
		{
			ofLogError(__FUNCTION__) << "File path cannot be empty!";
		}

		filepath = ofToDataPath(filepath, true);

		try
		{
			// Open playback file.
			this->playback = k4a::playback::open(filepath.c_str());

			// Read playback config.
			this->config = this->playback.get_record_configuration();

			// Get the serial number.
			this->playback.get_tag("K4A_DEVICE_SERIAL_NUMBER", &this->serialNumber);
			
			// Get the calibration.
			this->calibration = this->playback.get_calibration();

			// Get the duration.
			this->duration = this->playback.get_recording_length();
		}
		catch (const k4a::error& e)
		{
			ofLogError(__FUNCTION__) << e.what();

			this->playback.close();

			return false;
		}

		ofLogNotice(__FUNCTION__) << "Open success, reading from file " << filepath;

		this->bOpen = true;
		return true;
	}

	bool Playback::close()
	{
		if (!this->bOpen) return false;

		this->stopPlayback();

		this->playback.close();

		ofLogNotice(__FUNCTION__) << "Close success";

		this->serialNumber = "";
		this->bOpen = false;

		return true;
	}

	bool Playback::startPlayback(PlaybackSettings playbackSettings)
	{
		if (!this->bOpen)
		{
			ofLogError(__FUNCTION__) << "Open file before starting playback!";
			return false;
		}

		// Set update flags.
		this->bUpdateDepth = this->config.depth_track_enabled;
		this->bUpdateColor = this->config.color_track_enabled && playbackSettings.updateColor;
		this->bUpdateIr = this->config.ir_track_enabled && playbackSettings.updateIr;
		this->bUpdateWorld = this->config.depth_track_enabled && playbackSettings.updateWorld;
		this->bUpdateVbo = this->config.depth_track_enabled && playbackSettings.updateWorld && playbackSettings.updateVbo;
		this->bForceVboToDepthSize = playbackSettings.forceVboToDepthSize;
	
		this->bLoops = playbackSettings.autoloop;

		this->lastFrameSecs = 0;

		if (this->bUpdateDepth && this->bUpdateColor)
		{
			// Create transformation and images.
			this->transformation = k4a::transformation(this->calibration);

			this->setupTransformationImages();
		}

		if (this->bUpdateWorld)
		{
			// Load depth to world LUT.
			this->setupDepthToWorldTable();

			if (this->bUpdateColor)
			{
				// Load color to world LUT.
				this->setupColorToWorldTable();
			}
		}

		return this->startStreaming();
	}

	bool Playback::stopPlayback()
	{
		if (!this->bStreaming) return false;

		this->stopStreaming();

		this->transformation.destroy();

		return true;
	}

	void Playback::setPaused(bool paused)
	{
		this->bPaused = paused;
	}

	bool Playback::isPaused() const
	{
		return this->bPaused;
	}

	bool Playback::seekPct(float pct)
	{
		return this->seekUsecs(ofMap(pct, 0, 1, 0, this->getDurationUsecs(), true));
	}

	bool Playback::seekSecs(float seconds)
	{
		return this->seekUsecs(seconds * 1000000ll);
	}

	bool Playback::seekUsecs(long long usecs)
	{
		if (!this->bOpen) return false;

		try
		{
			this->playback.seek_timestamp(std::chrono::microseconds(usecs), K4A_PLAYBACK_SEEK_BEGIN);
			this->lastFrameSecs = 0;
		}
		catch (const k4a::error& e)
		{
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		return true;
	}

	bool Playback::updateCapture()
	{
		float nextFrameSecs = lastFrameSecs + 1 / static_cast<float>(this->getFramerate());
		if (this->bPaused || ofGetElapsedTimef() < nextFrameSecs)
		{
			// Not ready for another frame yet.
			return false;
		}

		try
		{
			if (this->playback.get_next_capture(&this->capture))
			{
				lastFrameSecs = ofGetElapsedTimef();
				return true;
			}
			else if (this->bLoops)
			{
				// Rewind and try again.
				this->seekUsecs(0);
				return this->updateCapture();
			}
			else
			{
				// Stop.
				this->stopPlayback();
				return false;
			}
		}
		catch (const k4a::error& e)
		{
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}
	}

	std::string Playback::readTag(const std::string& name)
	{
		if (!this->isOpen())
		{
			ofLogError(__FUNCTION__) << "Open playback before reading!";
			return "";
		}

		std::string value = "";
		this->playback.get_tag(name.c_str(), &value);
		return value;
	}

	DepthMode Playback::getDepthMode() const
	{
		return this->config.depth_mode;
	}

	ImageFormat Playback::getColorFormat() const
	{
		return this->config.color_format;
	}

	ColorResolution Playback::getColorResolution() const
	{
		return this->config.color_resolution;
	}

	FramesPerSecond Playback::getCameraFps() const
	{
		return this->config.camera_fps;
	}

	WiredSyncMode Playback::getWiredSyncMode() const
	{
		return this->config.wired_sync_mode;
	}

	uint32_t Playback::getDepthDelayUsec() const
	{
		return this->config.depth_delay_off_color_usec;
	}

	uint32_t Playback::getSubordinateDelayUsec() const
	{
		return this->config.subordinate_delay_off_master_usec;
	}

	float Playback::getDurationSecs() const
	{
		return getDurationUsecs() / 1000000.0f;
	}

	long long Playback::getDurationUsecs() const
	{
		return this->duration.count();
	}
}
