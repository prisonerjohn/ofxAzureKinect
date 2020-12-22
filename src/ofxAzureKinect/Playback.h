#pragma once

#include <k4arecord/playback.hpp>

#include "ofParameter.h"
#include "ofThread.h"

#include "Stream.h"

namespace ofxAzureKinect
{
	struct PlaybackSettings
	{
		bool updateColor;
		bool updateIr;
		bool updateWorld;
		bool updateVbo;

		bool autoloop;

		PlaybackSettings();
	};

	class Playback 
		: public Stream
	{
	public:
		Playback();
		~Playback();

		bool open(std::string filepath);
		bool close();

		bool startPlayback(PlaybackSettings playbackSettings = PlaybackSettings());
		bool stopPlayback();

		bool seekPct(float pct);
		bool seekSecs(float seconds);
		bool seekUsecs(long long usecs);

		std::string readTag(const std::string& name);

		bool isOpen() const;

		DepthMode getDepthMode() const override;
		ImageFormat getColorFormat() const override;
		ColorResolution getColorResolution() const override;
		FramesPerSecond getCameraFps() const override;

		WiredSyncMode getWiredSyncMode() const override;
		uint32_t getDepthDelayUsec() const override;
		uint32_t getSubordinateDelayUsec() const override;

		float getDurationSecs() const;
		long long getDurationUsecs() const;

	protected:
		bool updateCapture() override;

	private:
		bool bUpdateDepth;
		bool bLoops;

		std::chrono::microseconds duration;

		k4a_record_configuration_t config;
		k4a::playback playback;
	};
}
