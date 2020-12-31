#pragma once

#include <k4a/k4a.hpp>
#include <k4abt.h>

#include "ofParameter.h"
#include "ofPixels.h"
#include "ofTexture.h"

#include "Types.h"

namespace ofxAzureKinect
{
	struct BodyTrackerSettings
	{
		SensorOrientation sensorOrientation;
		ProcessingMode processingMode;
		int32_t gpuDeviceID;
		k4a_calibration_type_t imageType;
		bool updateBodyIndex;
		bool updateBodiesWorld;
		bool updateBodiesImage;

		BodyTrackerSettings();
	};

	class BodyTracker
	{
	public:
		BodyTracker();
		~BodyTracker();

		bool startTracking(const k4a::calibration& calibration, BodyTrackerSettings settings = BodyTrackerSettings());
		bool stopTracking();

		void processCapture(const k4a::capture& capture, const k4a::calibration& calibration, const k4a::transformation& transformation, const k4a::image& depthImg);
		void updateTextures();

		bool isTracking() const;

		const ofPixels& getBodyIndexPix() const;
		const ofTexture& getBodyIndexTex() const;

		size_t getNumBodies() const;
		const std::vector<k4abt_skeleton_t>& getBodySkeletons() const;
		const std::vector<uint32_t>& getBodyIDs() const;
		const std::vector<k4a_float2_t>& getBodyJointsProjected(int idx) const;

	public:
		ofParameter<float> jointSmoothing{ "Joint Smoothing", 0.0f, 0.0f, 1.0f };

	private:
		bool bTracking;

		bool bUpdateBodyIndex;
		bool bUpdateBodiesWorld;
		bool bUpdateBodiesImage;

		k4abt_tracker_configuration_t trackerConfig;
		k4abt_tracker_t bodyTracker;

		k4a_calibration_type_t imageType;

		ofPixels bodyIndexPix;
		ofTexture bodyIndexTex;

		std::vector<k4abt_skeleton_t> bodySkeletons;
		std::vector<uint32_t> bodyIDs;
		std::vector<std::vector<k4a_float2_t>> bodyJointsProjected;

		ofEventListeners eventListeners;
	};
}