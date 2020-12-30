#include "BodyTracker.h"

namespace ofxAzureKinect
{
	BodyTrackerSettings::BodyTrackerSettings()
		: sensorOrientation(K4ABT_SENSOR_ORIENTATION_DEFAULT)
		, processingMode(K4ABT_TRACKER_PROCESSING_MODE_GPU)
		, gpuDeviceID(0)
	{}

	bool BodyTracker::startTracking(const k4a::calibration& calibration, BodyTrackerSettings settings)
	{
		if (this->bTracking) return false;

		// Generate tracker config.
		this->trackerConfig.sensor_orientation = settings.sensorOrientation;
		this->trackerConfig.gpu_device_id = settings.gpuDeviceID;

		// Create tracker.
		k4abt_tracker_create(&calibration, this->trackerConfig, &this->bodyTracker);

		// Add joint smoothing parameter listener.
		this->eventListeners.push(this->jointSmoothing.newListener([this](float&)
		{
			k4abt_tracker_set_temporal_smoothing(this->bodyTracker, this->jointSmoothing);
		}));

		this->bTracking = true;
	
		return true;
	}

	bool BodyTracker::stopTracking()
	{
		if (!this->bTracking) return false;

		this->eventListeners.unsubscribeAll();

		k4abt_tracker_shutdown(this->bodyTracker);
		k4abt_tracker_destroy(this->bodyTracker);
		this->bodyTracker = nullptr;

		this->bTracking = false;

		return true;
	}

	void BodyTracker::processCapture(const k4a::capture& capture)
	{
		k4a_wait_result_t enqueueResult = k4abt_tracker_enqueue_capture(this->bodyTracker, capture.handle(), K4A_WAIT_INFINITE);
		if (enqueueResult == K4A_WAIT_RESULT_FAILED)
		{
			ofLogError(__FUNCTION__) << "Failed adding capture to tracker process queue!";
		}
		else
		{
			k4abt_frame_t bodyFrame = nullptr;
			k4a_wait_result_t popResult = k4abt_tracker_pop_result(this->bodyTracker, &bodyFrame, K4A_WAIT_INFINITE);
			if (popResult == K4A_WAIT_RESULT_SUCCEEDED)
			{
				// Probe for a body index map image.
				k4a::image bodyIndexImg = k4abt_frame_get_body_index_map(bodyFrame);
				const auto bodyIndexSize = glm::ivec2(bodyIndexImg.get_width_pixels(), bodyIndexImg.get_height_pixels());
				if (!this->bodyIndexPix.isAllocated())
				{
					this->bodyIndexPix.allocate(bodyIndexSize.x, bodyIndexSize.y, 1);
				}

				const auto bodyIndexData = reinterpret_cast<uint8_t*>(bodyIndexImg.get_buffer());
				this->bodyIndexPix.setFromPixels(bodyIndexData, bodyIndexSize.x, bodyIndexSize.y, 1);

				ofLogVerbose(__FUNCTION__) << "Capture BodyIndex " << bodyIndexSize.x << "x" << bodyIndexSize.y << " stride: " << bodyIndexImg.get_stride_bytes() << ".";
				bodyIndexImg.reset();

				size_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
				ofLogVerbose(__FUNCTION__) << numBodies << " bodies found!";

				this->bodySkeletons.resize(numBodies);
				this->bodyIDs.resize(numBodies);
				for (size_t i = 0; i < numBodies; i++)
				{
					k4abt_skeleton_t skeleton;
					k4abt_frame_get_body_skeleton(bodyFrame, i, &skeleton);
					this->bodySkeletons[i] = skeleton;
					uint32_t id = k4abt_frame_get_body_id(bodyFrame, i);
					this->bodyIDs[i] = id;
				}

				// Release body frame once we're finished.
				k4abt_frame_release(bodyFrame);
			}
		}
	}

	void BodyTracker::updateTextures()
	{
		if (this->bodyIndexPix.isAllocated())
		{
			if (!this->bodyIndexTex.isAllocated())
			{
				this->bodyIndexTex.allocate(this->bodyIndexPix);
				this->bodyIndexTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
			}

			this->bodyIndexTex.loadData(this->bodyIndexPix);
		}
	}

	bool BodyTracker::isTracking() const
	{
		return this->bTracking;
	}

	const ofPixels& BodyTracker::getBodyIndexPix() const
	{
		return this->bodyIndexPix;
	}

	const ofTexture& BodyTracker::getBodyIndexTex() const
	{
		return this->bodyIndexTex;
	}

	size_t BodyTracker::getNumBodies() const
	{
		return this->bodySkeletons.size();
	}

	const std::vector<k4abt_skeleton_t>& BodyTracker::getBodySkeletons() const
	{
		return this->bodySkeletons;
	}

	const std::vector<uint32_t>& BodyTracker::getBodyIDs() const
	{
		return this->bodyIDs;
	}
}