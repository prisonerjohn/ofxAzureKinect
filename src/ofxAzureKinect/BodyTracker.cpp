#include "BodyTracker.h"

namespace ofxAzureKinect
{
	BodyTracker::BodyTracker()
	{
	}

	BodyTracker::BodyTracker(k4a_calibration_t calibration, k4abt_tracker_configuration_t config)
	{
		k4abt_tracker_create(&calibration, config, &tracker);
	}

	void BodyTracker::update(k4a_capture_t capture)
	{
		k4a_wait_result_t enqueueResult = k4abt_tracker_enqueue_capture(tracker, capture, K4A_WAIT_INFINITE);
		if (enqueueResult == K4A_WAIT_RESULT_FAILED)
		{
			ofLogError(__FUNCTION__) << "Failed adding capture to tracker process queue!";
		}
		else
		{
			k4abt_frame_t bodyFrame = nullptr;
			k4a_wait_result_t result = k4abt_tracker_pop_result(tracker, &bodyFrame, K4A_WAIT_INFINITE);
			if (result == K4A_WAIT_RESULT_SUCCEEDED)
			{
				// Probe for a body index map image.
				k4a::image bodyIndexImg = k4abt_frame_get_body_index_map(bodyFrame);

				// Update the Body Index Map ofPixels and ofTexture
				const auto bodyIndexSize = glm::ivec2(bodyIndexImg.get_width_pixels(), bodyIndexImg.get_height_pixels());
				if (!this->bodyIndexPix.isAllocated())
				{
					this->bodyIndexPix.allocate(bodyIndexSize.x, bodyIndexSize.y, 1);
				}

				// Set the ofPixels
				const auto bodyIndexData = reinterpret_cast<uint8_t *>(bodyIndexImg.get_buffer());
				this->bodyIndexPix.setFromPixels(bodyIndexData, bodyIndexSize.x, bodyIndexSize.y, 1);

				// Release the body index map image
				ofLogVerbose(__FUNCTION__) << "Capture BodyIndex " << bodyIndexSize.x << "x" << bodyIndexSize.y << " stride: " << bodyIndexImg.get_stride_bytes() << ".";
				bodyIndexImg.reset();

				// Set number of bodies found
				num_bodies = k4abt_frame_get_num_bodies(bodyFrame);
				ofLogVerbose(__FUNCTION__) << num_bodies << " bodies found!";

				// Update Found Skeletons
				this->bodySkeletons.resize(num_bodies);
				this->bodyIDs.resize(num_bodies);
				for (size_t i = 0; i < num_bodies; i++)
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

	void BodyTracker::update_texture()
	{
		if (!this->bodyIndexTex.isAllocated())
		{
			this->bodyIndexTex.allocate(this->bodyIndexPix);
			this->bodyIndexTex.setTextureMinMagFilter(GL_NEAREST, GL_NEAREST);
		}
		this->bodyIndexTex.loadData(this->bodyIndexPix);
	}

	const ofPixels &BodyTracker::getBodyIndexPix() const
	{
		return this->bodyIndexPix;
	}

	const ofTexture &BodyTracker::getBodyIndexTex() const
	{
		return this->bodyIndexTex;
	}

	size_t BodyTracker::getNumBodies() const
	{
		return this->bodySkeletons.size();
	}

	const std::vector<k4abt_skeleton_t> &BodyTracker::getBodySkeletons() const
	{
		return this->bodySkeletons;
	}

	const std::vector<uint32_t> &BodyTracker::getBodyIDs() const
	{
		return this->bodyIDs;
	}

} // namespace ofxAzureKinect