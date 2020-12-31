#include "BodyTracker.h"

namespace ofxAzureKinect
{
	BodyTrackerSettings::BodyTrackerSettings()
		: sensorOrientation(K4ABT_SENSOR_ORIENTATION_DEFAULT)
		, processingMode(K4ABT_TRACKER_PROCESSING_MODE_GPU)
		, gpuDeviceID(0)
		, imageType(K4A_CALIBRATION_TYPE_DEPTH)
		, updateBodyIndex(true)
		, updateBodiesWorld(true)
		, updateBodiesImage(false)
	{}

	BodyTracker::BodyTracker()
		: bTracking(false)
		, imageType(K4A_CALIBRATION_TYPE_DEPTH)
		, bUpdateBodyIndex(false)
		, bUpdateBodiesWorld(false)
		, bUpdateBodiesImage(false)
	{}

	BodyTracker::~BodyTracker()
	{
		this->stopTracking();
	}

	bool BodyTracker::startTracking(const k4a::calibration& calibration, BodyTrackerSettings settings)
	{
		if (this->bTracking) return false;

		// Generate tracker config.
		this->trackerConfig.sensor_orientation = settings.sensorOrientation;
		this->trackerConfig.gpu_device_id = settings.gpuDeviceID;

		try
		{
			// Create tracker.
			this->bodyTracker = k4abt::tracker::create(calibration, this->trackerConfig);
		}
		catch (const k4a::error& e)
		{
			ofLogError(__FUNCTION__) << e.what();
			return false;
		}

		// Add joint smoothing parameter listener.
		this->eventListeners.push(this->jointSmoothing.newListener([this](float&)
		{
			this->bodyTracker.set_temporal_smoothing(this->jointSmoothing);
		}));

		// Save update flags.
		this->imageType = settings.imageType;
		this->bUpdateBodyIndex = settings.updateBodyIndex;
		this->bUpdateBodiesWorld = settings.updateBodiesWorld || settings.updateBodiesImage;
		this->bUpdateBodiesImage = settings.updateBodiesImage;

		this->bTracking = true;
	
		return true;
	}

	bool BodyTracker::stopTracking()
	{
		if (!this->bTracking) return false;

		this->eventListeners.unsubscribeAll();

		this->bodyIndexPix.clear();
		this->bodyIndexTex.clear();

		this->bodyTracker.shutdown();
		this->bodyTracker.destroy();

		this->bTracking = false;

		return true;
	}

	void BodyTracker::processCapture(const k4a::capture& capture, const k4a::calibration& calibration, const k4a::transformation& transformation, const k4a::image& depthImg)
	{
		if (!this->bodyTracker.enqueue_capture(capture))
		{
			ofLogError(__FUNCTION__) << "Failed adding capture to tracker process queue!";
			return;
		}
		
		k4abt::frame bodyFrame = this->bodyTracker.pop_result();
		if (bodyFrame == nullptr)
		{
			ofLogError(__FUNCTION__) << "Failed processing capture!";
			return;
		}

		if (this->bUpdateBodyIndex)
		{
			// Probe for a body index map image.
			k4a::image bodyIndexImg = bodyFrame.get_body_index_map();

			if (this->imageType == K4A_CALIBRATION_TYPE_COLOR)
			{
				try
				{
					k4a::image transformedBodyIndexImg = transformation.depth_image_to_color_camera_custom(depthImg, bodyIndexImg,
						K4A_TRANSFORMATION_INTERPOLATION_TYPE_NEAREST, K4ABT_BODY_INDEX_MAP_BACKGROUND).second;

					// Swap body index image with transformed version.
					bodyIndexImg.reset();
					bodyIndexImg = std::move(transformedBodyIndexImg);
				}
				catch (const k4a::error& e)
				{
					ofLogError(__FUNCTION__) << e.what();
				}
			}

			const auto bodyIndexDims = glm::ivec2(bodyIndexImg.get_width_pixels(), bodyIndexImg.get_height_pixels());
			if (!this->bodyIndexPix.isAllocated())
			{
				this->bodyIndexPix.allocate(bodyIndexDims.x, bodyIndexDims.y, 1);
			}

			const auto bodyIndexData = reinterpret_cast<uint8_t*>(bodyIndexImg.get_buffer());
			this->bodyIndexPix.setFromPixels(bodyIndexData, bodyIndexDims.x, bodyIndexDims.y, 1);
			ofLogVerbose(__FUNCTION__) << "Capture BodyIndex " << bodyIndexDims.x << "x" << bodyIndexDims.y << " stride: " << bodyIndexImg.get_stride_bytes() << ".";

			bodyIndexImg.reset();
		}

		size_t numBodies = bodyFrame.get_num_bodies();
		ofLogVerbose(__FUNCTION__) << numBodies << " bodies found!";

		if (this->bUpdateBodiesWorld)
		{
			this->bodySkeletons.resize(numBodies);

			for (size_t i = 0; i < numBodies; i++)
			{
				k4abt_skeleton_t skeleton = bodyFrame.get_body_skeleton(i);
				uint32_t id = bodyFrame.get_body_id(i);

				this->bodySkeletons[i].id = id;
						
				for (size_t j = 0; j < K4ABT_JOINT_COUNT; ++j)
				{
					this->bodySkeletons[i].joints[j].position = toGlm(skeleton.joints[j].position);
					this->bodySkeletons[i].joints[j].orientation = toGlm(skeleton.joints[j].orientation);
					this->bodySkeletons[i].joints[j].confidenceLevel = skeleton.joints[j].confidence_level;

					if (this->bUpdateBodiesImage)
					{
						try
						{
							k4a_float2_t projPos;
							calibration.convert_3d_to_2d(skeleton.joints[j].position, K4A_CALIBRATION_TYPE_DEPTH, this->imageType, &projPos);
							this->bodySkeletons[i].joints[j].projPos = toGlm(projPos);
						}
						catch (const k4a::error& e)
						{
							ofLogError(__FUNCTION__) << e.what();
						}
					}
				}
			}
		}

		// Release body frame once we're finished.
		bodyFrame.reset();
	}

	void BodyTracker::updateTextures()
	{
		if (this->bUpdateBodyIndex && this->bodyIndexPix.isAllocated())
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

	const std::vector<BodySkeleton>& BodyTracker::getBodySkeletons() const
	{
		return this->bodySkeletons;
	}
}