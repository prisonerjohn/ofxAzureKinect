#include "BodyTracker.h"

namespace ofxAzureKinect
{

	BodyTracker::BodyTracker(k4a_calibration_t calibration, k4abt_tracker_configuration_t config)
	{
		k4abt_tracker_create(&calibration, config, &tracker);
		active = true;
		load_shaders();
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

				// Update the Body Index Map ofPixels
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
				this->skeletons.resize(num_bodies);
				this->bodyIDs.resize(num_bodies);
				skeleton_meshes.clear();
				for (size_t i = 0; i < num_bodies; i++)
				{
					k4abt_skeleton_t skeleton;
					k4abt_frame_get_body_skeleton(bodyFrame, i, &skeleton);
					this->skeletons[i] = skeleton;
					uint32_t id = k4abt_frame_get_body_id(bodyFrame, i);
					this->bodyIDs[i] = id;

					skeleton_meshes.push_back(new ofVboMesh());
					skeleton_meshes[skeleton_meshes.size() - 1]->setMode(OF_PRIMITIVE_LINES);
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

	void BodyTracker::set_joint_smoothing(float val)
	{
		joint_smoothing = val;
		k4abt_tracker_set_temporal_smoothing(tracker, joint_smoothing);
	}

	float BodyTracker::get_joint_smoothing()
	{
		return joint_smoothing;
	}

	// The value for each pixel represents which body the pixel belongs to.
	// It can be either background (K4ABT_BODY_INDEX_MAP_BACKGROUND)
	// or the index of a detected k4abt_body_t.
	// https://docs.microsoft.com/en-us/azure/kinect-dk/body-index-map
	void BodyTracker::draw_body_map(int x, int y, int w, int h)
	{
		// Draw the 2D Map of all Detected Bodies
		bodyIndexTex.draw(x, y, w, h);
	}

	void BodyTracker::draw_point_clouds(ofTexture depth_texture, ofTexture depth_to_world_texture)
	{
		ofEnableDepthTest();
		constexpr int kMaxBodies = 6;
		int _bodyIDs[kMaxBodies];
		int i = 0;
		while (i < num_bodies)
		{
			_bodyIDs[i] = bodyIDs[i];
			++i;
		}
		while (i < kMaxBodies)
		{
			_bodyIDs[i] = 0;
			++i;
		}

		this->shader.begin();
		{
			this->shader.setUniformTexture("uDepthTex", depth_texture, 1);
			this->shader.setUniformTexture("uBodyIndexTex", bodyIndexTex, 2);
			this->shader.setUniformTexture("uWorldTex", depth_to_world_texture, 3);
			this->shader.setUniform2i("uFrameSize", depth_texture.getWidth(), depth_texture.getHeight());
			this->shader.setUniform1iv("uBodyIDs", _bodyIDs, kMaxBodies);

			int numPoints = depth_texture.getWidth() * depth_texture.getHeight();
			this->pointsVbo.drawInstanced(GL_POINTS, 0, 1, numPoints);
		}
		this->shader.end();
		ofDisableDepthTest();
	}

	void BodyTracker::draw_skeletons()
	{
		for (int i = 0; i < num_bodies; i++)
		{
			draw_skeleton(i);
		}
	}

	void BodyTracker::draw_skeleton(int id)
	{
		if (id >= num_bodies)
		{
			ofLogError(__FUNCTION__) << "Invaild Body Index {" << id << "}. Body does not exist.";
			return;
		}
		ofPushStyle();

		auto skeleton = skeletons[id];
		// Draw Joints
		for (int i = 0; i < K4ABT_JOINT_COUNT; ++i)
		{
			auto joint = skeleton.joints[i];
			ofPushMatrix();
			{
				glm::mat4 transform = glm::translate(toGlm(joint.position)) * glm::toMat4(toGlm(joint.orientation));
				ofMultMatrix(transform);
				ofNoFill();
				ofDrawBox(15);
				ofDrawAxis(75.0f);

				if (i == K4ABT_JOINT_SPINE_CHEST)
				{
					ofPushMatrix();
					ofTranslate(500, 0, 0);
					ofDrawBitmapStringHighlight(ofToString(bodyIDs[id]), 0, 0);
					ofPopMatrix();
				}
			}
			ofPopMatrix();
		}

		// Draw Bones
		auto &vertices = skeleton_meshes[id]->getVertices();
		vertices.resize(50);

		int vdx = 0;

		// Spine.
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_PELVIS].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SPINE_NAVEL].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SPINE_NAVEL].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_NECK].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_NECK].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HEAD].position);

		// Head.
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HEAD].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_NOSE].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_NOSE].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_EYE_LEFT].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_EYE_LEFT].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_EAR_LEFT].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_NOSE].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_EYE_RIGHT].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_EYE_RIGHT].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_EAR_RIGHT].position);

		// Left Leg.
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_PELVIS].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HIP_LEFT].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HIP_LEFT].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_KNEE_LEFT].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_KNEE_LEFT].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ANKLE_LEFT].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ANKLE_LEFT].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_FOOT_LEFT].position);

		// Right leg.
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_PELVIS].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HIP_RIGHT].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_HIP_RIGHT].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_KNEE_RIGHT].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_KNEE_RIGHT].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ANKLE_RIGHT].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ANKLE_RIGHT].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_FOOT_RIGHT].position);

		// Left arm.
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_NECK].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_CLAVICLE_LEFT].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_CLAVICLE_LEFT].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ELBOW_LEFT].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ELBOW_LEFT].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_WRIST_LEFT].position);

		// Right arm.
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_NECK].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_CLAVICLE_RIGHT].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_CLAVICLE_RIGHT].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT].position);

		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT].position);
		vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].position);

		ofSetColor(200);
		skeleton_meshes[id]->draw();

		ofPopStyle();
	}

	//--------------------------------------------------------------
	void BodyTracker::load_shaders()
	{
		auto shaderSettings = ofShaderSettings();
		shaderSettings.shaderFiles[GL_VERTEX_SHADER] = "shaders/render.vert";
		shaderSettings.shaderFiles[GL_FRAGMENT_SHADER] = "shaders/render.frag";
		shaderSettings.intDefines["BODY_INDEX_MAP_BACKGROUND"] = K4ABT_BODY_INDEX_MAP_BACKGROUND;
		shaderSettings.bindDefaults = true;

		if (this->shader.setup(shaderSettings))
		{
			ofLogNotice(__FUNCTION__) << "Success loading shader!";
		}
		else
		{
			ofLogError(__FUNCTION__) << "Could not load shader. Check that they are in the /bin/data/shader directory.";
		}

		// Setup PointCloud VBO
		std::vector<glm::vec3> verts(1);
		this->pointsVbo.setVertexData(verts.data(), verts.size(), GL_STATIC_DRAW);
	}

	size_t BodyTracker::get_num_bodies() const
	{
		return num_bodies;
	}

	const vector<k4abt_skeleton_t> &BodyTracker::get_skeletons() const
	{
		return skeletons;
	}

	const k4abt_skeleton_t &BodyTracker::get_skeleton(int i) const
	{
		return skeletons[i];
	}

	const ofPixels &BodyTracker::get_body_index_map_pix() const
	{
		return bodyIndexPix;
	}

	const ofTexture &BodyTracker::get_body_index_map_tex() const
	{
		return bodyIndexTex;
	}

} // namespace ofxAzureKinect