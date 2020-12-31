#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	if (kinectDevice.open())
	{
		auto deviceSettings = ofxAzureKinect::DeviceSettings();
		deviceSettings.syncImages = false;
		deviceSettings.depthMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
		deviceSettings.updateIr = false;
		deviceSettings.updateColor = true;
		//deviceSettings.colorResolution = K4A_COLOR_RESOLUTION_1080P;
		deviceSettings.updateWorld = true;
		deviceSettings.updateVbo = false;
		kinectDevice.startCameras(deviceSettings);
	
		auto bodyTrackerSettings = ofxAzureKinect::BodyTrackerSettings();
		bodyTrackerSettings.sensorOrientation = K4ABT_SENSOR_ORIENTATION_DEFAULT;
		//bodyTrackerSettings.processingMode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
		bodyTrackerSettings.imageType = K4A_CALIBRATION_TYPE_COLOR;
		bodyTrackerSettings.updateBodiesImage = true;
		kinectDevice.startBodyTracker(bodyTrackerSettings);
	}

	// Load shader.
	auto shaderSettings = ofShaderSettings();
	shaderSettings.shaderFiles[GL_VERTEX_SHADER] = "shaders/render.vert";
	shaderSettings.shaderFiles[GL_FRAGMENT_SHADER] = "shaders/render.frag";
	shaderSettings.intDefines["BODY_INDEX_MAP_BACKGROUND"] = K4ABT_BODY_INDEX_MAP_BACKGROUND;
	shaderSettings.bindDefaults = true;
	if (shader.setup(shaderSettings))
	{
		ofLogNotice(__FUNCTION__) << "Success loading shader!";
	}

	// Setup vbo.
	std::vector<glm::vec3> verts(1);
	pointsVbo.setVertexData(verts.data(), verts.size(), GL_STATIC_DRAW);
}

//--------------------------------------------------------------
void ofApp::exit()
{
	kinectDevice.close();
}

//--------------------------------------------------------------
void ofApp::update()
{

}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(0);

	if (kinectDevice.isStreaming())
	{
		// Scale down the 2D drawings.
		static const float kTargetHeight = 360;
		const auto texSize = glm::vec2(kinectDevice.getBodyIndexTex().getWidth(), kinectDevice.getBodyIndexTex().getHeight());
		const auto texScale = kTargetHeight / texSize.y;

		// Draw the body index texture. 
		// The pixels are not black, their color equals the body ID which is just a low number.
		kinectDevice.getBodyIndexTex().draw(0, 0, texSize.x * texScale, texSize.y * texScale);

		// Draw the projected joints onto the image.
		ofSetColor(ofColor::red);
		for (int i = 0; i < kinectDevice.getNumBodies(); ++i)
		{
			const auto joints = kinectDevice.getBodyJointsProjected(i);
			for (int j = 0; j < joints.size(); ++j)
			{
				ofDrawCircle(toGlm(joints[j]) * texScale, 5.0f);
			}
		}
		ofSetColor(ofColor::white);
	}

	camera.begin();
	{
		ofPushMatrix();
		{
			ofRotateXDeg(180);

			ofEnableDepthTest();

			constexpr int kMaxBodies = 6;
			int bodyIDs[kMaxBodies];
			int i = 0;
			while (i < kinectDevice.getNumBodies())
			{
				bodyIDs[i] = kinectDevice.getBodyIDs()[i];
				++i;
			}
			while (i < kMaxBodies)
			{
				bodyIDs[i] = 0;
				++i;
			}

			shader.begin();
			{
				shader.setUniformTexture("uDepthTex", kinectDevice.getDepthTex(), 1);
				shader.setUniformTexture("uBodyIndexTex", kinectDevice.getBodyIndexTex(), 2);
				shader.setUniformTexture("uWorldTex", kinectDevice.getDepthToWorldTex(), 3);
				shader.setUniform2i("uFrameSize", kinectDevice.getDepthTex().getWidth(), kinectDevice.getDepthTex().getHeight());
				shader.setUniform1iv("uBodyIDs", bodyIDs, kMaxBodies);

				int numPoints = kinectDevice.getDepthTex().getWidth() * kinectDevice.getDepthTex().getHeight();
				pointsVbo.drawInstanced(GL_POINTS, 0, 1, numPoints);
			}
			shader.end();

			ofDisableDepthTest();

			auto& bodySkeletons = kinectDevice.getBodySkeletons();
			for (auto& skeleton : bodySkeletons)
			{
				// Draw joints.
				for (int i = 0; i < K4ABT_JOINT_COUNT; ++i)
				{
					auto joint = skeleton.joints[i];
					ofPushMatrix();
					{
						glm::mat4 transform = glm::translate(toGlm(joint.position)) * glm::toMat4(toGlm(joint.orientation));
						ofMultMatrix(transform);

						ofDrawAxis(50.0f);
					}
					ofPopMatrix();
				}

				// Draw connections.
				skeletonMesh.setMode(OF_PRIMITIVE_LINES);
				auto& vertices = skeletonMesh.getVertices();
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

				skeletonMesh.draw();
			}
		}
		ofPopMatrix();
	}
	camera.end();

	std::ostringstream oss;
	oss << ofToString(ofGetFrameRate(), 2) + " FPS" << std::endl;
	oss << "Joint Smoothing: " << kinectDevice.getBodyTracker().jointSmoothing;
	ofDrawBitmapStringHighlight(oss.str(), 10, 20);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
	if (button == 1)
	{
		kinectDevice.getBodyTracker().jointSmoothing = ofMap(x, 0, ofGetWidth(), 0.0f, 1.0f, true);
	}
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
	
}
