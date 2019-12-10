#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	auto kinectSettings = ofxAzureKinect::DeviceSettings();
	kinectSettings.synchronized = false;
	kinectSettings.depthMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	kinectSettings.updateIr = false;
	kinectSettings.updateColor = false;
	//kinectSettings.colorResolution = K4A_COLOR_RESOLUTION_1080P;
	kinectSettings.updateBodies = true;
	kinectSettings.updateWorld = true;
	kinectSettings.updateVbo = false;
	if (this->kinectDevice.open(kinectSettings))
	{
		this->kinectDevice.startCameras();
	}

	// Load shader.
	auto shaderSettings = ofShaderSettings();
	shaderSettings.shaderFiles[GL_VERTEX_SHADER] = "shaders/render.vert";
	shaderSettings.shaderFiles[GL_FRAGMENT_SHADER] = "shaders/render.frag";
	shaderSettings.intDefines["BODY_INDEX_MAP_BACKGROUND"] = K4ABT_BODY_INDEX_MAP_BACKGROUND;
	shaderSettings.bindDefaults = true;
	if (this->shader.setup(shaderSettings))
	{
		ofLogNotice(__FUNCTION__) << "Success loading shader!";
	}

	// Setup vbo.
	std::vector<glm::vec3> verts(1);
	this->pointsVbo.setVertexData(verts.data(), verts.size(), GL_STATIC_DRAW);
}

//--------------------------------------------------------------
void ofApp::exit()
{
	this->kinectDevice.close();
}

//--------------------------------------------------------------
void ofApp::update()
{

}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(0);

	if (this->kinectDevice.isStreaming())
	{
		this->kinectDevice.getBodyIndexTex().draw(0, 0, 360, 360);
	}

	this->camera.begin();
	{
		ofPushMatrix();
		{
			ofRotateXDeg(180);

			ofEnableDepthTest();

			constexpr int kMaxBodies = 6;
			int bodyIDs[kMaxBodies];
			int i = 0;
			while (i < this->kinectDevice.getNumBodies())
			{
				bodyIDs[i] = this->kinectDevice.getBodyIDs()[i];
				++i;
			}
			while (i < kMaxBodies)
			{
				bodyIDs[i] = 0;
				++i;
			}

			this->shader.begin();
			{
				this->shader.setUniformTexture("uDepthTex", this->kinectDevice.getDepthTex(), 1);
				this->shader.setUniformTexture("uBodyIndexTex", this->kinectDevice.getBodyIndexTex(), 2);
				this->shader.setUniformTexture("uWorldTex", this->kinectDevice.getDepthToWorldTex(), 3);
				this->shader.setUniform2i("uFrameSize", this->kinectDevice.getDepthTex().getWidth(), this->kinectDevice.getDepthTex().getHeight());
				this->shader.setUniform1iv("uBodyIDs", bodyIDs, kMaxBodies);

				int numPoints = this->kinectDevice.getDepthTex().getWidth() * this->kinectDevice.getDepthTex().getHeight();
				this->pointsVbo.drawInstanced(GL_POINTS, 0, 1, numPoints);
			}
			this->shader.end();

			ofDisableDepthTest();

			auto& bodySkeletons = this->kinectDevice.getBodySkeletons();
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
				this->skeletonMesh.setMode(OF_PRIMITIVE_LINES);
				auto& vertices = this->skeletonMesh.getVertices();
				vertices.resize(50);
				int vdx = 0;

				// Spine.
				vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_PELVIS].position);
				vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SPINE_NAVAL].position);

				vertices[vdx++] = toGlm(skeleton.joints[K4ABT_JOINT_SPINE_NAVAL].position);
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

				this->skeletonMesh.draw();
			}
		}
		ofPopMatrix();
	}
	this->camera.end();

	ofDrawBitmapStringHighlight(ofToString(ofGetFrameRate(), 2) + " FPS", 10, 20);
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
void ofApp::mouseDragged(int x, int y, int button){

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
