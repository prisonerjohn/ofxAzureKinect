#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	// Open Kinect.
	if (this->kinectDevice.open())
	{
		auto kinectSettings = ofxAzureKinect::DeviceSettings();
		kinectSettings.updateIr = false;
		kinectSettings.updateColor = true;
		kinectSettings.colorResolution = K4A_COLOR_RESOLUTION_1080P;
		kinectSettings.updateVbo = false;
		this->kinectDevice.startCameras(kinectSettings);
	}

	// Load shader.
	auto shaderSettings = ofShaderSettings();
	shaderSettings.shaderFiles[GL_VERTEX_SHADER] = "shaders/render.vert";
	shaderSettings.shaderFiles[GL_GEOMETRY_SHADER] = "shaders/render.geom";
	shaderSettings.shaderFiles[GL_FRAGMENT_SHADER] = "shaders/render.frag";
	shaderSettings.bindDefaults = true;
	if (this->shader.setup(shaderSettings))
	{
		ofLogNotice(__FUNCTION__) << "Success loading shader!";
	}

	// Setup vbo.
	std::vector<glm::vec3> verts(1);
	this->vbo.setVertexData(verts.data(), verts.size(), GL_STATIC_DRAW);

	this->pointSize = 3.0f;
	this->useColorSpace = false;
}

//--------------------------------------------------------------
void ofApp::exit()
{
	this->kinectDevice.close();
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(0);

	if (this->kinectDevice.isStreaming())
	{
		this->cam.begin();
		{
			ofEnableDepthTest();

			ofDrawAxis(100.0f);

			ofPushMatrix();
			{
				ofRotateXDeg(180);

				this->shader.begin();
				{
					this->shader.setUniform1f("uSpriteSize", this->pointSize);

					int numPoints;
					
					if (this->useColorSpace)
					{
						this->shader.setUniformTexture("uDepthTex", this->kinectDevice.getDepthInColorTex(), 1);
						this->shader.setUniformTexture("uWorldTex", this->kinectDevice.getColorToWorldTex(), 2);
						this->shader.setUniformTexture("uColorTex", this->kinectDevice.getColorTex(), 3);
						this->shader.setUniform2i("uFrameSize", this->kinectDevice.getColorTex().getWidth(), this->kinectDevice.getColorTex().getHeight());
					
						numPoints = this->kinectDevice.getColorTex().getWidth() * this->kinectDevice.getColorTex().getHeight();
					}
					else
					{
						this->shader.setUniformTexture("uDepthTex", this->kinectDevice.getDepthTex(), 1);
						this->shader.setUniformTexture("uWorldTex", this->kinectDevice.getDepthToWorldTex(), 2);
						this->shader.setUniformTexture("uColorTex", this->kinectDevice.getColorInDepthTex(), 3);
						this->shader.setUniform2i("uFrameSize", this->kinectDevice.getDepthTex().getWidth(), this->kinectDevice.getDepthTex().getHeight());
					
						numPoints = this->kinectDevice.getDepthTex().getWidth() * this->kinectDevice.getDepthTex().getHeight();
					}

					this->vbo.drawInstanced(GL_POINTS, 0, 1, numPoints);
				}
				this->shader.end();
			}
			ofPopMatrix();
		}
		this->cam.end();
	}

	ofDrawBitmapStringHighlight(ofToString(ofGetFrameRate(), 2) + " FPS", 10, 20);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
	if (key == OF_KEY_UP)
	{
		this->pointSize *= 2;
	}
	else if (key == OF_KEY_DOWN)
	{
		this->pointSize /= 2;
	}
	else if (key == ' ')
	{
		this->useColorSpace ^= 1;
	}
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
