#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	auto kinectSettings = ofxAzureKinect::DeviceSettings();
	kinectSettings.updateIr = false;
	kinectSettings.updateColor = true;
	kinectSettings.colorResolution = K4A_COLOR_RESOLUTION_1080P;
	kinectSettings.updateVbo = true;
	if (this->kinectDevice.open(kinectSettings))
	{
		this->kinectDevice.startCameras();
	}
	glPointSize(2);
	cam.rotateDeg(160, glm::vec3(0, 1, 0));
	cam.rotateDeg(180, glm::vec3(0, 0, 1));

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
	ofBackground(128);
	ofEnableDepthTest();
	if (this->kinectDevice.isStreaming())
	{
		this->cam.begin();
		{
			ofDrawAxis(100.0f);

			if (this->kinectDevice.getColorInDepthTex().isAllocated())
			{
				this->kinectDevice.getColorInDepthTex().bind();
			}
			this->kinectDevice.getPointCloudVbo().draw(
				GL_POINTS,
				0, this->kinectDevice.getPointCloudVbo().getNumVertices());
			if (this->kinectDevice.getColorInDepthTex().isAllocated())
			{
				this->kinectDevice.getColorInDepthTex().unbind();
			}
		}
		this->cam.end();
	}
	ofDisableDepthTest();
	ofDrawBitmapString(ofToString(ofGetFrameRate(), 2) + " FPS", 10, 20);
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
