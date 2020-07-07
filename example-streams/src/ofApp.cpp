#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	if (this->kinectDevice.open())
	{
		auto kinectSettings = ofxAzureKinect::DeviceSettings();
		kinectSettings.syncImages = false;
		kinectSettings.updateWorld = false;
		this->kinectDevice.startCameras(kinectSettings);
	}
}

//--------------------------------------------------------------
void ofApp::exit()
{
	this->kinectDevice.close();
}

//--------------------------------------------------------------
void ofApp::update()
{
	if (this->kinectDevice.isFrameNew())
	{
		this->kinectFps.newFrame();
	}
}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(128);

	if (this->kinectDevice.isStreaming())
	{
		this->kinectDevice.getColorTex().draw(0, 0, 1280, 720);
		this->kinectDevice.getDepthTex().draw(1280, 0, 360, 360);
		this->kinectDevice.getIrTex().draw(1280, 360, 360, 360);
	}

	std::ostringstream oss;
	oss << std::fixed << std::setprecision(2)
		<< "APP: " << ofGetFrameRate() << " FPS" << std::endl
		<< "K4A: " << kinectFps.getFps() << " FPS";
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
