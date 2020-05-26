#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	int numConnected = ofxAzureKinect::Device::getInstalledCount();
	
	auto kinectSettings = ofxAzureKinect::DeviceSettings();
	kinectSettings.colorResolution = K4A_COLOR_RESOLUTION_720P;
	kinectSettings.syncImages = true;
	kinectSettings.updateWorld = false;
	
	for (int i = 0; i < numConnected; ++i)
	{
		kinectSettings.deviceIndex = i;

		auto device = std::make_shared<ofxAzureKinect::Device>();
		if (device->open(kinectSettings))
		{
			this->kinectDevices.push_back(device);
			device->startCameras();
		}
	}
}

//--------------------------------------------------------------
void ofApp::exit()
{
	for (auto device : this->kinectDevices)
	{
		device->close();
		device.reset();
	}
	this->kinectDevices.clear();
}

//--------------------------------------------------------------
void ofApp::update()
{

}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(128);

	int x = 0;
	for (auto device : this->kinectDevices)
	{
		if (device->isStreaming())
		{
			device->getColorTex().draw(x, 0, 640, 360);
			device->getDepthTex().draw(x, 360, 320, 320);
			device->getIrTex().draw(x + 320, 360, 320, 320);

			x += 640;
		}
	}

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
