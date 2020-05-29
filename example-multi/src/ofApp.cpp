#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);
	ofSetVerticalSync(false);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	// The following will start all connected devices as standalone (no sync).
	//this->setupStandalone();

	// The following will assign sync to devices based on serial number.
	this->setupMasterSubordinate();

	// Add FPS counter for each device.
	this->fpsCounters.resize(this->kinectDevices.size());
}

//--------------------------------------------------------------
void ofApp::setupStandalone()
{
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
void ofApp::setupMasterSubordinate()
{
	// Make sure to replace the following serials by the ones on your devices.
	const std::string serialMaster = "000224694712";
	const std::string serialSubordinate = "000569192412";

	auto kinectSettings = ofxAzureKinect::DeviceSettings();
	kinectSettings.colorResolution = K4A_COLOR_RESOLUTION_720P;
	kinectSettings.syncImages = true;
	kinectSettings.updateWorld = false;

	// Open Master device.
	kinectSettings.deviceSerial = serialMaster;
	kinectSettings.wiredSyncMode = K4A_WIRED_SYNC_MODE_MASTER;
	{
		auto device = std::make_shared<ofxAzureKinect::Device>();
		if (device->open(kinectSettings))
		{
			this->kinectDevices.push_back(device);
			device->startCameras();
		}
	}

	// Open Subordinate device.
	kinectSettings.deviceSerial = serialSubordinate;
	kinectSettings.wiredSyncMode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
	//kinectSettings.subordinateDelayUsec = 100;
	{
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
	for (int i = 0;i < this->kinectDevices.size(); ++i)
	{
		if (this->kinectDevices[i]->isFrameNew())
		{
			this->fpsCounters[i].newFrame();
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(128);

	int x = 0;
	for (int i = 0; i < this->kinectDevices.size(); ++i)
	{
		auto device = this->kinectDevices[i];
		if (device->isStreaming())
		{
			device->getColorTex().draw(x, 0, 640, 360);
			device->getDepthTex().draw(x, 360, 320, 320);
			device->getIrTex().draw(x + 320, 360, 320, 320);

			ofDrawBitmapStringHighlight(ofToString(this->fpsCounters[i].getFps(), 2) + " FPS", x + 10, 350, device->isFrameNew() ? ofColor::red : ofColor::black);

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
