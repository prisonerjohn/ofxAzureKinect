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
		auto device = std::make_shared<ofxAzureKinect::Device>();
		if (device->open(i))
		{
			device->startCameras(kinectSettings);

			this->kinectDevices.push_back(device);
		}
	}
}

//--------------------------------------------------------------
void ofApp::setupMasterSubordinate()
{
	auto kinectSettings = ofxAzureKinect::DeviceSettings();
	kinectSettings.colorResolution = K4A_COLOR_RESOLUTION_1080P;
	kinectSettings.syncImages = true;
	kinectSettings.updateWorld = false;

	// Open all device and check sync status.
	int numConnected = ofxAzureKinect::Device::getInstalledCount();
	int syncOutDeviceIndex = -1;
	int syncInOutDeviceIndex = -1;

	int connectedDeviceIndex = 0;
	for (int i = 0; i < numConnected; ++i)
	{
		auto device = std::make_shared<ofxAzureKinect::Device>();
		if (device->open(i))
		{
			this->kinectDevices.push_back(device);
			if (!device->isSyncInConnected() && device->isSyncOutConnected()) {
				syncOutDeviceIndex = connectedDeviceIndex;
			}
			else if (device->isSyncInConnected() && device->isSyncOutConnected()) {
				syncInOutDeviceIndex = connectedDeviceIndex;
			}
			connectedDeviceIndex++;
		}
	}

	int masterDeviceIndex = syncOutDeviceIndex >= 0 ? syncOutDeviceIndex : syncInOutDeviceIndex;

	if (masterDeviceIndex < 0) {
		ofLogWarning() << "No master device is detected. Start streams as standalone mode.";
		for (auto& device : this->kinectDevices) {
			device->startCameras(kinectSettings);
		}
	} else {
		sync.setMasterDevice(kinectDevices[masterDeviceIndex].get());

		// Open Subordinate devices first
		for (int i = 0; i < this->kinectDevices.size(); ++i) {
			this->kinectDevices[i]->setExposureTimeAbsolute(8000);
			if (i != masterDeviceIndex) {
				cerr << "sub device : " << this->kinectDevices[i]->getSerialNumber() << endl;
				kinectSettings.wiredSyncMode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
				kinectSettings.subordinateDelayUsec += 160;
				sync.addSubordinateDevice(kinectDevices[i].get());

				this->kinectDevices[i]->startCameras(kinectSettings);
			}
		}

		// Open Master device
		cerr << "master device : " << this->kinectDevices[masterDeviceIndex]->getSerialNumber() << endl;
		kinectSettings.wiredSyncMode = K4A_WIRED_SYNC_MODE_MASTER;
		kinectSettings.subordinateDelayUsec = 0;
		this->kinectDevices[masterDeviceIndex]->startCameras(kinectSettings);

		sync.start();
	}
}

//--------------------------------------------------------------
void ofApp::exit()
{
	sync.stop();
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

			if (device->isSyncInConnected()) {
				ofDrawBitmapStringHighlight("SyncIn", x + 100, 350, ofColor::blue);
			}
			if (device->isSyncOutConnected()) {
				ofDrawBitmapStringHighlight("SyncOut", x + 200, 350, ofColor::green);
			}
			ofDrawBitmapStringHighlight("DeviceTime : " + ofToString(device->getColorTexDeviceTime().count()), x + 10, 40);

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
