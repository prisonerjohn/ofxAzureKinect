#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	nearClip.set("Near Clip", 0.25f, 0.0f, 8.0f);
	farClip.set("Far Clip", 3.0f, 0.0f, 8.0f);

	guiPanel.setup("Scaled Depth", "settings.json");
	guiPanel.add(nearClip);
	guiPanel.add(farClip);

	if (kinectDevice.open())
	{
		auto kinectSettings = ofxAzureKinect::DeviceSettings();
		kinectSettings.syncImages = false;
		kinectSettings.updateColor = false;
		kinectSettings.updateWorld = false;
		kinectDevice.startCameras(kinectSettings);
	}
}

//--------------------------------------------------------------
void ofApp::exit()
{
	kinectDevice.close();
}

//--------------------------------------------------------------
void ofApp::update()
{
	if (kinectDevice.isFrameNew())
	{
		kinectFps.newFrame();
	}
}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(128);

	if (kinectDevice.isStreaming())
	{
		const auto& depthPix = kinectDevice.getDepthPix();

		if (!scaledDepthImage.isAllocated())
		{
			scaledDepthImage.allocate(depthPix.getWidth(), depthPix.getHeight(), depthPix.getImageType());
		}
		if (!linearDepthImage.isAllocated())
		{
			linearDepthImage.allocate(depthPix.getWidth(), depthPix.getHeight(), depthPix.getImageType());
		}

		auto& scaledPix = scaledDepthImage.getPixels();
		auto& linearPix = linearDepthImage.getPixels();

		// Convert from meters to millimeters.
		const float nearClipMm = nearClip * 1000;
		const float farClipMm = farClip * 1000;

		for (int i = 0; i < depthPix.size(); ++i)
		{
			if (depthPix[i] > farClipMm)
			{
				scaledPix[i] = linearPix[i] = 0;
			}
			else
			{
				scaledPix[i] = ofMap(depthPix[i], nearClipMm, farClipMm, 0, 255, true);
				linearPix[i] = ofMap(depthPix[i], nearClipMm, farClipMm, 0.0f, 1.0f, true);
			}
		}

		scaledDepthImage.update();
		linearDepthImage.update();

		kinectDevice.getDepthTex().draw(0, 0, 512, 512);
		scaledDepthImage.draw(512, 0, 512, 512);
		linearDepthImage.draw(1024, 0, 512, 512);
	}

	guiPanel.draw();
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
