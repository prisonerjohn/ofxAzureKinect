#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
	//ofSetLogLevel(OF_LOG_VERBOSE);

	ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

	bRecord = false;
	bPlayback = false;
	filename = "";

	openDevice();
}

//--------------------------------------------------------------
void ofApp::exit()
{
	kinectDevice.close();
	kinectPlayback.close();
}

//--------------------------------------------------------------
void ofApp::update()
{
	if (kinectDevice.isFrameNew())
	{
		fpsDevice.newFrame();
	}
	if (kinectPlayback.isFrameNew())
	{
		fpsPlayback.newFrame();
	}
}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(128);

	if (bPlayback)
	{
		if (kinectPlayback.isStreaming())
		{
			kinectPlayback.getColorTex().draw(0, 0, 1280, 720);
			kinectPlayback.getDepthTex().draw(1280, 0, 360, 360);
			kinectPlayback.getIrTex().draw(1280, 360, 360, 360);
		}
	}
	else
	{
		if (kinectDevice.isStreaming())
		{
			kinectDevice.getColorTex().draw(0, 0, 1280, 720);
			kinectDevice.getDepthTex().draw(1280, 0, 360, 360);
			kinectDevice.getIrTex().draw(1280, 360, 360, 360);
		}
	}

	std::ostringstream oss;
	oss << std::fixed << std::setprecision(2)
		<< (bPlayback ? "PLAYBACK" : "DEVICE") << std::endl
		<< std::endl
		<< "APP: " << ofGetFrameRate() << " FPS" << std::endl
		<< "K4A: " << (bPlayback ? fpsPlayback.getFps() : fpsDevice.getFps()) << " FPS" << std::endl
		<< std::endl
		<< "[TAB] : toggle mode" << std::endl
		<< "[SPACE] : " << (bPlayback ? "open file" : "toggle recording");
	ofDrawBitmapStringHighlight(oss.str(), 10, 20);
}

//--------------------------------------------------------------
void ofApp::openDevice()
{
	// Close the playback stream.
	closePlayback();

	// Open and start the live device stream.
	if (kinectDevice.open())
	{
		auto deviceSettings = ofxAzureKinect::DeviceSettings();
		deviceSettings.colorResolution = ofxAzureKinect::ColorResolution::K4A_COLOR_RESOLUTION_720P;
		deviceSettings.syncImages = false;
		deviceSettings.updateWorld = false;
		kinectDevice.startCameras(deviceSettings);
	}
}

//--------------------------------------------------------------
void ofApp::closeDevice()
{
	kinectDevice.close();
}

//--------------------------------------------------------------
void ofApp::openPlayback()
{
	// Select a video file to play.
	auto result = ofSystemLoadDialog("Select an MKV Kinect recorder file:");
	if (result.bSuccess)
	{
		// Close the live device stream.
		closeDevice();

		// Close the playback stream.
		closePlayback();

		// Open and start the playback stream.
		filename = result.fileName;
		if (kinectPlayback.open(result.filePath))
		{
			auto playbackSettings = ofxAzureKinect::PlaybackSettings();
			kinectPlayback.startPlayback(playbackSettings);
		}
		else
		{
			ofLogError(__FUNCTION__) << "Could not open file " << filename;
		}
	}
}

//--------------------------------------------------------------
void ofApp::closePlayback()
{
	kinectPlayback.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
	if (key == OF_KEY_TAB)
	{
		bPlayback = !bPlayback;
		bPlayback ? openPlayback() : openDevice();
		bRecord = false;
	}
	if (key == ' ')
	{
		if (bPlayback)
		{
			openPlayback();
		}
		else
		{
			bRecord = !bRecord;
			bRecord ? kinectDevice.startRecording() : kinectDevice.stopRecording();
		}
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
