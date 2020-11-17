#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup()
{
    ofLogNotice(__FUNCTION__) << "Found " << ofxAzureKinect::Device::getInstalledCount() << " installed devices.";

    auto settings = ofxAzureKinect::DeviceSettings();
    settings.colorResolution = K4A_COLOR_RESOLUTION_1080P;
    settings.depthMode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    settings.colorFormat = K4A_IMAGE_FORMAT_COLOR_MJPG; // BRGA32 not supported for recording
    settings.cameraFps = K4A_FRAMES_PER_SECOND_30;
    settings.syncImages = true;
    settings.updateWorld = false;
    settings.enableIMU = true;
    if (this->sensor.open())
    {
        this->sensor.startCameras(settings);
    }
}

//--------------------------------------------------------------
void ofApp::exit()
{
    this->sensor.close();
}

//--------------------------------------------------------------
void ofApp::update()
{

    // If we are recording, update the total recording time
    if (sensor.bRecord && ofGetElapsedTimef() > recording_start)
        recording_duration = ofGetElapsedTimef() - recording_start;
    
}

//--------------------------------------------------------------
void ofApp::draw()
{
    ofBackground(128);

    // visualize kinect streams
    if (this->sensor.isStreaming())
    {
        this->sensor.getColorTex().draw(0, 0, 1280, 720);
        this->sensor.getDepthTex().draw(1280, 0, 360, 360);
        this->sensor.getIrTex().draw(1280, 360, 360, 360);
    }

    if (sensor.bRecord)
    {
        draw_recording_animation();
    }

    ofDrawBitmapStringHighlight(ofToString(ofGetFrameRate(), 2) + " FPS", 10, 20);

    stringstream ss;
    ss << "Press SPACEBAR to start/stop a recording." << endl;
    ss << "\tRecordings are saved to the bin/data/ directory." << endl;
    ss << "Press 'f' to toggle fullscreen.";
    ofDrawBitmapStringHighlight(ss.str(), 10, ofGetHeight() - 50);
}

//--------------------------------------------------------------
void ofApp::draw_recording_animation()
{

    float a = ofMap(sin(ofGetElapsedTimef() * 3.5), -1, 1, 0, 120);
    ofColor col;
    string msg;

    float recording_countdown = sensor.getRecordingTimerDelay();
    if (recording_countdown > 0)
    {
        msg = "Starting Recording In: " + ofToString(recording_countdown, 2);
        col = ofColor(120, a);
    }
    if (recording_duration > 0)
    {
        msg = "Recording Time: " + ofToString(recording_duration, 2);
        col = ofColor(255, 0, 0, a);
    }

    ofPushStyle();
    ofSetColor(col);
    ofDrawRectangle(0, 0, ofGetWidth(), ofGetHeight());
    ofPopStyle();

    ofDrawBitmapStringHighlight(msg, 10, 40);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    switch (key)
    {
    case ' ':
    {
        sensor.bRecord = !sensor.bRecord;
        if (sensor.bRecord)
        {
            recording_delay = sensor.getRecordingTimerDelay();
            recording_start = ofGetElapsedTimef() + recording_delay;
        }
        else
        {
            recording_delay = 0;
            recording_start = 0;
            recording_duration = 0;
        }
        break;
    }
    case 'f':
    case 'F':
        ofToggleFullscreen();
        break;
    default:
        break;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg)
{
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)
{
}
