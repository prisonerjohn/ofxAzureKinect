#include "ofApp.h"

int main()
{
	ofGLFWWindowSettings settings;
	settings.setGLVersion(3, 2);
	settings.setSize(640 * 3, 360 + 320);
	ofCreateWindow(settings);

	ofRunApp(new ofApp());
}
