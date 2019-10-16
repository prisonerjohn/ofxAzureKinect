# ofxAzureKinect

ofxAddon that allows you to use [Azure Kinect](https://azure.microsoft.com/en-us/services/kinect-dk/) in [openFrameworks](https://github.com/openframeworks/openFrameworks).

* Get depth, color, depth to world, and color in depth frames as `ofPixels` or `ofTexture`.
* Get point cloud VBO with texture coordinates in depth space.
* Get body tracking skeleton and index texture.
* More coming soon... (undistort that crazy fisheye frame, read IMU values, sync between multi-devices, etc.)

## Installation

* Install the [Azure Kinect Sensor SDK](https://docs.microsoft.com/en-us/azure/Kinect-dk/sensor-sdk-download).
* Install the [Azure Kinect Body Tracking SDK](https://docs.microsoft.com/en-us/azure/Kinect-dk/body-sdk-download).
* Add an environment variable for `AZUREKINECT_SDK` and set it to the Sensor SDK installation path (no trailing slash). The default is `C:\Program Files\Azure Kinect SDK v1.3.0`.
* Add an environment variable for `AZUREKINECT_BODY_SDK` and set it to the Body SDK installation path (no trailing slash). The default is `C:\Program Files\Azure Kinect Body Tracking SDK`.

    ![Environment Variables](Install-EnvVars.png)

* Add the path to the Sensor SDK `bin` folder to the `PATH` variable. The default is `%AZUREKINECT_SDK%\sdk\windows-desktop\amd64\release\bin`.
* Add the path to the Body SDK `bin` folder to the `PATH` variable. The default is `%AZUREKINECT_BODY_SDK%\sdk\windows-desktop\amd64\release\bin`.
* Add the path to the Body SDK `tools` folder to the `PATH` variable. The default is `%AZUREKINECT_BODY_SDK%\tools`.

    ![Path](Install-Path.png)


* Clone this repository in your openFrameworks `addons` folder.
* You can then use the OF Project Generator to generate projects with the appropriate headers and libraries included. ✌️
* Note that if you want to use body tracking, you will need to copy the cuDNN model file `dnn_model_2_0.onnx` from the Body SDK `tools` folder into your project's `bin` folder!

## Compatibility

Tested with: 
* openFrameworks 0.10.x
* Windows 10
* Visual Studio 2017 / 2019

## Examples

* `example-streams` demonstrates how to get depth, color, infrared textures from the device.
* `example-pointCloud` demonstrates how to draw the basic point cloud VBO from the device.
* `example-shader` demonstrates how to reconstruct a point cloud using LUTs in a shader.
* `example-bodies` demonstrates how to get the body tracking index texture, and skeleton joint information.
