# ofxAzureKinect

ofxAddon that allows you to use [Azure Kinect](https://azure.microsoft.com/en-us/services/kinect-dk/) in [openFrameworks](https://github.com/openframeworks/openFrameworks).

## Installation

* Install the [Azure Kinect Sensor SDK](https://docs.microsoft.com/en-us/azure/Kinect-dk/sensor-sdk-download).

* Add an environment variable for `AZUREKINECT_SDK` and set it to the installation path. The default on Windows is `C:\Program Files\Azure Kinect SDK v1.1.0\sdk\`.

    ![Environment Variables](Install-EnvVars.png)

* Add the path to the SDK `bin` folder to the `PATH` variable. The default on Windows is `%AZUREKINECT_SDK%\windows-desktop\amd64\release\bin`.

    ![Path](Install-Path.png)

* You can then use the OF Project Generator to generate projects with the appropriate headers and libraries included.
