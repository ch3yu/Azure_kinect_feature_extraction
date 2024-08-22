
## Introduction
A growing body of research has been dedicated to <strong>human interaction recognition</strong>, which interprets the bodily movements and coordination between two (or more) individuals. There are already many existing modalities for human interaction recognition, such as RGB cameras and depth sensors. RGB cameras have been used ubiquitously for interaction recognition. While cameras provide an abundance of information, they give rise to the <strong>unauthorized use of personal information</strong>. Depth sensors have ushered in a new area of research focusing on recognizing interactions through <strong>skeletal movements</strong> while prioritizing the protection of privacy. The use of 3D key-point coordinates for skeletons not only improves computational efficiency, but also adds a third dimension that helps to overcome the issue of varying camera angles, a common limitation of camera-based systems.
This work aims to leverage <strong>Azure Kinect Development Kit (DK)</strong> to collect depth data and translate it to skeletal data with <strong>Azure Kinect SDK</strong>, which is one of the scaffolding components of <strong>skeleton-based human interaction recognition</strong> is collect skeletal data through depth sensors.
This work aims to leverage <strong>Azure Kinect Development Kit (DK)</strong> to collect depth data and translate it to skeletal data with <strong>Azure Kinect SDK</strong>, which is one of the scaffolding components of <strong>skeleton-based human interaction recognition</strong>.

## Visualization
The flow of collecting skeletal data is presented below:
1. RGB and depth data of dyadic human interactions are recorded, which are used as ground-truth and translation to skeletal data respectively.
2. We leverage <a href="https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/index.html" title="Azure Kinect SDK">Azure Kinect SDK documentation</a> and <a href="https://microsoft.github.io/Azure-Kinect-Body-Tracking/release/0.9.x/index.html" title="Azure Kinect Body Tracking SDK">Azure Kinect Body Tracking SDK documentation</a> to extract skeletal information (e.g., 3D joint coordinates, velocities, angles, and etc.).
   
:ghost: The diagram below only serves to visualize the procedure of data extraction. It doesn't reflect the real data!
   
![alt text](https://github.com/ch3yu/Azure_kinect_feature_extraction/blob/main/process.png)
## User Intructions
### Prerequisites
The following packages are required for extracting skeletal data collected by Azure Kinect DK, which includes runtime engine and libraries.
- <a href="https://learn.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download" title="Azure Kinect SDK">Azure Kinect SDK</a>
- <a href="https://learn.microsoft.com/en-us/azure/kinect-dk/body-sdk-download" title="Azure Kinect Body Tracking SDK">Azure Kinect Body Tracking SDK</a>
### Folder Structure
To compile the code with the terminal commands provided below, please arrange the folder structure as instructed after downloading packages aforementioned. The only file that can be pulled from this repository is <strong>skeleton_dyadic.c</strong>.
```
.
|----lib                # Directory for static libraries.
        |----k4a.lib
        |----k4abt.lib
        |----k4arecord.lib
|----include            # Directory to store header files.
        |----k4a.h
        |----k4abt.h
        |----k4abttypes
        |----k4atypes
        |----playback
|----src                # Directory for source code and dynamic libraries.
        |----depthengine_2_0.dll
        |----directml.dll
        |----dnn_model_2_0_op11.onnx
        |----k4a.dll
        |----k4abt.dll
        |----k4arecord.dll
        |----onnxruntime.dll
        |----onnxruntime_providers_cuda.dll
        |----onnxruntime_providers_shared.dll
        |----onnxruntime_providers_tensorrt.dll
        |----skeleton_dyadic.c
|----data              # Directory to store depth videos and timestamps
|----result            # Directory to store all skeletal data
```
### Compiling the code
After coordinating the folder structure as instructed, please redirect the command prompt to the home directory (i.e., the directory that contains <strong>lib</strong>, <strong>include</strong>, <strong>src</strong>, <strong>data</strong>, and <strong>result</strong>) to type in the following command.
```
>> gcc -g -c src\skeleton_dyadic.c -o src\skeleton_dyadic.o 
>> gcc -g src\skeleton_dyadic.o -o src\skeleton_dyadic -Llib -lk4a -lk4abt -lk4arecord
```
## Reference
