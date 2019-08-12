# Sensor Fusion Self-Driving Car Course

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

### Welcome to the Sensor Fusion course for self-driving cars.

In this course we will be talking about sensor fusion, whch is the process of taking data from multiple sensors and combining it to give us a better understanding of the world around us. we will mostly be focusing on two sensors, lidar, and radar. By the end we will be fusing the data from these two sensors to track multiple cars on the road, estimating their positions and speed.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar** data is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resoultion imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.


## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
$> cd SFND_Lidar_Obstacle_Detection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

### Windows 

The official [PCL's prebuilt binaries for Windows](http://www.pointclouds.org/downloads/windows.html) are out of date because they're built only for Visual Studio 2008 and 2010.

If you're using _Visual Studio 2017_ and you don't want to compile libraries you can download the [All In One installer](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.9.1) from the official _PCL's GitHub_ repository, but:

1. It will install in your computer lot of things that maybe you won't use never (like drivers for _Kinect_).
2. You can't use it with _Visual Studio 2019_.

The best way for Windows is to build the library from source, and the easiest way is using 
[Microsoft's VCPKG](https://docs.microsoft.com/en-us/cpp/build/vcpkg?view=vs-2019) because this tool will compile all the dependences for you.

To use _VCPKG_ first you have to build it. If you don't have it already, follow the next steps:

```
c:> cd c:\
c:> git clone https://github.com/microsoft/vcpkg.git
c:> cd vcpkg
c:\vcpkg> .\bootstrap-vcpkg.bat
```

Once you've _VCPKG_, you can build _PCL_ with the next command:

```
c:\vcpkg> vcpkg install pcl:x64-windows
```

Once finished (be patient) you've to do three last steps:

1. Add **C:\vcpkg\installed\x64-windows\bin** to your user's _PATH_.
2. Add **C:\vcpkg\installed\x64-windows\debug\bin** to your user's _PATH_.
3. Set the _CMake Toolchain File_ to **c:\vcpkg\scripts\buildsystems\vcpkg.cmake**.

	If you're using Visual _Studio 2019_ and you've installed the [C++ CMake tools for Windows](https://docs.microsoft.com/en-us/cpp/build/cmake-projects-in-visual-studio?view=vs-2019) component, you can do it directly from _Visual Studio 2019_: In _Solution Explorer_ right-click in _CMakeList.txt_ and select *CMake settings for playback*. Then in _General_ tab you have the _CMake Toolchain File_ setting. Set the path and press _Ctrl+S_ to save. _Visual Studio_ will generate the _CMake cache_ automatically.

The first time you generate the _CMake cache_, you'll get the next error:

**Property INTERFACE_LINK_LIBRARIES may not contain link-type keyword "optimized". The INTERFACE_LINK_LIBRARIES property may contain configuration-sensitive generator-expressions which may be used to specify per-configuration rules.**

This is a _PCL's CMake_ configuration file error and, as you can see in https://github.com/PointCloudLibrary/pcl/issues/2989#issuecomment-489433303, the solution is commenting a line in the _PCLConfig.cmake_ file, so:

1. Open **C:\vcpkg\installed\x64-windows\share\pcl\PCLConfig.cmake**
2. Comment the line: **list(APPEND PCL_${COMPONENT}_LIBRARIES "${${LIB}_LIBRARIES}")** adding a #
3. Save the file

And now you are ready to go.

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)
