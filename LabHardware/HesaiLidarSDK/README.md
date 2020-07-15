# Dependencies
## Clone
```
	git clone https://github.com/HesaiTechnology/HesaiLidarSDK.git --recursive
```

## Lidar only
```
	sudo apt install cmake libproj-dev libpcap-dev libboost-all-dev libyaml-cpp-dev libjpeg-dev libgdal-dev libpq-dev libvtk6-dev libvtk6-qt-dev libpcl-dev 
```

## Camera + Lidar

```
	sudo apt install cmake libproj-dev libcv-dev libpcap-dev libboost-all-dev libyaml-cpp-dev libjpeg-dev libgdal-dev libpq-dev libvtk6-dev libvtk6-qt-dev libpcl-dev 
```
# Out of source build

Make a directory somewhere out of your source directory. Let's say:
```
	mkdir build
	cd build 
```
Lidar only
```
	cmake <path to>/HesaiLidarSDK/  
```
Lidar + Camera
```
	cmake -DCamera_Enable=ON <path to>/HesaiLidarSDK/
```
Lidar + Camera + Samples
```
	cmake <path to>/HesaiLidarSDK/ -DENABLE_SAMPLE=ON
```
Then build the projects,
```
	make -j$(nproc)
```
Install HesaiLidarSDK to CMAKE_INSTALL_PREFIX, include (1) Headers (2) libhesaiLidarSDK.so in build/lib (3) binaries in build/bin
```
	make install
```

# Add to your project
```
	add_subdirectory(<path to>HesaiLidarSDK)

	target_link_libraries(${YOUR_PROJECT_NAME}
		...
		hesaiLidarSDK
		...
	)

```

# Note

##1. When compiling with camera functions, 
Add this line to your CMakeLists.txt
```
	add_definitions(-DHESAI_WITH_CAMERA)
```
And add '-DCamera_Enable=ON' to your cmake command line,
``` 
	cmake -DCamera_Enable=ON <your project source directory>
```

