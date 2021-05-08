# Introduction

ROS packages for large-scale metrology project, which extracts circle and sphere feature online with gocator_3210 and ABB robot. 

# Dependency

To install GoSDK dependency, the following steps are required: 

1. With a Serial Number of a device, register as a customer at the [LMI website](http://downloads.lmi3d.com/)
2. Download the SDK from the [LMI website](http://downloads.lmi3d.com/) (file 14400-4.2.5.17_SOFTWARE_GO_SDK.zip)
3. Uncompress GoSDK
4. To switch Build Type between "Debug" and "Release", you need to adjust three files in total. [modified_GO_SDK](https://github.com/Logan-Shi/GO_SDK)

```
/GO_SDK/Platform/kApi/kApi-Linux_X64.mk
/GO_SDK/Gocator/GoSdk/GoSdk-Linux_X64.mk
/GO_SDK/Gocator/GoSdk/GoSdkExample-Linux_X64.mk
```
5. Build GoSDK
```shell 
$ cd GO_SDK/Gocator
$ make -f GoSdk-Gnu.mk 
```

# Supported function

1. gocator 3210 point cloud capture: ```roslaunch gocator_3200 gocator_saver.launch``` (space for new shot, "s" to save)

2. can generate ideal target ball ply file: ```roslaunch target_generator target_generator.launch```

3. added online target ball measure with customizable params: ```roslaunch measure_node measure_target_ball```

4. added online workpiece measure with customizable params: ```roslaunch measure_node measure_workpiece```

5. can visualize saved ply using rviz: ```roslaunch gocator_publisher gocator_publisher.launch```

# Kinematic calibration

point clouds captured using this repo can be stitched with ABB forward kinematics, which is implemented with MATLAB in [repo](https://github.com/Ssz990220/Kinematic_Param_Calibration)

# Measure results

system achieved 0.1mm accuracy under the range of 640mm at the moment. 
