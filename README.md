# Introduction

packages for large-scale metrology project. 

# Dependency

To install GoSDK dependency, the following steps are required: 

1. With a Serial Number of a device, register as a customer at the [LMI website](http://downloads.lmi3d.com/)
2. Download the SDK from the [LMI website](http://downloads.lmi3d.com/) (file 14400-4.2.5.17_SOFTWARE_GO_SDK.zip)
3. Uncompress GoSDK
4. To switch Build Type between "Debug" and "Release", you need to adjust three files in total.[ziqi Chai's method]https://github.com/ZiqiChai/gocator_3x00)

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

# TODO list

[-] publisher

[-] subscriber

[-] data from gocator 3210

[-] inf data from gocator 3210 : used conditional removal

[-] gocator 3210 saver: ```roslaunch gocator_3200 gocator_saver.launch``` (space for new shot, "s" to save)

[-] offline-sim done 

[-] added target_generator: ```roslaunch target_generator target_generator.launch```

[-] output format

[-] saver capture after save off

[-] custom target_ball batch

[-] added online target ball measure: ```roslaunch measure_node measure_target_ball```

[-] added online workpiece measure: ```roslaunch measure_node measure_workpiece```

[ ] custom circle radius

[ ] filter redundant plane

[ ] read in custom params for workpiece measure

[-] test icp with 1/2.ply on sim

[-] visualize saved ply: ```roslaunch gocator_publisher gocator_publisher.launch```

[-] update pcl and vtk

[-] serial points measuring support

[-] send snap request with keyboard

[ ] QT user interface

[ ] publish transform

[-] ros param setting from roslaunch 
