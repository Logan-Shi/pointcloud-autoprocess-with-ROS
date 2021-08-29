# Introduction

ROS packages for large-scale metrology project, which extracts circle and sphere feature online with gocator_3210 and ABB robot. 

# Dependency

To install GoSDK dependency, download the following repo: 

[modified_GO_SDK](https://github.com/Logan-Shi/GO_SDK)

1. first build the sdk: 

```
run ./install.bash
```
2. then set the dir manually at /gocator_3200/CmakeLists.txt:

```
SET(GO_SDK_4 <dir>)
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
