# Introduction

ROS packages for large-scale metrology project, which extracts circle and sphere feature online with gocator_3210 and ABB robot. 

# Dependency

To install GoSDK dependency, download the following repo: 

[gocator_interface](https://github.com/Logan-Shi/gocator_interface)

and set up gocater_interface with instructions under [README](https://github.com/Logan-Shi/gocator_interface/blob/master/README.md)

# Install

after dependency is set up, installation can begin.

0. ```mkdir -p workspace/src && cd workspace/src```
1. ```git clone git@github.com:Logan-Shi/large-scale-variable-poses-online-measuring-system.git```
2. ```cd .. && catkin_make```
3. ```source devel/setup.bash```

# Current Supported function

1. gocator 3210 point cloud capture: ```roslaunch gocator_3200 gocator_saver.launch``` (space for new shot, "s" to save)

2. can generate ideal target ball ply file: ```roslaunch target_generator target_generator.launch```

3. added online target ball measure with customizable params: ```roslaunch measure_node measure_target_ball```

4. added online workpiece measure with customizable params: ```roslaunch measure_node measure_workpiece```

5. can visualize saved ply using rviz: ```roslaunch gocator_publisher gocator_publisher.launch```

# Measure results

system achieved 0.1mm accuracy under the range of 640mm at the moment. 

![gif](measure.gif)
