# Multitarget-tracker-ROS
The ROS version of the Multitarget-tracker project.
 More details about the Multitarget-tracker repository refers to [this](https://github.com/Smorodov/Multitarget-tracker).
 
Thanks to Smorodov for sharing his excellent projects!

#### Usage
Nodes: `multitarget_tracker_node`
You can use launch file to configure the node parameters and launch the nodes we want.
```
$ roslaunch multitarget_tracker_ros multitarget_tracker_ros.launch
```
You can specify the input image topics in launch file or in `multitarget_tracker_node.cpp` file.


#### License
GNU GPLv3: http://www.gnu.org/licenses/gpl-3.0.txt 
