# physbo_ros_bridge

physbo_ros_bridge is a package that includes a bridge software to commanding PHYSBO through ROS msgs.


## PHYSBO
PHYSBO is an optimization tools for PHYsics based on Bayesian Optimization

### doc
https://www.pasums.issp.u-tokyo.ac.jp/physbo/en

### source code
https://github.com/issp-center-dev/PHYSBO


## Install
- PHYSBO requires python3
```
sudo apt install python3-pip
sudo pip3 install rospkg
sudo pip3 install numpy
sudo pip3 install pandas
sudo pip3 install Cython
sudo pip3 install matplotlib
```

- install physbo
  - https://github.com/issp-center-dev/PHYSBO#install


## Usage
### physbo_interface
- test program node for only PHYSBO
- load test candidates (examples/data/candidates.csv)
- visualize mean, std and scores(acquisition functions)
```
cd examples
./test_physbo_interface.py
```

### physbo_ros_bridge
- bridge node for communication between PHYSBO and ROS system
- publish PHYSBO data to ROS system (e.g. next_param)
- subscribe evaluation value or flags to write the value to PHYSBO
```
roslaunch physbo_ros_bridge physbo_ros_bridge.launch
```
and then command the node from another terminal using ROS msgs.
```
rostopic pub -1 /bayesopt_ros_bridge/start_param_search std_msgs/Bool "data: true"
rostopic pub -1 /bayesopt_ros_bridge/result_to_bayesopt std_msgs/Float32 "data: 0.5"
```
repeat this process several times, and then terminate the terminal by Ctrl+C.


### memo
- Result and history data are saved in the dir specified by `policy_save_dir` (`data` dir as defalult).
- if you want to continue an experiment with the previous/existing data, `use_saved_policy=True` should be used.
  - The arg with `False` does not consider the existing data and learning history.
