# bayesopt_ros_bridge

bayesopt_ros_bridge is a package that includes bridge software to operate instance of bayesian optimization through ROS msgs.

## Use Case
- Experimental parameter optimization using bayesian optimization during robot experiments
- Integration of a ROS robot system with science experiments
- Lab-automation


## physbo_ros_bridge
- bridge node for communication between PHYSBO and ROS system
- publish PHYSBO data to ROS system (e.g. next_param)
- subscribe evaluation value or flags to write the value to PHYSBO
```
roslaunch bayesopt_ros_bridge physbo_ros_bridge.launch
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


## combo_ros_bridge
- bridge node for communication between COMBO and ROS system
- publish COMBO data to ROS system (e.g. next_param)
- subscribe evaluation value or flags to write the value to COMBO
```
roslaunch bayesopt_ros_bridge combo_ros_bridge.launch
```

### memo
- Result and history data are saved in the dir specified by `policy_save_dir` (`data` dir as defalult).
- if you want to continue an experiment with the previous/existing data, `use_saved_policy=True` should be used.
  - The arg with `False` does not consider the existing data and learning history.
- However, COMBO is not recommended when you want to continue experiments from existing data due to the following issue which was fixed in PHYSBO.
  - https://github.com/issp-center-dev/PHYSBO/issues/44
  - PHYSBO is a candidate for this purpose.
