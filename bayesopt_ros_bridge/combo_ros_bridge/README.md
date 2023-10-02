# combo_ros_bridge

combo_ros_bridge is a package that includes a bridge software to commanding COMBO through ROS msgs.


## COMBO
COMBO: COMmon Bayesian Optimization Library

### doc
https://github.com/tsudalab/combo/blob/master/docs/combo_document.pdf

### source code
- combo
  - https://github.com/tsudalab/combo
  - python2 ver
- combo3 (not recommended due to no maintaince)
  - https://github.com/tsudalab/combo3
  - python3 ver
  - PHYSBO is the next version and candidate for python3

## Install
### for combo
- install combo from source -> https://github.com/tsudalab/combo

### for combo3
- use bug-fix branch
```
cd ~/catkin_ws/src
git clone https://github.com/tsudalab/combo3.git
cd combo3
git remote add asano git@github.com:yuki-asano/combo3.git  # there is bug fix branch
git fetch asano
git checkout bug-fix
python3 setup.py install
```
see above link for latest information  

## Usage
### combo_interface
- test program node for only COMBO
- load test candidates (data/candidates.csv)
- visualize mean, std and scores(acquisition functions)
```
cd scripts
./test_combo_interface.py
```

### combo_ros_bridge
- bridge node for communication between COMBO and ROS system
- publish COMBO data to ROS system (e.g. next_param)
- subscribe evaluation value or flags to write the value to COMBO
```
roslaunch combo_ros_bridge combo_ros_bridge.launch
```

### memo
- Result and history data are saved in the dir specified by `policy_save_dir` (`data` dir as defalult).
- if you want to continue an experiment with the previous/existing data, `use_saved_policy=True` should be used.
  - The arg with `False` does not consider the existing data and learning history.
- However, COMBO is not recommended when you want to continue experiments from existing data due to the following issue which was fixed in PHYSBO.
  - https://github.com/issp-center-dev/PHYSBO/issues/44
  - PHYSBO is a candidate for this purpose.
