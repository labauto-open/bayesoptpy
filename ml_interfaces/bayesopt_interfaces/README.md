# bayesopt_interfaces
python interfaces of Bayesian optimizaion software for lab automation

## physbo_interfaces
- PHYSBO: optimization tools for PHYsics based on Bayesian Optimization
  - doc
    - https://www.pasums.issp.u-tokyo.ac.jp/physbo/en
  - source code
    - https://github.com/issp-center-dev/PHYSBO

### Install
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

### Usage
- test program node for only PHYSBO
- load test candidates (examples/data/candidates.csv)
- visualize mean, std and scores(acquisition functions)
```
cd examples
./test_physbo_interface.py
```

## combo_interfaces
- COMBO: COMmon Bayesian Optimization Library
 - doc
   - https://github.com/tsudalab/combo/blob/master/docs/combo_document.pdf
  - source code
    - combo 
      - https://github.com/tsudalab/combo
      - python2 ver
    - combo3 (not recommended due to no maintaince)
      - https://github.com/tsudalab/combo3
      - python3 ver
      - PHYSBO is the next version and candidate for python3

### Install
#### for combo
- install combo from source -> https://github.com/tsudalab/combo

#### for combo3
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

### Usage
- test program node for only COMBO
- load test candidates (data/candidates.csv)
- visualize mean, std and scores(acquisition functions)
```
cd scripts
./test_combo_interface.py
```
