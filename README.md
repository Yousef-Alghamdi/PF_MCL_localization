## Build on ROS
+ Clone this repository to your catkin workspace and catkin_make.
```
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone https://github.com/Yousef-Alghamdi/PF_MCL_localization.git
```
+ Change the name of the folder that you just cloned to "pf_localization" , which matches the name of ROS package.
```
  cd ../
  catkin_make
```
+ Modify the **value** of the parameter **package_path_param** in line 16 of launch/mcl_localization.launch to the path where the pf_localization package is located.  E.g : **value="/home/catkin_ws/src/"**

## Run the Simulation : 
roslaunch pf_localization mcl_localization.launch
