# PX4-SITL
px4 sitl with launch files

## Terminal commands
## No.0 (optional):
```
cd /home/alex/TEST-202201/PX4-SITL/PX4-Autopilot
```

## No.1:
```
make px4_sitl gazebo
```

## No.2:
```
source /home/alex/TEST-202201/PX4-SITL/PX4-Autopilot/Tools/setup_gazebo.bash /home/alex/TEST-202201/PX4-SITL/PX4-Autopilot /home/alex/TEST-202201/PX4-SITL/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```
```
source $(pwd)/Tools/setup_gazebo.bash $(pwd) $(pwd)/build_posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
```

## No.3.1 (launch with mavros, px4 sitl, Gazebo and spawn quadrotor):
```
roslaunch px4 mavros_posix_sitl.launch
```
## No.3.2 (px4 sitl, Gazebo and spawn quadrotor):
```
roslaunch px4 posix_sitl.launch
```
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

## No.4 (position control via mavros):
```
cd /home/alex/TEST-202201/PX4-SITL/PX4-Autopilot/integrationtests/python_src/px4_it/mavros
python test-poscl.py 
```

