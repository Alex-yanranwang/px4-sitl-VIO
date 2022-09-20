# px4-sitl-VIO
A px4-based simulation platform - combining with plugins needed by Vision-inertial-odometry (VIO)


## L515 - KinoJGM202207_H
```
roslaunch realsense2_camera rs_camera_l515.launch
roslaunch vid_estimator vid_realworld_kino.launch 
```

## D435i - KinoJGM202209_H
```
roslaunch realsense2_camera rs_camera_d435i.launch 
roslaunch vins vins_rviz.launch 
rosrun vins vins_node /home/alex/TEST-202206/KinoJGM202209_H/src/VINS-Fusion/config/realsense_d435i/realsense_stereo_imu_config.yaml 
```
