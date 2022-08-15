# Generate Disparity Image and PointCloud2 using [stereo_image_proc](https://github.com/ros-perception/image_pipeline/tree/galactic/stereo_image_proc) and OAK-D pro stereo Camera

## Clone the repository into workspace
```
mkdir image_ws
cd image_ws && mkdir src
cd image_ws/src/
git clone https://github.com/sumedhreddy90/tb4_stereo_image_proc.git
cd ~/image_ws/ && colcon build
source install/setup.bash
```

## Launch Turtlebot4 ignition bringup which includes stereo image proc Node

```
ros2 launch tb4_stereo_image_proc tb4_image_proc_view.launch.py
```

## Launch Disparity viewer
```
 ros2 run image_view disparity_view image:=/disparity DisparityImage
```

![disparity](https://user-images.githubusercontent.com/24978535/184568409-911c932b-cf09-412d-ba89-b0f5629f0cb9.png)

https://user-images.githubusercontent.com/24978535/184568269-64b268f1-ef04-4f53-b543-70a1d2bc50c0.mp4

## Environment
This repo is tested on 
- Ubuntu 20.04
- ROS2 Galactic
- Gazebo Ignition Edifice


