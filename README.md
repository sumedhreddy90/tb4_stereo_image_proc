# Generate Disparity Image and PointCloud2 using [stereo_image_proc](https://github.com/ros-perception/image_pipeline/tree/galactic/stereo_image_proc) and OAK-D pro stereo Camera

## Walk through on how to setup OAK-D pro stereo Camera using [stereo_image_proc](https://github.com/ros-perception/image_pipeline/tree/galactic/stereo_image_proc) in Iginition Gazebo

1. Setup `.urdf.xacro` file of your Camera sensor:
  Make sure that left and right camera links are referenced with gazebo rgbd_camera sensor. For example, let us configure left camera of  OAK-D pro stereo Camera by defining a rgbd_camera sensor. Similarly, you can perform the same for your right camera. Refer the complete code at [oakd.urdf.xacro](https://github.com/sumedhreddy90/tb4_stereo_image_proc/blob/main/turtlebot4/turtlebot4_description/urdf/sensors/oakd.urdf.xacro)
```
  <gazebo reference="${name}_left_camera_frame">
    <sensor name="rgbd_camera" type="rgbd_camera">
      <camera>
        <horizontal_fov>1.25</horizontal_fov>
        <image>
          <width>320</width>
          <height>240</height>
        </image>
        <clip>
          <near>0.3</near>
          <far>100</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
    </sensor>
    <xacro:material_darkgray/>
  </gazebo>
```
2. Establish a bridge between OAK-D pro stereo Camera and ros_ign_bridge. Here we pass arguments to `parameter_bridge` executable of `ros_ign_bridge` package. We pass all the required links with approriate msg's required for our application. For instance, we pass left and right camera frame links along with `sensor_msgs/msg/Image` and `ignition.msgs.Image` as an argument to `parameter_bridge`. Similarly, we can pass left and right `camera_info` with approriate msg. Complete code can be found at [Setting oakd_pro_camera_bridge](https://github.com/sumedhreddy90/tb4_stereo_image_proc/blob/main/turtlebot4_simulator/turtlebot4_ignition_bringup/launch/ros_ign_bridge.launch.py)

`'/link/oakd_pro_left_camera_frame/sensor/rgbd_camera/image' +
                '@sensor_msgs/msg/Image' +
                '[ignition.msgs.Image]'`

```
    # Camera sensor bridge
    oakd_pro_camera_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/world/', LaunchConfiguration('world'),
                '/model/', LaunchConfiguration('robot_name'),
                '/link/oakd_pro_left_camera_frame/sensor/rgbd_camera/image' +
                '@sensor_msgs/msg/Image' +
                '[ignition.msgs.Image'],
                # pass right camera frame as argument
            ['/world/', LaunchConfiguration('world'),
                '/model/', LaunchConfiguration('robot_name'),
                '/link/oakd_pro_right_camera_frame/sensor/rgbd_camera/image' +
                '@sensor_msgs/msg/Image' +
                '[ignition.msgs.Image'],
                # left camera info
            ['/world/', LaunchConfiguration('world'),
                '/model/', LaunchConfiguration('robot_name'),
                '/link/oakd_pro_left_camera_frame/sensor/rgbd_camera/camera_info' +
                '@sensor_msgs/msg/CameraInfo' +
                '[ignition.msgs.CameraInfo'],
                # right camera info
            ['/world/', LaunchConfiguration('world'),
                '/model/', LaunchConfiguration('robot_name'),
                '/link/oakd_pro_right_camera_frame/sensor/rgbd_camera/camera_info' +
                '@sensor_msgs/msg/CameraInfo' +
                '[ignition.msgs.CameraInfo'],
            ['/world/', LaunchConfiguration('world'),
                '/model/', LaunchConfiguration('robot_name'),
                '/link/oakd_pro_left_camera_frame/sensor/rgbd_camera/image' +
                '@sensor_msgs/msg/Image' +
                '[ignition.msgs.Image'],
            ['/world/', LaunchConfiguration('world'),
                '/model/', LaunchConfiguration('robot_name'),
                '/link/oakd_pro_left_camera_frame/sensor/rgbd_camera_rect/image' +
                '@sensor_msgs/msg/Image' +
                '[ignition.msgs.Image'],
                ],
        remappings=[
             # Remap left camera frame as /left/image_rect
            (['/world/', LaunchConfiguration('world'),
              '/model/',
              LaunchConfiguration('robot_name'),
              '/link/oakd_pro_left_camera_frame/sensor/rgbd_camera/image'],
             '/left/image_rect'),
             # Remap right camera frame as /right/image_rect
            (['/world/', LaunchConfiguration('world'),
              '/model/',
              LaunchConfiguration('robot_name'),
              '/link/oakd_pro_right_camera_frame/sensor/rgbd_camera/image'],
             '/right/image_rect'),
              # Remap left camera frame with camera_info msg as /left/camera_info
            (['/world/', LaunchConfiguration('world'),
              '/model/',
              LaunchConfiguration('robot_name'),
              '/link/oakd_pro_left_camera_frame/sensor/rgbd_camera/camera_info'],
             '/left/camera_info'),
             # Remap right camera frame with camera_info msg as /right/camera_info
            (['/world/', LaunchConfiguration('world'),
              '/model/',
              LaunchConfiguration('robot_name'),
              '/link/oakd_pro_right_camera_frame/sensor/rgbd_camera/camera_info'],
             '/right/camera_info'),
             # Remap left camera frame with image.msg as /left/image_rect_color for pointcloud2 data
            (['/world/', LaunchConfiguration('world'),
              '/model/',
              LaunchConfiguration('robot_name'),
              '/link/oakd_pro_left_camera_frame/sensor/rgbd_camera_rect/image'],
             '/left/image_rect_color'),
                ],
        condition=LaunchConfigurationEquals('model', 'standard'))
```
3. Now, we got all the topics that supports our [stereo_image_proc](https://github.com/ros-perception/image_pipeline/tree/galactic/stereo_image_proc) package. Create a package with launch file that launches all the required nodes of stereo_image_proc package along with turtlebot4 ignition bringup package with rempapping of topics as launch arguments. The launch file is self explanatory and can be found at [tb4_stereo_image_proc](https://github.com/sumedhreddy90/tb4_stereo_image_proc/blob/main/tb4_stereo_image_proc/launch/tb4_image_proc_view.launch.py). Topics that are essential for generation of disparity and pointcloud2 are: (Make sure your stereo camera is publishing the below topics)
```
 /left/image_rect
 /left/camera_info
 /right/image_rect
 /right/camera_info
 /left/image_rect_color
```

# Instructions to reproduce the Tutorial
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


