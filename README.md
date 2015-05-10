# realsense_camera
use realsense camera in ROS



## New

* [2015/04/10] Added infrared stream for realsense camera

        If you want IR stream, please visit
        http://solsticlipse.com/2015/03/31/intel-real-sense-3d-on-linux-macos.html
        https://github.com/teknotus/depthview/tree/kernelpatchfmt




## Notes
This package do not contains uvmap data(use to register RGB data)

So it is only publish point cloud without RGB

If you want RGB data for point cloud

Please visit [realsense_camera_tools](https://github.com/BlazingForests/realsense_camera_tools)

You can copy the uvmap files by realsense's serial number to data/uvmap/

```
realsense_camera
├── data
│   └── uvmap
│       ├── 039140070906
│       │   ├── uvmap_0000.bin
│       │   ├── uvmap_0001.bin
│       │   ├── ...
│       │   ├── ...
│       │   ├── uvmap_2047.bin
```

039140070906 is realsense's serial number



## Dependencies

```
sudo apt-get install libudev-dev
```

## Usage

* launch realsense camera and rviz
```
roslaunch realsense_camera realsense_rviz.launch 
```

* launch realsense camera only
```
roslaunch realsense_camera realsense_camera.launch
```


## Publish topics

default

```
sensor_msgs::PointCloud2
/camera/depth/points                point cloud without RGB
/camera/depth_registered/points     point cloud with RGB

sensor_msgs::Image
/camera/image/rgb_raw               raw image for RGB sensor
/camera/image/depth_raw             raw image for depth sensor
/camera/image/ir_raw                raw image for infrared sensor
```

you can custom topic in file realsense_camera.launch

```
<arg name="topic_depth_points_id" default="$(arg camera)/depth/points" />
<arg name="topic_depth_registered_points_id" default="$(arg camera)/depth_registered/points" />
    
<arg name="topic_image_rgb_raw_id" default="$(arg camera)/image/rgb_raw" />
<arg name="topic_image_depth_raw_id" default="$(arg camera)/image/depth_raw" />
<arg name="topic_image_infrared_raw_id" default="$(arg camera)/image/ir_raw" />
```

## Q&A

* If have "select timeout" issue
```
Stop realsense_camera node
Re-plug realsense device then retry
```

* RGB to depth misalignment

[uvmap help](https://github.com/BlazingForests/realsense_camera_tools/blob/master/README.md#qa)


## TODO

* add use TF between RGB & depth to register rgb data
* add usb reset
* add read properties from device
* add RGB to depth camera calibration


## Issues

![](https://github.com/BlazingForests/realsense_camera/blob/master/issues/rgb/realsense_rgb_1431245702.jpg)
![](https://github.com/BlazingForests/realsense_camera/blob/master/issues/rgb/realsense_rgb_1431248335.jpg)
![](https://github.com/BlazingForests/realsense_camera/blob/master/issues/rgb/realsense_rgb_1431248344.jpg)



