# realsense_camera
use realsense camera in ROS



## New

* [2015/11/26] Add print center Z value and realsense temperature at the same time for `F200`.  
Can use to calibration Z value with realsense's temperature(Thx to @teknotus).

```
0) sudo apt-get install libusb-1.0-0-dev  
1) sudo cp config/80-realsense-usb.rules /etc/udev/rules.d  
2) reboot to reload the rule  
3) rebuild realsense_camera  
4) roslaunch realsense_camera realsense_rviz.launch debug_depth_unit:=true

If everything is OK, in terminal you will see  

getRealsenseUSBHandle OK!  
average center z value = 0.388359    temp = 31    depth_unit = 33.099998
```

* Modify default image topic name like openni.

* Add R200 support, But still in test, Because i don't have the R200 device now.

* Intel maybe release official ROS-Realsense package.

		visit http://roscon.ros.org/2015/   then search realsense


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
sudo apt-get install libudev-dev libv4l-dev libusb-1.0-0-dev
```

To make sure the dynamic reconfigure works, Please visit

```
https://github.com/teknotus/depthview/tree/udev
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
/camera/rgb/image_raw               raw image for RGB sensor
/camera/depth/image_raw             raw image for depth sensor
/camera/ir/image_raw                raw image for infrared sensor
```

you can custom topic in file realsense_camera.launch

```
<arg name="topic_depth_points_id" default="$(arg camera)/depth/points" />
<arg name="topic_depth_registered_points_id" default="$(arg camera)/depth_registered/points" />

<arg name="topic_image_rgb_raw_id" default="$(arg camera)/rgb/image_raw" />
<arg name="topic_image_depth_raw_id" default="$(arg camera)/depth/image_raw" />
<arg name="topic_image_infrared_raw_id" default="$(arg camera)/ir/image_raw" />
```

### Camera Info
To specify a yaml file with intrinsic calibration parameters for the RGB & IR camera (for example generated using [this](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) tool), modify the value in the realsense_(r200_)camera.launch to your calibration file (eg. package://package_name/path_to_yaml_file/calibration_file.yaml )
```
<!-- camera calibration file url -->
<arg name="rgb_camera_info_url" default="" />
<arg name="ir_camera_info_url" default="" />
```
Make sure the image width and height specified in the calibration file is the same as that specified in launch file.

The calibration info will be published under the topic  `/camera/rgb/camera_info` `/camera/depth/camera_info` `/camera/ir/camera_info`


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

```
terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >' what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument

```

```
//fill point cloud data time: [0.035777 s]     <----  need optimize
```
