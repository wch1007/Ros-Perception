# Perception517

## Project Overview

This project is a robotic perception system integrating ArUco marker detection and pose estimation based on RealSense cameras, YOLO object detection and 3D localization, and multi-frame transformation. The system enables accurate recognition and localization of objects in the workspace, and transforms target poses into the robot arm base frame for subsequent grasping and manipulation.
https://docs.google.com/document/d/11xjWABXkmtUkdhgVQmDyfbheEC6ylWzkDBqWH_Du0WM/edit?tab=t.0


## Developers

- Chenghao Wang
- Yifan Li
- Xiny Hu

## Main Features

- ArUco marker detection and pose estimation
- YOLO 2D/3D object detection
- Coordinate transformation (camera, marker, arm base)
- ROS2 multi-topic publishing and debugging tool integration
- Visualization support with rqt_image_view and rviz2

Workflow

1. Detect QR Code using RealSense Camera
   - Detect the QR code attached to the workspace.
   - Estimate the pose of the QR code relative to the camera frame (T_camera_qrcode).

2. Calculate Camera Pose in QR Code Frame (the relative position between camera and QR code)
   - Invert the transform:
     T_qrcode_camera = inverse(T_camera_qrcode)

3. Define QR Code Pose in Arm Frame
   - Manually set or measure the static transform from the robot base to the QR code (T_arm_qrcode).

4. Compute Camera Pose in Arm Frame
   - Chain transforms:
     T_arm_camera = T_arm_qrcode * T_qrcode_camera

5. Detect Object in Camera Frame
   - Use Yolo to find object pose:
     T_camera_object

6. Get Object Pose in Arm Frame
   - Combine transforms:
     T_arm_object = T_arm_camera * T_camera_object

7. Send Target Pose to Motion Planner
   - Use T_arm_object as the grasp target for the robot's motion planning module.





## How to transform

```bash
ros2 topic echo /yolo/detections_3d # object对于camera frame的坐标

ros2 run tf2_ros static_transform_publisher _ _ _ _ _ _ map camera_link # 中间填写camera对于qrcode frame的坐标，记得计算欧拉角，作用是发布一个setting，最终可以替换成arm的

ros2 run yolov8_tf_node transform_detections # 最终计算object对于qrcode frame的坐标（中间用相机位姿进行计算）
```
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

## Setting

```bash
colcon build
```

```bash
source install/setup.bash
```
## Aruco

#### T1
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

#### T2
```bash
colcon build
```
If you only rebuild the package of aruco
```bash
colcon build --packages-select aruco_realsense_ros
```

```bash
source install/setup.bash
```
```bash
ros2 run aruco_realsense_ros aruco_node
```

#### T3 (topics)
Check topic list
```bash
ros2 topic echo /aruco/marker_pose_rpy
```
Raw image
```bash
ros2 topic echo /camera/color/image_raw
```
Marker pose in (x,y,z,r,p,t) form
```bash
ros2 topic echo /aruco/marker_pose_rpy
```
Camera pose in marker frame (x,y,z,r,p,t) form
```bash
ros2 topic echo /aruco/camera_pose_rpy
```
You need to set the marker in arm_base_link's position in aruco_node.py, the default is:
```bash
        self.marker_in_arm = {
            'x': 0.1,
            'y': -0.2,
            'z': 0.05,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }
```
The tf matrix from camera_link to arm_base_link
```bash
ros2 topic echo /aruco/tf
```

For the beta version, I got a mimic object position in camera link published on camera/mimic_xyzrpt topic, this should be replaced by the real xyzrpt of the object determined by yolo and depth info
```bash
ros2 topic echo /camera/mimic_xyzrpt
```
After tf, the position of the object in arm_base_link
```bash
ros2 topic echo /arm/mimic_xyzrpt
```

## Yolo

```bash
ros2 launch realsense2_camera rs_launch.py
```
#### 1 (👍) - 2d bbox
```bash
ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/camera/color/image_raw device:=CPU
```
#### 2 (👍) - 2d seg+bbox
```bash
ros2 launch yolo_bringup yolo.launch.py model:=yolov8m-seg input_image_topic:=/camera/color/image_raw device:=CPU 
```
#### 3 (👍) - 3d bbox
```bash
ros2 launch yolo_bringup yolov8.launch.py input_image_topic:=/camera/color/image_raw device:=CPU input_depth_topic:=/camera/depth/image_rect_raw target_frame:=camera_link use_3d:=True
```

# rqt
ros2 run rqt_image_view rqt_image_view
# rviz
ros2 run rviz2 rviz2

ros2 topic echo /yolo/detections
ros2 topic echo /yolo/tracking
ros2 topic echo /yolo/dbg_image
ros2 topic echo /yolo/detections_3d
```



## Yolo_3d_work

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true

ros2 launch yolo_bringup yolov8.launch.py \
  input_image_topic:=/camera/color/image_raw \
  input_depth_topic:=/camera/aligned_depth_to_color/image_raw \
  input_depth_info_topic:=/camera/aligned_depth_to_color/camera_info \
  device:=CPU \
  target_frame:=camera_link \
  use_3d:=True

ros2 topic echo /camera/aligned_depth_to_color/image_raw
ros2 topic echo /camera/color/image_raw

ros2 run rqt_image_view rqt_image_view

ros2 topic echo /yolo/detections_3d
```



## Yolo with Depth Transform

```bash
ros2 launch realsense2_camera rs_launch.py

ros2 launch yolo_bringup yolov8.launch.py input_image_topic:=/camera/color/image_raw device:=CPU input_depth_topic:=/camera/depth/image_rect_raw target_frame:=camera_link use_3d:=True

ros2 topic echo /yolo/detections_3d

ros2 run tf2_ros static_transform_publisher 0 0 1.2 0 0 0 map camera_link

ros2 run yolov8_tf_node transform_detections
```
#### Bug
Reason:
/camera/color/image_raw（1280×720）
/camera/depth/image_rect_raw（848×480）

https://github.com/mgonzs13/yolo_ros/issues/11
https://github.com/mgonzs13/yolo_ros/pull/13
```bash
[detect_3d_node-3] Traceback (most recent call last):
[detect_3d_node-3]   File "/home/rosdev/ros2_ws/install/yolo_ros/lib/yolo_ros/detect_3d_node", line 33, in <module>
[detect_3d_node-3]     sys.exit(load_entry_point('yolo-ros==0.0.0', 'console_scripts', 'detect_3d_node')())
[detect_3d_node-3]   File "/home/rosdev/ros2_ws/install/yolo_ros/lib/python3.10/site-packages/yolo_ros/detect_3d_node.py", line 444, in main
[detect_3d_node-3]     rclpy.spin(node)
[detect_3d_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 226, in spin
[detect_3d_node-3]     executor.spin_once()
[detect_3d_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 751, in spin_once
[detect_3d_node-3]     self._spin_once_impl(timeout_sec)
[detect_3d_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 748, in _spin_once_impl
[detect_3d_node-3]     raise handler.exception()
[detect_3d_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/task.py", line 254, in __call__
[detect_3d_node-3]     self._handler.send(None)
[detect_3d_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 447, in handler
[detect_3d_node-3]     await call_coroutine(entity, arg)
[detect_3d_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 372, in _execute_subscription
[detect_3d_node-3]     await await_or_execute(sub.callback, msg)
[detect_3d_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 107, in await_or_execute
[detect_3d_node-3]     return callback(*args)
[detect_3d_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/message_filters/__init__.py", line 83, in callback
[detect_3d_node-3]     self.signalMessage(msg)
[detect_3d_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/message_filters/__init__.py", line 64, in signalMessage
[detect_3d_node-3]     cb(*(msg + args))
[detect_3d_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/message_filters/__init__.py", line 313, in add
[detect_3d_node-3]     self.signalMessage(*msgs)
[detect_3d_node-3]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/message_filters/__init__.py", line 64, in signalMessage
[detect_3d_node-3]     cb(*(msg + args))
[detect_3d_node-3]   File "/home/rosdev/ros2_ws/install/yolo_ros/lib/python3.10/site-packages/yolo_ros/detect_3d_node.py", line 177, in on_detections
[detect_3d_node-3]     new_detections_msg.detections = self.process_detections(
[detect_3d_node-3]   File "/home/rosdev/ros2_ws/install/yolo_ros/lib/python3.10/site-packages/yolo_ros/detect_3d_node.py", line 204, in process_detections
[detect_3d_node-3]     bbox3d = self.convert_bb_to_3d(depth_image, depth_info_msg, detection)
[detect_3d_node-3]   File "/home/rosdev/ros2_ws/install/yolo_ros/lib/python3.10/site-packages/yolo_ros/detect_3d_node.py", line 266, in convert_bb_to_3d
[detect_3d_node-3]     depth_image[int(center_y)][int(center_x)] / self.depth_image_units_divisor
[detect_3d_node-3] IndexError: index 584 is out of bounds for axis 0 with size 480
[ERROR] [detect_3d_node-3]: process has died [pid 1390770, exit code 1, cmd '/home/rosdev/ros2_ws/install/yolo_ros/lib/yolo_ros/detect_3d_node --ros-args -r __node:=detect_3d_node -r __ns:=/yolo --params-file /tmp/launch_params_x9uiuoqj -r depth_image:=/camera/depth/image_rect_raw -r depth_info:=/camera/depth/camera_info -r detections:=tracking'].
```

---

## Shared Drive
https://drive.google.com/drive/folders/1Ca7cZbU5AXZ1mbn07E53e9UEEOHd-OuB?usp=sharing


## ArUco Marker
```bash
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py
```

## Appedix

---

### /yolo/detections

```
header:
  stamp:
    sec: 1745366042
    nanosec: 681903320
  frame_id: camera_color_optical_frame
detections:
- class_id: 69
  class_name: oven
  score: 0.5432252883911133
  id: ''
  bbox:
    center:
      position:
        x: 583.5409545898438
        y: 360.6185302734375
      theta: 0.0
    size:
      x: 1155.0013427734375
      y: 708.1890869140625
  bbox3d:
    center:
      position:
        x: 0.0
        y: 0.0
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    size:
      x: 0.0
      y: 0.0
      z: 0.0
    frame_id: ''
  mask:
    height: 0
    width: 0
    data: []
  keypoints:
    data: []
  keypoints3d:
    data: []
    frame_id: ''
```


---

### /yolo/dbg_image

```
header:
  stamp:
    sec: 1746593972
    nanosec: 919295410
  frame_id: camera_color_optical_frame
height: 720
width: 1280
encoding: bgr8
is_bigendian: 0
step: 3840
data:
- 132
- 135
- 152
- 134
- 137
- 155
- 134
- 137
- 155
- 136
- 139
- 157
- 137
- 138
- 156
```

---

### /yolo/detections (with seg)

```
  keypoints:
    data: []
  keypoints3d:
    data: []
    frame_id: ''
- class_id: 63
  class_name: laptop
  score: 0.6110746264457703
  id: ''
  bbox:
    center:
      position:
        x: 48.437564849853516
        y: 552.5540771484375
      theta: 0.0
    size:
      x: 96.27031707763672
      y: 103.369873046875
  bbox3d:
    center:
      position:
        x: 0.0
        y: 0.0
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    size:
      x: 0.0
      y: 0.0
      z: 0.0
    frame_id: ''
  mask:
    height: 720
    width: 1280
    data:
    - x: 4.0
      y: 500.0
    - x: 4.0
      y: 610.0
    - x: 20.0
      y: 610.0
    - x: 20.0
      y: 604.0
    - x: 24.0
      y: 600.0
    - x: 30.0
      y: 600.0
    - x: 32.0
      y: 598.0
    - x: 38.0
      y: 598.0
    - x: 40.0
      y: 596.0
    - x: 44.0
      y: 596.0
    - x: 46.0
      y: 594.0
    - x: 52.0
      y: 594.0
    - x: 54.0
      y: 592.0
    - x: 64.0
      y: 592.0
    - x: 66.0
      y: 590.0
    - x: 78.0
      y: 590.0
    - x: 80.0
      y: 588.0
    - x: 84.0
      y: 588.0
    - x: 86.0
      y: 586.0
    - x: 88.0
      y: 586.0
    - x: 94.0
      y: 580.0
    - x: 94.0
      y: 558.0
    - x: 92.0
      y: 556.0
    - x: 92.0
      y: 552.0
    - x: 90.0
      y: 550.0
    - x: 90.0
      y: 548.0
    - x: 88.0
      y: 546.0
    - x: 88.0
      y: 542.0
    - x: 86.0
      y: 540.0
    - x: 86.0
      y: 536.0
    - x: 84.0
      y: 534.0
    - x: 84.0
      y: 530.0
    - x: 82.0
      y: 528.0
    - x: 82.0
      y: 524.0
    - x: 80.0
      y: 522.0
    - x: 80.0
      y: 518.0
    - x: 78.0
      y: 516.0
    - x: 78.0
      y: 512.0
    - x: 76.0
      y: 510.0
    - x: 76.0
      y: 500.0
```

### A single command that installs all commonly needed ROS 2 Humble packages for realsense2_camera, yolo_ros, and aruco_pose_estimation so you don't have to chase them down one-by-one:

sudo apt update && sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-diagnostic-updater \
  ros-humble-camera-info-manager \
  ros-humble-compressed-image-transport \
  ros-humble-pcl-conversions \
  ros-humble-tf2-ros \
  ros-humble-tf2-eigen \
  ros-humble-vision-opencv \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-ament-cmake \
  ros-humble-ros-base

### After that, just rebuild:

cd ~/ros2_ws
colcon build --symlink-install


---

### /yolo/detections_3d

```
header:
  stamp:
    sec: 1746920233
    nanosec: 131079590
  frame_id: camera_color_optical_frame
detections:
- class_id: 0
  class_name: person
  score: 0.9451740980148315
  id: '1'
  bbox:
    center:
      position:
        x: 791.4027099609375
        y: 394.0580749511719
      theta: 0.0
    size:
      x: 908.68994140625
      y: 642.4336547851562
  bbox3d:
    center:
      position:
        x: 1.9895000000000007
        y: -1.7179545540492693
        z: -0.7357633608419807
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    size:
      x: 0.5930000000000013
      y: 4.2525406995990735
      z: 3.0067523448707103
    frame_id: camera_link
  mask:
    height: 0
    width: 0
    data: []
  keypoints:
    data: []
  keypoints3d:
    data: []
    frame_id: ''
- class_id: 62
  class_name: tv
  score: 0.8102542757987976
  id: '2'
  bbox:
    center:
      position:
        x: 79.16596984863281
        y: 188.20114135742188
      theta: 0.0
    size:
      x: 168.17138671875
      y: 224.76947021484375
  bbox3d:
    center:
      position:
        x: 3.7314999999999996
        y: 3.0321590032115546
        z: 0.42954938930629805
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    size:
      x: 0.5710000000000008
      y: 1.4757453194541508
      z: 1.9676604259388681
    frame_id: camera_link
  mask:
    height: 0
    width: 0
    data: []
  keypoints:
    data: []
  keypoints3d:
    data: []
    frame_id: ''
- class_id: 62
  class_name: tv
  score: 0.7937374711036682
  id: '6'
  bbox:
    center:
      position:
        x: 198.95523071289062
        y: 231.7283935546875
      theta: 0.0
    size:
      x: 160.74110412597656
      y: 199.8680419921875
  bbox3d:
    center:
      position:
        x: 4.8505
        y: 2.582651619742433
        z: 0.06737126914834146
      orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
    size:
      x: 0.5870000000000006
      y: 1.8269437598852392
      z: 2.2722613013572666
    frame_id: camera_link
  mask:
    height: 0
    width: 0
    data: []
  keypoints:
    data: []
  keypoints3d:
    data: []
    frame_id: ''
---
```


---

### ros2 run yolov8_tf_node transform_detections

```
rosdev@GIX-30258983:~/ros2_ws$ ros2 run yolov8_tf_node transform_detections
[INFO] [1746922358.054725814] [yolo_tf_transformer]: person in map: (-0.11, -0.40, 1.33)
[INFO] [1746922358.056954043] [yolo_tf_transformer]: tv in map: (0.42, -3.71, -1.88)
[INFO] [1746922358.518233124] [yolo_tf_transformer]: person in map: (-0.11, -0.40, 1.33)
[INFO] [1746922358.520171709] [yolo_tf_transformer]: tv in map: (0.41, -3.59, -1.79)
[INFO] [1746922359.857307606] [yolo_tf_transformer]: person in map: (-0.12, -0.41, 1.33)
[INFO] [1746922359.859062908] [yolo_tf_transformer]: tv in map: (0.43, -3.84, -1.99)
[INFO] [1746922360.471215062] [yolo_tf_transformer]: person in map: (-0.11, -0.39, 1.37)
[INFO] [1746922361.101167765] [yolo_tf_transformer]: person in map: (-0.12, -0.42, 1.37)
[INFO] [1746922361.103084241] [yolo_tf_transformer]: tv in map: (0.42, -3.73, -1.90)
```


### ros2 topic echo /aruco/marker_pose_rpy

```bash
layout:
  dim:
  - label: pose_with_rpy
    size: 7
    stride: 7
  data_offset: 0
data:
- 7.0
- -0.00031442761362477744
- -0.0340867049892565
- 0.14513442522948167
- 2.9217575760589862
- -0.012766291348430553
- -0.006772688466906401
---
```
# tf
rosdev@GIX-30258983:~/ros2_ws$ ros2 run tf2_tools view_frames
[INFO] [1747777076.910348477] [view_frames]: Listening to tf data for 5.0 seconds...
[INFO] [1747777081.993956707] [view_frames]: Generating graph in frames.pdf file...
[INFO] [1747777081.998440337] [view_frames]: Result:tf2_msgs.srv.FrameGraph_Response(frame_yaml="camera_depth_frame: \n  parent: 'camera_link'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\ncamera_depth_optical_frame: \n  parent: 'camera_depth_frame'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\ncamera_color_frame: \n  parent: 'camera_link'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\ncamera_color_optical_frame: \n  parent: 'arm_link'\n  broadcaster: 'default_authority'\n  rate: 5.218\n  most_recent_transform: 1747777081.891744\n  oldest_transform: 1747777077.292192\n  buffer_length: 4.600\naruco_marker_7: \n  parent: 'camera_color_optical_frame'\n  broadcaster: 'default_authority'\n  rate: 5.217\n  most_recent_transform: 1747777081.888900\n  oldest_transform: 1747777077.288602\n  buffer_length: 4.600\n")
