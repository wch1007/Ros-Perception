# Perception517

```bash
colcon build
```

```bash
source install/setup.bash
```

## yolo
ros2 launch realsense2_camera rs_launch.py

ros2 launch yolo_bringup yolo.launch.py input_image_topic:=/camera/color/image_raw device:=CPU

ros2 run rqt_image_view rqt_image_view

ros2 topic echo /yolo/detections

## ArUco marker
```bash
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py
```

---
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
