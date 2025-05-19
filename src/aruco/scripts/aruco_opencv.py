#!/usr/bin/env python3
import numpy as np
import cv2
import cv2.aruco as aruco

# 设置相机
camera_id = 10  # 使用 /dev/video10
cap = cv2.VideoCapture(camera_id)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 设置 ArUco 参数
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
marker_size = 0.2  # 单位：米

try:
    while True:
        # 读取一帧
        ret, frame = cap.read()
        if not ret:
            print("未能读取帧")
            continue

        # 检测 ArUco 标记
        corners, ids, rejected = detector.detectMarkers(frame)

        # 如果检测到标记
        if ids is not None:
            # 绘制标记边框和 ID
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            # 打印检测到的标记 ID 和位置
            for i, corner in enumerate(corners):
                marker_id = ids[i][0]
                center = np.mean(corner[0], axis=0)
                print(f"标记 ID: {marker_id}, 中心位置: ({center[0]:.1f}, {center[1]:.1f})")

        # 显示图像
        cv2.imshow('OpenCV ArUco Detection', frame)
        
        # 按 'q' 退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # 释放资源
    cap.release()
    cv2.destroyAllWindows() 