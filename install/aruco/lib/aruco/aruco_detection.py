#!/usr/bin/env python3
import numpy as np
import cv2
import cv2.aruco as aruco
import pyrealsense2 as rs
import time

def main():
    print("正在初始化RealSense管道...")
    try:
        # 创建上下文和设备列表
        ctx = rs.context()
        devices = ctx.query_devices()
        print(f"检测到 {len(devices)} 个RealSense设备")
        
        if len(devices) == 0:
            print("未检测到任何RealSense设备！")
            return
        
        # 列出设备信息
        for i, dev in enumerate(devices):
            print(f"设备 {i}: {dev.get_info(rs.camera_info.name)}")
            print(f"  序列号: {dev.get_info(rs.camera_info.serial_number)}")
            print(f"  固件版本: {dev.get_info(rs.camera_info.firmware_version)}")
            
            # 枚举传感器
            sensors = dev.query_sensors()
            print(f"  设备有 {len(sensors)} 个传感器")
            
            for j, sensor in enumerate(sensors):
                print(f"    传感器 {j}: {sensor.get_info(rs.camera_info.name) if sensor.supports(rs.camera_info.name) else '未知'}")
                
                # 尝试枚举支持的流
                try:
                    profiles = sensor.get_stream_profiles()
                    print(f"      支持 {len(profiles)} 种流类型")
                    
                    # 只打印一些关键的流类型
                    for profile in profiles[:5]:  # 只显示前5个，避免信息过多
                        stream_name = "未知"
                        if profile.stream_type() == rs.stream.depth:
                            stream_name = "深度"
                        elif profile.stream_type() == rs.stream.color:
                            stream_name = "彩色"
                        elif profile.stream_type() == rs.stream.infrared:
                            stream_name = "红外"
                            
                        if profile.is_video_stream_profile():
                            video_profile = profile.as_video_stream_profile()
                            width, height = video_profile.width(), video_profile.height()
                            fps = video_profile.fps()
                            print(f"        {stream_name} 流: {width}x{height} @ {fps}fps")
                except Exception as e:
                    print(f"      获取流信息失败: {e}")
        
        # 创建管道和配置
        pipeline = rs.pipeline()
        config = rs.config()
        
        # 只配置彩色流
        print("\n正在只配置彩色流...")
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # 启动管道
        print("正在启动管道...")
        try:
            profile = pipeline.start(config)
            print("管道成功启动！")
            
            # 获取设备信息
            dev = profile.get_device()
            print(f"已连接到设备: {dev.get_info(rs.camera_info.name)}")
            
            # 等待相机稳定
            print("等待相机稳定...")
            time.sleep(2)

            # 设置 ArUco 参数
            print("正在配置ArUco检测器...")
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

            # 相机内参（建议用RealSense标定参数替换）
            camera_matrix = np.array([[615.95431, 0, 325.26983],
                                    [0, 617.92586, 257.57722],
                                    [0, 0, 1]], dtype=np.float32)
            dist_coeffs = np.array([0.142588, -0.311967, 0.003950, -0.006346, 0.000000], dtype=np.float32)

            # 标记尺寸（米）
            marker_size = 0.2

            frame_count = 0
            print("开始获取帧并处理...")
            
            while True:
                print(f"正在等待第 {frame_count + 1} 帧...", end='\r')
                
                # 等待帧到达
                try:
                    frames = pipeline.wait_for_frames(5000)  # 等待5秒
                except Exception as e:
                    print(f"\n等待帧超时: {e}")
                    continue
                
                # 获取彩色帧
                color_frame = frames.get_color_frame()
                
                # 检查彩色帧是否有效
                if not color_frame:
                    print("\n未能获取彩色帧")
                    continue

                # 转换为numpy数组
                color_image = np.asanyarray(color_frame.get_data())
                print(f"\n成功获取第 {frame_count + 1} 帧")
                
                # 检查彩色图像尺寸
                height, width, channels = color_image.shape
                print(f"帧大小: {width}x{height}, 通道数: {channels}")
                
                frame_count += 1
                
                # 转换为灰度图像用于ArUco检测
                gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

                # 检测ArUco标记
                corners, ids, rejected = detector.detectMarkers(gray)

                # 如果检测到标记
                if ids is not None and len(ids) > 0:
                    print(f"检测到 {len(ids)} 个标记")
                    # 绘制标记边框和ID
                    color_image = cv2.aruco.drawDetectedMarkers(color_image, corners, ids)

                    for i in range(len(ids)):
                        marker_corners = corners[i]
                        objPoints = np.array([
                            [-marker_size/2, marker_size/2, 0],
                            [marker_size/2, marker_size/2, 0],
                            [marker_size/2, -marker_size/2, 0],
                            [-marker_size/2, -marker_size/2, 0]
                        ], dtype=np.float32)
                        
                        success, rvec, tvec = cv2.solvePnP(
                            objPoints, marker_corners, camera_matrix, dist_coeffs)
                        
                        if success:
                            cv2.drawFrameAxes(color_image, camera_matrix, dist_coeffs, 
                                        rvec, tvec, marker_size/2)
                            marker_id = ids[i][0]
                            position = tvec.flatten()
                            text = f"ID: {marker_id}"
                            cv2.putText(color_image, text, 
                                     (int(marker_corners[0][0][0]), int(marker_corners[0][0][1]) - 10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            print(f"\n标记 ID: {marker_id}")
                            print(f"位置 (米):")
                            print(f"  X: {position[0]:.3f}")
                            print(f"  Y: {position[1]:.3f}")
                            print(f"  Z: {position[2]:.3f}")

                # 显示彩色图像
                cv2.imshow('ArUco Detection (RGB)', color_image)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except Exception as e:
            print(f"处理过程中发生错误: {e}")
        finally:
            print("正在停止管道...")
            pipeline.stop()
            cv2.destroyAllWindows()
            print("程序结束")
            
    except Exception as e:
        print(f"初始化出错: {e}")

if __name__ == '__main__':
    main() 