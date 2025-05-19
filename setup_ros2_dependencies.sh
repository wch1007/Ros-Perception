#!/bin/bash
set -e

echo "🔧 Updating package lists..."
sudo apt update

echo "📦 Installing core ROS 2 Humble dependencies..."
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-ament-cmake \
  ros-humble-tf2-ros \
  ros-humble-tf2-eigen \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-diagnostic-updater \
  ros-humble-camera-info-manager \
  ros-humble-compressed-image-transport \
  ros-humble-pcl-conversions \
  ros-humble-vision-opencv \
  ros-humble-rmw-cyclonedds-cpp

echo "🧪 Installing build/test dependencies for tf2..."
sudo apt install -y ros-humble-ament-cmake-google-benchmark

echo "📷 Installing RealSense system dependencies..."
sudo apt install -y \
  libssl-dev libusb-1.0-0-dev libudev-dev pkg-config \
  libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

echo "✅ All dependencies installed successfully."
