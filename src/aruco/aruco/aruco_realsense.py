# Subscribe to color image topic
self.color_sub = self.create_subscription(
    Image,
    '/camera/color/image_raw',
    self.color_callback,
    10)
# Subscribe to depth image topic
self.depth_sub = self.create_subscription(
    Image,
    '/camera/depth/image_rect_raw',
    self.depth_callback,
    10)
# Subscribe to camera info topic
self.camera_info_sub = self.create_subscription(
    CameraInfo,
    '/camera/color/camera_info',
    self.camera_info_callback,
    10) 