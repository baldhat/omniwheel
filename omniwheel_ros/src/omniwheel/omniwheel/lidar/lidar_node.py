# First import the library
import pyrealsense2 as rs
import numpy as np
import cv2

# Create a context object. This object owns the handles to all connected realsense devices
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
profile = pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        if not depth: continue

        depth = frames.get_depth_frame()
        depth_data = depth.as_frame().get_data()
        np_image = np.asanyarray(depth_data)
        depth_image_3d = np.dstack((np_image, np_image, np_image))
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image_3d, alpha=0.03), cv2.COLORMAP_JET)

        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

        cv2.imshow('Frame', depth_colormap)


finally:
    pipeline.stop()
