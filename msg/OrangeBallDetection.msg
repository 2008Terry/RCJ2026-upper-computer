std_msgs/Header header
bool detected
float32 confidence
# Detector image-ground frame. See docs/coordinate_frames.md.
# +x is camera-image down / robot rear, +y is camera-image left / robot left.
geometry_msgs/Point ball_center_m
geometry_msgs/Point ball_center_raw_px
geometry_msgs/Point ball_top_raw_px
