ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    rgb_camera.color_profile:=640x480x30 \
    enable_depth:=true \
    depth_module.depth_profile:=640x480x30 \
    enable_infra:=false \
    enable_infra1:=false \
    enable_infra2:=false \
    depth_module.infra_profile:=640x480x30 \
    enable_rgbd:=true \
    align_depth.enable:=true \
    enable_sync:=true \
    enable_gyro:=false \
    enable_accel:=false \
    pointcloud.enable:=true

