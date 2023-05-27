# rm_vision_ros2_galaxy_camera

A ROS2 package for Daheng Imaging Galaxy USB 3.0 industrial camera, for use in [rm_vision](https://github.com/chenjunnn/rm_vision) project.

Based on HikVision camera package from the aforementioned project. When used in rm_vision, change `camera` to `galaxy` in `launch_params.yaml`.

## Usage

```
ros2 launch galaxy_camera galaxy_camera.launch.py
```

## Notes

This package is experimental, and its integration into real world RoboMaster applications has not been thoroughly tested. Issue reports welcomed.
