# UAV Tracking with Solid-State Lidars: Dynamic Multi-Frequency Scan Integration

[System Pipeline](doc/pipeline_diagram.pdf)

This is the code implementation for the paper [UAV Tracking with Solid-State Lidars: Dynamic Multi-Frequency Scan Integration](https://arxiv.org/abs/2304.12125).

## Install
The code has been tested on Ubuntu 20.04 with ROS Noetic

### Dependencies
- PCL

- Eigen

- Boost

- Livox_ros_driver, Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

The drone detection requires the point cloud to be in PointCloud2 format. For the conversion use [this](https://github.com/koide3/livox_to_pointcloud2) repository and follow the instructions. If you use a different repository, change the value for the ```self.lidar_sub``` variable in the ```livox_to_img.py``` file accordingly.

### Build
```
  cd ~/catkin_ws/src
  git clone https://github.com/TIERS/dynamic_scan_tracking
  cd ..
  catkin build
  ```

## Run
  ```
  roslaunch dynamic_scan_tracking dynamic_scan_tracking.launch
  ```

To enable/disable the drone detection modify the parameters in the ```config.yaml``` file.

## Citation
If you use this code for any academic work, please cite the following publication:

```
@misc{catalano2023uav,
    title={UAV Tracking with Solid-State Lidars:Dynamic Multi-Frequency Scan Integration}, 
    author={Iacopo Catalano and Ha Sier and Xianjia Yu and Jorge Pena Queralta and Tomi Westerlund},
    year={2023},
    eprint={2304.12125},
    archivePrefix={arXiv},
    primaryClass={cs.RO}
}
```