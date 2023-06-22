# UAV Tracking with Solid-State Lidars: Dynamic Multi-Frequency Scan Integration

Image system Pipeline

This is the code implementation for the paper [UAV Tracking with Solid-State Lidars: Dynamic Multi-Frequency Scan Integration](https://arxiv.org/abs/2304.12125).

If you use any of this code, please cite the following publication:

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

# Dependencies

Livox ROS Drivers
Livo Undistortion

# Introduction


# RUN with rosbags:
## Compile
  ```
  cd ~/catkin_ws/src
  git clone https://github.com/TIERS/multi-modal-loam.git
  cd ..
  catkin_make -DCATKIN_WHITELIST_PACKAGES="mm_loam"
  ```

## Play rosbag
```
rosbag play office_2022-04-21-19-14-23.bag --clock
```

## Run launch file
```
roslaunch mm_loam mm_lio_full.launch
```