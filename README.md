# QR-DQN Robot Simulation Assets

This repository provides the simulation assets used in the paper:

**"DESIGN A PATH – PLANNING STRATEGY FOR MOBILE ROBOT IN MULTI-STRUCTURED ENVIRONMENT BASED ON DISTRIBUTIONAL REINFORCEMENT LEARNING"**  


| <img src="https://github.com/phamduyaaaa/qr-dqn-robot-simulation-assets/blob/main/src/file_launch/image_map/resized/file_DiffrobotAndMap_crop.png?raw=true" height="175"/> | <img src="https://github.com/phamduyaaaa/qr-dqn-robot-simulation-assets/blob/main/src/file_launch/image_map/resized/file_all_crop.png?raw=true" height="175"/> | <img src="https://github.com/phamduyaaaa/qr-dqn-robot-simulation-assets/blob/main/src/file_launch/image_map/resized/file_mapSmallAndDiffrb_crop.png?raw=true" height="175"/> | <img src="https://github.com/phamduyaaaa/qr-dqn-robot-simulation-assets/blob/main/src/file_launch/image_map/resized/file_mapSmallPersonAndDiffrb_crop.png?raw=true" height="175"/> |
|---|---|---|---|
| file_DiffrobotAndMap | file_all | file_mapSmallAndDiffrb | file_mapSmallPersonAndDiffrb |

| <img src="https://github.com/phamduyaaaa/qr-dqn-robot-simulation-assets/blob/main/src/file_launch/image_map/resized/file_mapSmall_2AndDiffrb_crop.png?raw=true" height="175"/> | <img src="https://github.com/phamduyaaaa/qr-dqn-robot-simulation-assets/blob/main/src/file_launch/image_map/resized/map1_crop.png?raw=true" height="175"/> | <img src="https://github.com/phamduyaaaa/qr-dqn-robot-simulation-assets/blob/main/src/file_launch/image_map/resized/map2_crop.png?raw=true" height="175"/> | <img src="https://github.com/phamduyaaaa/qr-dqn-robot-simulation-assets/blob/main/src/file_launch/image_map/resized/map4_crop.png?raw=true" height="175"/> |
|---|---|---|---|
| file_mapSmall_2AndDiffrb | map1 | map2 | map4 |

---

## Contents
```
src/
├── file_launch/                     # Launch files and map images
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── image_map/                   # Sample environment images
│   │   ├── file_all.png
│   │   ├── file_DiffrobotAndMap.png
│   │   ├── file_mapSmall_2AndDiffrb.png
│   │   ├── file_mapSmallAndDiffrb.png
│   │   ├── file_mapSmallPersonAndDiffrb.png
│   │   ├── small_1.png
│   │   ├── small_2.png
│   │   ├── small3.png
│   │   └── small4.png
│   └── launch/                      # ROS launch files
│       ├── file_all.launch
│       ├── file_DiffrobotAndMap.launch
│       ├── file_map.launch
│       ├── file_mapAndModelperson.launch
│       ├── file_mapSmallAndDiffrb.launch
│       ├── file_mapSmall_2AndDiffrb.launch
│       ├── file_mapSmall_3AndDiffrb.launch
│       ├── file_mapSmallPersonAndDiffrb.launch
│       ├── small1.launch
│       ├── small2.launch
│       ├── small3.launch
│       └── small4.launch
│
├── model_robot/                     # Robot model definition
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── meshes/
│   │   ├── base_link.STL
│   │   ├── cover_link.STL
│   │   ├── wheels/
│   │   │   ├── left_tire.stl
│   │   │   └── right_tire.stl
│   │   └── sensors/                 # Sensor meshes (LiDAR, camera, etc.)
│   │       ├── astra.dae
│   │       ├── HDL32E_base.dae
│   │       ├── HDL32E_base.stl
│   │       ├── HDL32E_scan.dae
│   │       ├── HDL32E_scan.stl
│   │       ├── hokuyo.dae
│   │       ├── kinectv2.dae
│   │       ├── kinectv2.stl
│   │       ├── kinectv2_nobase.dae
│   │       ├── kinectv2_nobase.stl
│   │       ├── lds.stl
│   │       ├── r200.dae
│   │       ├── VLP16_base_1.dae
│   │       ├── VLP16_base_1.stl
│   │       ├── VLP16_base_2.dae
│   │       ├── VLP16_base_2.stl
│   │       ├── VLP16_scan.dae
│   │       └── VLP16_scan.stl
│   └── urdf/
│       └── diffbot_realsize.urdf
│
└── mymap/                           # Simulation maps
    ├── CMakeLists.txt
    ├── package.xml
    ├── replace_path.py
    ├── control_model_person/         # Scripts for human models
    │   ├── control_personStanding.py
    │   ├── control_personWalking.py
    │   └── control_personWalking_mS3.py
    └── src/
        ├── map.world
        ├── map_small.world
        ├── map_small_person.world
        ├── map_modelperson.world
        ├── map_Small_2.world
        ├── map_Small_3.world
        ├── small_1.world
        ├── small_2.world
        ├── small3.world
        └── small4.world

```
---

## Usage Instructions

### 1. Clone the Repository
Download the external models used in the simulation and place them in your home directory:
```bash
git clone https://github.com/phamduyaaaa/qr-dqn-robot-simulation-assets.git
```
### 2. Build the Workspace
From your catkin workspace root:    
```bash
catkin_make
```
### 3. Adjust Device Username in Map Files
Some .world files contain absolute paths.    
To update these paths to your local username:    
#### Open mymap/replace_path.py.
#### Set the variable device_name to your system username.
#### Run the script:
```bash
python3 replace_path.py
```
### 4. Launch a Simulation Example
Use the corresponding launch file. For example:    
```bash
roslaunch filelaunch file_map.launch
```

## Notes

This repository provides simulation assets (SDF/World maps and URDF models) to ensure reproducibility.

The QR-DQN training and evaluation code is not included here but is available upon request from the corresponding author.

The external models folder is required for correct rendering of robots and environments.

## Citation

If you use these assets in your research, please cite:

Nguyen, A.-T., et al. (2025). Design a Path–Planning Strategy for Mobile Robot in Multi-Structured Environment Based on Distributional Reinforcement Learning.

## Contact

For questions or requests (including access to the training code and configuration files), please contact:   

Nguyen Anh Tu – Corresponding Author    

Email: tuna@haui.edu.vn

## License
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)  
This repository is licensed under the [MIT License](./LICENSE).  
You are free to use, modify, and distribute this work for research and educational purposes, provided that proper credit is given.
