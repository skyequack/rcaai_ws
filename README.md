# GraspNet Simulation Stack

A comprehensive ROS2-based simulation stack for robotic grasping using the UR5 robot arm with GraspNet deep learning algorithms.

## Overview

This project provides a complete simulation environment for developing and testing robotic grasping algorithms. The system is built around a UR5 robot arm and includes perception, motion planning, and control components.

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    GraspNet System                          │
├─────────────────────────────────────────────────────────────┤
│  graspnet_bringup/     │  System orchestration & launch    │
│  graspnet_hardware/    │  Hardware interfaces              │
│  graspnet_description/ │  Robot models & URDF files       │
│  graspnet_gazebo/      │  Simulation environments         │
│  graspnet_moveit/      │  Motion planning configuration    │
│  graspnet_control/     │  Robot control & grasp planning  │
│  graspnet_perception/  │  Vision & object detection       │
└─────────────────────────────────────────────────────────────┘
```

## Package Structure

### Core Packages

- **`graspnet_hardware/`** - Hardware interface package for real robot integration
- **`graspnet_description/`** - Robot description files (URDF, meshes, configurations)
- **`graspnet_gazebo/`** - Gazebo simulation worlds and models
- **`graspnet_moveit/`** - MoveIt motion planning configuration
- **`graspnet_control/`** - Robot control and grasp planning algorithms
- **`graspnet_perception/`** - Computer vision and object detection
- **`graspnet_bringup/`** - Launch files and system orchestration

### Key Features

- **Modular Design**: Each component is separate and can be developed independently
- **Scalable Architecture**: Easy to add new sensors, algorithms, or robot configurations
- **Simulation Ready**: Complete Gazebo integration with physics simulation
- **Real Hardware Ready**: Hardware interfaces prepared for real robot deployment
- **ROS2 Native**: Built with ROS2 best practices and modern launch system

## Quick Start

### Prerequisites

- ROS2 (Humble)
- Gazebo (Garden or later)
- MoveIt2
- Python 3.8+
- OpenCV
- NumPy

### Installation

1. Clone the repository:
```bash
cd ~/rcaai_ws/src
git clone https://github.com/skyequack/rcaai_ws 
```

2. Install dependencies:
```bash
cd ~/rcaai_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Clone repos:
```bash
cd ~/rcaai_ws/src
source clone_repos.sh
```

4. Build the workspace:
```bash
colcon build
source install/setup.bash
```

### Running the System

#### Full System Launch
```bash
ros2 launch graspnet_bringup graspnet_system.launch.py
```

#### Simulation Only
```bash
ros2 launch graspnet_bringup simulation.launch.py
```

#### Individual Components
```bash
# Robot visualization
ros2 launch graspnet_description display.launch.py

# Gazebo simulation
ros2 launch graspnet_gazebo gazebo.launch.py

# Motion planning
ros2 launch graspnet_moveit moveit.launch.py
```

## Development Roadmap

### Phase 1: Basic Simulation 
- [x] Package structure creation
- [x] Basic URDF models
- [x] Gazebo world setup
- [x] Launch file integration

### Phase 2: Robot Integration (In Progress)
- [ ] UR5 robot description integration
- [ ] Gripper design and integration
- [ ] Joint controllers setup
- [ ] MoveIt configuration

### Phase 3: Perception System
- [ ] Camera sensor integration
- [ ] Point cloud processing
- [ ] Object detection implementation
- [ ] GraspNet algorithm integration

### Phase 4: Grasping Pipeline
- [ ] Grasp pose generation
- [ ] Motion planning integration
- [ ] Grasp execution
- [ ] Success evaluation

### Phase 5: Real Hardware
- [ ] Hardware interface implementation
- [ ] Real robot testing
- [ ] Performance optimization
- [ ] Safety features

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## File Structure

```
graspnet_simulation_stack/
├── graspnet_hardware/          # Hardware interfaces
│   ├── src/                    # C++ source files
│   ├── include/               # Header files
│   └── package.xml
│
├── graspnet_description/       # Robot models
│   ├── urdf/                  # URDF files
│   ├── meshes/                # 3D meshes
│   ├── config/                # Configuration files
│   └── launch/                # Launch files
│
├── graspnet_gazebo/           # Simulation
│   ├── worlds/                # Gazebo world files
│   ├── models/                # Custom models
│   ├── launch/                # Launch files
│   └── config/                # Gazebo configurations
│
├── graspnet_moveit/           # Motion planning
│   ├── config/                # MoveIt configurations
│   └── launch/                # Launch files
│
├── graspnet_control/          # Control & planning
│   ├── graspnet_control/      # Python modules
│   │   ├── robot_controller.py
│   │   └── grasp_planner.py
│   └── launch/                # Launch files
│
├── graspnet_perception/       # Vision system
│   ├── graspnet_perception/   # Python modules
│   │   ├── camera_manager.py
│   │   └── object_detector.py
│   └── config/                # Vision configurations
│
└── graspnet_bringup/          # System orchestration
    ├── launch/                # Main launch files
    └── config/                # System configurations
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Maintainer

- **Omer** - [omer.mitmpl2022@learner.manipal.edu](mailto:omer.mitmpl2022@learner.manipal.edu)

## Acknowledgments

- Universal Robots for UR5 robot specifications
- GraspNet research community
- ROS2 and MoveIt2 development teams
