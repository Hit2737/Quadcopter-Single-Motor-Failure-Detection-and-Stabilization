
# PX4 Drone Simulation with ROS 2 and Gazebo

This repository provides a workspace for simulating and controlling an Iris drone using PX4, ROS 2, and Gazebo. It includes packages for low-level motor control, motor failure detection using LSTM neural networks, and ROS 2 interfaces for communication with PX4.

## Workspace Structure

```
.
├── .gitignore
├── .gitmodules
├── build/
├── install/
│   ├── setup.bash
│   └── ...
├── log/
├── README.md
└── src/
    ├── px4_model/
    │   ├── dataset/
    │   │   ├── DATA_0.csv
    │   │   ├── DATA_1.csv
    │   │   └── ...
    │   ├── models/
    │   │   └── Failure-Detection-0.pth
    │   └── failure_model/
    │       └── model.ipynb
    ├── px4_motor_control/
    │   ├── launch/
    │   │   └── iris_sitl.launch.py
    │   ├── include/
    │   │   └── px4_motor_control/
    │   │       ├── controller.h
    │   │       └── controller_node.h
    │   └── src/
    │       ├── controller_node.cpp
    │       └── controller.cpp
    ├── px4_msgs/
    └── px4_ros_com/
```

### Key Packages

1. **px4_model**: Implements LSTM-based motor failure detection.
2. **px4_motor_control**: Provides low-level control of the drone in offboard mode.
3. **px4_msgs**: Contains ROS 2 message definitions for PX4.
4. **px4_ros_com**: Provides ROS 2-PX4 interface for seamless data exchange.

## Installation and Setup

### Prerequisites

- Ubuntu 22.04
- ROS 2 (Humble)
- PX4-Autopilot
- Gazebo Classic
- PlotJuggler (optional for data visualization)

### Steps to Install
```bash
cd ROS_PX4
colcon build --symlink-install
```

## Running the Simulation

To launch the Iris drone simulation in Gazebo:

```bash
cd PX4-Autopilot
make px4_sitl gazebo-classic_iris
```

### Launching Motor Failure Detection

The `px4_model` package includes an LSTM model for motor failure detection. To collect data and train the model:

1. **Collect data**:
   ```bash
   ros2 run px4_model collector.py
   ```
   
2. **Train the model**:
   - Open `src/px4_model/failure_model/model.ipynb`
   - Follow the instructions in the notebook to preprocess data, train, and evaluate the model.

### Visualization with PlotJuggler

For real-time visualization of thrust, torque commands, and position tracking:
```bash
ros2 run plotjuggler plotjuggler
```

## Acknowledgments

This project is uses the [Github Repository](https://github.com/SaxionMechatronics/px4_offboard_lowlevel) by the SMART research group at Saxion University of Applied Sciences and the PX4 community.

For more information, visit the [PX4 documentation](https://docs.px4.io/).

---

Enjoy your PX4 simulation and control experience!
