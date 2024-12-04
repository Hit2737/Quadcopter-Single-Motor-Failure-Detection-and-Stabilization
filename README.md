# PX4 Drone Simulation with ROS 2 and Gazebo 🚁  

This repository provides a workspace for simulating and controlling an **Iris drone** using **PX4**, **ROS 2**, and **Gazebo Classic**. It includes packages for low-level motor control, motor failure detection, trajectory publishing, and real-time visualization. The workspace is designed to streamline drone simulation and testing, offering tools for motor control, parameter configuration, and motor failure analysis.  

---

## **Workspace Structure** 🗂️  

```
ROS_PX4/
├── src/
│   ├── px4_detector/              # Package for motor failure detection and trajectory control
│   │   └── px4_detector/
│   │       ├── trajectory_publisher.py  # Publishes trajectories for the drone ✈️
│   │       └── detector.py             # Motor failure detection logic ⚙️
│   ├── px4_motor_control/        # Low-level motor control package 🔧
│   │   ├── config/
│   │   │   ├── initial_gains_iris.yaml  # Initial PID gain values
│   │   │   ├── iris_param.yaml          # Iris drone parameters
│   │   │   └── sitl_params.yaml         # Simulation parameters
│   │   ├── include/
│   │   │   └── px4_offboard_lowlevel/
│   │   │       ├── controller_node.h    # Node header for motor control 🛠️
│   │   │       └── controller.h         # Main motor control logic
│   │   ├── launch/
│   │   │   └── setup.launch.py          # ROS 2 launch file for simulation 🚀
│   │   ├── src/
│   │   │   ├── controller_node.cpp      # ROS 2 node for motor control
│   │   │   └── controller.cpp           # Motor control implementation
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── px4_msgs/                  # ROS 2 custom messages for PX4 📬
│   │   └── msgs/
│   │       ├── ActionRequest.msg        # Custom message definitions
│   │       ...
│   └── px4_ros_com/               # Interface for ROS 2 and PX4 communication
├── .gitignore
├── .gitmodules
└── README.md
```

---

## **Features** ✨  

1. **Drone Simulation** 🛸:  
   Simulate the Iris drone in Gazebo Classic using PX4's SITL (Software in the Loop) environment.

2. **Low-Level Motor Control** 🔧:  
   Implement precise motor control using a ROS 2 node with configurable PID gains.

3. **Motor Failure Detection** ⚠️:  
   Simulate motor failures for testing robust flight control and fault tolerance.

4. **Trajectory Control** 📈:  
   Publish desired drone trajectories using Python scripts.

5. **Real-Time Visualization** 📊:  
   Visualize drone parameters like thrust, torque, and position tracking using PlotJuggler.

6. **Configurable Parameters** 🛠️:  
   Customize drone parameters, PID gains, and SITL settings via YAML files.

---

## **Installation and Setup**  

### **Prerequisites** ✅  

- **Operating System**: Ubuntu 22.04 🐧  
- **ROS 2**: Humble 🤖  
- **PX4-Autopilot**: Follow the [PX4 installation guide](https://docs.px4.io/main/en/index.html).  
- **Gazebo Classic**: Install for realistic drone simulation.  
- **PlotJuggler** *(optional)*: For real-time data visualization.  

---

### **Cloning the Repository** 📂  

```bash
git clone https://github.com/Hit2737/ROS_PX4.git
```

---

### **Building the Workspace** 🏗️  

1. Navigate to the workspace directory:  
   ```bash
   cd ROS_PX4
   ```

2. Build the workspace using `colcon`:  
   ```bash
   colcon build --symlink-install
   ```

---

## **Running the Simulation** 🎮  

### **1. Launch the Iris Drone in Gazebo Classic** 🌍  

Start the PX4 SITL simulation for the Iris drone:  
```bash
cd PX4-Autopilot
make px4_sitl gazebo-classic_iris
```

---

### **2. Source the Workspace** 🧩  

Before running any ROS 2 commands, source the workspace:  
```bash
source install/setup.bash
```

---

### **3. Launch the ROS 2 Setup** 🚀  

Run the main launch file for motor control and trajectory configuration:  
```bash
ros2 launch px4_motor_control setup.launch.py
```

---

### **4. Publish a Trajectory** ✈️  

In a new terminal, source the workspace and run the trajectory publisher:  
```bash
ros2 run px4_detector trajectory_publisher
```

---

### **5. Simulate Motor Failure** ⚙️  

To fail a specific motor during the simulation, use the following command:  
```bash
gz topic -p /gazebo/motor_failure_num -m "data: <motor_number>"
```

Replace `<motor_number>` with the motor index you want to fail (e.g., `2`).

---

### **6. Visualize Data with PlotJuggler** *(Optional)* 📊  

For real-time data visualization, launch PlotJuggler:  
```bash
ros2 run plotjuggler plotjuggler
```

---

## **Configuration Files** 🛠️  

The `px4_motor_control/config/` directory contains key configuration files:  

- **`initial_gains_iris.yaml`**:  
  Configures initial PID gains for motor control.  

- **`iris_param.yaml`**:  
  Drone-specific parameters such as mass, inertia, and motor coefficients.  

- **`sitl_params.yaml`**:  
  SITL-specific parameters for tuning the simulation environment.  

---

## **Acknowledgments** 🙌  

This repository is inspired by:  
- [SMART Research Group's Repository](https://github.com/SaxionMechatronics/px4_offboard_lowlevel).  
- The PX4 open-source community for providing tools and documentation.  

For detailed documentation and updates, visit the [PX4 website](https://docs.px4.io/main/en/index.html).  

---

## **Enjoy Your PX4 Drone Simulation!** 🚁💻  

Explore the possibilities of PX4, ROS 2, and Gazebo Classic for building robust drone systems. 🛸🎉  