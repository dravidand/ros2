# ROSbot 2R Navigation Project
![ROSBot 2R](https://github.com/user-attachments/assets/ee375e10-59eb-4242-afa7-da5692c88dd9)

## Overview
This project explores autonomous navigation using the **ROSbot 2R**, implementing two distinct path-planning approaches to efficiently reach a target destination while avoiding obstacles. The goal was to measure **total travel time** and **distance covered** while ensuring smooth navigation in both simulated and real-world environments.

## Navigation Approaches 🚀
### 1️⃣ **Multi-Waypoint Navigation**
- The robot follows a **predefined set of waypoints**, optimizing the path to reach the destination efficiently.
- This approach enables flexible route planning and smooth movement across multiple checkpoints.
- It is particularly effective for **structured environments** where obstacle placement is predictable.

### 2️⃣ **Single-Coordinate Navigation**
- The robot directly moves toward a **single target coordinate** while actively avoiding obstacles.
- Suitable for **dynamic environments** where predefining waypoints is impractical.
- Provides a more adaptive response to unexpected obstacles and changes in surroundings.
<img width="957" height="724" alt="image" src="https://github.com/user-attachments/assets/474ff8b3-089b-406a-b5fc-f854931771c2" />

## Implementation 🔧
The project was implemented in **simulation** using **Gazebo** and **RViz**, followed by real-world deployment on the **ROSbot 2R**.

<img width="682" height="474" alt="image" src="https://github.com/user-attachments/assets/333c654e-4e4e-447f-968d-394935dc3b88" />


### 🔍 **Sensor Suite**
To achieve robust navigation, the robot utilized:
- **LiDAR** → Distance measuring and obstacle detection.
- **Time-of-Flight (ToF) Sensors (FL & FR)** → Short-range obstacle avoidance.
- **IMU** → Orientation and motion tracking.
- **Wheel Encoders** → Precise odometry data.
- **Camera** → Used for visualization purposes only.

### 🎯 **Simulation Results**
- Both algorithms were successfully tested in **Gazebo**.
- The robot consistently **reached the goal without collisions**, efficiently selecting routes based on sensor data.
- **Waypoint-based navigation** proved effective in structured layouts, while **single-coordinate navigation** adapted better to dynamic environments.
<img width="1344" height="756" alt="image" src="https://github.com/user-attachments/assets/18ad9b98-db87-46e2-af80-80401ed06976" />


### 📹 **Real-World Testing**
- The system was successfully transferred to a physical **ROSbot 2R**.
- Real-world performance aligned closely with simulation, demonstrating **robust obstacle avoidance and smooth navigation**.
- The results validated the **sensor fusion and path-planning** techniques implemented.
<img width="569" height="591" alt="image" src="https://github.com/user-attachments/assets/34f4f2ee-ed40-439f-be47-cb6d779f727c" />


## Key Takeaways 🎓
- **Multi-waypoint navigation** is better for predefined paths, while **single-coordinate navigation** offers adaptability.
- **Simulation closely matched real-world results**, showcasing the reliability of the approach.
- **Sensor fusion** played a crucial role in precise movement and real-time decision-making.

---

💡 **Next Steps:** Expanding the project to incorporate **dynamic obstacle avoidance using deep learning** and refining real-world implementation.

🚀 **Stay tuned for updates!**
