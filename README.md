cat > ~/drone_project/README.md << 'EOF'
# 🚁 Autonomous Drone Inspection System

A fully autonomous drone inspection system built with PX4 SITL, Gazebo Harmonic, MAVSDK Python, and ROS2 Jazzy — simulating real-world urban navigation between buildings.

## 🎯 Project Highlights
- **RRT* Path Planning** — finds optimal gap-weaving path through a city environment
- **Dynamic Obstacle Avoidance** — detects and replans route mid-mission in real time
- **Autonomous Takeoff, Navigation & Landing** — zero manual control
- **Simulated Photo Capture** — logs GPS coordinates at every waypoint
- **Pre/Post Flight Health Checks** — battery, EKF alignment, GPS lock validation

## 🛠️ Tech Stack
| Tool | Purpose |
|------|---------|
| PX4 SITL | Flight controller simulation |
| Gazebo Harmonic | 3D urban world simulation |
| MAVSDK Python | Drone communication & control |
| ROS2 Jazzy | Middleware |
| RRT* Algorithm | 3D path planning |
| Python asyncio | Async mission execution |

## 🏙️ Simulation Environment
Custom Gazebo world featuring:
- 10 houses with grass patches
- 14 trees (Oak & Pine)
- 4 cars
- Skyscrapers and mid-rise buildings
- Boundary walls enclosing a 96x96m city block

## 🚀 How to Run

### 1. Launch Simulation
```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 PX4_GZ_WORLD=mission_world make px4_sitl gz_x500
```

### 2. Run Mission
```bash
cd ~/drone_project
python3 full_mission.py
```

## 📁 Project Structure
