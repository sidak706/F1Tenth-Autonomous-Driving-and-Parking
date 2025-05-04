# 🚗 F1TENTH Autonomous Driving & Parking

## 🅿️ Autonomous Parking System

## Project Overview
Our project focused on developing an autonomous parking system that implements a forward parking algorithm. It does so through using LIDAR data and a finite state machine.

## 🧠 How It Works
The algorithm can be broken down into 4 states in a state machine.
- Drive forward until you reach a gap to park
- Reverse into the middle of the spot next to it
- Turn into the parking spot
- Straighten the car and stop

## 🛠️ How to Run the Code

```bash
ros2 run parking parking.py

### 📦 Prerequisites
- ROS2 (Foxy or later)
- F1Tenth Gym simulator
- Python 3.x or C++ compiler (depending on the implementation)

🎥 Demo

![Parking Demo](video.gif)
