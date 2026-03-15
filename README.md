# 🦾 Senior Project: WhitePost - Main Arm Control

This repository contains the control system for a 5-DOF Robotic Arm and End Effector using ROS2 (Humble). The system is designed to run on a **Raspberry Pi 5** (acting as the Arm Station/Master) and a **Raspberry Pi Zero 2W** (acting as the End Effector/Vision node).

## 📌 Table of Contents

1. [Environment Setup](#1-environment-setup)
2. [Installation & Development Workflow](#2-installation--development-workflow)
3. [Remote Access (SSH)](#3-remote-access-ssh)
4. [Networking (Hotspot Configuration)](#4-networking-hotspot-configuration)
5. [Running the System (ROS2)](#5-running-the-system-ros2)
6. [Control Interface (ROS2 Topics)](#6-control-interface-ros2-topics)
7. [Hardware & Vision (Pi Zero)](#7-hardware--vision-pi-zero)
8. [Service Management (Auto-start)](#8-service-management-auto-start)
9. [Maintenance & Troubleshooting](#9-maintenance--troubleshooting)

---

## 1. Environment Setup

Before running any ROS2 commands, ensure you source the ROS2 installation and your local workspace:

```bash
# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source local workspace
source install/setup.bash
```

Tip: You can add these lines to your ~/.bashrc to load them automatically.

---

# 2. Installation & Development Workflow

*Initial Setup*

Clone the repository to your Raspberry Pi or local machine: