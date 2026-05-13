# Daisy ( Dual-arm UR5) bringup Guide

This repository contains the configuration, launch files, and instructions required to bring up and control Daisy robot.

## Table of Contents
- [Hardware Setup](#hardware-setup)
- [Network Configuration](#network-configuration)
- [Teach Pendant Setup](#teach-pendant-setup)
- [Installation](#installation)
- [Bringup Instructions](#bringup-instructions)

---

## Hardware Setup

This repo was tested on following setup

* 2x Universal Robots UR5 (CB3 or e-Series) with Teach Pendants
* 1x Control PC with (Ubuntu 24.04)
* 1x Gigabit Ethernet Switch
* Ethernet cables for connecting both robots to the switch and the switch to the PC
---

## Network Configuration

This was the network configuration for this robot setup

* **Host PC IP:** `192.168.102.180` (Subnet: `255.255.252.0`)
* **Left Arm IP:** `192.168.102.44`
* **Right Arm IP:** `192.168.102.43`

---

## Teach Pendant Setup

After succesfully booting up the robot by pressing power button which turns Green when robot is on, follow below steps on Teaching Pendent

### 1. Initialize the robot,

When the robot is switched on, following screen will appear. When everything is safe and ready, select 'Go to Initialization screen' and release Emergency Stop button.

<img src="docs/image/initial_screen.png" width="400" alt="Initial Screen">

Once the robot is initialized, Select **ON** and then **START**. Robot's motors will be activated and then it will be ready-to-use.
Exit the `Initialize Robot` screen by selecting **OK** on bottom right corner of Teaching pendent.

<img src="docs/image/exit_menu.png" width="400" alt="Exit Initialize Robot">
### 2. Load External Program

In order work with ROS2, we need to load `external_control.urp` and then launch the ros2 launch file to bringup Daisy robot.

From below screen Select **Run Program** 

<img src="docs/image/run_program.png" width="400" alt="Run Program">

Then from top-left corner, select **File** and from drop-down menu, choose **Load Program**,

<img src="docs/image/load_program.png" width="400" alt="Load Program">

A file menu will appear and choose the file named `external_control.urp` and Open it. 

---

## Installation 

### Clone this repository into your workspace.
```bash
git clone https://github.com/code-iai/iai_robots.git -b ros-jazzy
```
### Install dependency
```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
```
### Build the workspace
```bash
colcon build
source install/setup.bash
```

---

## Bringup Instructions

### Launch Daisy bringup file
```bash
ros2 launch iai_daisy_description daisy_bringup.launch.py
```
Press Play button after launching ROS2 launch file and you're ready to go.

<img src="docs/image/external_control.png" width="400" alt="External Control">

  
