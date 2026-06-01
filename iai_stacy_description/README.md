# iai_stacy_description

URDF description and bringup launch files for the Stacy robot at AICOR Institute for Artificial Intelligence, University of Bremen.

## Network Setup

To connect to the robot, plug an ethernet cable directly from the robot controller into your PC.
Then configure a static IPv4 interface on that network adapter:

| Field   | Value           |
|---------|-----------------|
| IP      | 192.168.1.10    |
| Netmask | 255.255.255.0   |
| Gateway | 192.168.1.1     |

The robot controller is reachable at `192.168.1.2`.

## Launch

**RViz visualization (no hardware required):**
```bash
ros2 launch iai_stacy_description display.launch.py
```

**Real hardware bringup:**
```bash
ros2 launch iai_stacy_description hardware.launch.py
```
