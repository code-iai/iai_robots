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

## Package structure

The hardware bringup uses a two-URDF pattern required by `ur_robot_driver`:

- **`urdf/stacy.urdf.xacro`** — plain description used by `display.launch.py` (no ros2_control tag).
- **`urdf/stacy_controlled.urdf.xacro`** — hardware description: same geometry plus the `<ros2_control>` hardware interface tag that `ur_control.launch.py` needs.
- **`launch/rsp.launch.py`** — custom robot state publisher launch that processes `stacy_controlled.urdf.xacro`. Passed to `ur_control.launch.py` via the `description_launchfile` argument, overriding the driver's default `ur_rsp.launch.py` so the full robot (column, camera, tool) is published on `/robot_description` instead of just the bare UR5.

## Launch

**RViz visualization (no hardware required):**
```bash
ros2 launch iai_stacy_description display.launch.py
```

**Real hardware bringup:**
```bash
ros2 launch iai_stacy_description hardware.launch.py
```
