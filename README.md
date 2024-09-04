iai_robots
==========

Repo holding descriptions and launch-files for robots in the iai lab.

This repository was cleaned up and many packages containing descriptions of robot peripherals were removed. The package can be found [here](https://github.com/code-iai/iai_robot_peripherals/tree/cleanup-repository).
Many packages in this repository still have run dependencies on the removed packages. If you need to use a robot from this repository, you might need to install the missing packages from the repository linked above.
The removed packages are:

* iai_adapter_iso50_kinect2_description
* iai_adapters_description
* iai_kinect2_description
* iai_kinect2_look_target
* iai_kinect_description
* iai_roller_description
* iai_schunk_pw070_v6
* iai_weiss_wsg50
* iai_wsg_50_description

If you experience any issues running the packages contained in this repository, clone the above mentioned repository and install the missing packages by running

```bash
git clone https://github.com/code-iai/iai_robot_peripherals.git
```
