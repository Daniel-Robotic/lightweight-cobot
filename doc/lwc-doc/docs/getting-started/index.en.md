# Getting started

This section explains how to prepare the environment, connect to the controller, install the project, and start controlling the robot.

## Quick start

1. [**Sunrise Workbench setup**](sunrise-setup.md) — prepare the KUKA controller, upload `ServerFriRos2`, and configure the network. *(Physical robot only.)*
2. [**Connect to the server**](remote-access.md) — choose local or remote deployment and connect to the control server over SSH.
3. [**Install the project**](installation.md) — install LWC using `curl`, `git`, or manually.
4. [**Configure the system**](configuration.md) — configure `cobot-setting.yaml` with the robot IP, ports, and tools.
5. [**cobot CLI**](cli-reference.md) — learn the commands for running, building, and updating the project.

!!! tip "Simulation only?"
    Skip the physical-controller setup and run `cobot run --simulate` after installation.

## Concepts and control

- [System architecture](concepts/architecture.md)
- [FRI protocol](concepts/fri-protocol.md)
- [Webots simulation](concepts/simulation.md)
- [Motion planning](concepts/motion-planning.md)
- [ROS 2 Control](control/ros2-control.md)
- [Foxglove](control/foxglove.md)
- [REST API](control/rest-api.md)
