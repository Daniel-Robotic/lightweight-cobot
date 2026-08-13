# System architecture

!!! info "Work in progress"
    A detailed description of the system architecture is being prepared.

LWC is built as a set of interconnected ROS 2 packages. Key components:

- **iiwa_bringup** — launch files and the entry point for starting the system
- **iiwa_controller** — hardware interface connecting to the KUKA controller over FRI
- **iiwa_planning** — MoveIt 2-based motion planning
- **iiwa_web** — REST API and MCP server for external control
- **iiwa_description** — URDF robot description and Webots worlds
- **iiwa_config** — configuration files for MoveIt, controllers, and cameras
- **iiwa_utils** — helper Python utilities and configuration loading
- **iiwa_msgs** — custom ROS 2 message types (action and srv)
