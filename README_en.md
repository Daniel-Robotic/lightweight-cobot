# Lightweight Cobot

> **Draft documentation — work in progress**

ROS 2 packages for controlling the **KUKA LBR IIWA 7 R800**: communication with the real robot via [FRI](https://github.com/lbr-stack/fri) (Fast Robot Interface) and simulation in [Webots](https://cyberbotics.com/).

<table>
  <tr>
    <th align="center">LBR IIWA 7 R800</th>
  </tr>
  <tr>
    <td align="center">
      <img src="https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/jazzy/lbr_fri_ros2_stack/doc/img/foxglove/iiwa7_r800.png" alt="LBR IIWA 7 R800" width="300">
    </td>
  </tr>
</table>

---

## Status

| OS | ROS Distribution | FRI Version |
| :--- | :--- | :--- |
| `Ubuntu 24.04` | `jazzy` | `1.15` |

---

## Quick Start

Install everything with a single command:

```bash
bash <(curl -fsSL https://gitverse.ru/api/repos/daniel-robotics/lightweight-cobot/raw/branch/main/install.sh)
```

The script installs ROS 2 Jazzy, Webots, builds the workspace, and installs the `cobot` CLI.

---

## `cobot` Commands

After installation, manage the system via CLI:

```
cobot <command>
```

### Setup

| Command | Description |
| :--- | :--- |
| `cobot setup` | First-time setup: documentation server, build environment, robot config |
| `cobot local-setup` | Install ROS 2 Jazzy natively and build the project with colcon |
| `cobot docker-setup` | Build or pull Docker images |
| `cobot doc-setup` | Deploy or stop the MkDocs documentation server |
| `cobot robot-setup` | Configure `cobot-setting.yaml` interactively |

### Run

| Command | Description |
| :--- | :--- |
| `cobot run` | Launch the robot controller or Webots simulator (local or Docker) |

### Build

| Command | Description |
| :--- | :--- |
| `cobot rebuild` | Rebuild ROS 2 packages in `src/` with colcon |
| `cobot clean` | Remove colcon build artifacts (`build/` `install/` `log/`) |

### Management

| Command | Description |
| :--- | :--- |
| `cobot update` | Pull latest changes from the remote git branch and reinstall `cobot` |
| `cobot delete` | Remove the project, Docker images, containers, and optionally ROS 2 |

---

## Demo

> GIF animations will be added in upcoming releases

<table>
  <tr>
    <th align="center" width="33%">Webots Simulation</th>
    <th align="center" width="33%">Joint-Space Control</th>
    <th align="center" width="33%">Cartesian Control</th>
  </tr>
  <tr>
    <td align="center"><i>— coming soon —</i></td>
    <td align="center"><i>— coming soon —</i></td>
    <td align="center"><i>— coming soon —</i></td>
  </tr>
</table>

---

## Packages

| Package | Description |
| :--- | :--- |
| `iiwa_bringup` | Launch files: Webots simulation, real robot via FRI, MoveIt motion planning and RViz visualization |
| `iiwa_config` | Configuration files: MoveIt, ros2_control controllers, kinematics and general system settings |
| `iiwa_controller` | Hardware interface: real-time joint control via FRI within the ros2_control ecosystem |
| `iiwa_description` | URDF/XACRO robot description and Webots world configuration |
| `iiwa_msgs` | ROS 2 interfaces: action messages for joint-space and Cartesian motion, services for named poses |
| `iiwa_planning` | Motion planning: C++ and Python nodes built on MoveIt 2 (OMPL, Pilz, moveit_py) |
| `iiwa_utils` | System utilities: configuration loading, object and camera spawning in Webots, data conversion |
| `iiwa_web` | Web interface for monitoring and remote control of the cobot via browser |

---

## Citation

If you use this project in your work, please leave a star ⭐ and cite it:

```bibtex
@software{lightweight_cobot_2026,
  author  = {Hrabar, Daniil},
  title   = {Lightweight Cobot: ROS 2 stack for KUKA LBR IIWA 7},
  year    = {2026},
  url     = {https://gitverse.ru/daniel-robotics/lightweight-cobot}
}
```

---

## Acknowledgements

We gratefully acknowledge the support of the following organizations and grants:

| Organization | Notes |
| :--- | :--- |
| [Komsomolsk-on-Amur State University (KnAGU)](https://knastu.ru/) | Research was conducted at KnAGU |
| [Russian Science Foundation (RSF)](https://rscf.ru/) | Work supported by the Russian Science Foundation |
| <!-- TODO --> | <!-- TODO --> |
| <!-- TODO --> | <!-- TODO --> |
