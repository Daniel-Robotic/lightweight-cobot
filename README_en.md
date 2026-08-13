# Lightweight Cobot

<p align="center">
  <a href="README.md">Русский</a> · <strong>English</strong>
</p>

**Lightweight Cobot (LWC)** is an open control system for the **KUKA LBR iiwa 7 R800** collaborative robot built on ROS 2. It combines physical robot control through FRI and `ros2_control`, a Webots digital twin, MoveIt 2 motion planning, RViz and Foxglove visualization, plus REST and MCP interfaces for external applications and AI agents.

<table>
  <tr>
    <th align="center">LBR iiwa 7 R800</th>
  </tr>
  <tr>
    <td align="center">
      <img src="https://raw.githubusercontent.com/lbr-stack/lbr_fri_ros2_stack/jazzy/lbr_fri_ros2_stack/doc/img/foxglove/iiwa7_r800.png" alt="LBR iiwa 7 R800" width="300">
    </td>
  </tr>
</table>

## Problems solved by the project

- Provides one software stack for both the physical robot and simulation without duplicating control code.
- Connects KUKA Sunrise Cabinet to ROS 2 through FRI and exposes standard `ros2_control` interfaces.
- Executes joint-space and Cartesian motions using MoveIt 2, OMPL, and Pilz.
- Simplifies installation, configuration, builds, and startup through the `cobot` CLI.
- Keeps the main robot, tool, and service parameters in one `cobot-setting.yaml` file.
- Provides monitoring and integration through RViz, Foxglove, HTTP/WebSocket APIs, and MCP.

## Features

| Component | Purpose |
|---|---|
| Physical robot | KUKA LBR iiwa 7 R800 control through FRI and `ServerFriRos2` |
| Digital twin | Robot, tool, and environment simulation in Webots |
| Motion planning | Joint-space and Cartesian trajectories through MoveIt 2 |
| Control | `ros2_control`, ROS 2 actions/services, REST API, and MCP |
| Monitoring | RViz, Foxglove, and system state through the web interface |
| Infrastructure | Native or Docker environment, a unified CLI, and centralized configuration |

## Compatibility

| Component | Supported version |
|---|---|
| Operating system | **Ubuntu 24.04 LTS** — verified for native installation |
| ROS 2 | Jazzy |
| Webots | 2025a |
| CLI Python | 3.11 |
| KUKA Sunrise OS | 1.16 |
| KUKA FRI | 1.16 |

Docker is available as an alternative environment on a compatible Linux host. Full Windows and macOS support is not claimed. Sunrise Workbench is used separately to prepare and synchronize the KUKA controller project.

## Repositories and documentation

| Resource | Link |
|---|---|
| Primary repository | [GitVerse](https://gitverse.ru/daniel-robotics/lightweight-cobot) |
| Mirror | [GitHub](https://github.com/Daniel-Robotic/lightweight-cobot) |
| Online documentation | [GitVerse Pages](https://daniel-robotics.gitverse.site/lightweight-cobot/) |
| Documentation mirror | [GitHub Pages](https://daniel-robotic.github.io/lightweight-cobot/) |

The detailed guide starts on the [Overview](doc/lwc-doc/docs/getting-started/index.md) page. Documentation sources are stored under `doc/lwc-doc/docs`.

## Quick start

### Requirements

- Ubuntu 24.04 LTS;
- internet access;
- `sudo` privileges;
- a physical KUKA LBR iiwa 7 R800, or a computer if only the simulator will be used.

### Install the CLI

Run the installer:

```bash
curl -fsSL https://gitverse.ru/api/repos/daniel-robotics/lightweight-cobot/raw/branch/master/install.sh | bash
```

The installer checks the basic tools, installs Docker, `uv`, and Python 3.11 when required, clones the project into `~/.lwc`, and installs the `cobot` CLI. Set `COBOT_INSTALL_DIR` to use a different location.

Open a new terminal or reload the shell environment, then start the first-time setup wizard:

```bash
cobot setup
```

The wizard offers to start the local documentation, configure `cobot-setting.yaml`, and prepare either a native ROS 2 or Docker build environment.

### Simulation only

A physical robot and Sunrise Workbench are not required for Webots simulation. Run:

```bash
cobot run
```

Choose the native or Docker environment, then select **Webots simulator**.

### Physical robot

Before the first run, prepare the controller and `ServerFriRos2` as described in [Sunrise Workbench setup](doc/lwc-doc/docs/getting-started/sunrise-setup.md). Verify the KONI/KLI network, IP addresses, FRI period, selected tool, and its Load Data.

Then run:

```bash
cobot run
```

Choose the native or Docker environment, then select **Physical controller**. See the [ServerFriRos2](doc/lwc-doc/docs/sunrise/kuka/programs/server-fri-ros2.md) page for details about the controller-side application.

## Main commands

| Command | Purpose |
|---|---|
| `cobot setup` | Configure documentation, robot parameters, and the build environment |
| `cobot robot-setup` | Edit `cobot-setting.yaml` interactively |
| `cobot local-setup` | Install ROS 2 Jazzy and build the workspace natively |
| `cobot docker-setup` | Pull or build the Docker images |
| `cobot run` | Select an environment and launch the physical robot or Webots interactively |
| `cobot run local` | Use native ROS 2, then select the physical robot or Webots |
| `cobot run docker` | Use Docker, then select the physical robot or Webots |
| `cobot rebuild` | Rebuild the ROS 2 workspace |
| `cobot clean` | Remove the `build`, `install`, and `log` artifacts |
| `cobot update` | Update the project and reinstall the CLI |
| `cobot --help` | Show every available command |

## Local documentation

Docker is required for the local documentation server.

```bash
cobot doc-setup
```

By default, the site is available at [http://localhost:8000](http://localhost:8000). Markdown source changes are watched automatically.

| Command | Purpose |
|---|---|
| `cobot doc-setup` | Start the local documentation server |
| `cobot doc-setup build` | Build the static site and combined PDF under `doc/lwc-doc/site` |
| `cobot doc-setup rebuild` | Rebuild the Docker image and restart the server |
| `cobot doc-setup down` | Stop the local server |

The online documentation is hosted on [GitVerse Pages](https://daniel-robotics.gitverse.site/lightweight-cobot/), with a mirror on [GitHub Pages](https://daniel-robotic.github.io/lightweight-cobot/).

## Packages

| Package | Description |
|---|---|
| `iiwa_bringup` | Launch files for Webots, the physical FRI robot, MoveIt, and RViz |
| `iiwa_config` | MoveIt, `ros2_control`, kinematics, and shared configuration files |
| `iiwa_controller` | Real-time FRI hardware interface for `ros2_control` |
| `iiwa_description` | URDF/Xacro robot description, meshes, tools, and Webots worlds |
| `iiwa_msgs` | ROS 2 actions and services for joint, Cartesian, and named-pose motions |
| `iiwa_planning` | C++ and Python motion nodes based on MoveIt 2, OMPL, Pilz, and `moveit_py` |
| `iiwa_utils` | Configuration loading, data conversion, and Webots object/camera utilities |
| `iiwa_web` | REST API, WebSocket, and MCP interfaces for monitoring and external control |

Java applications for KUKA Sunrise Cabinet live separately under `src/iiwa_sunrise` and are not part of the colcon build.

## Safety

Before commanding the physical robot, verify the work area, joint limits, active tool, load model, and selected control mode. LWC does not replace KUKA safety functions, a robotic-cell risk assessment, or operator supervision.

## License

This project is available under the [Apache License 2.0](LICENSE).

## Citation

If you use the project in research or development, cite the repository:

```bibtex
@software{lightweight_cobot_2026,
  author  = {Hrabar, Daniil},
  title   = {Lightweight Cobot: ROS 2 stack for KUKA LBR IIWA 7},
  year    = {2026},
  url     = {https://gitverse.ru/daniel-robotics/lightweight-cobot}
}
```

## Acknowledgements

| Organization | Notes |
|---|---|
| [Komsomolsk-on-Amur State University (KnAGU)](https://knastu.ru/) | Research was conducted at KnAGU |
| [Russian Science Foundation (RSF)](https://rscf.ru/) | Work supported by the Russian Science Foundation |
