---
hide:
  - navigation
  - toc
  - footer
---

<meta http-equiv="refresh" content="0; url=./getting-started/">
# Overview

**Lightweight Cobot (LWC)** is an open system for controlling the **KUKA LBR IIWA 7 R800** collaborative robot based on **ROS 2 Jazzy**. It supports both a physical robot through the FRI protocol and a virtual Webots simulation. The project includes the `cobot` CLI, a ROS 2 Control hardware interface, MoveIt 2 motion planning, and REST/MCP APIs for AI-agent integration.

## Repositories

| Platform | Link | Status |
|---|---|---|
| **GitVerse** (preferred) | [daniel-robotics/lightweight-cobot](https://gitverse.ru/daniel-robotics/lightweight-cobot) | primary |
| GitHub | [Daniel-Robotic/lightweight-cobot](https://github.com/Daniel-Robotic/lightweight-cobot) | mirror |

## Online documentation

- [GitVerse Pages](https://daniel-robotics.gitverse.site/lightweight-cobot) — primary
- [GitHub Pages](https://daniel-robotic.github.io/lightweight-cobot/) — mirror

## Requirements

- A physical **KUKA LBR IIWA 7 R800** for real-robot operation, or
- **Webots** if you only want to use simulation;
- A PC or server running **Ubuntu 24.04**, or SSH access to an existing server;
- Network access to the robot controller.

## Recommended order

1. [**Sunrise Workbench setup**](getting-started/sunrise-setup.md) — prepare the KUKA controller. *(Physical robot only.)*
2. [**Connect to the server**](getting-started/remote-access.md) — choose local or remote deployment and connect over SSH.
3. [**Install the project**](getting-started/installation.md) — install LWC using `curl`, `git`, or manually, then run `cobot setup`.
4. [**Configure the system**](getting-started/configuration.md) — set the robot IP, ports, and tools in `cobot-setting.yaml`.
5. [**cobot CLI**](getting-started/cli-reference.md) — review commands for running, building, and updating the project.

!!! tip "Simulation only?"
    If a physical robot is unavailable, skip step 1 and start with the [project installation](getting-started/installation.md). Launch the simulator with `cobot run --simulate`.

!!! info "`cobot setup` automates steps 3–4"
    After installing the project, run `cobot setup`. The wizard will configure the documentation, robot parameters, and the ROS 2 or Docker build environment.
