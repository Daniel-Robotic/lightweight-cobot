# cobot CLI commands

`cobot` is the single entry point for managing the entire project. Use it for operations involving the robot, simulator, Docker containers, and documentation.

## Help

```bash
cobot -h
```

The help output is divided into four command groups:

- **Setup commands** — system configuration;
- **Run commands** — starting the project;
- **Build commands** — building ROS 2;
- **Management commands** — package management.

Some commands have their own subcommands. For example:

```bash
cobot doc-setup rebuild   # rebuild subcommand of doc-setup
cobot doc-setup --help    # help for a command's subcommands
```

---

## Setup commands

Commands for initial and repeated system setup.

### `cobot setup`

First-run setup wizard. It guides you through three steps:

1. Configure the documentation server.
2. Configure robot parameters in `cobot-setting.yaml`: IP address, FRI port, and tool.
3. Select a build environment: native ROS 2 Jazzy or a Docker image.

```bash
cobot setup
```

Use this command for the first installation instead of running each setup command manually.

---

### `cobot local-setup`

Installs ROS 2 Jazzy locally without Docker: downloads dependencies with `rosdep` and builds the workspace with `colcon`.

```bash
cobot local-setup
```

!!! note
    After the command finishes, run `source ~/.bashrc` or open a new terminal.

---

### `cobot docker-setup`

Builds or downloads the Docker images used to run the project in isolation. Two options are available:

- **Build from the Dockerfile** — slower, but produces an up-to-date image;
- **Download a prebuilt image** — faster, using a published image.

```bash
cobot docker-setup
```

---

### `cobot doc-setup`

Deploys a local MkDocs documentation server containing a full copy of the [online documentation](https://daniel-robotics.gitverse.site/lightweight-cobot/).

```bash
cobot doc-setup          # start/build the documentation
cobot doc-setup rebuild  # rebuild the documentation image
```

After startup, the documentation is available at `http://localhost:8000`.

---

### `cobot robot-setup`

Interactive wizard for configuring `cobot-setting.yaml`. It asks for the robot IP address, FRI port, active tool, and other parameters.

```bash
cobot robot-setup
```

!!! tip
    Use this command to change the configuration. It validates the entered values and prevents YAML syntax errors. See [System configuration](configuration.md) for parameter details.

---

## Run commands

### `cobot run`

Starts the full stack: hardware interface, MoveIt 2, RViz, and optional components such as Foxglove and the REST API. At startup, it asks you to select:

- **Docker or local execution**;
- **Webots simulation or the physical robot**.

```bash
cobot run                 # select the mode interactively
cobot run --simulate      # force simulation mode
```

---

## Build commands

### `cobot rebuild`

Rebuilds the ROS 2 workspace with `colcon`. Use it after changing package source code.

```bash
cobot rebuild
```

!!! note
    This command is available only for a local installation, not Docker. It is equivalent to `colcon build --mixin release`.

---

### `cobot clean`

Removes generated build directories. It prompts you to select which directories to remove:

- `build/` — compilation artifacts;
- `install/` — installed package files;
- `log/` — build logs.

```bash
cobot clean
```

---

## Management commands

### `cobot update`

Downloads the latest project version from GitVerse and reinstalls the `cobot` CLI.

```bash
cobot update
```

---

### `cobot delete`

Removes project components from the system. It lets you remove only the project, the Docker images and containers, or ROS 2 as well.

```bash
cobot delete
```

!!! danger
    This operation is irreversible. Removed files and Docker images must be installed again.

---

**Robot control:** [Control via the REST API](control/rest-api.md)
