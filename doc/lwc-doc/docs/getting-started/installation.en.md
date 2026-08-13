# Project installation

## Quick installation

The easiest option is to install the project with a single `curl` command. Make sure that `curl` is installed:

```bash
sudo apt update && sudo apt upgrade -y && sudo apt install curl
```

Go to your home directory and run the installation script:

=== "Stable version (master)"

    ```bash
    cd ~
    curl -fsSL https://gitverse.ru/api/repos/daniel-robotics/lightweight-cobot/raw/branch/master/install.sh | bash
    ```

=== "Development version (dev)"

    ```bash
    cd ~
    curl -fsSL https://gitverse.ru/api/repos/daniel-robotics/lightweight-cobot/raw/branch/dev/install.sh | bash -s dev
    ```

---

## Installation with Git

The project is available on both [GitVerse](https://gitverse.ru/daniel-robotics/lightweight-cobot) (preferred) and [GitHub](https://github.com/Daniel-Robotic/lightweight-cobot).

!!! tip "New to Git?"
    If this is your first time using Git and GitHub, see the [GitHub getting-started guide](https://docs.github.com/en/get-started/start-your-journey/hello-world).

Clone the repository and run the installation script:

=== "GitVerse"

    ```bash
    cd ~
    git clone https://gitverse.ru/daniel-robotics/lightweight-cobot.git
    cd ~/lightweight-cobot
    sudo chmod +x ./install.sh
    ./install.sh
    ```

=== "GitHub"

    ```bash
    cd ~
    git clone https://github.com/Daniel-Robotic/lightweight-cobot.git
    cd ~/lightweight-cobot
    sudo chmod +x ./install.sh
    ./install.sh
    ```

---

## Manual installation

If neither `curl` nor `git` is available, download the project archive manually from the repository page using the **Download ZIP** button, extract it, and run:

```bash
cd ~/lightweight-cobot
sudo chmod +x ./install.sh
./install.sh
```

---

## Installation process

The [`install.sh`](https://gitverse.ru/daniel-robotics/lightweight-cobot/raw/branch/master/install.sh) script automatically installs:

- **Git** — version control system;
- **Docker** — containerization for isolated execution;
- **`cobot` CLI** — the main project management tool;
- Ubuntu system dependencies.

### Restarting after installation

After the script finishes, a restart may be required for Docker to work:

```bash
sudo reboot now
```

Watch the terminal output: the script will indicate whether a restart is required.

### If the log is empty

If the script produces no output, reload the Bash environment and continue setup manually:

```bash
source ~/.bashrc   # refresh environment variables
cobot setup        # continue system setup
```

---

## Verifying the installation

After installation, make sure that `cobot` is available:

```bash
cobot -h
```

If the command displays the list of available subcommands, installation was successful.

---

**Next step:** [System configuration](configuration.md)
