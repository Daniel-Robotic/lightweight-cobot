# SunriseWorkbench setup

!!! info "Physical robot only"
    This section applies only when working with a physical KUKA LBR IIWA 7. For simulation, proceed to [Project installation](installation.md).

---

## Physical hardware setup

### Ethernet connection

Connect an Ethernet cable from your PC or control server to one of the KUKA controller's network ports:

- **KLI** (KUKA Line Interface) — the primary port used for control and programming;
- **KONI** (KUKA Optional Network Interface) — the additional port used for FRI.

Both ports can be connected at the same time. You can select the interface when configuring the server.

> Connect the KLI and KONI ports according to the KUKA controller wiring diagram.

---

## Synchronizing the SunriseWorkbench project

### Checking for ServerFriRos2

Make sure that your Sunrise project contains `ServerFriRos2.java`. If the file is missing, download it from the repository. Its path is `src/iiwa_sunrise/src/ServerFriRos2.java`.

=== "curl"

    ```bash
    curl -fsSL https://gitverse.ru/api/repos/daniel-robotics/lightweight-cobot/raw/branch/master/src/iiwa_sunrise/src/ServerFriRos2.java \
         -o ServerFriRos2.java
    ```

=== "wget"

    ```bash
    wget -O ServerFriRos2.java \
         https://gitverse.ru/api/repos/daniel-robotics/lightweight-cobot/raw/branch/master/src/iiwa_sunrise/src/ServerFriRos2.java
    ```

After downloading it, add the file to the Sunrise project and synchronize the project with the controller.

### Synchronizing with the controller

Open **SunriseWorkbench** and click the project synchronization button:

> In SunriseWorkbench, use the project synchronization button.

Before synchronizing, make sure that the PC and KUKA controller are on the same network. The current controller network settings can be checked directly in SunriseWorkbench:

> The controller network parameters are available in the SunriseWorkbench settings window.

---

## Configuring ServerFriRos2

Open `ServerFriRos2.java` in SunriseWorkbench and change the following parameters to match your network configuration:

```java
// IP address of the KONI interface
KONI_IP = "192.170.10.10";

// IP address of the KLI interface
KLI_IP  = "192.168.21.31";

// Zero position (all joints at 0°)
ZERO_POSITION = {0, 0, 0, 0, 0, 0, 0};

// Working position for monitoring
MONITOR_WORKING_POSITION = {0, 0, 0, -1.57, 0, 1.57, 0};

// Tool used by default
@Named("tool1")
```

!!! warning "Important"
    `KONI_IP` and `KLI_IP` in the Java program are addresses of the ROS 2 computer that the controller can reach through the corresponding networks. Conversely, `robot.ip` in `cobot-setting.yaml` is the address of the KUKA controller as seen from the computer. Incorrect or swapped addresses prevent the FRI connection from being established.

After making the changes, synchronize the project with the controller again.

---

**Next step:** [Connecting to the control server](remote-access.md)
