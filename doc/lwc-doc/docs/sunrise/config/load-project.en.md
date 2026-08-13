# Loading a project from the controller

This section explains how to import an existing project directly from the KUKA controller into SunriseWorkbench.

!!! note "Prerequisite"
    Make sure that SunriseWorkbench is installed and running. See the [Windows](../sun_install/windows.md) and [Linux](../sun_install/linux/linux.md) installation guides.

## Starting the project import wizard

In the main SunriseWorkbench window, click **New Sunrise Project**.

![Main SunriseWorkbench window](../assets/config/new-project/step-01.png)

In the dialog that opens, select **Load project from controller** and enter the IP address of the KUKA Sunrise Cabinet controller.

![Controller connection dialog](../assets/config/load-config/step_01.png)

!!! info "Controller IP address"
    The default controller IP address is `172.31.1.147`. If the controller has been reconfigured, enter its current address. This configuration uses `192.168.21.147`. To find the current address, see [Station configuration](../kuka/features/station.md).

Click **Next** to begin downloading the project from the controller.

![Project download process](../assets/config/load-config/step_02.png)

When the download is complete, the imported project appears in the SunriseWorkbench project tree. In this example, the project is named `SunriseProject`.

![Imported project in the workspace](../assets/config/load-config/step_03.png)

!!! tip "Next step"
    All required libraries must be installed for full robot operation. See [Installing libraries](libraries.md).
