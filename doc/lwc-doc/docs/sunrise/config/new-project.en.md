# Creating a new project

This section explains how to create a SunriseWorkbench project and perform the initial configuration for a KUKA LBR IIWA 7.

!!! note "Prerequisite"
    Before creating a project, make sure that SunriseWorkbench is installed. See the [Windows](../sun_install/windows.md) and [Linux](../sun_install/linux/linux.md) installation guides.

## Starting the new-project wizard

In the main SunriseWorkbench window, click **New Sunrise Project**.

![Main SunriseWorkbench window](../assets/config/new-project/step-01.png)

## Configuring the controller connection

Enter the IP address of the KUKA Sunrise Cabinet controller in the dialog.

![Entering the controller IP address](../assets/config/new-project/step-02.png)

!!! warning "Controller IP address"
    The default controller IP address is `172.31.1.147`. If your configuration uses a different address, replace it with the current value.

## Project name

Enter the project name in the corresponding field.

![Entering the project name](../assets/config/new-project/step-03.png)

## Selecting the robot model

Select the robot model from the drop-down list. For a KUKA LBR IIWA 7, choose **LBR iiwa 7 R800**.

![Selecting the robot model](../assets/config/new-project/step-04.png)

## Selecting the flange

Select the flange type that matches your robot configuration.

![Selecting the flange](../assets/config/new-project/step-05.png)

!!! warning "Flange selection"
    The flange type must exactly match the physical robot configuration. This setup uses **Medien-Flansch elektrisch**. Leave its orientation at the default value of 0°.

## Reviewing the configuration

Review all parameters in the summary window. If they are correct, click **Finish**.

![Reviewing the parameters](../assets/config/new-project/step-06.png)

## Selecting an application template

After the project is created, the template selection dialog opens. Select one of the examples and click **Finish**.

![Selecting a template](../assets/config/new-project/step-07.png)

## Main editor window

After the wizard completes successfully, SunriseWorkbench opens the new project in its main editor window.

![Main editor window](../assets/config/new-project/step-08.png)

!!! tip "Next step"
    All required libraries must be installed for full robot operation. See [Installing libraries](libraries.md).
