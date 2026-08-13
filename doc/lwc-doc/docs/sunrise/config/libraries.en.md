# Installing libraries

Additional libraries distributed as `.zip` archives must be installed for the SunriseWorkbench project to work fully.

!!! note "Prerequisite"
    Before installing the libraries, make sure that the project has been created or loaded. See [Creating a new project](new-project.md) or [Loading an existing project](load-project.md).

## Opening the settings

In the SunriseWorkbench main menu, select **Window → Preferences**. In the window that opens, go to **Install/Update → Available Software Sites**.

![Window → Preferences menu](../assets/config/libraries/step-01.png)

## Adding library archives

Click **Add**. In the dialog, click **Archive...** and add a library `.zip` archive.

![Available Software Sites](../assets/config/libraries/step-02.png)

### Windows users

Use the standard file-selection dialog to locate and select the archive.

### Linux users

Because SunriseWorkbench runs in an emulated Windows environment, use the following procedure to access the Linux file system:

1. In the file-selection dialog, click **Look in:** and select **My Computer**.

    ![Adding a software source](../assets/config/libraries/step-03.png)

2. A list of mounted drives appears. It may contain more entries than the number of physical drives because of how the compatibility environment works.

    ![Selecting an archive](../assets/config/libraries/step-04.png)

3. Check each drive in turn. One of them contains the Linux file system (drive H in this example).

    ![My Computer](../assets/config/libraries/step-05.png)

4. Open the directory containing the libraries, select one of the `.zip` archives, and click **OK**.

    ![Drive list](../assets/config/libraries/step-06.png)

5. Confirm the selected archive by clicking **OK** in the next window.

    ![Linux file system](../assets/config/libraries/step-07.png)

## Adding the remaining archives

The archive appears in the **Available Software Sites** list. Repeat the procedure for every remaining library `.zip` file.

![Selecting an archive](../assets/config/libraries/step-08.png)

After adding all archives, click **OK** to save the settings.

## Installing the libraries

Select **Help → Install New Software...** from the menu. In the **Work with** field, select **All Available Sites**. Components from all added archives appear in the list.

![List of added archives](../assets/config/libraries/step-09.png)

Select every available component and click **Next**. Review the installation summary and click **Next** again.

![Help → Install New Software](../assets/config/libraries/step-10.png)

Accept the license agreements and click **Finish** to begin installation.

![Component list](../assets/config/libraries/step-11.png)

!!! warning "Installation duration"
    Installing the libraries may take a significant amount of time. Do not interrupt the process.
    ![Selecting components](../assets/config/libraries/step-12.png)

## Restarting the application

When installation finishes, SunriseWorkbench prompts you to restart. Click **Restart Now**.

![Installation process](../assets/config/libraries/step-13.png)

After the restart, the interface switches to Russian and all installed libraries appear in `StationSetup.cat`.

![Restart prompt](../assets/config/libraries/step-14.png)

!!! tip "Next step"
    If the project has not yet been configured, see [Creating a new project](new-project.md) or [Loading an existing project](load-project.md).
