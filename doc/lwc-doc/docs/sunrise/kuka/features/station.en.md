# Station

The **Station** section is the main navigation level of KUKA smartHMI. Open it by pressing **Station** in the smartPAD navigation bar. It provides access to the primary robot-cell control functions.

![Station main window](../../assets/kuka/station/main.jpg)

## Menu structure

The Station interface contains four functional areas:

| Area | Description |
|---|---|
| Navigation menu | Station, Applications, Robot menu, and IO Group menu |
| Station menu | Process data, Safety, Frames, KUKA_Sunrise_cabinet, HMI status, Information, and Log |
| Extra menu | Motion mode, clock, and user buttons |
| smartPAD function buttons | Program and motion control |

## Process data

**Process data** displays the current state of the active application, for example `Ok`. Use it to monitor parameters of the running program in real time.

## Safety

The **Safety** section provides access to robot safety-system settings and status.

![Safety](../../assets/kuka/station/safety1.jpg)

### Safety functions

| Function | Description |
|---|---|
| Status | Displays the current safety configuration state |
| Activation | Activates or deactivates the safety configuration |

### Actions on the Activation page

| Action | Description |
|---|---|
| Activate | Apply and activate the current safety configuration |
| Deactivate | Disable the active safety configuration |
| Reset | Restore the previous safety configuration state |

The **Safety configuration ID** field displays the unique identifier of the loaded configuration, for example `2BCAB6DD`.

![Safety — Activation](../../assets/kuka/station/safety2.jpg)

## Frames

The **Frames** section opens the coordinate-system editor.

![Frames overview](../../assets/kuka/station/frames.jpg)

It lists all frames defined in the Sunrise project and lets you inspect, correct, and navigate their hierarchy.

### Frame table structure

| Column | Description |
|---|---|
| Frame name | Frame name in the project |
| X, Y, Z | Axis offsets in millimeters |
| A, B, C | Orientation in degrees |

Frame data is also available in SunriseWorkbench.

### Navigation and correction

To open child frames, press **>** next to the required frame. The breadcrumb path updates automatically. Select an item in the breadcrumb to return to a previous level.

![Nested frames](../../assets/kuka/station/frames2.jpg)

Press **Correct** to open a dialog that compares current and new values. Press **Save** to confirm or **Cancel** to discard the changes.

![Correcting a frame](../../assets/kuka/station/frames3.jpg)

Frames support multiple nesting levels. The navigation bar displays the complete hierarchy path, for example `World > grant_RNF > P4`.

![Frame hierarchy](../../assets/kuka/station/frames4.jpg)

## KUKA_Sunrise_cabinet

**KUKA_Sunrise_Cabinet** displays the status of the controller hardware components.

![KUKA Sunrise Cabinet](../../assets/kuka/station/cabinet.jpg)

| Component | Description |
|---|---|
| Boot status | Controller boot status |
| Fieldbuses | EtherCAT bus status |

## HMI status

**HMI status** displays the connection state between smartHMI and the Sunrise Cabinet controller.

## Log

The **Log** section opens the system event log.

![Log](../../assets/kuka/station/protocol.jpg)

### Log filters

| Filter | Description |
|---|---|
| Source(s) | Station, LBR_iiwa_7_R800, or both |
| Level | Information, warning, or error |
| Time period | Time range to display |

Each entry contains a severity icon, event date and time, source, name, and description.

## Information

The **Information** section contains detailed system information about the controller and connected robot.

![Information](../../assets/kuka/station/info.jpg)

## smartPAD function buttons

The physical smartPAD buttons are divided into three groups: program control buttons on the left, manual axis control buttons on the right, and user buttons.

### Program control buttons

| Button | Description |
|---|---|
| Edit | Enters Teach mode and enables manual modification of program points |
| Stop | Stops program execution or robot motion |
| Backward step | Executes one program step in reverse; used for debugging |
| Start | Starts the selected application or resumes a stopped program; in T1/T2, the enabling device must be held |

!!! note
    Editing from the smartPAD is not used in this project. Programs are written in Java and changed only in SunriseWorkbench.

### Axis control buttons (T1 and T2)

| Button | Description |
|---|---|
| A1 − / A1 + | Move axis 1 in the negative or positive direction |
| A2 − / A2 + | Move axis 2 in the negative or positive direction |
| A3 − / A3 + | Move axis 3 in the negative or positive direction |
| A4 − / A4 + | Move axis 4 in the negative or positive direction |
| A5 − / A5 + | Move axis 5 in the negative or positive direction |
| A6 − / A6 + | Move axis 6 in the negative or positive direction |
| A7 − / A7 + | Move axis 7 in the negative or positive direction |

In Cartesian control mode, the same buttons move the TCP along X, Y, and Z and rotate it around A, B, and C.

### Speed control (Override)

| Button | Description |
|---|---|
| 0 | Decrease manual motion speed |
| 100 | Increase manual motion speed |

The value is displayed as a percentage of maximum speed. In T1 mode, TCP speed is hardware-limited to 250 mm/s.

### User buttons

Four white round buttons are located at the bottom of the left panel. Their behavior is programmed through the Sunrise project API. They are unassigned by default.

## Operating modes

| Mode | Description |
|---|---|
| T1 | Manual control with TCP speed limited to 250 mm/s; the enabling device must be held |
| T2 | Manual control at normal speed; the enabling device must be held |
| AUT | Automatic mode; axis buttons are unavailable and Start/Stop buttons control execution |

!!! tip "Extra menu"
    See [Extra menu](extra-menu.md) for additional control parameters.
