# Robot menu

The **Robot** section is one of the main areas of KUKA smartHMI. It provides diagnostic robot-state information, mastering functions, tool and base calibration, and load parameters.

![Robot menu](../../assets/kuka/robot-menu/step-01.jpg)

The table below lists the main menu items.

| Item | Description |
|---|---|
| Axis position | Displays the current position of each robot axis in degrees |
| Cartesian position | Displays the current tool position in Cartesian coordinates |
| Axis torques | Displays the current torque on each robot axis |
| Mastering | Provides axis mastering and tool-offset teaching functions |
| Load data | Enters or calibrates the load parameters at the flange |
| Move enable | Displays the manual-motion enable signal state |
| Log | Displays events and errors; equivalent to the item under [Station](station.md) |
| Device state | Displays the current device state with a color indicator |
| Calibration | Provides tool and base calibration functions |

## Axis position

This section displays the current angular position of each of the robot's seven axes in degrees. Values update in real time. Software limits are also shown as minimum and maximum permitted values for each axis.

![Axis position](../../assets/kuka/robot-menu/step-02.jpg)

## Cartesian position

This section displays the tool center point (TCP) position in Cartesian coordinates relative to the selected base. The following parameters are available:

- **X, Y, Z** — linear TCP coordinates in millimeters;
- **A, B, C** — orientation angles in degrees.

!!! info "Angle-axis correspondence"
    **A** is rotation around Z, **B** around Y, and **C** around X, following the ZYX convention.

The current calculation context is also displayed:

- selected Tool;
- active TCP;
- selected Base.

Change these values under **Manual method options**; see [Extra menu](extra-menu.md). After selecting another tool, TCP, or base, the values on this page are recalculated accordingly.

![Cartesian position](../../assets/kuka/robot-menu/step-03.jpg)

## Axis torques

This section displays current torque on each of the seven axes in newton-meters (Nm). Values update in real time. This information lets the operator:

- monitor the load on each axis;
- diagnose possible mechanical problems;
- identify characteristic torque values for later control-program tuning.

![Axis torques](../../assets/kuka/robot-menu/step-04.jpg)

## Mastering

Mastering maps the mechanical robot position to its software model. Without correct mastering, software coordinates do not match the actual axis positions.

The main mastering menu provides functions for updating mastering data, unmastering individual axes, and teaching tool offsets.

![Main mastering menu](../../assets/kuka/robot-menu/step-05.jpg)

Use **Update mastering data** to save new mastering values after completing the procedure. The controller records the current mechanical axis positions as references.

![Updating mastering data](../../assets/kuka/robot-menu/step-06.jpg)

**Unmaster** removes mastering data from a selected axis. An unmastered axis is considered uncalibrated and may move beyond software limits.

![Unmastering an axis](../../assets/kuka/robot-menu/step-07.jpg)

!!! warning "When should an axis be unmastered?"
    Unmaster an axis if it reaches a software limit and cannot continue moving. After moving it away from the limit, master it again to restore correct robot operation.

**Teach tool offset** applies a correction to an axis zero position without repeating the complete mastering procedure. Use it for small mechanical offsets.

![Teaching a tool offset](../../assets/kuka/robot-menu/step-08.jpg)

!!! warning "Important"
    Select the tool whose offset will be taught before activating this function.

## Load data

Correct load parameters are required for accurate motion planning, prevention of axis overload, and proper operation of Power and Force Limiting (PFL).

The main load-data menu lists the available tool slots. Load parameters can be entered or calibrated for each tool.

![Main load-data menu](../../assets/kuka/robot-menu/step-09.jpg)

Under **Determine load data**, enter or automatically determine mass, center of mass, and inertia tensor.

![Determining load data](../../assets/kuka/robot-menu/step-10.jpg)

The **Tool mass calibration** procedure lets the controller measure the attached tool mass automatically by performing test motions. Follow the on-screen instructions.

![Tool mass calibration](../../assets/kuka/robot-menu/step-11.jpg)

When the procedure finishes, the controller displays the measured load parameters. Review them and verify that they match the actual tool characteristics.

![Calibration results](../../assets/kuka/robot-menu/step-12.jpg)

After confirmation, the controller saves and immediately applies the updated load data.

![Updated load data](../../assets/kuka/robot-menu/step-13.jpg)

## Move enable

This item displays the state of the manual-motion enable signal. The signal is activated by pressing the enable button on the manipulator body. smartHMI indicates the active state by changing the axis indicators from gray to white.

!!! note "Mode limitation"
    Move enable is unavailable in **automatic mode** (AUT). The signal is active only in manual modes T1 and T2.

## Log

This section is equivalent to **Log** under [Station](station.md). It displays controller events, warnings, and errors so that the operator can review their chronology and diagnose faults.

## Device state

This section displays the current device state with a color indicator:

| Color | State |
|---|---|
| Green | The device is operating normally |
| Yellow | A warning or potential issue requires attention |
| Red | A critical error or fault has been detected |

## Calibration

The **Calibration** section contains procedures for determining the geometric parameters of tools and bases used by the controller to calculate Cartesian coordinates.

The main menu contains two categories: base calibration and tool calibration.

![Main calibration menu](../../assets/kuka/robot-menu/step-14.jpg)

**Base calibration** defines the position of a working coordinate system relative to the World coordinate system. It associates the program with the physical location of a workpiece or equipment in the robot cell.

![Base calibration](../../assets/kuka/robot-menu/step-15.jpg)

**Tool calibration** determines the TCP position and tool orientation relative to the robot flange. Several calibration methods are available for each tool.

![Tool calibration](../../assets/kuka/robot-menu/step-16.jpg)

The selected method determines the procedure. The common **XYZ 4-Point** method approaches one reference point from four different orientations.

![Selected tool calibration method](../../assets/kuka/robot-menu/step-17.jpg)
