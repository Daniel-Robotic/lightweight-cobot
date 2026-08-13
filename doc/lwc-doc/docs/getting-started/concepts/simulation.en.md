# Simulation (Webots)

!!! info "Work in progress"
    A detailed simulator guide is being prepared.

**Webots** is an open-source robot simulator. LWC uses it as a digital twin of the KUKA LBR IIWA 7, allowing control algorithms to be developed and debugged without a physical robot.

## Key features

- The simulator uses the same ROS 2 topics and interfaces as the real robot.
- The simulation world is set in `cobot-setting.yaml` → `digital_twin.webots.world`.
- Start it with `cobot run --simulate`.

## Differences from the real robot

- There are no real safety constraints, so motion can be faster.
- Physics is approximate, including inertia, friction, and elasticity.
- FRI is not used; communication goes through the Webots ROS 2 driver.
