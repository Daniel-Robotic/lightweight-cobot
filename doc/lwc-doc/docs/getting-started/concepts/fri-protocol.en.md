# FRI protocol

!!! info "Work in progress"
    A detailed description of the FRI protocol is being prepared.

**FRI (Fast Robot Interface)** is a UDP protocol for low-level real-time control of a KUKA robot. It runs over Ethernet and provides a deterministic data exchange cycle between an external PC and the KUKA controller.

## Main characteristics

- **Transport:** UDP (no delivery guarantee, which is important for real-time operation)
- **Cycle period:** 5 ms (200 Hz) or 10 ms (100 Hz), configured in `cobot-setting.yaml` → `robot.fri_cycle_ms`
- **Control modes:** position, torque, and impedance

## Network requirements

!!! warning "Important: a 5 ms cycle requires KONI"
    A **5 ms (200 Hz)** cycle requires the **KONI** port (KUKA Optional Network Interface).
    The KLI port supports only a 10 ms cycle. Set `fri_cycle_ms: 10` when using KLI.

| Port | Minimum cycle | Purpose |
|---|---|---|
| **KONI** | 5 ms | High-frequency control, recommended for FRI |
| **KLI** | 10 ms | Standard control and programming |
