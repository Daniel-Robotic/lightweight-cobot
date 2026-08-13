# Connecting to the server

## Deployment options

The project can be deployed in two ways:

| Option | Description | When to use it |
|---|---|---|
| **Local** | Install on your PC | Development, simulation, and debugging |
| **Remote (server)** | Install on a dedicated server connected to the KUKA controller | Working with the physical robot |

With remote deployment, you control the server from your PC over an **SSH connection**.

---

## SSH clients

Choose any of the following applications:

- [**Termius**](https://termius.com/) — cross-platform SSH client with a convenient GUI;
- [**MobaXterm**](https://mobaxterm.mobatek.net/) — multifunctional terminal for Windows;
- [**PuTTY**](https://putty.software/) — classic SSH client for Windows;
- the **built-in terminal or command prompt**, as described below.

For instructions on configuring Termius, MobaXterm, or PuTTY, refer to their official documentation.

---

## Connection details

```
IP address: 192.168.21.1
Username:   cobot
Password:   12345678
```

!!! warning "Network requirement"
    Your PC must be on the **same network/subnet as the robot** (for example, the KnASU network). Otherwise, the connection cannot be established.

---

## Connecting from the built-in terminal

=== "Linux"

    Any distribution can be used. Open a terminal and run:

    ```bash
    ssh cobot@192.168.21.1
    ```

=== "Windows"

    **Windows 10** or later is required for the built-in SSH client. Open **Command Prompt** or **PowerShell** and run:

    ```powershell
    ssh cobot@192.168.21.1
    ```

The command will prompt for a password:

```
cobot@192.168.21.1's password:
```

Enter `12345678`. Characters are not displayed while you type; this is normal security behavior. Press ++enter++.

After a successful connection, the server command prompt appears:

```
cobot@server:~$
```

---

**Next step:** [Project installation](installation.md)
