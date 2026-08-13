# SSL error during installation

## Problem description

When running `cobot setup` or `rosdep update`, an SSL handshake error may occur while downloading ROS dependency indexes from GitHub.

**Possible causes:**

- network restrictions such as a corporate firewall or ISP filtering;
- GitHub being blocked by the router or ISP;
- DNS resolution problems for `raw.githubusercontent.com`;
- restricted TLS connections caused by Deep Packet Inspection.

## Symptoms

=== "cobot setup"
    ```
    [rosdep] Initializing rosdep...
    ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml]:
            <urlopen error _ssl.c:983: The handshake operation timed out>
    ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml]:
            <urlopen error _ssl.c:983: The handshake operation timed out>
    ERROR: Not all sources were able to be updated.
    ```

=== "rosdep update"
    ```
    /usr/bin/rosdep:6: DeprecationWarning: pkg_resources is deprecated as an API.
        from pkg_resources import load_entry_point
    ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml]:
            <urlopen error _ssl.c:983: The handshake operation timed out> (https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml)
    ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml]:
            <urlopen error _ssl.c:983: The handshake operation timed out> (https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml)
    ERROR: Not all sources were able to be updated.
    [[[
    ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml]:
            <urlopen error _ssl.c:983: The handshake operation timed out> (https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml)
    ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml]:
            <urlopen error _ssl.c:983: The handshake operation timed out> (https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml)
    ```

---

## Solution: WireGuard VPN

The recommended way to bypass network restrictions is to establish a WireGuard VPN tunnel. For **WireGuard server** setup, see the [official WireGuard documentation](https://www.wireguard.com/quickstart/) or your cloud provider's documentation.

The following steps configure the **client** on the workstation.

---

### 1. Install WireGuard

```bash
sudo apt update && sudo apt install -y wireguard-tools
```

---

### 2. Configure the client

Create the configuration file:

```bash
sudo nano /etc/wireguard/wg0.conf
```

Add the following content and substitute your server details:

```ini
[Interface]
# Client private key (generate with: wg genkey)
PrivateKey = <YOUR_PRIVATE_KEY>
# Client address on the VPN network
Address = 10.0.0.2/24
# DNS servers (optional)
DNS = 8.8.8.8, 1.1.1.1

[Peer]
# WireGuard server public key
PublicKey = <SERVER_PUBLIC_KEY>
# Server address and UDP port
Endpoint = <SERVER_IP>:51820
# Route all traffic through the VPN
AllowedIPs = 0.0.0.0/0
# Keepalive for clients behind NAT
PersistentKeepalive = 25
```

!!! tip "Generating keys"
    If you do not have a key pair, generate one:
    ```bash
    # Private key
    wg genkey | tee privatekey

    # Public key (send it to the server administrator)
    cat privatekey | wg pubkey
    ```

!!! note "Split routing"
    To route only GitHub traffic through the VPN, replace `AllowedIPs` with:
    ```ini
    AllowedIPs = 140.82.112.0/20, 185.199.108.0/22
    ```

!!! warning "MTU problems"
    If the connection is established but packets are lost, reduce the MTU in `[Interface]`:
    ```ini
    MTU = 1420
    ```

---

### 3. Manage the tunnel

```bash
# Start the tunnel
sudo wg-quick up wg0

# Check connection status and statistics
sudo wg show

# Stop the tunnel
sudo wg-quick down wg0
```

---

### 4. Start automatically at boot

```bash
sudo systemctl enable wg-quick@wg0
sudo systemctl start wg-quick@wg0
```

---

### 5. Verify the connection

```bash
# Verify that the interface is up
ip addr show wg0

# Check routes
ip route show

# Ping the VPN server
ping 10.0.0.1

# Check the external IP (it should match the VPN server IP)
curl -s ifconfig.me
```

After the connection succeeds, run installation again:

```bash
cobot setup
```

or update only rosdep:

```bash
rosdep update
```

---

### 6. Diagnostics

If the tunnel does not start, inspect the system log:

```bash
sudo journalctl -u wg-quick@wg0 -f
```

Make sure that UDP port `51820` is open on the **server**:

```bash
# Check on the server
sudo ufw status
# or
sudo iptables -L -n | grep 51820
```
