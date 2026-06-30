# Ошибка SSL при установке

## Описание проблемы

При выполнении `cobot setup` или `rosdep update` может возникать ошибка SSL-рукопожатия при попытке загрузить индексы зависимостей ROS с серверов GitHub.

**Возможные причины:**

- Сетевые ограничения (корпоративный брандмауэр, интернет-провайдер)
- Блокировка GitHub на уровне маршрутизатора или провайдера
- Проблемы с DNS-разрешением `raw.githubusercontent.com`
- Ограниченный доступ к TLS-соединениям (Deep Packet Inspection)

## Симптомы

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

## Решение: VPN через WireGuard

Рекомендуемый способ обойти сетевые ограничения — поднять WireGuard VPN-туннель.
По настройке **WireGuard-сервера** обратитесь к [официальной документации WireGuard](https://www.wireguard.com/quickstart/) или документации вашего облачного провайдера.

Ниже приведена настройка **клиентской части** на рабочей машине.

---

### 1. Установка WireGuard

```bash
sudo apt update && sudo apt install -y wireguard-tools
```

---

### 2. Конфигурация клиента

Создайте файл конфигурации:

```bash
sudo nano /etc/wireguard/wg0.conf
```

Добавьте следующее содержимое, подставив данные вашего сервера:

```ini
[Interface]
# Приватный ключ клиента (генерируется командой: wg genkey)
PrivateKey = <ВАШ_ПРИВАТНЫЙ_КЛЮЧ>
# IP-адрес клиента в VPN-сети
Address = 10.0.0.2/24
# DNS-серверы (опционально)
DNS = 8.8.8.8, 1.1.1.1

[Peer]
# Публичный ключ WireGuard-сервера
PublicKey = <ПУБЛИЧНЫЙ_КЛЮЧ_СЕРВЕРА>
# Адрес и UDP-порт сервера
Endpoint = <IP_СЕРВЕРА>:51820
# Маршрутизировать весь трафик через VPN
AllowedIPs = 0.0.0.0/0
# Keepalive для клиентов за NAT
PersistentKeepalive = 25
```

!!! tip "Генерация ключей"
    Если у вас ещё нет ключевой пары, сгенерируйте её:
    ```bash
    # Приватный ключ
    wg genkey | tee privatekey

    # Публичный ключ (передайте администратору сервера)
    cat privatekey | wg pubkey
    ```

!!! note "Частичная маршрутизация"
    Если нужно направлять через VPN только трафик к GitHub, замените `AllowedIPs`:
    ```ini
    AllowedIPs = 140.82.112.0/20, 185.199.108.0/22
    ```

!!! warning "Проблемы с MTU"
    Если соединение установлено, но есть потери пакетов — уменьшите MTU в секции `[Interface]`:
    ```ini
    MTU = 1420
    ```

---

### 3. Управление туннелем

```bash
# Поднять туннель
sudo wg-quick up wg0

# Проверить статус и статистику соединения
sudo wg show

# Остановить туннель
sudo wg-quick down wg0
```

---

### 4. Автозапуск при загрузке системы

```bash
sudo systemctl enable wg-quick@wg0
sudo systemctl start wg-quick@wg0
```

---

### 5. Проверка подключения

```bash
# Убедиться, что интерфейс поднят
ip addr show wg0

# Проверить маршруты
ip route show

# Пинг до VPN-сервера
ping 10.0.0.1

# Проверить внешний IP (должен совпадать с IP VPN-сервера)
curl -s ifconfig.me
```

После успешного подключения повторно запустите установку:

```bash
cobot setup
```

или только обновление rosdep:

```bash
rosdep update
```

---

### 6. Диагностика

Если туннель не поднимается, смотрите системные логи:

```bash
sudo journalctl -u wg-quick@wg0 -f
```

Убедитесь, что на **сервере** открыт UDP-порт `51820`:

```bash
# Проверить на сервере
sudo ufw status
# или
sudo iptables -L -n | grep 51820
```
