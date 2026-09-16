# FIFO-приоритет для FRI

## Зачем это нужно

FRI передаёт позиционную команду каждый цикл. При выбранном периоде 10 мс новый пакет должен обрабатываться примерно каждые 10 мс. Обычный планировщик Linux может задержать поток другим процессом. В журнале это видно по сообщениям:

```text
Could not enable FIFO RT scheduling: Operation not permitted
Overrun might occur ... Read time: 11500 us
```

`SCHED_FIFO` — политика планировщика Linux для потоков с фиксированным приоритетом. В проекте `controller_manager` запрашивает приоритет 50, а FRI-поток — 80. Поток FRI получает более высокий приоритет, чтобы вовремя принять пакет KUKA и отправить ответ.

Настройку выполняют **на компьютере с ROS 2**, с которого запускается `cobot run`. Sunrise Cabinet и `cobot-setting.yaml` менять для этого не нужно.

## Настройка Ubuntu 24.04

Выполните на компьютере с ROS 2 под пользователем, который запускает `cobot`:

```bash
sudo groupadd -f realtime
sudo usermod -aG realtime "$USER"
sudo tee /etc/security/limits.d/99-ros2-realtime.conf >/dev/null <<'EOF'
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock unlimited
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock unlimited
EOF
```

Затем полностью выйдите из графического сеанса и войдите снова либо перезагрузите компьютер. Открытый терминал не получает новую группу и лимиты.

## Проверка

В новом терминале проверьте группу и лимиты:

```bash
id -nG | tr ' ' '\n' | grep -x realtime
ulimit -r
ulimit -l
```

Ожидаемый результат: первая команда выводит `realtime`, вторая — `99`, третья — `unlimited`.

Запустите FRI без движения робота и убедитесь, что в журнале есть:

```text
FRI worker uses FIFO priority 80
```

После запуска можно дополнительно посмотреть политики всех потоков процесса:

```bash
PID=$(pgrep -n ros2_control_node)
ps -L -p "$PID" -o pid,tid,cls,rtprio,pri,comm
```

Для потоков реального времени в столбце `CLS` будет `FF`; у FRI-потока `RTPRIO` равен 80, у основного цикла `controller_manager` — 50.

!!! warning "Перед первым движением"
    Сначала проверьте успешное подключение FRI и строку `FRI worker uses FIFO priority 80`. Затем выполните короткое перемещение на скорости 0.1 при наблюдении оператора. FIFO уменьшает jitter компьютера, но не отменяет контроль сети, ограничений робота и рабочей зоны.

## Если настройка не сработала

- Если `realtime` отсутствует в выводе `id`, выйдите из системы и войдите снова.
- Если `ulimit -r` меньше 80, проверьте файл `/etc/security/limits.d/99-ros2-realtime.conf` и наличие `pam_limits.so` в конфигурации PAM.
- Если лог всё ещё сообщает `Operation not permitted`, убедитесь, что `cobot run` запущен тем же пользователем, для которого выполнен `usermod`.
- В Python virtualenv дополнительных действий не требуется: права задаются пользователю и наследуются процессом Python.
- Для Docker нужны права контейнера: `--cap-add=sys_nice --ulimit rtprio=99 --ulimit memlock=-1`.

## Откат

Чтобы удалить настройку, выполните:

```bash
sudo rm /etc/security/limits.d/99-ros2-realtime.conf
sudo gpasswd -d "$USER" realtime
```

После повторного входа процессы снова будут работать с обычным планировщиком Linux.

Подробнее о лимитах и приоритете `controller_manager` — в [документации ros2_control](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html#determinism).
