# iiwa_controller

Аппаратный `hardware_interface::SystemInterface` для KUKA LBR iiwa 7 R800.
Проверенная версия: встроенный **FRI C++ SDK 1.16**, ROS 2 Jazzy,
`hardware_interface` **4.48.0**, C++17. [English](README.en.md).

## Основания реализации

Перед ревизией реализации изучены Doxygen SDK 1.16 из навыка `kuka-fri`
и документация ros2_control Jazzy. Затем проверены поставляемые исходники SDK,
`ServerFriRos2.java`, URDF и полное руководство
[KUKA Sunrise.FRI 1.16](../../doc/pdf/KUKA_SunriseFRI_116_en.pdf).

Требования KUKA: разделы 2.1.4, 6.2.1–6.2.5 и 6.3.3. `monitor()` зеркалирует
commanded position, `waitForCommand()` — IPO; в обоих commanding-состояниях
нужны непрерывные команды. Измеренная позиция, commanded position и IPO — разные
величины. Позиционный интерфейс содержит 7 значений A1…A7 в радианах.

Интеграция ROS следует [Writing a Hardware Component (Jazzy)](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html),
сигнатурам установленных заголовков и автоматически создаваемым интерфейсам.

## Поведение

- ROS-интерфейсы: `joint1`…`joint7`; команда `position` [рад]; состояния
  `position` [рад], `velocity` [рад/с], `effort` и `external_torque` [Нм].
- `position` всегда содержит измерение энкодеров. Скорость — конечная разность
  измеренных позиций по временным меткам робота. EMA-фильтров нет.
- Команды передаются без EMA. Ограничение шага `max_velocity * FRI period`
  сохраняется как ограничение скорости, а не сглаживающий фильтр. Позиция вне
  ограничений и NaN/Inf приводят к ошибке. Пределы берутся из URDF через Jazzy
  `HardwareInfo::limits`; численные пределы робота не изменены.
- Поддерживается POSITION + JOINT overlay с position, joint impedance или
  Cartesian impedance control, согласно `ServerFriRos2`. TORQUE/WRENCH
  намеренно не реализованы. Мониторинг без overlay поддерживается.
- Единственный запускаемый контроллер движения — `iiwa_arm_controller`
  (`joint_trajectory_controller/JointTrajectoryController`). Action:
  `/iiwa_arm_controller/follow_joint_trajectory`. Joint state broadcaster
  публикует состояния с существующими настройками QoS. Новых тем и TF нет.
- IP и порт передаются из `robot.ip` / `robot.port` в аппаратный URDF;
  исходные значения проекта — `192.170.10.2:30200`. Период manager задаётся
  `robot.fri_cycle_ms`; его нужно согласовать с выбранным периодом Sunrise.

Один рабочий поток владеет `ClientApplication::step()`. Межпоточный обмен
в цикле использует `try_lock`: при конкуренции цикл не ждёт mutex. `read()`
использует последний целый снимок с проверкой давности, `write()` повторяет
передачу в следующем цикле. До первого UDP-пакета допускается ожидание Sunrise
до 15 с; после первого пакета ошибка `step()` завершает поток. Активация требует
свежей сессии как минимум MONITORING_READY и качества GOOD/EXCELLENT.

При отсутствии пакетов/команд 100 мс, нерастущем timestamp, нечисловой
телеметрии, недопустимом режиме, safety stop, неактивных приводах или плохом
качестве в commanding приложение прекращает выдачу команд; ROS получает ERROR.
Выход из COMMANDING_ACTIVE требует явного восстановления lifecycle и новой
сессии Sunrise. Пороги 15 с и 100 мс, ограничения команд и политика явного
восстановления — решения драйвера, **не универсальные требования KUKA**.
Остановка потока предшествует закрытию сокета. Объекты уничтожаются в порядке
application → connection → client; cleanup, error, shutdown и деструктор
выполняют это одинаково. Прекращение FRI не заменяет аппаратную аварийную остановку.

Зависимости: `hardware_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`,
существующие зависимости package.xml; SDK собирается как `fri_client_sdk`.
Публичные SDK заголовки: `friLBRClient.h`, `friClientApplication.h`,
`friUdpConnection.h`; исключения обрабатываются как `FRIException`.
Поставляемые файлы SDK не изменены.

## Проверка без робота

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select iiwa_controller iiwa_bringup iiwa_config iiwa_description iiwa_utils --symlink-install
source install/setup.bash
colcon test --base-paths src --packages-select iiwa_controller
colcon test-result --test-result-base build/iiwa_controller --verbose
/usr/bin/python3 src/iiwa_controller/test/smoke_jtc.py
```

Тесты используют реальный SDK с искусственными данными и локальный режим
`simulate=true` аппаратного плагина. Они проверяют измерения, IPO, отсутствие
EMA, режимы, ограничения, ошибки и повторные переходы lifecycle. Они не
доказывают временные характеристики реального FRI-соединения.

Webots smoke-test перед аппаратным запуском: `cobot run local` → **Webots
simulator**; проверить, что `joint_state_broadcaster` и `iiwa_arm_controller`
активны, выполнить небольшую допустимую траекторию MoveIt и отменить её.
Проверить результат action и остановку симулятора. Webots использует отдельный
плагин и сам по себе не проверяет UDP/FRI. Реальный робот в этой ревизии не запускался.
Для аппаратной проверки нужны оператор, свободная рабочая зона, проверенные
сеть, tool/load data, режим, ограничения и процедура останова из AGENTS.md.

Результаты ревизии: сборка пяти затронутых ROS-пакетов прошла; 14 C++-сценариев
в двух CTest-проверках прошли. `smoke_jtc.py` загрузил настоящий URDF с локальной
имитацией нашего плагина, активировал JSB/JTC и завершил траекторию +0.02 рад
сустава 1 со статусом SUCCEEDED. При SIGINT библиотека `pal_statistics`
Controller Manager вывела сообщение об остановленном контексте; lifecycle
аппаратного плагина завершился успешно. Webots и физический FRI не запускались.
