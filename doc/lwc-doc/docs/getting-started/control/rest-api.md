# Управление через REST API

REST API предоставляет HTTP-интерфейс для управления роботом из любой среды: Bash, Python, MATLAB, Postman и других инструментов.

## Веб-панель (Swagger UI)

Интерактивная документация со встроенным тест-клиентом доступна по адресу:

- **Локально:** [http://localhost:8007/docs](http://localhost:8007/docs)
- **Удалённо:** `http://ip-сервера:8007/docs`

---

## Подготовка к работе

!!! tip "Не знакомы с инструментами?"
    - **Bash / curl** — [введение в Bash-скрипты](https://easy-quest.github.io/my-docs/en/docs/Bash-x/bash_scripts_001/)
    - **Python** — [руководство для начинающих](https://metanit.com/python/tutorial/) или [официальная документация](https://docs.python.org/3/)
    - **MATLAB** — [официальная документация MATLAB](https://www.mathworks.com/help/matlab/index.html)

### Константы

Для Python и MATLAB определите константы в начале скрипта. Для Bash они указываются непосредственно в командах.

=== "Python"

    ```python
    import httpx, math

    HOST   = "http://localhost:8007"
    T_FAST = 10   # тайм-аут для чтения состояния [с]
    T_MOVE = 60   # тайм-аут для команд движения [с]
    ```

=== "MATLAB"

    ```matlab
    HOST   = 'http://localhost:8007';
    T_FAST = 10;
    T_MOVE = 60;
    ```

---

## Эндпоинты

### 1. GET `/robot/joint_states` — углы суставов

Возвращает текущие значения углов (в радианах) всех 7 суставов.
Используйте для мониторинга положения робота перед отправкой команд.

=== "Bash"

    ```bash
    curl -s --max-time 10 http://localhost:8007/robot/joint_states | python3 -m json.tool
    ```

=== "Python"

    ```python
    r = httpx.get(f"{HOST}/robot/joint_states", timeout=T_FAST)
    print(r.json()["position"])
    ```

=== "MATLAB"

    ```matlab
    r = webread([HOST '/robot/joint_states'], weboptions('Timeout', T_FAST));
    disp(r.position)
    ```

---

### 2. GET `/robot/pose` — текущая поза TCP

Возвращает декартову позу инструментального центра (TCP): координаты `x, y, z` [м] и ориентацию в углах Эйлера [градусы].
Используйте для контроля положения инструмента в пространстве.

=== "Bash"

    ```bash
    curl -s --max-time 10 http://localhost:8007/robot/pose | python3 -m json.tool
    ```

=== "Python"

    ```python
    r = httpx.get(f"{HOST}/robot/pose", timeout=T_FAST)
    pose = r.json()
    print(pose)
    ```

=== "MATLAB"

    ```matlab
    r = webread([HOST '/robot/pose'], weboptions('Timeout', T_FAST));
    fprintf('x=%.4f  y=%.4f  z=%.4f\n', r.position.x, r.position.y, r.position.z);
    fprintf('A=%.4f  B=%.4f  C=%.4f\n', r.orientation.euler_deg.a, r.orientation.euler_deg.b, r.orientation.euler_deg.c);
    ```

---

### 3. GET `/robot/positions` — именованные позиции

Возвращает список всех сохранённых именованных позиций (name + description).
Используйте для получения допустимых значений для `/robot/move/named`.

=== "Bash"

    ```bash
    curl -s --max-time 10 http://localhost:8007/robot/positions
    ```

=== "Python"

    ```python
    r = httpx.get(f"{HOST}/robot/positions", timeout=T_FAST)
    for pos in r.json():
        print(pos["name"], "—", pos["description"])
    ```

=== "MATLAB"

    ```matlab
    r = webread([HOST '/robot/positions'], weboptions('Timeout', T_FAST));
    for i = 1:numel(r)
        fprintf('%s — %s\n', r(i).name, r(i).description);
    end
    ```

---

### 4. POST `/robot/stop` — экстренная остановка

Немедленно останавливает текущее движение робота.
Используйте при необходимости прервать выполняемую команду.

=== "Bash"

    ```bash
    curl -s --max-time 10 -X POST http://localhost:8007/robot/stop
    ```

=== "Python"

    ```python
    r = httpx.post(f"{HOST}/robot/stop", timeout=T_FAST)
    print(r.json())
    ```

=== "MATLAB"

    ```matlab
    opts = weboptions('RequestMethod', 'post', 'MediaType', 'application/json', 'Timeout', T_FAST);
    r = webwrite([HOST '/robot/stop'], struct(), opts);
    disp(r.success)
    ```

---

### 5. POST `/robot/move/named` — движение в именованную позицию

Перемещает робота в одну из заранее сохранённых позиций (см. `/robot/positions`).
Используйте для воспроизводимых переходов между рабочими позициями.

| Параметр | Тип | Описание |
|---|---|---|
| `name` | string | Имя позиции |
| `speed` | float | Масштаб скорости (0.0–1.0) |
| `accel_scale` | float | Масштаб ускорения (0.0 = по умолчанию) |

=== "Bash"

    ```bash
    curl -s --max-time 60 -X POST http://localhost:8007/robot/move/named \
      -H "Content-Type: application/json" \
      -d '{"name": "home", "speed": 0.1, "accel_scale": 0.0}'
    ```

=== "Python"

    ```python
    r = httpx.post(f"{HOST}/robot/move/named",
                   json={"name": "home", "speed": 0.1, "accel_scale": 0.0},
                   timeout=T_MOVE)
    print(r.json())
    ```

=== "MATLAB"

    ```matlab
    opts = weboptions('RequestMethod', 'post', 'MediaType', 'application/json', 'Timeout', T_MOVE);
    body = struct('name', 'home', 'speed', 0.1, 'accel_scale', 0.0);
    r = webwrite([HOST '/robot/move/named'], body, opts);
    disp(r.success)
    ```

---

### 6. POST `/robot/move/pose` — движение по декартовой позе

Перемещает TCP в заданную точку пространства. Ориентация задаётся углами Эйлера (A, B, C) в радианах.
Используйте когда нужно задать точное положение и ориентацию инструмента.

| Параметр | Тип | Описание |
|---|---|---|
| `x, y, z` | float | Координаты [м] |
| `a, b, c` | float | Углы Эйлера [рад] |
| `speed` | float | Масштаб скорости (0.0–1.0) |
| `planner` | string | Планировщик: `ptp`, `lin`, `circ` |
| `frame_id` | string | Система отсчёта (пусто = `base_link`) |

=== "Bash"

    ```bash
    curl -s --max-time 60 -X POST http://localhost:8007/robot/move/pose \
      -H "Content-Type: application/json" \
      -d '{"x": 0.4, "y": 0.0, "z": 0.5, "a": 0.0, "b": 3.14159, "c": 0.0, "speed": 0.1, "planner": "ptp"}'
    ```

=== "Python"

    ```python
    r = httpx.post(f"{HOST}/robot/move/pose",
                   json={"x": 0.4, "y": 0.0, "z": 0.5,
                         "a": 0.0, "b": math.pi, "c": 0.0,
                         "speed": 0.1, "planner": "ptp", "frame_id": ""},
                   timeout=T_MOVE)
    print(r.json())
    ```

=== "MATLAB"

    ```matlab
    opts = weboptions('RequestMethod', 'post', 'MediaType', 'application/json', 'Timeout', T_MOVE);
    body = struct('x', 0.4, 'y', 0.0, 'z', 0.5, ...
                  'a', 0.0, 'b', pi,   'c', 0.0, ...
                  'speed', 0.1, 'planner', 'ptp', 'frame_id', '');
    r = webwrite([HOST '/robot/move/pose'], body, opts);
    disp(r.success)
    ```

---

### 7. POST `/robot/move/joints` — движение по углам суставов

Перемещает робота в позицию, заданную углами всех 7 суставов (в радианах).
Используйте когда нужен прямой контроль над конфигурацией робота без планирования в декартовом пространстве.

| Параметр | Тип | Описание |
|---|---|---|
| `joints` | float[7] | Углы суставов J1–J7 [рад] |
| `speed` | float | Масштаб скорости (0.0–1.0) |

=== "Bash"

    ```bash
    curl -s --max-time 60 -X POST http://localhost:8007/robot/move/joints \
      -H "Content-Type: application/json" \
      -d '{"joints": [0.0, 0.5, 0.0, -1.5708, 0.0, 1.5708, 0.0], "speed": 0.1}'
    ```

=== "Python"

    ```python
    r = httpx.post(f"{HOST}/robot/move/joints",
                   json={"joints": [0.0, 0.5, 0.0, -math.pi/2, 0.0, math.pi/2, 0.0],
                         "speed": 0.1},
                   timeout=T_MOVE)
    print(r.json())
    ```

=== "MATLAB"

    ```matlab
    opts = weboptions('RequestMethod', 'post', 'MediaType', 'application/json', 'Timeout', T_MOVE);
    body = struct('joints', {{0.0, 0.5, 0.0, -pi/2, 0.0, pi/2, 0.0}}, 'speed', 0.1);
    r = webwrite([HOST '/robot/move/joints'], body, opts);
    disp(r.success)
    ```

---

### 8. POST `/trajectory/send` — отправка траектории

Выполняет многоточечную траекторию, заданную набором waypoints с метками времени.
Используйте для плавного воспроизведения записанных или синтезированных движений.

| Параметр | Тип | Описание |
|---|---|---|
| `points[].positions` | float[7] | Углы суставов в точке [рад] |
| `points[].time_from_start` | float | Время от старта [с] |
| `validate_limits` | bool | Проверять ли ограничения суставов |

=== "Bash"

    ```bash
    curl -s --max-time 60 -X POST http://localhost:8007/trajectory/send \
      -H "Content-Type: application/json" \
      -d '{
        "points": [
          {"positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "time_from_start": 0.0},
          {"positions": [0.0, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0], "time_from_start": 3.0},
          {"positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "time_from_start": 6.0}
        ],
        "validate_limits": true
      }'
    ```

=== "Python"

    ```python
    r = httpx.post(f"{HOST}/trajectory/send",
                   json={
                       "points": [
                           {"positions": [0.0]*7,                                "time_from_start": 0.0},
                           {"positions": [0.0, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0], "time_from_start": 3.0},
                           {"positions": [0.0]*7,                                "time_from_start": 6.0},
                       ],
                       "validate_limits": True,
                   },
                   timeout=T_MOVE)
    print(r.json())
    ```

=== "MATLAB"

    ```matlab
    opts = weboptions('RequestMethod', 'post', 'MediaType', 'application/json', 'Timeout', T_MOVE);
    p1 = struct('positions', {{0,0,0,0,0,0,0}},        'time_from_start', 0.0);
    p2 = struct('positions', {{0,0.5,0,-1.0,0,1.0,0}}, 'time_from_start', 3.0);
    p3 = struct('positions', {{0,0,0,0,0,0,0}},        'time_from_start', 6.0);
    body = struct('points', {{p1, p2, p3}}, 'validate_limits', true);
    r = webwrite([HOST '/trajectory/send'], body, opts);
    disp(r)
    ```

---

### 9. POST `/trajectory/send_csv` — загрузка траектории из CSV

Загружает траекторию из CSV-файла и выполняет её. Формат файла: строки — точки, столбцы — углы суставов + время.
Используйте для воспроизведения траекторий, подготовленных во внешних инструментах.

=== "Bash"

    ```bash
    curl -s --max-time 60 -X POST http://localhost:8007/trajectory/send_csv \
      -F "file=@trajectory.csv" \
      -F "separator=," \
      -F "validate_limits=true"
    ```

=== "Python"

    ```python
    with open("trajectory.csv", "rb") as f:
        r = httpx.post(f"{HOST}/trajectory/send_csv",
                       files={"file": ("trajectory.csv", f, "text/csv")},
                       data={"separator": ",", "validate_limits": "true"},
                       timeout=T_MOVE)
    print(r.json())
    ```

=== "MATLAB"

    !!! warning "Требует уточнения"
        Код ниже использует `matlab.net.http` и может потребовать доработки в зависимости от вашей версии MATLAB.

    ```matlab
    import matlab.net.http.*
    import matlab.net.http.io.*

    csvData = fileread('trajectory.csv');
    part = FormProvider(FormField('file', csvData, 'filename', 'trajectory.csv', ...
                                  'content-type', 'text/csv'), ...
                       FormField('separator', ','), ...
                       FormField('validate_limits', 'true'));
    req = RequestMessage('POST', [], part);
    opts = matlab.net.http.HTTPOptions('ConnectTimeout', T_MOVE, 'ResponseTimeout', T_MOVE);
    [resp, ~] = req.send([HOST '/trajectory/send_csv'], opts);
    disp(resp.Body.Data)
    ```

---

### 10. POST `/trajectory/stop` — остановка траектории

Прерывает выполнение текущей траектории.
Используйте для аварийной остановки во время воспроизведения.

=== "Bash"

    ```bash
    curl -s --max-time 10 -X POST http://localhost:8007/trajectory/stop
    ```

=== "Python"

    ```python
    r = httpx.post(f"{HOST}/trajectory/stop", timeout=T_FAST)
    print(r.json())
    ```

=== "MATLAB"

    ```matlab
    opts = weboptions('RequestMethod', 'post', 'Timeout', T_FAST);
    r = webread([HOST '/trajectory/stop'], opts);
    disp(r)
    ```

---

### 11. GET `/trajectory/logs` — лог выполнения траектории

Возвращает последние `n` строк лога выполнения траектории.
Используйте для диагностики после выполнения команды.

=== "Bash"

    ```bash
    curl -s --max-time 10 "http://localhost:8007/trajectory/logs?n=20"
    ```

=== "Python"

    ```python
    r = httpx.get(f"{HOST}/trajectory/logs", params={"n": 20}, timeout=T_FAST)
    for line in r.json()["lines"]:
        print(line)
    ```

=== "MATLAB"

    ```matlab
    r = webread([HOST '/trajectory/logs'], weboptions('Timeout', T_FAST), 'n', 20);
    disp(r.lines)
    ```

---

### 12. POST `/sequences/start` — запуск последовательности движений

Запускает автоматическую последовательность движений из JSON-конфига (`motion_sequence_config.json`).
Используйте для воспроизведения повторяющихся циклов работы.

| Параметр | Тип | Описание |
|---|---|---|
| `config` | file | JSON-файл конфигурации последовательности |
| `n_iterations` | int | Число повторений (0 = бесконечно) |
| `delay_between_iterations` | float | Пауза между итерациями [с] |

=== "Bash"

    ```bash
    curl -s --max-time 10 -X POST http://localhost:8007/sequences/start \
      -F "config=@motion_sequence_config.json" \
      -F "n_iterations=3" \
      -F "delay_between_iterations=5.0"
    ```

=== "Python"

    ```python
    with open("motion_sequence_config.json", "rb") as f:
        r = httpx.post(f"{HOST}/sequences/start",
                       files={"config": ("config.json", f, "application/json")},
                       data={"n_iterations": "3", "delay_between_iterations": "5.0"},
                       timeout=T_FAST)
    print(r.json())
    ```

=== "MATLAB"

    !!! warning "Требует уточнения"
        Код ниже использует `matlab.net.http` и может потребовать доработки в зависимости от вашей версии MATLAB.

    ```matlab
    import matlab.net.http.*
    import matlab.net.http.io.*

    jsonData = fileread('motion_sequence_config.json');
    part = FormProvider(FormField('config', jsonData, 'filename', 'config.json', ...
                                  'content-type', 'application/json'), ...
                       FormField('n_iterations', '3'), ...
                       FormField('delay_between_iterations', '5.0'));
    req = RequestMessage('POST', [], part);
    opts = matlab.net.http.HTTPOptions('ConnectTimeout', T_FAST, 'ResponseTimeout', T_FAST);
    [resp, ~] = req.send([HOST '/sequences/start'], opts);
    disp(resp.Body.Data)
    ```

---

### 13. GET `/sequences/status` — статус последовательности

Возвращает текущий статус выполнения последовательности.
Используйте для polling-мониторинга выполнения.

=== "Bash"

    ```bash
    curl -s --max-time 10 http://localhost:8007/sequences/status
    ```

=== "Python"

    ```python
    r = httpx.get(f"{HOST}/sequences/status", timeout=T_FAST)
    print(r.json())
    ```

=== "MATLAB"

    ```matlab
    r = webread([HOST '/sequences/status'], weboptions('Timeout', T_FAST));
    disp(r.status)
    ```

---

### 14. GET `/sequences/logs` — лог последовательности

Возвращает последние `n` строк лога выполнения последовательности.

=== "Bash"

    ```bash
    curl -s --max-time 10 "http://localhost:8007/sequences/logs?n=50"
    ```

=== "Python"

    ```python
    r = httpx.get(f"{HOST}/sequences/logs", params={"n": 50}, timeout=T_FAST)
    for line in r.json()["lines"]:
        print(line)
    ```

=== "MATLAB"

    ```matlab
    r = webread([HOST '/sequences/logs'], weboptions('Timeout', T_FAST), 'n', 50);
    disp(r.lines)
    ```

---

### 15. POST `/sequences/stop` — остановка последовательности

Прерывает выполнение текущей последовательности движений.

=== "Bash"

    ```bash
    curl -s --max-time 10 -X POST http://localhost:8007/sequences/stop
    ```

=== "Python"

    ```python
    r = httpx.post(f"{HOST}/sequences/stop", timeout=T_FAST)
    print(r.json())
    ```

=== "MATLAB"

    ```matlab
    opts = weboptions('RequestMethod', 'post', 'Timeout', T_FAST);
    r = webread([HOST '/sequences/stop'], opts);
    disp(r)
    ```
