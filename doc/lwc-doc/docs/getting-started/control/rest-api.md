# Управление через REST API

REST API позволяет читать состояние робота и отправлять ему команды по HTTP. Он рассчитан на прикладные скрипты, интеграции с другими системами и быстрые проверки через Swagger UI.

Запросы к перемещению выполняются синхронно: ответ приходит после завершения планирования и выполнения команды либо после внутреннего тайм-аута. Очереди команд в API нет — дождитесь ответа на текущий запрос, прежде чем отправлять следующий.

!!! warning "Безопасность"
    REST API не заменяет штатную систему безопасности KUKA и кнопку аварийного останова. Перед первым запуском на реальном роботе проверьте программу Sunrise, зоны безопасности, инструмент и рабочую область. Начинать знакомство с API лучше в симуляции.

## Запуск и доступ

Веб-сервер запускается вместе со стеком робота, если в корневом файле **cobot-setting.yaml** включён блок **web**:

~~~ yaml
web:
  enabled: true
  host: 0.0.0.0
  port: 8007
  endpoints: pkg://iiwa_config/config/api_endpoints.yaml
  joint_limits: pkg://iiwa_config/config/moveit/joint_limits.yaml
~~~

После запуска через **cobot run** сервер будет доступен по адресу **http://адрес-сервера:8007**. Swagger UI помогает посмотреть фактическую схему запросов и выполнить одиночный тест:

- локально: [http://localhost:8007/docs](http://localhost:8007/docs);
- с другой машины: http://адрес-сервера:8007/docs;
- JSON-схема OpenAPI: http://адрес-сервера:8007/openapi.json.

Отдельного health-check в сервере нет. Если открывается Swagger UI, HTTP-сервер запущен. Готовность ROS-компонентов проверяется при обращении к конкретному маршруту.

По умолчанию сервер слушает все сетевые интерфейсы и не использует аутентификацию. Не публикуйте порт 8007 в недоверенную сеть. Для локальной работы укажите **host: 127.0.0.1**, а для удалённого доступа ограничьте сеть правилами firewall или VPN.

## Подготовка к примерам

Вкладки на этой странице синхронизированы: выберите удобный язык один раз, и тот же вариант будет открыт у следующих примеров.

=== "curl"

    ~~~ bash
    HOST=http://localhost:8007
    ~~~

=== "Python"

    ~~~ python
    import httpx

    HOST = "http://localhost:8007"
    T_READ = 10
    T_MOVE = 60
    ~~~

=== "MATLAB"

    ~~~ matlab
    HOST = 'http://localhost:8007';
    T_READ = 10;
    T_MOVE = 60;

    readOpts = weboptions('Timeout', T_READ);
    moveOpts = weboptions('MediaType', 'application/json', 'Timeout', T_MOVE);
    ~~~

Для JSON-запросов MATLAB использует встроенный webwrite. Загрузка CSV и JSON-файлов требует интерфейса matlab.net.http, который есть в современных desktop-версиях MATLAB.

## Состав API

| Метод | Маршрут | Назначение |
|---|---|---|
| GET | /robot/joint_states | Текущее состояние суставов |
| GET | /robot/pose | Поза TCP относительно base_link |
| GET | /robot/positions | Именованные положения из SRDF |
| POST | /robot/move/named | Переход в именованное положение |
| POST | /robot/move/pose | Декартово перемещение TCP |
| POST | /robot/move/joints | Перемещение по углам семи суставов |
| POST | /trajectory/send | Публикация траектории из JSON |
| POST | /trajectory/send_csv | Загрузка и публикация траектории из CSV |
| GET | /trajectory/logs | Последние записи траекторного модуля |
| POST | /sequences/start | Запуск последовательности из JSON-файла |
| GET | /sequences/status | Статус запущенной последовательности |
| GET | /sequences/logs | Вывод процесса последовательности |
| POST | /stop | Остановка команд API и планировщика |

## Получение состояния

### Состояние суставов

GET **/robot/joint_states** возвращает последнее сообщение ROS-топика **/joint_states**. Поля **position**, **velocity** и **effort** расположены в том же порядке, что и соответствующий массив **name**. Углы в **position** заданы в радианах.

=== "curl"

    ~~~ bash
    curl -sS --max-time 10 $HOST/robot/joint_states | python3 -m json.tool
    ~~~

=== "Python"

    ~~~ python
    response = httpx.get(f"{HOST}/robot/joint_states", timeout=T_READ)
    response.raise_for_status()
    state = response.json()
    print(dict(zip(state["name"], state["position"])))
    ~~~

=== "MATLAB"

    ~~~ matlab
    jointState = webread([HOST '/robot/joint_states'], readOpts);
    disp(jointState.position)
    ~~~

Типичный ответ:

~~~ json
{
  "name": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
  "position": [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0],
  "velocity": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "effort": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}
~~~

Если сообщения от контроллера не поступают в течение двух секунд, API вернёт 503. Обычно это означает, что контроллер или робот ещё не запущен.

### Поза TCP

GET **/robot/pose** вычисляет прямую кинематику через сервис MoveIt **/compute_fk**. Положение задаётся в метрах, а ориентация возвращается одновременно кватернионом и углами Эйлера:

- **euler_rad** — радианы;
- **euler_deg** — градусы;
- углы **A, B, C** соответствуют конвенции KUKA ABC: поворот вокруг Z, затем Y и затем X.

=== "curl"

    ~~~ bash
    curl -sS --max-time 10 $HOST/robot/pose | python3 -m json.tool
    ~~~

=== "Python"

    ~~~ python
    response = httpx.get(f"{HOST}/robot/pose", timeout=T_READ)
    response.raise_for_status()
    pose = response.json()
    print(pose["position"])
    ~~~

=== "MATLAB"

    ~~~ matlab
    pose = webread([HOST '/robot/pose'], readOpts);
    fprintf('TCP: x=%.3f, y=%.3f, z=%.3f м\n', ...
        pose.position.x, pose.position.y, pose.position.z);
    fprintf('ABC: A=%.1f, B=%.1f, C=%.1f град\n', ...
        pose.orientation.euler_deg.a, ...
        pose.orientation.euler_deg.b, ...
        pose.orientation.euler_deg.c);
    ~~~

Маршрут зависит и от **/joint_states**, и от работающего MoveIt. При недоступности любого из них будет возвращён 503.

### Именованные положения

GET **/robot/positions** читает положения group_state из SRDF. Список не зашит в API: он отражает текущую конфигурацию робота. В стандартной конфигурации есть положения **home**, **work** и **transport**.

=== "curl"

    ~~~ bash
    curl -sS $HOST/robot/positions | python3 -m json.tool
    ~~~

=== "Python"

    ~~~ python
    response = httpx.get(f"{HOST}/robot/positions", timeout=T_READ)
    response.raise_for_status()
    for position in response.json():
        print(position["name"], "—", position["description"])
    ~~~

=== "MATLAB"

    ~~~ matlab
    namedPositions = webread([HOST '/robot/positions'], readOpts);
    for i = 1:numel(namedPositions)
        fprintf('%s — %s\n', namedPositions(i).name, ...
            namedPositions(i).description);
    end
    ~~~

Перед вызовом **/robot/move/named** всегда полезно получить этот список: он показывает точное имя, группу планирования и целевые углы суставов.

## Команды перемещения

Все три команды ниже используют MoveIt. Ответ имеет вид:

~~~ json
{"success": true, "message": "Движение выполнено успешно"}
~~~

Поле **success: false** означает, что планировщик не смог построить или выполнить траекторию. HTTP-статус при этом может остаться 200, поэтому в прикладном коде проверяйте и статус HTTP, и поле **success**.

### Переход в именованное положение

POST **/robot/move/named** перемещает манипулятор в положение из SRDF.

| Поле | Обязательное | Значение |
|---|---:|---|
| name | да | Имя положения из /robot/positions |
| speed | нет | Масштаб скорости от 0.01 до 1.0; по умолчанию 0.1 |
| accel_scale | нет | Масштаб ускорения от 0 до 1.0; 0 означает использовать speed |

=== "curl"

    ~~~ bash
    curl -sS --max-time 60 -X POST $HOST/robot/move/named \
      -H "Content-Type: application/json" \
      -d '{"name": "home", "speed": 0.1, "accel_scale": 0.0}'
    ~~~

=== "Python"

    ~~~ python
    response = httpx.post(
        f"{HOST}/robot/move/named",
        json={"name": "home", "speed": 0.1, "accel_scale": 0.0},
        timeout=T_MOVE,
    )
    response.raise_for_status()
    result = response.json()
    if not result["success"]:
        raise RuntimeError(result["message"])
    ~~~

=== "MATLAB"

    ~~~ matlab
    body = struct('name', 'home', 'speed', 0.1, 'accel_scale', 0.0);
    reply = webwrite([HOST '/robot/move/named'], body, moveOpts);
    assert(reply.success, reply.message)
    ~~~

### Декартово перемещение TCP

POST **/robot/move/pose** принимает положение TCP в метрах и ориентацию ABC в радианах. Если **frame_id** пуст, используется фрейм, заданный в настройках планирования; в стандартной конфигурации это **base_link**.

| Поле | Обязательное | Значение |
|---|---:|---|
| x, y, z | да | Координаты TCP, м |
| a, b, c | нет | Углы KUKA ABC, рад; по умолчанию 0 |
| speed | нет | Масштаб скорости от 0.01 до 1.0; по умолчанию 0.1 |
| planner | нет | ompl, ptp, lin, circ или chomp; по умолчанию ptp |
| frame_id | нет | Фрейм целевой позы; пустая строка использует фрейм по умолчанию |

Значение **planner** приводится к нижнему регистру. PTP подходит для переходов между точками, LIN — для прямолинейного движения инструмента. Выбор CIRC имеет смысл только для задач, где он поддерживается вашим планировщиком и целевой позой.

=== "curl"

    ~~~ bash
    curl -sS --max-time 60 -X POST $HOST/robot/move/pose \
      -H "Content-Type: application/json" \
      -d '{
        "x": 0.40, "y": 0.00, "z": 0.50,
        "a": 0.0, "b": 3.14159, "c": 0.0,
        "speed": 0.1, "planner": "ptp", "frame_id": ""
      }'
    ~~~

=== "Python"

    ~~~ python
    target = {
        "x": 0.40, "y": 0.00, "z": 0.50,
        "a": 0.0, "b": 3.14159, "c": 0.0,
        "speed": 0.1, "planner": "ptp", "frame_id": "",
    }
    response = httpx.post(f"{HOST}/robot/move/pose", json=target, timeout=T_MOVE)
    response.raise_for_status()
    print(response.json())
    ~~~

=== "MATLAB"

    ~~~ matlab
    body = struct( ...
        'x', 0.40, 'y', 0.00, 'z', 0.50, ...
        'a', 0.0, 'b', pi, 'c', 0.0, ...
        'speed', 0.1, 'planner', 'ptp', 'frame_id', '');
    reply = webwrite([HOST '/robot/move/pose'], body, moveOpts);
    assert(reply.success, reply.message)
    ~~~

### Перемещение по углам суставов

POST **/robot/move/joints** принимает ровно семь углов в порядке J1–J7. API проверяет количество значений и текущие границы из файла **joint_limits.yaml**.

| Сустав | Допустимый угол, рад |
|---|---:|
| J1 | от -2.97 до 2.97 |
| J2 | от -2.10 до 2.10 |
| J3 | от -2.97 до 2.97 |
| J4 | от -2.10 до 2.10 |
| J5 | от -2.97 до 2.97 |
| J6 | от -2.10 до 2.10 |
| J7 | от -3.05 до 3.05 |

При изменении файла ограничений ориентируйтесь на Swagger UI: значения в этой таблице относятся к поставляемой конфигурации.

=== "curl"

    ~~~ bash
    curl -sS --max-time 60 -X POST $HOST/robot/move/joints \
      -H "Content-Type: application/json" \
      -d '{"joints": [0.0, 0.5, 0.0, -1.57, 0.0, 1.57, 0.0], "speed": 0.1}'
    ~~~

=== "Python"

    ~~~ python
    response = httpx.post(
        f"{HOST}/robot/move/joints",
        json={
            "joints": [0.0, 0.5, 0.0, -1.57, 0.0, 1.57, 0.0],
            "speed": 0.1,
        },
        timeout=T_MOVE,
    )
    response.raise_for_status()
    print(response.json())
    ~~~

=== "MATLAB"

    ~~~ matlab
    body = struct( ...
        'joints', [0.0, 0.5, 0.0, -1.57, 0.0, 1.57, 0.0], ...
        'speed', 0.1);
    reply = webwrite([HOST '/robot/move/joints'], body, moveOpts);
    assert(reply.success, reply.message)
    ~~~

## Траектории по суставам

Маршруты раздела **/trajectory** публикуют сообщение JointTrajectory напрямую в контроллер **/iiwa_arm_controller/joint_trajectory**. Ответ **status: sent** подтверждает публикацию сообщения, но не завершение движения и не отсутствие ошибок контроллера. Отслеживайте состояние робота через **/robot/joint_states** и при необходимости смотрите **/trajectory/logs**.

### Траектория в JSON

POST **/trajectory/send** принимает одну или несколько точек.

| Поле | Значение |
|---|---|
| points | Непустой список точек |
| points[].positions | Ровно 7 углов J1–J7 в радианах |
| points[].time_from_start | Время от начала траектории в секундах, не меньше 0 |
| validate_limits | Проверять границы суставов; по умолчанию true |

Сервер не проверяет возрастание времени между точками, поэтому задавайте его самостоятельно. Для контроллера траектория с возрастающими значениями времени предсказуемее.

=== "curl"

    ~~~ bash
    curl -sS --max-time 20 -X POST $HOST/trajectory/send \
      -H "Content-Type: application/json" \
      -d '{
        "points": [
          {"positions": [0, 0, 0, 0, 0, 0, 0], "time_from_start": 0.0},
          {"positions": [0, 0.5, 0, -1.0, 0, 1.0, 0], "time_from_start": 3.0},
          {"positions": [0, 0, 0, 0, 0, 0, 0], "time_from_start": 6.0}
        ],
        "validate_limits": true
      }'
    ~~~

=== "Python"

    ~~~ python
    trajectory = {
        "points": [
            {"positions": [0.0] * 7, "time_from_start": 0.0},
            {"positions": [0.0, 0.5, 0.0, -1.0, 0.0, 1.0, 0.0], "time_from_start": 3.0},
            {"positions": [0.0] * 7, "time_from_start": 6.0},
        ],
        "validate_limits": True,
    }
    response = httpx.post(f"{HOST}/trajectory/send", json=trajectory, timeout=T_READ)
    response.raise_for_status()
    print(response.json())
    ~~~

=== "MATLAB"

    ~~~ matlab
    p1 = struct('positions', [0, 0, 0, 0, 0, 0, 0], ...
                'time_from_start', 0.0);
    p2 = struct('positions', [0, 0.5, 0, -1.0, 0, 1.0, 0], ...
                'time_from_start', 3.0);
    trajectory.points = [p1, p2];
    trajectory.validate_limits = true;

    opts = weboptions('MediaType', 'application/json', 'Timeout', T_READ);
    reply = webwrite([HOST '/trajectory/send'], trajectory, opts);
    disp(reply)
    ~~~

### Загрузка CSV

POST **/trajectory/send_csv** принимает CSV-файл в multipart-поле **file**. Первая строка должна быть заголовком. Имена колонок суставов могут быть записаны как **joint1** или **joint_1**, регистр не важен; колонка времени называется **t**, **time** или **time_from_start**. Порядок колонок произвольный.

Пример файла:

~~~ csv
joint1,joint2,joint3,joint4,joint5,joint6,joint7,t
0,0,0,0,0,0,0,0.0
0,0.5,0,-1.0,0,1.0,0,3.0
~~~

Параметры **separator** и **validate_limits** передаются в строке запроса, а не как поля формы. По умолчанию разделитель — запятая, проверка ограничений включена.

=== "curl"

    ~~~ bash
    curl -sS --max-time 20 -X POST \
      "$HOST/trajectory/send_csv?separator=%2C&validate_limits=true" \
      -F "file=@trajectory.csv;type=text/csv"
    ~~~

=== "Python"

    ~~~ python
    with open("trajectory.csv", "rb") as csv_file:
        response = httpx.post(
            f"{HOST}/trajectory/send_csv",
            params={"separator": ",", "validate_limits": True},
            files={"file": ("trajectory.csv", csv_file, "text/csv")},
            timeout=T_READ,
        )
    response.raise_for_status()
    print(response.json())
    ~~~

=== "MATLAB"

    ~~~ matlab
    import matlab.net.http.*
    import matlab.net.http.io.*

    uri = URI([HOST '/trajectory/send_csv?separator=%2C&validate_limits=true']);
    form = MultipartFormProvider('file', FileProvider('trajectory.csv'));
    request = RequestMessage('post', [], form);
    httpOpts = HTTPOptions('ConnectTimeout', T_READ, 'ResponseTimeout', T_READ);
    response = request.send(uri, httpOpts);

    disp(response.Body.Data)
    ~~~

Для файла с точкой с запятой замените %2C на %3B.

### Лог траекторного модуля

GET **/trajectory/logs?n=50** возвращает до 300 последних записей. Параметр **n** должен быть в диапазоне от 1 до 300.

=== "curl"

    ~~~ bash
    curl -sS "$HOST/trajectory/logs?n=20" | python3 -m json.tool
    ~~~

=== "Python"

    ~~~ python
    response = httpx.get(f"{HOST}/trajectory/logs", params={"n": 20}, timeout=T_READ)
    response.raise_for_status()
    for line in response.json()["lines"]:
        print(line)
    ~~~

=== "MATLAB"

    ~~~ matlab
    logs = webread([HOST '/trajectory/logs?n=20'], readOpts);
    disp(logs.lines)
    ~~~

Чтобы прервать траекторию, используйте общий маршрут **POST /stop**. Отдельного маршрута **/trajectory/stop** в API нет.

## Последовательности движений

POST **/sequences/start** запускает отдельный процесс motion_sequence_runner. Он читает загруженный JSON-файл и поочерёдно отправляет цели MoveToJoints или MoveToPose.

| Поле формы | Значение по умолчанию | Назначение |
|---|---:|---|
| config | — | JSON-файл последовательности, обязательное поле |
| n_iterations | 3 | Число повторений, не меньше 1 |
| delay_between_iterations | 5.0 | Пауза между итерациями, с |
| bag_path | пусто | Путь для записи rosbag; пустая строка отключает запись |
| topics | пусто | Топики для rosbag через запятую; пусто означает все обнаруженные топики |
| joints_action | cobot/move_to_joints | Имя action для суставных целей |
| pose_action | cobot/move_to_pose | Имя action для декартовых целей |

Минимальная структура конфигурации:

~~~ json
{
  "home": {
    "joints": [0, 0, 0, -1.57, 0, 1.57, 0],
    "speed": 0.1
  },
  "waypoints": [
    {
      "x": 0.6, "y": 0.1, "z": 0.55,
      "a": 3.14, "b": 0.31, "c": 2.79,
      "speed": 0.2, "planner": "lin"
    },
    {
      "joints": [0.5, 0.3, 0, -1.2, 0, 1.4, 0],
      "speed": 0.2
    }
  ]
}
~~~

Если в точке есть поле **joints**, она считается суставной. Иначе runner ожидает декартовы поля **x**, **y**, **z**, **a**, **b** и **c**.

### Запуск последовательности

=== "curl"

    ~~~ bash
    curl -sS --max-time 10 -X POST $HOST/sequences/start \
      -F "config=@motion_sequence_config.json;type=application/json" \
      -F "n_iterations=3" \
      -F "delay_between_iterations=5.0"
    ~~~

=== "Python"

    ~~~ python
    with open("motion_sequence_config.json", "rb") as config:
        response = httpx.post(
            f"{HOST}/sequences/start",
            files={"config": ("motion_sequence_config.json", config, "application/json")},
            data={"n_iterations": "3", "delay_between_iterations": "5.0"},
            timeout=T_READ,
        )
    response.raise_for_status()
    print(response.json())
    ~~~

=== "MATLAB"

    ~~~ matlab
    import matlab.net.http.*
    import matlab.net.http.io.*

    form = MultipartFormProvider( ...
        'config', FileProvider('motion_sequence_config.json'), ...
        'n_iterations', '3', ...
        'delay_between_iterations', '5.0');
    request = RequestMessage('post', [], form);
    httpOpts = HTTPOptions('ConnectTimeout', T_READ, 'ResponseTimeout', T_READ);
    response = request.send(URI([HOST '/sequences/start']), httpOpts);

    disp(response.Body.Data)
    ~~~

Ответ **status: started** подтверждает запуск процесса, но не корректность содержимого JSON и не успешность каждого движения. Если runner завершится с ошибкой, проверьте его состояние и лог.

### Статус и журнал последовательности

=== "curl"

    ~~~ bash
    curl -sS $HOST/sequences/status | python3 -m json.tool
    curl -sS "$HOST/sequences/logs?n=50" | python3 -m json.tool
    ~~~

=== "Python"

    ~~~ python
    status = httpx.get(f"{HOST}/sequences/status", timeout=T_READ)
    status.raise_for_status()
    print(status.json())

    logs = httpx.get(f"{HOST}/sequences/logs", params={"n": 50}, timeout=T_READ)
    logs.raise_for_status()
    for line in logs.json()["lines"]:
        print(line)
    ~~~

=== "MATLAB"

    ~~~ matlab
    status = webread([HOST '/sequences/status'], readOpts);
    logs = webread([HOST '/sequences/logs?n=50'], readOpts);
    disp(status)
    disp(logs.lines)
    ~~~

Статусы:

- **idle** — последовательность ещё не запускалась;
- **running** — процесс выполняется;
- **finished** — процесс завершён; в ответе будет код **returncode**.

Одновременно может работать только одна последовательность. Повторный POST **/sequences/start** во время её выполнения вернёт 409. Для остановки используйте **POST /stop**: отдельного **/sequences/stop** нет.

## Общая остановка

POST **/stop** останавливает запущенный runner, публикует точку удержания текущей позиции для траекторного контроллера и вызывает сервис MoveIt **cobot/stop**. Если текущие состояния суставов недоступны, вместо точки удержания публикуется пустая траектория.

=== "curl"

    ~~~ bash
    curl -sS --max-time 10 -X POST $HOST/stop | python3 -m json.tool
    ~~~

=== "Python"

    ~~~ python
    response = httpx.post(f"{HOST}/stop", timeout=T_READ)
    response.raise_for_status()
    print(response.json())
    ~~~

=== "MATLAB"

    ~~~ matlab
    reply = webwrite([HOST '/stop'], struct(), ...
        weboptions('MediaType', 'application/json', 'Timeout', T_READ));
    disp(reply)
    ~~~

Команда отменяет программные операции, но не снимает питание с робота и не заменяет штатную аварийную остановку. После её вызова проверьте сообщение в ответе и реальное состояние робота.

## Ошибки и диагностика

| Код | Когда возникает |
|---:|---|
| 200 | Запрос обработан; для команд движения дополнительно проверьте поле success |
| 409 | Уже запущена последовательность движений |
| 422 | Некорректная структура запроса, число суставов, скорость, планировщик или лимиты суставов |
| 503 | ROS-топик, сервис, action-сервер или MoveIt недоступен; также возможен тайм-аут ожидания |

При проблемах идите от простого к сложному:

1. Откройте **/docs** и убедитесь, что сервер запущен и маршрут присутствует в схеме.
2. Проверьте **/robot/joint_states**. Без него не будет работать получение позы, а остановка траектории не сможет сформировать точку удержания.
3. Убедитесь, что стек запущен полностью: controller_manager, MoveIt и iiwa_motion_server.
4. После запуска последовательности посмотрите **/sequences/logs**; после публикации траектории — **/trajectory/logs**.

MCP-сервер работает в том же процессе, но это отдельный интерфейс: его адрес — **http://адрес-сервера:8007/mcp/mcp**. Для обычных HTTP-интеграций используйте маршруты из этой страницы.
