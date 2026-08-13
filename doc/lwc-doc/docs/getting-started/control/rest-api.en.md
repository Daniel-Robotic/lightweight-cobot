# Control via the REST API

The REST API reads robot state and sends commands over HTTP. It is intended for application scripts, integrations with other systems, and quick checks through Swagger UI.

Motion requests are synchronous: the response is returned after planning and execution finish or an internal timeout occurs. The API has no command queue. Wait for the current request to finish before sending another one.

!!! warning "Safety"
    The REST API does not replace the standard KUKA safety system or emergency stop. Before the first run on a physical robot, check the Sunrise program, safety zones, tool, and workspace. Begin using the API in simulation.

## Starting and accessing the server

The web server starts with the robot stack when the **web** section is enabled in the root **cobot-setting.yaml** file:

~~~ yaml
web:
  enabled: true
  host: 0.0.0.0
  port: 8007
  endpoints: pkg://iiwa_config/config/api_endpoints.yaml
  joint_limits: pkg://iiwa_config/config/moveit/joint_limits.yaml
~~~

After starting the stack with **cobot run**, the server is available at **http://server-address:8007**. Swagger UI shows the actual request schema and lets you run individual tests:

- locally: [http://localhost:8007/docs](http://localhost:8007/docs);
- from another computer: `http://server-address:8007/docs`;
- OpenAPI JSON schema: `http://server-address:8007/openapi.json`.

There is no separate health-check endpoint. If Swagger UI opens, the HTTP server is running. Readiness of ROS components is checked when a specific endpoint is called.

By default, the server listens on all network interfaces and does not use authentication. Do not expose port 8007 to an untrusted network. For local access, set **host: 127.0.0.1**. For remote access, restrict the network with firewall rules or a VPN.

## Preparing the examples

The tabs on this page are synchronized. Select a language once and the same tab will be selected in subsequent examples.

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

MATLAB uses the built-in `webwrite` function for JSON requests. Uploading CSV and JSON files requires `matlab.net.http`, available in modern desktop versions of MATLAB.

## API overview

| Method | Endpoint | Purpose |
|---|---|---|
| GET | /robot/joint_states | Current joint state |
| GET | /robot/pose | TCP pose relative to base_link |
| GET | /robot/positions | Named positions from the SRDF |
| POST | /robot/move/named | Move to a named position |
| POST | /robot/move/pose | Cartesian TCP motion |
| POST | /robot/move/joints | Move the seven joints to specified angles |
| POST | /trajectory/send | Publish a trajectory from JSON |
| POST | /trajectory/send_csv | Upload and publish a trajectory from CSV |
| GET | /trajectory/logs | Latest trajectory module log entries |
| POST | /sequences/start | Start a sequence from a JSON file |
| GET | /sequences/status | Running sequence status |
| GET | /sequences/logs | Sequence process output |
| POST | /stop | Stop API commands and the planner |

## Reading robot state

### Joint state

GET **/robot/joint_states** returns the latest message from the ROS **/joint_states** topic. The **position**, **velocity**, and **effort** fields use the same order as the **name** array. Angles in **position** are in radians.

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

Typical response:

~~~ json
{
  "name": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
  "position": [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0],
  "velocity": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "effort": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}
~~~

If no controller messages arrive within two seconds, the API returns 503. This usually means that the controller or robot has not started yet.

### TCP pose

GET **/robot/pose** computes forward kinematics with the MoveIt **/compute_fk** service. Position is specified in meters. Orientation is returned as both a quaternion and Euler angles:

- **euler_rad** — radians;
- **euler_deg** — degrees;
- **A, B, C** follow the KUKA ABC convention: rotation around Z, then Y, then X.

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
    fprintf('TCP: x=%.3f, y=%.3f, z=%.3f m\n', ...
        pose.position.x, pose.position.y, pose.position.z);
    fprintf('ABC: A=%.1f, B=%.1f, C=%.1f deg\n', ...
        pose.orientation.euler_deg.a, ...
        pose.orientation.euler_deg.b, ...
        pose.orientation.euler_deg.c);
    ~~~

This endpoint depends on both **/joint_states** and MoveIt. If either is unavailable, it returns 503.

### Named positions

GET **/robot/positions** reads `group_state` positions from the SRDF. The list is not hardcoded in the API; it reflects the current robot configuration. The standard configuration includes **home**, **work**, and **transport**.

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

Call this endpoint before **/robot/move/named** to obtain the exact name, planning group, and target joint angles.

## Motion commands

All three commands below use MoveIt. A response has this form:

~~~ json
{"success": true, "message": "Motion completed successfully"}
~~~

**success: false** means that the planner could not plan or execute the trajectory. The HTTP status may still be 200, so application code must check both the HTTP status and the **success** field.

### Moving to a named position

POST **/robot/move/named** moves the manipulator to a position from the SRDF.

| Field | Required | Value |
|---|---:|---|
| name | yes | Position name from /robot/positions |
| speed | no | Speed scale from 0.01 to 1.0; default: 0.1 |
| accel_scale | no | Acceleration scale from 0 to 1.0; 0 uses speed |

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

### Cartesian TCP motion

POST **/robot/move/pose** accepts a TCP position in meters and an ABC orientation in radians. If **frame_id** is empty, the frame from the planning settings is used; in the standard configuration this is **base_link**.

| Field | Required | Value |
|---|---:|---|
| x, y, z | yes | TCP coordinates, m |
| a, b, c | no | KUKA ABC angles, rad; default: 0 |
| speed | no | Speed scale from 0.01 to 1.0; default: 0.1 |
| planner | no | ompl, ptp, lin, circ, or chomp; default: ptp |
| frame_id | no | Target pose frame; an empty string uses the default frame |

The **planner** value is converted to lowercase. PTP is suitable for transitions between points; LIN produces straight-line tool motion. CIRC is appropriate only when it is supported by the planner and target pose.

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

### Moving by joint angles

POST **/robot/move/joints** accepts exactly seven angles in J1–J7 order. The API validates the number of values and the current limits from **joint_limits.yaml**.

| Joint | Allowed angle, rad |
|---|---:|
| J1 | -2.97 to 2.97 |
| J2 | -2.10 to 2.10 |
| J3 | -2.97 to 2.97 |
| J4 | -2.10 to 2.10 |
| J5 | -2.97 to 2.97 |
| J6 | -2.10 to 2.10 |
| J7 | -3.05 to 3.05 |

If the limits file changes, use Swagger UI as the reference. The table above describes the supplied configuration.

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

## Joint trajectories

Endpoints under **/trajectory** publish a `JointTrajectory` message directly to **/iiwa_arm_controller/joint_trajectory**. A **status: sent** response confirms publication, not completion of motion or absence of controller errors. Monitor **/robot/joint_states** and inspect **/trajectory/logs** when necessary.

### JSON trajectory

POST **/trajectory/send** accepts one or more points.

| Field | Value |
|---|---|
| points | Non-empty point list |
| points[].positions | Exactly 7 J1–J7 angles in radians |
| points[].time_from_start | Time from trajectory start in seconds, at least 0 |
| validate_limits | Validate joint limits; default: true |

The server does not check that time increases between points. Set increasing values yourself to make controller behavior predictable.

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

### Uploading CSV

POST **/trajectory/send_csv** accepts a CSV file in the multipart **file** field. The first row must be a header. Joint columns may be named **joint1** or **joint_1**, case-insensitively. The time column may be named **t**, **time**, or **time_from_start**. Columns may appear in any order.

Example file:

~~~ csv
joint1,joint2,joint3,joint4,joint5,joint6,joint7,t
0,0,0,0,0,0,0,0.0
0,0.5,0,-1.0,0,1.0,0,3.0
~~~

Pass **separator** and **validate_limits** in the query string, not as form fields. The default separator is a comma and limit validation is enabled.

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

For a semicolon-delimited file, replace `%2C` with `%3B`.

### Trajectory module log

GET **/trajectory/logs?n=50** returns up to 300 latest entries. The **n** parameter must be between 1 and 300.

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

Use the common **POST /stop** endpoint to interrupt a trajectory. The API has no **/trajectory/stop** endpoint.

## Motion sequences

POST **/sequences/start** launches a separate `motion_sequence_runner` process. It reads the uploaded JSON file and sends `MoveToJoints` or `MoveToPose` targets in sequence.

| Form field | Default | Purpose |
|---|---:|---|
| config | — | Sequence JSON file; required |
| n_iterations | 3 | Number of repetitions; at least 1 |
| delay_between_iterations | 5.0 | Delay between iterations, s |
| bag_path | empty | rosbag output path; an empty string disables recording |
| topics | empty | Comma-separated rosbag topics; empty means all discovered topics |
| joints_action | cobot/move_to_joints | Action name for joint targets |
| pose_action | cobot/move_to_pose | Action name for Cartesian targets |

Minimal configuration:

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

A point containing **joints** is treated as a joint target. Otherwise, the runner expects Cartesian fields **x**, **y**, **z**, **a**, **b**, and **c**.

### Starting a sequence

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

A **status: started** response confirms that the process started, not that the JSON is valid or that every motion succeeds. If the runner exits with an error, inspect its status and log.

### Sequence status and log

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

Statuses:

- **idle** — no sequence has been started;
- **running** — the process is running;
- **finished** — the process has exited; the response includes **returncode**.

Only one sequence can run at a time. A second POST to **/sequences/start** while one is running returns 409. Use **POST /stop** to stop it; there is no separate **/sequences/stop** endpoint.

## Common stop command

POST **/stop** stops the running sequence runner, publishes a hold point at the current position to the trajectory controller, and calls the MoveIt **cobot/stop** service. If current joint states are unavailable, it publishes an empty trajectory instead.

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

This command cancels software operations, but does not remove robot power or replace the standard emergency stop. After calling it, verify both the response message and the physical robot state.

## Errors and diagnostics

| Code | When it occurs |
|---:|---|
| 200 | The request was processed; for motion commands, also check the success field |
| 409 | A motion sequence is already running |
| 422 | Invalid request structure, joint count, speed, planner, or joint limits |
| 503 | A ROS topic, service, action server, or MoveIt is unavailable; a wait timeout may also have occurred |

When troubleshooting, proceed from simple checks to more complex ones:

1. Open **/docs** and verify that the server is running and the endpoint appears in the schema.
2. Check **/robot/joint_states**. Without it, pose retrieval does not work and trajectory stopping cannot generate a hold point.
3. Make sure that the complete stack is running: `controller_manager`, MoveIt, and `iiwa_motion_server`.
4. After starting a sequence, inspect **/sequences/logs**. After publishing a trajectory, inspect **/trajectory/logs**.

The MCP server runs in the same process but provides a separate interface at **http://server-address:8007/mcp/mcp**. For ordinary HTTP integrations, use the endpoints documented on this page.
