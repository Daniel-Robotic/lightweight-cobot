# Интерфейсы `iiwa_llm_plan_executor`

Версия контракта графа: **2** (совпадает с `iiwa_scene_context` 2).
Исполнитель не вызывает Qwen3 и не переписывает граф модели. Система
принимает готовый JSON-граф.

## Ответ графа

```json
{
  "status": "ready",
  "entry": "p1_find",
  "plan": [
    {
      "id": "p1_find",
      "skill": "detect",
      "object": "cap",
      "target": "",
      "params": {},
      "transitions": [
        {"event": "found", "next": "p1_grab"},
        {"event": "not_found", "next": "replan_required"},
        {"event": "failed", "next": "replan_required"}
      ]
    }
  ]
}
```

* `status` — `ready` или `blocked`; `blocked` никогда не исполняется.
* `plan` — не более 256 узлов. Отклоняются неизвестные ключи, дубли `id`,
  зарезервированные терминалы, отсутствующий `entry`, пропущенные события,
  неизвестные `next`, недостижимые узлы и узлы без пути к терминалу.
* Терминалы: `completed`, `replan_required`, `operator_required`.
* `params` пока поддерживает только `dispense.start_fraction` /
  `dispense.end_fraction` в диапазоне `0..1`.

## Навыки

Реализованные навыки: `detect`, `approach`, `lower`, `grab`, `lift`,
`transfer`, `align`, `release`, `retreat`, `capture`, `verify`, `stop`,
`wait_scene_clear`, `inspect_state`, `notify_operator`, `dispense`,
`dock_tool`, `handoff_screw`.

Алиасы: `pick` (= `grab`), `move_to_storage` (= `transfer` + осевой
`lower`), `inspect` (= `verify`). `tighten` не входит в профиль и
отклоняется до любого движения.

Набор навыков и правила приходят из `iiwa_scene_context/catalog.py` и живого
`scene_json`; модель не может добавить навык или контракт. В сцене без
герметика `dispense` не рекламируется, без винтов — `handoff_screw`, без
инструментов — `dock_tool`.

## Приём графа

`parse_response(raw) -> PlanGraph` выполняет строгую проверку формы.
`validate_graph(graph, scene_envelope, catalog)` возвращает
`ValidationResult(ok, code, reason, details)`.

Порядок приёма:

1. формат JSON и схема графа;
2. готовность сцены/MoveIt/контроллера и истёкшая задержка старта;
3. собственный свежий снимок сцены исполнителем;
4. проверка свежести снимка;
5. валидация структуры, ветвей и владения относительно этого снимка;
6. публикация принятого графа.

`RunRecord` создаётся **только после** успешного прохождения всех шагов и
публикации. Поэтому отклонённый `run_id` свободен: исправленный граф можно
отправить снова. Отклонённый запрос не появляется в
`GET /planner/runs/{run_id}`, в `/iiwa_llm_plan_executor/plan` и в JSONL-журнале
запусков. API возвращает конкретные `code`/`reason`.

Коды отклонения: `invalid`, `rejected`, `blocked`, `replan_required`,
`not_ready`, `stale_scene`, `busy`, `conflict`.

Правила `run_id`:

* пока запуск активен, другой `run_id` получает `busy`;
* повтор идентичного принятого графа с тем же `run_id` — `duplicate`, второго
  движения нет;
* другой граф с уже принятым `run_id` — `conflict`;
* после `completed`, `operator_required`, `replan_required`, `failed` или
  `canceled` `run_id` остаётся занятым;
* одновременные отправки защищены блокировкой, активный граф на робота один.

Перед первым движением исполнитель повторно проверяет свежесть сцены и
`state_version`. Если сцена изменилась или стала недоступной, движение не
начинается. Во время выполнения внутренний счётчик (не публикуется) ловит
внешнее изменение сцены; изменения владения, подтверждённые собственными
`grab`/`release`/`dock_tool`/`handoff_screw`, разрешены.

## ROS-интерфейсы

| Направление | Имя | Тип | Payload |
| --- | --- | --- | --- |
| сервис | `/iiwa_llm_plan_executor/submit_plan` | `iiwa_msgs/SubmitPlan` | request `run_id`, `response` (JSON-строка графа). Ответ: `accepted`, `code`, `reason`, `status_json`. |
| сервис | `/iiwa_llm_plan_executor/submit` | `webots_ros2_msgs/SetString` | Устаревший совместимый вход `{"run_id","response"}`; направляется в тот же механизм приёма. Поле `scene_revision` игнорируется. |
| топик | `/iiwa_llm_plan_executor/plan` | `std_msgs/String` | `{"run_id","response"}` для принятого графа; transient-local, depth 1. Отклонённые графы сюда не попадают. |
| топик | `/iiwa_llm_plan_executor/status` | `std_msgs/String` | События и терминал: `run_id`, `state`, `reason`, `node_id`, `skill`, `event`, `next`, `terminal`, `mounted_tool`, `held_object`, `timestamp`, `event_detail`/`timing`. Поля `scene_revision` нет. |
| сервис | `/iiwa_llm_plan_executor/get_status` | `iiwa_msgs/GetRunStatus` | request `string run_id`; response `bool found`, `string status_json`. |
| сервис | `/iiwa_llm_plan_executor/cancel` | `webots_ros2_msgs/SetString` | `{"run_id": "..."}` или строка; останавливает выполнение, сохраняя захват. |

## Тайминги этапов

Каждый запуск пишет JSONL в `~/.ros/iiwa_llm_plan_executor/runs/<run_id>.jsonl`.
События `motion_timing` и `timing` содержат монотонные `started_at`,
`finished_at`, `duration_sec` для этапов:

* завершение предыдущей траектории — `since_previous_trajectory_sec`;
* `_wait_stationary` — `wait_stationary`;
* синхронизация PlanningScene — `planning_scene_check` /
  `planning_scene_sync`;
* построение плана MoveIt — `moveit_plan`;
* проверка траектории — `trajectory_check`;
* ожидание контроллера — `controller_execution`;
* ответ supervisor — `supervisor_command` (`service_wait_sec`,
  `service_call_sec`, `result_wait_sec`);
* захват кадра — `frame_capture`.

`start_delay_sec` — задержка после загрузки сцены, применяется один раз перед
стартом графа. Это не пауза между движениями.

## Терминалы и журнал

`completed` — граф и его проверенные постусловия завершены.
`replan_required` — нужен новый снимок и, возможно, новый вызов Qwen3.
`operator_required` — нужно решение человека; робот остановлен.
`failed` / `canceled` — безопасная остановка (устаревшее состояние, лимиты,
столкновение, отмена, лимит шагов); состояние захвата сохраняется.

## Ограничения движения

Свободные перемещения между рабочими точками — Pilz PTP. Осевое опускание,
подъём и нанесение герметика — Pilz LIN. Каждый план плотно пересэмплируется
и проверяется по реальным лимитам суставов, self/scene collision, зазору до
камер, наклону TCP (≤10°) и скачкам суставов. Боковое движение из контактной
позы предваряется осевым LIN-отводом. Исполнитель не откатывается к
неограниченному `MoveToPose`.

Это проверка в Webots, а не валидация физического робота и не оценка
качества Qwen3.
