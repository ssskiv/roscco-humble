# roscco (ROS 2 Humble)

Порт `PolySync/roscco` с ROS 1 (catkin) на ROS 2 Humble (ament_cmake).

## Сборка

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
# положить сюда каталог roscco/
cd roscco
git submodule add https://github.com/PolySync/oscc.git oscc   # или скопировать вручную
git submodule update --init oscc

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select roscco --cmake-args -DKIA_SOUL=ON
source install/setup.bash
```

Флаг машины (`-DKIA_SOUL=ON`, `-DKIA_SOUL_EV=ON`, `-DKIA_NIRO=ON`) обрабатывается
`oscc/api/OsccConfig.cmake` — тот же набор, что и в апстриме.

## Запуск

```bash
sudo ip link set can0 up type can bitrate 500000
ros2 launch roscco example.launch.py can_channel:=0
```

Только мост, без геймпада:

```bash
ros2 run roscco roscco_node --ros-args -p can_channel:=0
```

## Топики

| Направление | Топик | Тип |
|---|---|---|
| вход | `brake_command` | `roscco/msg/BrakeCommand` |
| вход | `throttle_command` | `roscco/msg/ThrottleCommand` |
| вход | `steering_command` | `roscco/msg/SteeringCommand` |
| вход | `enable_disable` | `roscco/msg/EnableDisable` |
| выход | `brake_report` | `roscco/msg/BrakeReport` |
| выход | `steering_report` | `roscco/msg/SteeringReport` |
| выход | `throttle_report` | `roscco/msg/ThrottleReport` |
| выход | `fault_report` | `roscco/msg/FaultReport` |
| выход | `can_frame` | `roscco/msg/CanFrame` |
| выход | `steering_angle` | `roscco/msg/SteeringAngleReport` |
| выход | `brake_pedal` | `roscco/msg/BrakePedalReport` |

Ручная проверка:

```bash
ros2 topic hz /brake_report
ros2 topic echo /fault_report
ros2 topic pub --once /enable_disable roscco/msg/EnableDisable '{enable_control: true}'
```

## Параметры

Все параметры объявлены с дескрипторами и диапазонами, так что rclcpp отклоняет
мусор и при старте, и при каждом `ros2 param set`. Посмотреть границы:

```bash
ros2 param describe /roscco_node limits.throttle_max
ros2 param list /roscco_node
```

### Ограничения команд

| Параметр | Диапазон | По умолчанию |
|---|---|---|
| `limits.brake_min` / `limits.brake_max` | 0…1 | 0 / 1 |
| `limits.throttle_min` / `limits.throttle_max` | 0…1 | 0 / 1 |
| `limits.steering_torque_min` | −1…0 | −1 |
| `limits.steering_torque_max` | 0…1 | 1 |

Входящие команды клампятся перед уходом в OSCC, факт обрезки логируется
(throttled, раз в секунду). `NaN`/`inf` заменяются нулём — во firmware улетает
float, и NaN там хуже нуля.

Это не косметика: прошивка принимает полный диапазон, и кроме этих клампов
между кривым выходом планировщика и полной тягой ничего нет. В
`example.launch.py` по умолчанию стоят осторожные 0.30 тормоза / 0.15 газа /
±0.25 руля — поднимай по мере доверия к стеку.

`roscco_teleop` имеет собственные `max_brake`, `max_throttle`,
`max_steering_torque` — они масштабируют вход геймпада ещё до публикации.

### Обратная связь: угол руля и педаль тормоза

Декодируется из OBD-кадров, которые CAN gateway перекладывает на control CAN.
**Без модуля gateway на шине кадров нет, и оба топика молчат** — это ожидаемо.

Сигнал описывается как в DBC, все поля — параметры:

| Параметр | Смысл |
|---|---|
| `*.enabled` | включить декодирование |
| `*.can_id` | идентификатор кадра (0…0x1FFFFFFF) |
| `*.start_bit` | Intel — позиция LSB, Motorola — позиция MSB |
| `*.bit_length` | ширина в битах (1…64) |
| `*.little_endian` | true = Intel, false = Motorola (пилообразная нумерация как в DBC) |
| `*.is_signed` | дополнительный код |
| `*.scale`, `*.offset` | `физ = raw * scale + offset` |

Префиксы — `steering_feedback.` и `brake_pedal_feedback.`.

Дополнительно у руля `min_angle` / `max_angle`: выход за них не глушит
сообщение, а ставит `in_range=false` и пишет warning — так неверный
scale или порядок байт виден сразу, а не превращается в тишину.

У тормоза `bit_length: 1` читает педальный выключатель. Если у тебя вместо
выключателя аналог (давление, положение), поставь реальную ширину и порог
`press_threshold`; `active_high: false` инвертирует смысл.

`feedback_timeout` (по умолчанию 1 с) — через сколько молчания по настроенному
ID писать предупреждение.

Дефолты `0x2B0` (угол, 0.1°/LSB) и `0x220` (тормоз) взяты из
`oscc/firmware/vehicles/kia_soul/vehicles.h`. **Для любой другой машины они
неверны** — придётся реверсить самому, `cansniffer` в помощь.

Подбор на живой машине:

```bash
ros2 topic echo /steering_angle
ros2 param set /roscco_node steering_feedback.scale 0.0625
ros2 param set /roscco_node steering_feedback.little_endian false
```

Параметры сигнала читаются на каждый кадр, так что крутить их можно не
перезапуская узел. `enabled` и проверка «влезает ли сигнал в 8 байт» —
только на старте.

## Что изменилось не только синтаксически

**Публикация вынесена из обработчика сигнала.** OSCC отдаёт репорты из
SIGIO-хендлера. ROS 1 публиковал прямо оттуда; в rclcpp это небезопасно —
publisher аллоцирует и берёт блокировки. Теперь колбэк только кладёт POD в
lock-free SPSC-очередь (`include/roscco/signal_queue.hpp`), а таймер
(`drain_period_ms`, по умолчанию 5 мс) публикует их в потоке экзекьютора.
Переполнение очереди считается и логируется, а не молча портит память.

**Убран reinterpret_cast репортов в сообщения.** `cast_callback<>` полагался на
совпадение layout OSCC-структуры и сгенерированного ROS-типа. В ROS 2 это
заведомо неверно: генерируемые типы содержат `std::array` и аллокатор. Копирование
полей явное.

**Метка времени снимается в момент прихода кадра** через `clock_gettime` внутри
хендлера, а не в момент публикации. Если включён `use_sim_time`, эти штампы
останутся системным временем — учитывай при проигрывании бэгов.

**Teleop публикует по таймеру 50 Гц, а не по колбэку joy.** Модули OSCC
дизейблятся, если не получают команду 200 мс, а частота `joy_node` ничем не
гарантирована. Добавлен watchdog: при пропадании `joy` дольше `joy_timeout`
команды обнуляются и уходит disable.

**Индексы осей и кнопок стали параметрами.** Дефолты — F310/Xbox в режиме
XInput. Для другого геймпада правится в launch-файле, а не в исходнике.

**Константы `FaultOriginId`** переименованы в `FAULT_ORIGIN_BRAKE`,
`FAULT_ORIGIN_STEERING`, `FAULT_ORIGIN_THROTTLE` — ROS 2 требует UPPER_SNAKE_CASE.

## Что выброшено

- **Apollo-пример и `pid_control`** — завязаны на bazel-сборку Apollo и ROS 1.
- **Тесты на rapidcheck + rostest** — требуют портирования на `ament_cmake_gtest`
  и `launch_testing`. Каркас сообщений и логика конвертации не менялись по смыслу,
  так что тесты переносятся почти механически.
- **`roscpp_code_format`** — заменён на `ament_lint_auto`.

## Возможная правка под твой чекаут OSCC

`src/roscco_node.cpp` вызывает `oscc_open(channel)` / `oscc_close(channel)`.
Оригинальный roscco звал `oscc_init()` без аргументов — видимо, под более старый
API. Если у тебя в `oscc/api/include/oscc.h` объявлен `oscc_init()`, поменяй
вызов в `main()`.

## Безопасность

Узел вызывает `oscc_disable()` при штатном завершении по SIGINT. При `SIGKILL`,
падении процесса или отвале CAN-адаптера сработает только внутренний watchdog
модулей на 200 мс. Физический E-stop в разрыве питания модулей обязателен.
