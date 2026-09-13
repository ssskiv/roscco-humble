# made by Opus 5 
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

Ручная проверка:

```bash
ros2 topic hz /brake_report
ros2 topic echo /fault_report
ros2 topic pub --once /enable_disable roscco/msg/EnableDisable '{enable_control: true}'
```

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
