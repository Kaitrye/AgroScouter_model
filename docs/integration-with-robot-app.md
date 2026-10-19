# Контекст: подключить приложение робота к симуляции

Передайте этот файл ассистенту, работающему в репозитории
`/media/kaitrye/free/Robot-for-agricultural-scouting`. Цель — запускать задачи
C++ приложения из `Robot/src` на симуляции, сохранив режим физического робота.
Репозитории остаются раздельными; связь идёт через ROS 2.

## Симуляция

Путь: `/home/kaitrye/Загрузки/robot_sim_harmonic/robot_sim`.
Платформа: ROS 2 Jazzy и Gazebo Harmonic. Запуск: `bash start_sim.sh` из корня
репозитория. `start_gazebo.sh` не запускает ROS-мосты. Интерфейс описан в
`docs/ros-and-configuration.md`; узел моторов и энкодеров находится в
`tools/motor_interface.py`.

| Топик | Тип | Значение |
|---|---|---|
| `/motors` | `std_msgs/msg/Int32MultiArray` | Команда `[левый, правый]`, каждое число от −100 до 100. `[40, 40]` ведёт вперёд; `[0, 0]` останавливает. |
| `/encoders` | `std_msgs/msg/UInt32MultiArray` | Два накопительных счётчика `[левый, правый]`, 50 Гц. |
| `/ground_truth/odom` | `nav_msgs/msg/Odometry` | Истинная поза для оценки результата. |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU симуляции; формат отличается от `gps_imu/angle` приложения. |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Камера, направленная вниз. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Двумерный лидар; это не облако Livox. |
| `/clock` | `rosgraph_msgs/msg/Clock` | Симуляционное время. |

Команды `/motors` надо повторять: после 0,5 с без сообщений привод
останавливается. Модель имеет небольшую постоянную разницу скоростей
левого и правого моторов.

`/encoders` **не вычисляется из перемещения корпуса**. Gazebo выдаёт скорость
каждой гусеницы после применения погрешности мотора; виртуальные энкодеры
интегрируют эти две скорости независимо, затем округляют накопленные тики.
Поэтому при буксовании гусениц тики могут расти, даже если корпус почти
стоит. Внутренние телеметрические топики: `/robot/left_track/speed` и
`/robot/right_track/speed` (`std_msgs/msg/Float64`). В модели нет отдельного
вала, редуктора, датчика Холла и физики заклинивания двигателя — это
скоростная модель виртуального энкодера, а не полная модель электроники.

Начальный масштаб: `encoder_ticks_per_meter = 1000` тиков на метр пути
гусеницы; `motor_command_speed_m_s = 0.4` м/с при команде 100. Значения
находятся в `config/parameters.json` симуляции и пока не сверены с
физическим роботом. `1000` тиков могут соответствовать примерно метру
движения гусеницы, но при пробуксовке не метру пути корпуса.

## Приложение: где менять

- Активное приложение и HTTP API — `Robot/src`; сборка Conan/CMake описана в
  `Robot/README.md`.
- `Robot/src/RobotController/robot_controller.cpp` сейчас автоматически
  запускает аппаратные `motors`, `encoders` и `gps_imu` через ModuleSupervisor.
- `Robot/src/TaskExecutor/tasks/move_straight.cpp` публикует `motors` и
  подписывается на `encoders`.
- `Robot/src/TaskExecutor/tasks/calibrate_encoders.cpp` использует те же
  топики. `Robot/src/ApiServer/request_executer.cpp` принимает задачи через
  `POST /api/tasks` и сообщает статус через `GET /api/tasks/{id}`.
- `Robot/Modules` и корневой `src` внешнего репозитория содержат драйверы
  физического оборудования. Они не нужны для первого прогона в Gazebo.

Скрипты `Robot/Modules/scripts/run_*.sh` привязаны к `/home/nvidia/...` и
ROS 2 Humble. Имеющаяся сборка `Robot/src/build/Release` также использует
Humble. На машине симуляции установлен Jazzy. При новой конфигурации
`Robot/src/RosBridge/CMakeLists.txt` может незаметно выбрать заглушку без
ROS, если зависимости не найдены: требуется подтвердить сообщение
`RosBridge: ROS 2 support enabled`.

## Задачи для ассистента приложения

1. Добавить явный режим `sim`. В нём не запускать аппаратные модули
   `motors`, `encoders` и `gps_imu`; оставить RosBridge, TaskExecutor и
   HTTP API. Обычный запуск физического робота должен сохраниться.
2. Собрать приложение под Jazzy после загрузки
   `/opt/ros/jazzy/setup.bash`, не используя старый кэш CMake от Humble.
   Убедиться, что `move_straight` не возвращает `requires a ROS2 build`.
3. Проверить порядок левого и правого мотора, знак коррекции и параметр
   `switch_motors` в `move_straight.cpp`. Симуляция принимает именно
   `[левый, правый]`. Начинать с короткой команды.
4. Проверить остановку `[0, 0]` после успеха, ошибки, таймаута и прерывания
   задачи. Не запускать параллельно `teleop.sh` как второй источник команд.
5. В первом тесте использовать `use_imu=false`. Для `use_imu=true`
   потребуется согласовать `gps_imu/angle` типа
   `geometry_msgs/msg/Vector3` с `/imu/data` типа `sensor_msgs/msg/Imu`.
   Также в `move_straight.cpp` обработчик IMU не устанавливает
   `angle_trace->ready=true`, поэтому ожидание IMU заканчивается таймаутом.
6. Не запускать драйвер физического мотора вместе с симуляцией. Камера,
   YOLO, Livox, ToF и RTK — отдельные этапы интеграции.

## Минимальная совместная проверка

Запустить симуляцию:

```bash
cd /home/kaitrye/Загрузки/robot_sim_harmonic/robot_sim
bash start_sim.sh
```

Затем в другом терминале проверить наличие `/motors`, `/encoders` и
`/ground_truth/odom`, запустить приложение в новом режиме `sim` и отправить:

```bash
curl -sS -X POST http://localhost:8080/api/tasks \
  -H 'Content-Type: application/json' \
  -d '{"task":"move_straight","params":{"ticks":1000,"base_speed":40,"use_imu":false}}'
```

Запрос возвращает `task_id`. Получить статус через
`GET http://localhost:8080/api/tasks/<task_id>`; проверить `DONE`,
остановку и фактическое расстояние по `/ground_truth/odom`. Сравнивать
`/encoders` с путём корпуса: при проскальзывании они могут расходиться.

Топики симуляции и расчёт виртуальных тиков проверены отдельно; полный
совместный прогон C++ приложения и Gazebo ещё не выполнялся.
