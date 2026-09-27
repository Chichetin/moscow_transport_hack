# Штатное завершение ноды (#205, D-099)

27.09.2026. База — `fccfba998f3bbfb3b44e8723ea54bafb458371bd`.
Исключение из заморозки `src/` — [#24](https://github.com/Chichetin/moscow_transport_hack/issues/24#issuecomment-5859205279).

`main()` ловил только `KeyboardInterrupt`. В Humble закрытие контекста может прервать
`spin` исключением `ExternalShutdownException`; в runner #203 оно выходило наружу с exit 1.
Добавлены импорт исключения и его обработка вместе с `KeyboardInterrupt`.
Оба штатных пути проходят существующий `finally`: уничтожение ноды и `try_shutdown()`.
Неожиданный `RuntimeError` после очистки продолжает выходить вызывающему коду.

## Красный тест и зелёные проверки

Данные не монтировались. Использован существующий образ `tram-odom:dev`
(`28af3e7e7d23`), ROS 2 Humble, сеть отключена, 1 CPU, 512 МБ.
Рабочая копия монтировалась в `/ws`; `TRAM_DATA_DIR=/tmp/nonexistent-205`,
`PYTHONDONTWRITEBYTECODE=1`. Перед тестами в контейнере:

```bash
source /opt/ros/humble/setup.bash
colcon build --executor sequential --event-handlers console_cohesion-
source install/setup.bash
```

| Проверка | Команда | Результат |
|---|---|---|
| База, существующие ROS-тесты | `python3 -m pytest -p no:cacheprovider src/tram_odometry/test -q` | 32 passed |
| Новый тест до исправления | `python3 -m pytest -p no:cacheprovider -rA src/tram_odometry/test/test_main.py` | 1 failed, 3 passed; падает `ExternalShutdownException` |
| После исправления и пересборки `tram_odometry` | `python3 -m pytest -p no:cacheprovider -rA src/tram_odometry/test` | 36 passed |
| Обнаружение тестов через colcon | `colcon test --packages-select tram_odometry --return-code-on-test-failure --event-handlers console_cohesion-` и `colcon test-result --verbose` | 36 tests, 0 errors, 0 failures, 0 skipped |
| Полный pytest на хосте без данных | `TRAM_DATA_DIR=/tmp/nonexistent-205 PYTHONDONTWRITEBYTECODE=1 /home/m1hairu/Documents/msk_transport/repo/.venv/bin/python -m pytest -p no:cacheprovider` | 810 passed, 8 skipped |

Регрессия вызывает настоящий `main()` и создаёт настоящую ROS-ноду. Для ветви
`ExternalShutdownException` подменённый `spin` сначала закрывает контекст, затем выбрасывает
исключение: повторная очистка контекста безопасна. Проверяются также обычный возврат `spin`,
`KeyboardInterrupt` при живом контексте и передача исходного `RuntimeError` после очистки.

## Проверка сигналом и её пределы

В контейнере запускался `ros2 launch tram_odometry odometry.launch.py`. После появления всех
шести входных подписок наблюдатель посылал **один SIGINT процессу launch**; launch пересылал
сигнал своей ноде. После исправления:

```text
node_ready=True
[INFO] [odometry_node-1]: process has finished cleanly
launch_exit=0
clean_shutdown=True
```

Такой одиночный сигнал успешно завершал и базу: конкретный путь исключения зависит от
порядка обработки сигнала. Исправление ветви `ExternalShutdownException` подтверждено
красным/зелёным тестом выше. Две попытки отдельного закрытия контекста во время настоящего
`spin` также закончились обычным возвратом и не воспроизвели это исключение.

Предварительная проверка SIGINT всей группе процессов на базе дала другой traceback:
`KeyboardInterrupt` во время конструктора либо `destroy_node()` — launch может переслать
ребёнку второй сигнал. Эта гонка нескольких сигналов данной правкой не закрыта.
Локальные логи и скрипты проверки: `out/verification/205-shutdown/`.

## Точность и область изменения

Holdout повторно не запускался: меняется только обработка исключения при завершении.
Побайтная идентичность ядра, параметров и карт базе проверена командой с exit 0:

```bash
git diff --exit-code fccfba998f3bbfb3b44e8723ea54bafb458371bd -- \
  src/tram_odometry_core src/tram_odometry/config src/tram_odometry/maps
```

Новых чисел точности не заявляется. Контракты, входы, GNSS-окно, обработка сообщений и
значения выходов не меняются.
