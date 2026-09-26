# Проверка типов и stamp записи жюри (#97)

Коммит кода `dee1c556fcd39992a7e9389463087c33ab426157`, команда из worktree:

```bash
bash tools/submission/jury_layouts.sh --run
```

Команда завершилась с кодом 0. Отчёт:
`out/submission/layouts-dee1c556fcd39992a7e9389463087c33ab426157.json`,
`commit=dee1c55`, `dirty=0`, 10/10 раскладок `build=ok`.

| Исходный bag | Статус | `/result/velocity` | `/result/position` |
|---|---|---:|---:|
| `30618_082f1d65` (с GNSS) | ok, типы и stamp из входов | 891 | 891 |
| `30618_5036aa78` (без GNSS) | ok, типы и stamp из входов | 1116 | 1116 |

`jury_layouts.sh --run` воспроизводит по 25 секунд каждого bag. Для каждого записанного
сообщения проверены контрактный тип и ненулевой `header.stamp`, совпадающий точно с одним
из трёх `/vehicle/*` входов исходного bag. Ошибка делает `status=fail` и попадает в
`runs[].note`. Тесты `test_check_recording.py` отвергают неверный тип, чужой/нулевой stamp,
проверяют каждую публикацию и не разрешают GNSS stamp как основание для выхода;
`test_layout_report.py` проверяет сохранность JSON при многострочных логах ROS.

`bash docker/dev.sh python3 -m pytest`: 384 passed, 8 skipped. `bash docker/dev.sh colcon build`:
3 packages finished. Проверка содержимого полей `Odometry` и перевода км/ч → м/с остаётся
в тестах ноды `test_position_message_follows_contract` и
`test_real_core_converts_kmh_to_mps_once`; архивный гейт проверяет типы и stamp.
