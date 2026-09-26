# Эксперименты с публичными решениями

`compare_public.py` читает настоящие bag через штатный `tram_eval`, вызывает ядра
пяти зафиксированных публичных репозиториев и наш pipeline. `stress_public_ideas.py`
проверяет два opt-in варианта нашего pipeline и сценарий зависания обеих тележек.
Исполняемое ядро проекта и `params.yaml` этими скриптами не изменяются.

Это эксперимент с адаптерами вычислительных ядер. ROS-публикации, таймеры, QoS,
сборка Humble и совместимость пакетов не проверяются этим запуском. Для mttex/ktoyart
сохраняются исходные callback-вычисления, но публикация перехватывается. Для ilush
поданы две скорости и нормированная команда, карта отключена, step вызывается на
входных событиях. Для Ange вызывается исходный C++ DualEkf: dt из событий, среднее
свежих колёс, разрешена адаптация массы; таймер и ограничения ROS-обёртки отсутствуют.
Их результаты описывают именно такие адаптации, а не штатные ноды.

## Подготовка

Скачать репозитории в `PUBLIC_REPO_ROOT` (по умолчанию `/tmp`) под именами ниже
и выбрать **точные commits**. Код соперников не копируется в этот репозиторий.

| Каталог | Репозиторий | Commit |
|---|---|---|
| msk-competitors-lcm | TellSamm/LCM-Solution | 77bb4de3ddd3bc4337db6cb2bbbe8fbdcf2703c3 |
| msk-competitors-mttex | b1ank37/mttex-tram-odometry-hackathon | 67664a62df3a00fe6132b25a61bc03ed346ed692 |
| msk-competitors-ktoyart | ktoyart/tram_odometry_pkg | 9bc59b1d10b891ba83c6c268ae0600c8ad34a66d |
| msk-competitors-ilushenssss | ilushenssss/odometer-on-model | a4b016de373422ff93393f87825668f7ef511fdf |
| msk-competitors-ange1ika | Ange1ika/tram_backup_odometry_ws | 4df9d7d7328fe5e5cf22699bb8c020a48304fe0b |

Зависимости Python: `requirements-dev.txt` плюс `pyproj`; точные использованные
версии — manifest рядом с отчётом. C++: g++ с C++17, заголовки Eigen 3.4.0.
`tram_map.pkl` проекта ktoyart не загружается: штатный fallback карты не меняет
уравнений скорости, а результаты положения для этого адаптера не считаются.

Пример сборки адаптера Ange (пути к Eigen и клону задать по своей раскладке):

```bash
mkdir -p out/public-comparison
g++ -std=c++17 -O2 -I /path/to/eigen-3.4.0 \
  -I /tmp/msk-competitors-ange1ika/src/tram_model_core/include \
  tools/research/ange_runner.cpp \
  /tmp/msk-competitors-ange1ika/src/tram_model_core/src/model.cpp \
  -o out/public-comparison/ange_runner
```

Переменные: `TRAM_DATA_DIR` — каталог bag; `BENCH_REPO` — checkout нашего зафиксированного
baseline (по умолчанию checkout скрипта); `PUBLIC_REPO_ROOT` — родитель клонов;
`ANGE_RUNNER` — собранный бинарник (по умолчанию `out/public-comparison/ange_runner`
в BENCH_REPO). `LCM_REPO` при необходимости переопределяет путь к LCM.

## Команды

```bash
python tools/research/compare_public.py --split holdout --jobs 4 \
  --variants ours lcm lcm_pertram mttex ktoyart ktoyart_si ilush ange \
  --out out/public-comparison/holdout
python tools/research/compare_public.py --split train --jobs 3 \
  --variants ours ours_frozen ours_adapt --out out/public-comparison/ideas-train
python tools/research/stress_public_ideas.py --split train6 \
  --out out/public-comparison/stress-train6
```

`train6` — заранее зафиксированные три bag 30618 и три 30639 из train; holdout
в этот набор не входит. Варианты:

- `ours` — штатный pipeline без изменений.
- `ours_trapezoid` — только интеграл пути заменён трапецией.
- `ours_frozen` — экспериментальное снятие доверия при ≥4 повторах ненулевой скорости
  >0,3 м/с и накопленной модельной Δv >0,3 м/с. Повторные stamp не добавляют наблюдение.
  **Это прототип:** отдельно проверить большие разрывы времени, ресинхронизацию и
  случай постоянной реальной скорости при ошибочной тяговой модели до включения в ядро.
- `ours_adapt` — экспериментальное увеличение дисперсии по медиане модуля последних
  30 инноваций (масштаб 1,4826, начало после 10, порог 3 номинальных σ, предел 1 м/с).
  Масштабирование адаптировано к дисперсии `r_wheel` нашего фильтра, а не перенесено
  как коэффициент с другими единицами.
- `lcm` — опубликованная общая калибровка и параметры ноды, callback-порядок событий.
- `lcm_pertram` — диагностический вариант, использующий номер вагона из имени bag;
  это дополнительная информация, не гарантированная контрактом ноды.
- `lcm_adapt` — опция adapt_r в чужом ядре.
- `ktoyart_si` — отдельно обозначенная правка входных единиц; `ktoyart` сохраняет ошибку оригинала.

По каждому bag сохраняются исходные метрики, покрытие эталона и `common_speed_rmse`:
RMSE на пересечении сопоставленных точек всех вариантов **конкретного запуска**.
Поэтому общий набор точек меняется при изменении списка `--variants`.
Для позиции LCM сохранены два соглашения: raw base_link и proxy master со смещением
−9,873 м по касательной карты. Это заданное геометрическое преобразование, не подбор
сдвига по эталону. UTM переводится в ENU эталона; GNSS вне окна не поступает оценщику.

Калибровки/карта LCM опубликованы после обучения с пересечением нашего holdout.
Даже общий evaluator не делает такую оценку независимой от обучения конкурента.
Метрики годятся для сравнения зафиксированных артефактов и поиска идей.
