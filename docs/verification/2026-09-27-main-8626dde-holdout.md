# Holdout на `main` `8626dde` после D-095/D-096

27.09.2026, 26 bag. Из корня worktree:

```bash
TRAM_DATA_DIR=/home/ir6/my/moscow_transport_hack/dataset/data \
TRAM_OUT_DIR="$PWD/out" \
/home/ir6/my/moscow_transport_hack/.venv/bin/python tools/eval/run_eval.py --split holdout
```

Полный результат: `out/eval/8626dde-holdout/metrics.json`. В файле `commit: 8626dde`,
`split: holdout`, окно GNSS 5,0 с, точка сравнения `base_link`. Ни один bag не упал;
оценок NaN/inf и несовпадений `Estimate.t` со stamp входа нет.

| Метрика | Медиана по bag | Худший bag | Значение |
|---|---:|---|---:|
| speed RMSE, м/с | 0,0319 | `30618_27e994fc` | 0,1606 |
| speed MAE, м/с | 0,02135 | `30618_defd0170` | 0,0643 |
| bias разгон, м/с | −0,0138 | `30618_082f1d65` | −0,1124 |
| bias торможение, м/с | −0,0042 | `30639_0ab96c59` | +0,1416 |
| bias стоянка, м/с | −0,0063 | `30618_082f1d65` | −0,0174 |
| bias ход, м/с | −0,0171 | `30618_defd0170` | −0,1013 |
| along RMSE, м | 2,18825 | `30618_0686195f` | 8,0489 |
| drift, % | 0,0123 | `30618_27e994fc` | 0,4352 |
| cross RMSE, м | 0,31885 | `30639_0ab96c59` | 4,7486 |
| pos3d RMSE, м | 2,43265 | `30618_0686195f` | 9,7779 |

Медианы D-096: along 2,188, cross 0,319, pos3d 2,433 м. Повторный прогон совпал
с ними после округления. D-095 меняет только публикацию `/result/velocity` нодой;
`tools/eval` сравнивает выход ядра с GNSS и эту задержку не учитывает.

Ограничение: при D-096 holdout использовали для выбора между двумя вариантами геометрии
карты, поэтому выигрыш на holdout может быть завышен. Независимый контрольный bag
организаторов `30618_88aea4d9` описан в D-096 и
`docs/verification/2026-09-27-judge-checker.md`.
