# Stress eval review fixes (#15): holdout verification

- Baseline: `153704a` (`origin/main` at test time); candidate changes only `tools/eval` and documentation.
- Holdout: 26 bags, GNSS window 5 s. Run with the shared `.venv` on Linux; no ROS runtime needed for this evaluator.
- Commands:

```bash
.venv/bin/python tools/eval/run_eval.py --split holdout --jobs 8 \
  --out out/eval/issue15-followup-main
.venv/bin/python tools/eval/run_eval.py --split holdout \
  --compare out/eval/issue15-followup-main/metrics.json --stress --jobs 8 \
  --out out/eval/issue15-followup-branch
```

## Contract metrics

All 16 metrics matched `origin/main` exactly (0.000% change). The CLI reported `D-012: главные метрики не хуже`; 26/26 bags had no crash, nonfinite estimate, or output/input stamp mismatch.

| Metric (median over bags) | origin/main | candidate | Δ, % |
|---|---:|---:|---:|
| speed_rmse | 0.062 | 0.062 | 0.000 |
| speed_mae | 0.039 | 0.039 | 0.000 |
| speed_bias_accel | -0.059 | -0.059 | 0.000 |
| speed_bias_brake | 0.039 | 0.039 | 0.000 |
| speed_bias_stop | -0.006 | -0.006 | 0.000 |
| speed_bias_cruise | -0.025 | -0.025 | 0.000 |
| drift_pct | 0.028 | 0.028 | 0.000 |
| along_mean | 1.889 | 1.889 | 0.000 |
| along_max | 9.443 | 9.443 | 0.000 |
| along_rmse | 2.610 | 2.610 | 0.000 |
| cross_mean | 0.329 | 0.329 | 0.000 |
| cross_max | 5.003 | 5.003 | 0.000 |
| cross_rmse | 1.075 | 1.075 | 0.000 |
| pos3d_rmse | 3.219 | 3.219 | 0.000 |
| pos3d_max | 11.2 | 11.2 | 0.000 |
| slip_flag_frac | 0.001 | 0.001 | 0.000 |

## Stress diagnostics after fixes

Every scenario that ran produced estimates during its event; there were 0 crashes. `gap_70` skipped one bag because the event and recovery tail did not fit. Duplicate timestamps remain separate observations for event peaks; recovery uses the maximum excess at each duplicate time.

| Scenario | bags with result | speed peak median/max, m/s | speed excess median, m/s | speed recovery median / no recovery, s | position peak median/max, m | position excess median, m | position recovery median / no recovery, s | no event estimates | crashes |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| outlier | 26/26 | 0.078/50.0 | 0.022 | 0.025 (0) | 5.395/2007.3 | 0.001 | 0.021 (2) | 0 | 0 |
| gap_1 | 26/26 | 0.124/2.476 | 0.050 | 0.016 (0) | 3.751/2007.4 | 0.003 | 0.016 (0) | 0 | 0 |
| gap_10 | 26/26 | 0.172/2.476 | 0.069 | 0.022 (0) | 4.793/2007.6 | 0.134 | 0.022 (0) | 0 | 0 |
| gap_70 | 25/26 | 0.246/2.476 | 0.097 | 0.017 (0) | 6.393/2012.7 | 0.266 | 0.017 (0) | 0 | 0 |
| spike | 26/26 | 0.191/6.943 | 0.139 | 0.023 (0) | 6.046/2007.6 | 0.183 | 0.016 (4) | 0 | 0 |
| noise | 26/26 | 1.192/3.148 | 1.090 | 0.163 (0) | 7.132/2011.5 | 3.644 | 6.515 (2) | 0 | 0 |
| jitter | 26/26 | 0.164/2.475 | 0.057 | 0.017 (0) | 4.784/2007.9 | 0.360 | 0.017 (0) | 0 | 0 |
| rollback | 26/26 | 0.165/2.475 | 0.046 | 0.022 (0) | 4.786/2007.6 | 0.010 | 0.022 (0) | 0 | 0 |

The updated `outlier` and `rollback` aggregates differ slightly from the original PR report because the corrected duplicate-stamp handling now retains the worst publication. The large absolute position peaks are pre-existing alignment error in `30639_4285f2bc` (#70); excess versus the clean pass isolates the injected disturbance.
