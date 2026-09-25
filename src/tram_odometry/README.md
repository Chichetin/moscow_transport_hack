# tram_odometry

ROS 2 ноды — тонкая обёртка над `tram_odometry_core`: подписка на `/vehicle/*` (и GNSS в
окне выставки) → `pipeline.Odometry.step` → `/result/velocity`, `/result/position`,
`/result/diagnostics`. Топики и формат — `docs/contracts.md` §1, параметры —
`config/params.yaml`, карта — `maps/route.csv`.
