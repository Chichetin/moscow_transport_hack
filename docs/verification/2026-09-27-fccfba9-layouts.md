# Exact-HEAD layouts fccfba9 — evaluator act

Commit: `fccfba998f3bbfb3b44e8723ea54bafb458371bd`. Tracked tree clean before and after. Existing `.venv` symlink preserved.

[Unmodified report published with this act](2026-09-27-fccfba9-layouts.json). The local paths below identify the measurement environment; they are not required installation paths.

- Command: `bash tools/submission/jury_layouts.sh --run`.
- UTC: 2026-09-27T19:24:28.243841+00:00 → 2026-09-27T19:26:52.666341+00:00; 144.423 s.
- Exit: 0; 12/12 builds and 3/3 runs OK.
- Unmodified report: `/home/m1hairu/Documents/msk_transport/repo/.claude/worktrees/docker-verify-fccfba9/out/submission/layouts-fccfba998f3bbfb3b44e8723ea54bafb458371bd.json`. SHA256 `22f66825a5ba55c0b207980ba6ed85a8ac181d0b62153f653bbde2e8012b2c05`.
- Follow-up `check_submission.py`: exit 1; only blockers #200/#201 FAIL. layouts/tree/artifacts/markers/compliance/params OK.

| Replay | Velocity messages | Position messages | Result |
|---|---:|---:|---|
| 30618_082f1d65, full messages | 891 | 891 | node alive, 0 logged errors, types/stamps OK |
| 30618_5036aa78, no GNSS | 1125 | 937 | node alive, 0 logged errors, types/stamps OK |
| 30618_082f1d65, judge messages | 430 | 429 | node alive, 0 logged errors, types/stamps OK |

Exposure: three real replays, including two of holdout 30618_082f1d65; one replay of no_gnss_train 30618_5036aa78. The unchanged script limits playback to 25 s, so the 98.84 s no-GNSS bag was not replayed in full. Three input stamp-validation scans also ran. This is an authorized frozen-code evaluator check, not a blind test or accuracy evaluation. No tuning, no access to sealed154.

Container: ordinary user 1000:1000, 2 CPU, 512 MiB RAM/swap total, network none; exact snapshot in container-settings.json. These layouts do not establish late-start or persistent-node/multiple-bag safety. Per-process player/node exit values are not separately retained by the published script.

Logs and metadata are in this directory; detailed layout/build/run logs are next to the unmodified report under `/home/m1hairu/Documents/msk_transport/repo/.claude/worktrees/docker-verify-fccfba9/out/submission/layouts-fccfba998f3bbfb3b44e8723ea54bafb458371bd`. No product changes, commits, pushes, GitHub messages, issue state changes, or tag operations performed.
