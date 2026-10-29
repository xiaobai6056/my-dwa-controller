# Historical scoring ablation

[REPORT.md](REPORT.md) preserves the fixed A–G comparison, exact weights,
classification definitions and negative conclusion. No scheme qualified, so
6/8 s confirmation was not performed. [The index](../INDEX.md) identifies the
current final-snapshot reproduction; it does not reproduce predecessor weights
using the final polyline runner.

Historical `run_trials.py` requires the matching archived ablation `build/rolling`
plus an explicit `EXPERIMENT_DATA_ROOT` with original `input.json`, `params/*.json`
and a new results destination. `EXPERIMENT_PREVIOUS_ROOT` selects the matching
rolling evidence archive for A-prefix parity. Result directories are exclusive;
A parity must pass before B–G. Do not use a current final-snapshot binary to
claim historical ablation reproduction or run new confirmation without evidence.

`EXPERIMENT_DATA_ROOT=/path/to/copied_ablation_evidence python3 experiment/score_ablation/analyze.py`
recomputes derived JSON/plots from original `results/phase1` and `input.json`.
It writes only derived outputs in the selected directory; these classifications
never enter control or safety logic. Raw archives and binary identities are
external, not bundled. Ideal geometry/association and future-truth gates remain
model assumptions; real perception/execution and safety are unverified.
