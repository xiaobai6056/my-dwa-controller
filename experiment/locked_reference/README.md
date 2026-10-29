# Historical locked-reference experiment

[REPORT.md](REPORT.md) preserves findings and limitations;
[design.json](design.json) records geometry, weights, lifecycle and screening.
The early active-bypass screen was negative; 6/8 s, runtime integration and
Gazebo were not performed. See [the index](../INDEX.md) for the current snapshot.

Historical `run_trials.py` requires `EXPERIMENT_DATA_ROOT` with original
`input.json`, `params/design.json` and the matching archived `build/rolling`.
It creates exclusive result directories. Phase two requires a positive screen.
Current rolling sources include later changes and cannot substitute for that
historical source/binary. `analyze.py` reads original `results/phase1/{A_t4,B_t4}`
and writes derived analysis/plots under the selected external evidence root.

Original phase1 execution preceded the conservative selected-zero fail-safe.
The original proof found its predicate true for all 240 B proposals, so the added
boundary did not change that archived trajectory; no new run was used to validate
that boundary. Original producer source/ELF and final source/ELF identities are
external evidence. A locked target is neither an obstacle prediction nor a frozen
exclusion zone, and the connectivity certificate is not supplied as a DWA path.

Source timestamps retain +1 s; findings use navigation-relative time. Full scene
geometry, ideal association, current-pose rays and future-truth gates are
conditional assumptions. Circular-footprint clearance is not a real safety margin.
