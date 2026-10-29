# Historical Heading and reference-margin sweep

[REPORT.md](REPORT.md) retains the fixed 12 B variants plus parking A, their
findings, negative screen and limitations. [matrix.json](matrix.json) records the
predeclared parameters, stop rules and confirmation thresholds. None qualified,
so 6/8 s confirmation and further search were not performed.

For historical tooling, set `EXPERIMENT_DATA_ROOT` to a copied external sweep
archive with original `input.json`, `params/matrix.json`, per-case inputs,
parameters and `results/phase1` telemetry. `analyze.py` computes output metrics;
`diagnose.py` also requires the matching `build/audit_reference` and `logs/`;
`summarize_audit.py` reads resulting per-case score_audit.json. They write derived
evidence under that selected root and do not launch rolling trials.

`run_sweep.py` also needs `EXPERIMENT_PREVIOUS_ROOT` for the matching locked-point
archive and the original sweep `build/rolling`. Its exclusive result directories,
A/B old-field parity and safety/chatter stop gates remain. Use original source
and binary identities for exact historical reproduction; the current final
polyline runner is a later implementation. See [the index](../INDEX.md).

Heading is multiplied only during guidance; ordinary weights are unchanged.
0.2/0.3 m affect reference construction, not a universal execution clearance.
When command_held is true, score fields describe proposed_candidate; only
selected/next_pose represent movement. Source stamps retain +1 s; report times
are navigation-relative. Scoring replay reconstructs the archived queue and
current-pose map and checks proposal scores within 1e-12, with ideal geometry
and association. It is not a sensor replay or real-system safety evidence.

Raw traces, binary/library SHA records, original tests and source bundle remain
external evidence inventories described in the report, rather than bundled files.
