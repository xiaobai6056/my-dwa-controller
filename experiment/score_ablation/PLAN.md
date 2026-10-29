# Dynamic scoring ablation plan

Scope approved by the user: independent offline experiments only. Formal baseline f7b6a45; parent rolling experiment 5cd10fc. No Controller, safety gate, scene, sampler, horizon, guide point or release changes.

- [x] Verify raw and weighted score semantics, Heading enable condition and exact unnormalized A–G table with tests.
- [x] Preserve every original geometry/prediction/queue helper and hard-filter block byte-for-byte. Change only scorer weights between schemes.
- [x] Make one common observation-only harness extension: run every trial to navigation time 14 s, record first passage without terminating there, and log weighted score components and rear-entry-predicting selected actions.
- [x] Run A–G at start=4 s; compare A's original prefix exactly and require entry/pass 8.95/12.95 s.
- [x] Screen for meaningful rearward displacement (diagnostic 0.05 m before 8.8 s) together with earlier passage (diagnostic 0.20 s), subject to all original gates. Also report continuous raw metrics so threshold choice is visible.
- [x] Recheck promising candidates at start=6 and 8 with corresponding A controls. Prefer smaller weight changes when effects are close; do not expand the given grid if no candidate qualifies.
- [x] Record fixed-window safety, stops, slow movement, yaw reversals, no progress and winner score terms; do not equate motion or yaw with rear passage.
- [x] Independently review, verify all protected SHA values and save exact code, parameters, raw traces, commands and binaries.

The 0.05 m and 0.20 s screening thresholds classify results only; they never enter the sampler, score, safety constraints or robot execution. Future-truth geometry and complete scene observation remain inherited idealizations.

Result: no scheme qualified; the conditional t=6/8 step was deliberately skipped, not claimed completed as trials.

Result: no scheme qualified; the conditional t=6/8 step was deliberately skipped, not claimed completed as trials.
