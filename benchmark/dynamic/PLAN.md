# Dynamic Benchmark V0.1 implementation record — 2026-10-20

User-authorized direct implementation in existing workspace, no commits or parameter changes.

- [x] Inspect baseline: clean feature/dynamic-obstacle branch at 1bad3b6, workspace parent is not Git; no AGENTS.md under /home/user. Existing scene generator, Collector, preflight, ActionWorker, process cleanup and YAML switching reused.
- [x] Choose restart-per-trial rather than in-process reset (isolation); avoid importing RL environment. DynNav-Bench MIT reviewed conceptually; no third-party source copied.
- [x] Add deterministic D01 crossing / D02 temporary blockage; consistent world/map from existing generator, side pocket for D02.
- [x] Test independent polygon gap, motion phases, command statistics, result classification before implementation.
- [x] Add world state plugin and physical contact sensors to benchmark obstacles/walls only. Keep robot sensor and Nav2 YAML unchanged.
- [x] Reuse existing Collector and real NavigateToPose action; separate motion and evaluator helpers. Record ground truth, physical contacts, scans, Costmap observations, commands, feedback, BT/action events and configuration hashes.
- [x] Smoke D01 MyDWA / D02 DWB; investigate only infrastructure defects, retain failed attempts.
- [x] Freeze final assets/evaluator then run 12 formal attempts, alternating controller order and restarting all simulation processes per attempt.
- [x] Recompute metrics, audit raw records, build/test existing package, document results and limitations.

Timing: motion epoch = task dispatch observed /clock; fixed simulation-time keyframes independent of controller/robot pose. Wall watchdog remains separate. Completion = observed action terminal minus dispatch, success only real SUCCEEDED plus valid measurements and no contact. Failed times excluded from successful mean. Contact sensors (100 Hz) determine physical collisions; projected conservative robot rectangle gives explicitly approximate dynamic gap at nominal 50 Hz model states (unstamped, reception /clock quantized ~0.1 s). Geometry checks are not substituted for physical contact.

D01: 4 m straight in 3 m corridor, 0.5 m square crossing at x=1.6 from y=-1.1 to +1.1 at 0.25 m/s, t=4..12.8.
D02: 4 m straight, 0.9 m corridor with dead-end storage pocket, 0.4 x 0.8 m blocker at x=2; y=1.1 -> 0 at t=2..6.4, hold until t=20, exit by t=24.4. Geometric release when lower edge >= corridor upper edge 0.45. Static map omits moving entity. Existing localization uncertainty retained; no noise or Nav2 parameters adjusted.
Completed: 12 formal NavigateToPose goals accepted; 5 success, 6 contact cancellations, 1 true Nav2 FAILED. All original attempts retained. See RESULTS.md, first_run/ and evidence/raw_audit_handoff.log. Post-review exception-path fixes did not change measured results; one additional D01/DWB smoke retained separately. Existing uncrustify source formatting failure remains under freeze.
