# Scan Emergency V2 archive

This branch preserves the Scan Emergency V2 proof of concept as it stood on
2026-10-23. Its parent is static V1.1, commit
`1bad3b66aa64384d5955f1933dcfb5c1017dcd62` (`v1.1-static`).

The archive includes controller source, configuration, all available C++ tests
(including previously ignored static tests), Benchmark definitions and supporting
scripts, experiment reports, and selected compact result tables and figures.
The source and configuration are archived without algorithm changes.

Read `docs/scan_emergency_v02.md`,
`benchmark/scan_emergency_20261022/REPORT.md`, and
`benchmark/scan_v2_matrix_20261022/REPORT.md` for scope, limitations, and the
observed t4 collision. This is a proof of concept, not a general dynamic obstacle
avoidance release. Reported historical test results are not new test runs.

Raw bags/JSONL, runtime and build logs, compiled libraries, replay recordings,
source ZIP/TAR snapshots, caches, and temporary/build output are deliberately
excluded from Git. Their complete external workspace snapshot remains separate
and unchanged. Replaying or regenerating results requires those external inputs;
paths in historical reports refer to their original experiment environment.

No dynamic Benchmark is migrated to the new V3 branch. `feature/dynamic-v3`
starts at the exact static V1.1 commit, without dynamic source or configuration.
Note that static V1.1 originally tracks neither the test directory nor the
Benchmark directory; its CMake test references therefore need separately scoped
test restoration before a fresh BUILD_TESTING build. This archive preserves the
available tests without changing that baseline.
