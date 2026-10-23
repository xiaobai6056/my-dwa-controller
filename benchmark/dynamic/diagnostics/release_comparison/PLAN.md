# Release build comparison plan

Goal: measure only the existing MyDWA package build configuration effect without business logic/parameter changes.

Architecture: two fresh CMake build/install prefixes outside source, same current sources and compiler, empty build type vs Release. Original install stays intact. Reuse frozen V0.2 external probe and dynamic trial/evaluation. A copied launcher adds selected local_setup after original underlay. Read /proc maps to prove actual plugin path/hash and reject mismatches. Separate diagnostic results, never V0.1 success statistics.

- [x] Snapshot Git status/diff, all existing repository hashes, original install/build configuration, source/headers/config hash, environment and probe hash.
- [x] Configure/build both variants with BUILD_TESTING=ON and compile_commands; prove only intended flags differ (-O3 -DNDEBUG in Release).
- [x] Run both variants' existing 8 GTest targets and benchmark Python tests; build no core source changes.
- [x] Add tested path/hash selection and independent wrapper, preserving V0.2 entry/files unchanged.
- [x] Six actual D01 Headless runs in preregistered order U1,R1,R2,U2,U3,R3 (paired order alternates), domain91/master11436, same probe and parameters. Full restart each; compare actual mappings and all runtime parameters. Failures retained, cleanup failure halts.
- [x] Report per-run and pooled wall/threadCPU/score distributions, deadline fraction, call/output Hz, missed warnings, collision; use normal-returning subset too. Show run variability, not confidence claims from 3 runs.
- [x] Fresh tests/integrity/load audits; preserve binaries/build logs/raw data and Windows-readable report. No Git commit/push; original install remains default rollback.