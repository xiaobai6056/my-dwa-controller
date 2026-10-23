# D01 crossing timing scan

User authorized full execution with no algorithm/parameter changes. Independent timing_sweep directory; original dynamic sources and all older records stay byte-identical.

- [x] Freeze initial repository, original and Release/System DWB libraries, source/parameters/scenes, Git diff and statuses.
- [x] Tested scene adapter: only shift movement keyframe times/release_after, preserve 8.8s duration, .25m/s/path/size/start/goal; t4 exact equality to original.
- [x] Reuse original trial main with only load(scene) bound to effective scene; raw original summary retained and effective scene/provenance appended. Original Motion/Observer/evaluator unchanged. New launch sources Release local_setup last; no LD_PRELOAD, same observation method for both plugins; maps+SHA verify per run.
- [x] Preregister 30 fresh runs: 2/3/4/5/6 s x MyDWA/DWB x3. Alternate controller order and interleave timing points across repeats. Original t4 data not reused because old MyDWA formal build unoptimized and prior Release runs diagnostic.
- [x] For every trial, verify actual first GT displacement and full physical trace against intended schedule, not configuration alone. Keep complete invalid attempts, no selective rerun. Full restart/cleanup every run; abort on failed cleanup.
- [x] Independent offline metrics: Nav2 action, physical contacts, successful-only duration, approximate footprint clearance, sampled first sustained brake/turn, nearby contact position. Conservative no-interaction/observed waiting/turn-with-lateral-motion/unknown classification with explicit evidence boundaries.
- [x] Per-run CSV/JSON, 5-point collision plot, interaction XY/velocity/gap plots, Markdown report and commands, tests and frozen-source/raw consistency audits. No git add/commit/push.