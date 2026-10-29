# Strong Heading and reference-margin offline experiment

Continuation authorized by the user's request to substantially increase reward
parameters and test 0.2/0.3 m instead of 0.1 m. No new planner/control logic.
The inherited locked-reference lifecycle is the approved design.

Goal: determine whether stronger existing Heading guidance and more conservative
reference-point offsets produce meaningful early rear passage under identical
hard constraints. This plan uses the writing-plans and verification workflow.

Architecture: parameterize only the existing offline referenceScore Heading
multiplier and reference geometry's two constants. Preserve ordinary scoring on
exit, the existing four-frame permission and selected-zero fail-safe. Independent
no-hardlink clone of locked_reference commit b43493e; no runtime deployment.

Fixed pre-run matrix: H multiplier {1,10,20,40}; lateral extra clearance and
forward offset beyond the exit both {0.10,0.20,0.30} m, paired. This is 12 B
variants plus one ordinary latched-wait A. H absolute {0.05,0.5,1,2}; P/G/V/O
remain 0.2/0.6/0.1/0.1. No normalization. The reference-point clearance certificate
changes from 0.27+0.10 to 0.27+configured_extra; the original hard clearance,
Costmap inflation, exclusion zone and brake gates do not change.

- [x] Snapshot immutable inputs/old experiments/formal repo and independent clone;
  inspect applicable AGENTS instructions and commit this plan.
- [x] Add failing unit tests for configurable .2/.3 reference position, stricter
  narrow-channel rejection, Heading multiplication with other terms unchanged.
- [x] Implement only those parameters in reference.hpp/cpp; feed from per-run
  input JSON into the copied runner. Record effective Heading weight and whether
  scores refer to a held proposal. Preserve original safety blocks byte-for-byte.
- [x] Build with immutable baseline libraries and pass all inherited/new tests.
- [x] Run A and original B(.1,H1) at 4 s once; compare all action/state/score fields
  with archived original traces to validate exact behavioral reproduction.
- [x] Run the remaining fixed 4 s matrix once each, same scene/initial state,
  t=4..16 s, no termination on first pass. Stop sweep on contact, exclusion
  violation, unsafe recovery/fallback failure or abnormal repeated start-stop;
  preserve evidence without tuning or rerunning the unsafe variant.
- [x] Classify actual motion vs rear displacement vs actual passage. Keep strict
  pre-8.8 displacement and sustained .05 m/.2 s screen, total pass gain >=.2 s
  versus parking A, plus absence of stalls/oscillation and meaningful benefit
  over original B. Save every target, actual command and score component.
- [x] Only if a safe, clearly improved candidate exists, check the least aggressive
  eligible candidate and matching A at starts 6 and 8, using the same end 16 s.
  Otherwise stop at the fixed matrix; never extend the search automatically.
- [x] Analyze key scoring/geometry evidence; archive code/ELF/plugin and static
  library SHA, full parameters/traces/raw logs/commands/tests and protection hash.
  Commit only the independent experiment branch, no push/merge/Gazebo.

The paired margins are reference-point construction settings, not a guaranteed
extra margin on every executed trajectory. Ideal scene geometry/current-pose
synthetic rays and future truth gates remain declared assumptions, not real Scan.
Interpret any early motion as approach until actual rear entry and passage occur.

Completed: fixed 12 B + A matrix; conditional 6/8 checks skipped because none eligible. No tuning after results.
