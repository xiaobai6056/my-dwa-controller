# Fallback non-acceleration preference implementation plan

Goal: test whether unnecessary speed recovery can be suppressed without removing any original DWA escape candidate. No detector changes, score changes, new tuning parameters or unchecked commands.

Architecture: RiskGovernor keeps the original hard maximum in DwaFallback and supplies an optional preferred maximum equal to the current nonnegative velocity, bounded by the hard maximum. Controller retains the original sampled candidates, adds the exact preferred-speed boundary using the existing angular samples if missing, and gives collision-free candidates at/below that boundary priority. Within each priority group the original score and angular tie-break remain. If all preferred candidates fail existing checks, the original accelerating candidates remain eligible. This is a feasibility-based exception, not proof of dynamic safety.

Alternatives: a hard current-speed cap is simpler but can remove the only feasible escape. A score penalty retains all candidates but requires tuning and can still accelerate when a non-accelerating candidate exists. The chosen lexicographic preference has a precise, testable behavior and no weight.

Files: risk_governor.hpp/.cpp (decision output), controller.hpp/.cpp (sampling and selection), controller_diagnostics.hpp/.cpp (counts/preference/escape reason), existing governor/controller/diagnostics GTests. Monitor, sampler, predictor, obstacle checks, scorer and config remain unchanged.

- [x] Preserve source, configuration and old experiment hashes. Run current binary D01 2s/4s as same-version controls before changing src/include.
- [x] Add failing governor and plugin tests: current-speed preference; all original escape candidates retained; turn/deceleration choices; exact boundary at zero/coarse sampling; no unchecked zero when all invalid; clear confirmation releases preference; disabled baseline signature unchanged.
- [x] Implement optional `preferred_max_linear_speed` and priority selection. Add only required diagnostic fields: preferred cap, feasible preferred count, accelerating escape selected.
- [x] Build to a NEW isolated prefix; run focused and existing functional tests. Preserve failure output before implementation and green output afterward.
- [x] Reuse existing smoke helper via a thin adapter to a new results directory. Run modified D01 2s/4s once; compare source-stamped odometry, command receipt times, collision outcomes and selector evidence. Run D01 6s/S04/S05 only as limited regression checks if resources allow. No parameter search or altered scenes.
- [x] Review exact diff and audit input hashes; report failures as well as successes, distinguish candidate-level guarantee from collision avoidance. Roll back if the experiment does not support retaining the change.

Build: `colcon --log-base log_fallback_acceleration build --build-base build_fallback_acceleration --install-base install_fallback_acceleration --packages-select my_dwa_controller --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON` from /home/user/ros2_ws after sourcing Humble.

Validation is authorized by the current request and the standing instruction to implement after inspecting the project without stopping at a proposal. Keep all existing uncommitted work; no commits/resets or large refactor.
