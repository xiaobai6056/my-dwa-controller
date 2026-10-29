# Polyline offline reproduction

From the repository root, in Linux/Bash:

```bash
source /opt/ros/humble/setup.bash
source /path/to/controller_install/setup.bash
bash experiment/polyline_bypass/reproduce.sh /path/to/new_external_output
```

Replace both `/path/to` locations with real paths. The output directory must be
new and outside this repository. No ROS node, Gazebo, installation or deployment
is launched. Dependencies are listed in [the index](../INDEX.md).
Use an installation built from this branch's matching controller sources;
the index identifies the validated baseline. A different API-compatible
installation is not a numerical reproducibility guarantee.

This entrypoint builds the final checked-in rolling sources, runs five CTest
executables (41 individual tests), and executes bundled A0/A1/A2 at start=4 s.
A0 uses mode A (latched parking); A1/A2 use mode B with their exact per-input
reference parameters. These names are bundled reproduction cases, distinct from
the historical A–H matrix names in [REPORT.md](REPORT.md). Input provenance paths
are relative archive identifiers; runtime consumes embedded numerical scene data.

The original experiment completed A0/B0/A–D through 16 s. E stopped at 7.15 s
on a nominal braking exclusion failure, before advancing the abort row. F/G/H,
12/16 multipliers and 6/8 s confirmation were not run. There was no qualifying
best/second-best scheme and no real Controller/Gazebo trial. Preserve this stop;
do not resume or tune the failed matrix.

Design: entrance s=min(robot_s,zone.lo-margin), exit s=zone.hi+margin,
q=zone.rear-margin; check every robot→entrance→exit segment against walls and the
original exclusion zone. Four distinct fresh stopped observations admit a locked
polyline; original path projection/cropping supplies P/G/H, with H multiplied only
while guided. Other weights and all nominal/braking/queued safety gates remain.
The predeclared screen required sustained >=0.05 m rearward displacement for
>=0.2 s before 8.8 s, earlier actual entry, >=0.2 s full-pass gain over B0, release
and path recovery without abnormal rotation. No case met the complete screen.

Historical analysis requires the external original archive:

```bash
export EXPERIMENT_DATA_ROOT=/path/to/copied_polyline_evidence
python3 experiment/polyline_bypass/analyze.py
python3 experiment/polyline_bypass/diagnose.py
```

These tools expect historical `results/phase1/{A0,B0,A,B,C,D,E}`, per-case
`parameters.json`, original `input.json`, and (for diagnose.py) `failure_check.json`.
They write derived files under the selected evidence root. The three-case output
of reproduce.sh is not the complete historical archive. To recreate only E's
read-only failure diagnostic, using archived E commands:

```bash
/path/to/build/failure_check "$EXPERIMENT_DATA_ROOT/input.json" \
  "$EXPERIMENT_DATA_ROOT/results/phase1/E/start_4.00.jsonl" \
  /path/to/new_failure_check.json
```

This checks pose parity and the failed exclusion subcondition, without selecting
new actions or advancing beyond the abort; it is not an independent full dynamic
filter audit. Scores during command_held describe proposed_candidate; selected
is actual output. Source timestamps have the inherited +1 s offset; reports use
navigation-relative time. Ideal geometry, association, synthetic rays and future
truth gates are conditional model assumptions. Minimum clearance is a circular
footprint approximation, not a real sensor safety claim.

Original red/final XML, immutable ELF/library hashes, raw traces, commands,
protection manifests and source bundle are external evidence inventories described
by the historical report, not additional bundled files.
