# Scan trend implementation and bounded validation

Goal: extend the current dirty working tree with default-off scan tracking, unified
path evidence, and the existing risk governor. Preserve DWA and soft preferences.

Architecture: ScanObstacleTracker owns subscription and exact-time TF processing
in odom, adjacency clusters and a 0.8 s linear fit. PathConflictMonitor combines
scan motion with current Costmap occupation in ONE ConflictEvidence. RiskGovernor
uses the same stop/wait/clear/recover state machine. Missing/ambiguous scans cannot
confirm clear. Scan states, not control cycles, count as new sensor evidence.

- [x] Save baseline hashes/archive; inspect live source, D01/D02 and recent runs.
- [x] Red: tracker synthetic scans (ego motion, crossing speed, dropout, bad TF,
  invalid points, static wall, out-of-order time); monitor and governor regressions.
- [x] Green: tracker header/source; sensor_msgs dependency and independent target.
- [x] Green: extend monitor optional scan snapshot, fuse geometry, preserve legacy
  when omitted; governor blocks clearing on stale/lost data and preserves cap when
  a previously certified stop becomes uncertifiable.
- [x] Wire configure/activate/deactivate/cleanup and per-cycle snapshot; add source
  stamped diagnostics; verify real plugin integration and disabled signatures.
- [x] Build/install only build_scan_trend/install_scan_trend/log_scan_trend.
- [x] Round 1: original D01 2s and 4s (two fresh launches), motion/path/governor
  evidence and known 0.25 m/s truth. Diagnose layers before any correction.
- [x] Round 2: original S05 (one fresh launch), static motion and risk false alerts.
- [x] Round 3: select one D01 condition from evidence (one fresh launch), confirm
  braking/wait/recovery or document exact failure layer. Maximum four launches in
  three purposeful rounds, with no threshold sweeps.
- [x] Full CTest, functional CTest, scoped C++ formatting separately; archive final
  source/plugin/parameters and verify protected baseline hashes. Report limitations.

Alternatives considered: Costmap-only cannot estimate physical speed; an external
tracking node adds lifecycle/transport work without benefit here. A small tracker
class with an optional subscription keeps the existing Controller orchestration.
The user has explicitly authorized implementation and selected responsibilities;
execute inline without another design/permission gate. No existing result edited.
