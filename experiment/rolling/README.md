# Historical rolling model

See [the experiment index](../INDEX.md) for the current build/reproduction entrypoint
and [the original report](../../docs/experiments/rolling_bypass_20261029.md) for
results and limitations. Current C++ includes later reference/polyline changes;
use the original archived source/binary for exact historical rolling trials.

`analyze.py` requires `EXPERIMENT_DATA_ROOT` pointing to a copied external rolling
evidence archive containing original `input.json` and `results/summary.json` plus
`results/start_*.jsonl`. It writes derived metrics, connectivity certificates and
plots there. Certificates are diagnostics and never enter the reference path,
sampler or scorer. `audit_selected EXTERNAL_EVIDENCE_ROOT` replays selected nominal
predictions from that same layout; it does not select new actions.

The model checks D01 input size/timing and uses complete geometry, future-truth
collision admission, aligned maps and persistent delayed execution. Real sensor
and executor behavior are unverified; offline clearances do not establish safety.
