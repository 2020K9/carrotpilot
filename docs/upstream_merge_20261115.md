# Upstream merge 2026-11-15

Merge ajouatom/carrot-wip through 4fb490e17 into 2020K9/carrot-wip based on d3c4944ff (27 upstream commits).

- Preserve previously deleted Bluetooth B1 firmware/monitor; do not resurrect its new standalone feedback tests without that component. Upstream versions retained outside the repository for review.
- Preserve K9 braking/steering code, tests and all seven K9/PathOffset catalog entries byte-for-byte. Cluster renderer unchanged; upstream cut-in validation data is updated.
- Integrate upstream Tesla opt-in speed setting, radar matching, indicator logic, Jetson startup/status and SD/NVMe installer changes.
- Resolve settings counts to the actual 218 entries and regenerate web bundles from merged source. No model selection or vehicle installation performed.

## Worker verification

- Native params/IPC/vision IPC build passes. Changed Python sources compile; git diff whitespace check passes.
- Full web suite: 887 passed.
- Focused JetLink and wiki suites: 441 passed, 1 skipped, 1 failed. The tar extraction DeprecationWarning failure in test_refresh_selects_committed_release_and_repairs_hud reproduces on the parent (1 failed, 8 passed in that file).
- Expanded K9 + radar + settings + Tesla selection: merge 1299 passed, 21 failed, 231 errors, 23 subtests passed; parent 1517 passed, 7 failed, 5 subtests passed. Existing parent and new worktree have independently built native prerequisites, so aggregate counts are not an equivalent whole-system pass.
- New 18 failures and 231 setup errors are frozen-K9 comparison admission assertions: they reject the upstream Tesla interface.py change before executing their controller comparison. The checks are preserved, NOT weakened or reclassified as passed. Hyundai/K9 code and K9 tests are unchanged. Three Ray/Casper settings failures occur in both runs. Four Tesla failures in the baseline were not present in the merged run.
- No complete integration build, physical Jetson timing, HUD visual acceptance or driving validation is claimed.
