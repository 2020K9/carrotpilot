# Latest carrot-wip integration — 2026-11-09

Integrates ajouatom/openpilot carrot-wip `0436c7f0a7c9741f55bbb175f68c53d0df0c85d5` into 2020K9/carrotpilot `53bf47d6579cf793f0d41c3145d94da96971450f` at the owner's request. This is the upstream update, not just the earlier brake-mode patch.

## Preserved fork behavior

- K9 creep implementation, saved-value migration, and every K9 settings entry are byte/content-identical to the published parent. Mode 3 retains the half-gap addition below approximately 1.5 km/h.
- Existing fork pedal-interceptor and B1-monitor/display deletions remain; conflicting upstream edits do not silently restore those retired components. Their removed test/workflow entries remain excluded. The new import-path test exercises all retained module identity assertions and explicitly checks B1 monitor remains absent.
- Retain the fork's CanfdBlinkerControl exclusion before USB2CAN startup while adopting the upstream source reaper and owner support.
- Rebuild generated web assets from merged source; merge settings by content, retaining K9 entries and new upstream entries (216 total). Wiki count assertions reflect this actual catalog.
- Includes upstream external Light speed 2 catalog/runtime support; does not restart the retired local student-model research, select a model, install weights, connect to a vehicle, or initiate training.

## Verification

- K9 None/0/1/2/3, five synthetic sequences each: exact CAN equality between pre-merge and merged worktrees, independently executed in each tree.
- K9 focused/schema and memory diagnostics: 124 passed, 20 setup errors. The 20 historic frozen-controller cases reject the new common hyundaicanfd changes before running. Their assertions remain unchanged; the separate direct two-tree CAN comparison above is not reported as those tests passing.
- Upstream controlsd publish, vehicle fault handling, CAN-FD stop-object/stopping and UI helper tests: 270 passed with the repository's isolated offline runner.
- Assistant HTTP/services, model rejection/retry and retained import paths: 137 passed.
- Full browser suite: 885 passed after installing pinned dependencies and selecting the test Python interpreter.
- Wiki checks: 15 passed. Web asset build and Python/JSON parse checks passed.
- Tests executed within worker /k9bot CPU cap. Raw logs remain private, outside the repository.

No full native vehicle build, road validation, live install, or whole-repository all-green claim. Upstream feature validation limits remain applicable.
