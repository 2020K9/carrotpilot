# Fairy Tale retirement

Fairy Tale is removed from the selectable model list using the existing retired catalog mechanism. Existing specifications and signed release metadata remain solely for boot continuity and exact cache identification. Other model entries are unchanged.

An already-running Fairy Tale boot remains frozen. If its saved selection is still Fairy Tale at the next boot, existing catalog policy selects N16; choosing another available model before reboot takes precedence. Offroad maintenance queues the unused Fairy Tale cache for the existing Jetson cleanup protocol. No live vehicle installation or cache deletion was performed during publication. Published release assets are retained.

Validation: standalone Python 3.12 model-library tests, including selection rejection, hidden UI row, frozen running boot, next-boot fallback, and retired Jetson-only cleanup queue. This is not a vehicle inference test.

2026-11-28 (대감님 "지워"): the signed Jetson release `fairy-tale-release.json` and its catalog pointer are removed, and the Fairy Tale GitHub releases are deleted. Only the retired catalog entry and `fairy-tale.json` spec stay, so offroad cleanup can still find and delete an old Fairy Tale cache by its exact sha256. A boot still frozen on Fairy Tale advertises no Jetson release. Other models are unchanged.
