# K9 HUD2 — active GitHub driving renderer port

## Provenance and permission

GRT47/HKG_5W_TurzxMonitor `android-hud`, commit
`1e0538c223dc78e09a5f74a624875d24ba8e1e99`:
https://github.com/GRT47/HKG_5W_TurzxMonitor/tree/1e0538c223dc78e09a5f74a624875d24ba8e1e99

Sources: `package_t88_hud/src/com/hkg/huddisplay/HudRenderer.java`,
`Car3dView.java`, `HudState.java`; original traffic assets under
`package_turzx_monitor/design_tesla/navdy_*`. Existing `assets/hud2/SHA256SUMS`
records byte-for-byte sprite provenance. User reported private permission on
2026-11-28 00:23 KST. This does not assert an independently verified public license
or general redistribution/sublicensing rights.

The earlier `drawRoad` fallback implementation was the WRONG active source.
`drawVizBgContent(external3dCar=true)` calls `drawRoad3d` (lines711–728).
`stepSimulation` publishes shared `camVpMat` (564–605), and `Car3dView` uses that
matrix at1004. Its local fallback camera is not the active video camera either.
Video frame1 is D/78km/h; frame5/9 show parking and are not driving references.

## Direct source functions mapped to Python/raylib

- Shared `setLookAtM`/`perspectiveM` -> `Camera`, `camera`, `project_world`.
  Reset target(-.24,3.10,0),distance23,pitch3.60°,FOV20°,aspect864/576 retained.
  Chase yaw clamp±6.5°,curve/.001*4.2; original orbit blend/distance/FOV/zoom
  equations retained. Original864x576 scene uniformly fits a720x480 virtual
  viewport(x612..1332,y0..480) centered inside allowed driving region. Horizontal
  and vertical focal pixels match exactly; no wide-HUD stretch or sprite gain.
- `stepSimulation` -> `advance_motion`: exponential speed/steer14Hz,curve/lead9Hz,
  dt bounded. Curve clamp±.0025,steering fallback−steer*.000025; spatial dash phase
  integrates speed/3.6 and wraps7000. Inputs are renderer state, no demo telemetry.
- `clipAddQuad` -> `_world_quad`: homogeneous near-plane clipping at.85 before
  division. `emitClippedGradientQuad` -> `_gradient_world`: near.35 clipping with
  interpolated alpha/color attributes, then original triangle-fan colors via rlgl.
- `fillRoadBand` -> `_road_band`: original decorative asphalt span−48..44 and
  44..118m,width±8.8m,8/14m steps,dark white alpha16/7. This is visual ground,
  not a claim of detected road edges. It renders even without perceived lanes.
- Observed lane/edge sample adapter retains source solid/dashed metric halfwidths
  .075/.06,colors,44/86/128m fade bands1/.52/.18 and dash3.5m/period7m. A valid
  near0m observation enables source-style decorative rear continuation to−48m.
  Missing/invalid near samples do not invent additional lines or fill gaps.
- `drawFeatheredPath3d` -> `path_strips`: original seven lateral positions,
  lateral alpha/core arrays,source behind fade−24..−2m,quadratic front fade,
 82m no-lead maximum,lead center2.2+compressed-distance−1.6 cutoff,AP/close/critical
  gray/blue/amber/red and alpha58/185. Valid observed path at0m permits decorative
  continuation behind ego to−24m; forward valid samples remain authoritative.
- `drawRoad3d` traffic -> `traffic_pose`, `_car`, `_faceted_car`: +2.2m center,
  >30m distance compression0.7,curve rotation/center displacement,clip-depth sort,
  −2.10m rear anchor,eye-relative elevation/yaw and original sprite selection.
  Original alpha clamp(1.25−depth/115,.35,1),lead/other tint and gain1.08/.92.
  Camera-relative face visibility and original body/cabin vertices/colors render
  source faceted fallback for bearing>25°/elevation>19°/missing sprite.
- `drawNav` -> `_car`: radial shadow fan; original brake threshold accel<−1.5;
  source white-alpha lamp mask tinted(255,38,56),additive passes with6% enlarged
  glow alpha.55 then two normal passes. No unsupported headlamp state fabricated.
- Original `isStationaryTarget` behavior applies only when actual absolute speed
  exists: ego>=20km/h and |target m/s|<1.5,except primary lead path.

## Minimal platform/data adapters and intentional differences

1. **Ego-only substitution requested by user:** existing black K9
   `assets/images/k9_navdy_ego.png` and metadata helpers with original
   `_draw_ego_brake_light` remain. No white Niro asset prerequisite. A photo cannot
   reproduce mesh wheel rotation,physical materials,steering/body roll or dynamic
   ego yaw. Original traffic is unchanged-source imagery/faceted fallback.
2. Existing K9 outer speed,gear,navigation,panels,alerts/background remain owned by
   `_k9l_draw_hud`; mode2 only supplies road callback. Original 1920x720 outer UI
   is not copied. The original3:2 scene is uniformly fitted/letterboxed inside
   the short/wide HUD. Original world-metre sprite sizing is preserved,so neither
   camera nor K9/traffic is stretched or shifted to force composition.
3. During road callback only,the source-aspect viewport gets its matching clip;
   outer full driving clip is restored before top helper. Bottom of the virtual
   scene is its frame edge,not an internal road geometry endpoint. Outer unchanged.
4. Lane/path arrays replace original always-present nominal analytic lanes; this
   preserves measured valid geometry and gaps. Rear continuation is display-only,
   tied to a valid near observation, never returned as sensor data. Curvature
   controls ground/camera/traffic exactly as source. Therefore inconsistent
   synthetic path and curvature inputs can visibly disagree; no silent correction.
5. Temporal state is renderer-local and camera context is reset in finally.
   Same timestamp doesn't advance animation (deterministic multiple render passes);
   inactive gap>.5s resets smoothing to current telemetry,avoiding stale-mode drift.
   Original touch orbit/zoom formulas exist but K9 scene has no bound touch handler:
   default yaw/pitch0/zoom1 used. No new UI or settings were invented.
6. Source headlamp status is absent in current state schema; ordinary taillamp-on
   inference is omitted,while actual acceleration-based traffic brake works.
   Source traffic freshness timestamp is also absent; adapter uses current state
   detections, never mode0's held objects. Original soft Canvas rasterization is
   approximated by32-sector raylib radial gradient; antialiasing differs.
7. Dark driving palette is the current selected HUD source path. Android's entire
   light-theme dashboard/material pipeline is not imported. Existing outer theme
   handling remains untouched. No networking,Android runtime,park showcase copied.

The source mesh files are absent from the pinned archive,but they do not block
road/shared-camera import. No user model request or separate white-SUV requirement.
This is an actual Python/raylib port of active source math,not binary Android reuse
or a pixel-identical clone. Differences above must remain explicit.

## Scope and evidence

Only `cluster_k9_video.py`, `tests/test_k9_hud_video.py`, this document changed in
this correction; inherited dirty files/assets/integration untouched. P/R/modes0/1
preserved. No commit,push,deploy,vehicle access.

Focused tests:
```
PYTHONDONTWRITEBYTECODE=1 /home/ec2-user/k9_research/checker-hud2-env/bin/python -m pytest -p no:cacheprovider -o addopts= --confcutdir=openpilot/selfdrive/carrot/tests openpilot/selfdrive/carrot/tests/test_k9_hud_video.py openpilot/selfdrive/carrot/tests/test_k9_hud_video_integration.py openpilot/selfdrive/carrot/tests/test_k9_hud_navdy_res.py
```
35 tests cover source analytic camera,uniform aspect/x-y pixel metric,curve/orbit formulas,temporal exponential
smoothing,phase,same-time and stale-gap behavior,clip finiteness,gradient attributes,
source asset anchors,traffic compression,rear anchors,faceted fallback,brake passes,
invalid/bounded data,K9 helper delegation and mode integration.

Worker `/home/ec2-user/k9_research/checker-hud2-out/VALIDATION.md` contains final
comparison/benchmark results. `source-demo.py` uses exact original HudState.demo
formulas only in offline explicitly synthetic fixtures(t0,t2),not production.
