# Understanding Settings

[한국어](../ko/settings.md)

> [!NOTE]
> This is the canonical English user guide maintained with the `carrot-wip` code. When user-visible behavior changes, update this document together with the related code and tests.

Use **Carrot Web** to view and change all carrotpilot-specific settings. The device settings screen remains useful for Wi-Fi, device information, standard openpilot toggles, and software updates. Parameters defined by `carrot_settings.json` belong in the **Settings** screen in Carrot Web.

An accelerometer-detected suspected horizontal impact of at least 1.5g displays a warning and a ten-second cancellation notice. Without a touch, it saves `OpenpilotEnabledToggle` OFF and reboots into Dashcam mode, preventing control until manually enabled again. See [detection, cancellation, recovery, and recording interruption](dashcam-log-sharing.md#automatic-dashcam-mode-after-a-suspected-impact).

> [!IMPORTANT]
> **Current support status**
>
> - Supported settings interface: **Carrot Web**
> - CarrotMan: not supported as a user app or external integration
> - CarrotLink: not supported
> - Older CarrotMan or CarrotLink connection instructions no longer describe the current workflow
>
> A `carrotMan` name may still appear inside code or messages. It does not mean that the former user app is supported.

## Connecting to Carrot Web

1. Connect the comma device and your phone or computer to the same network.
2. Open `http://DEVICE-IP:7000` in a browser.
3. Select **Settings** from the top menu.

For example, if the device IP is `192.168.0.25`, open:

    http://192.168.0.25:7000

See [Carrot Web](https://github.com/ajouatom/openpilot/wiki/Guide-Carrot-Web) for connection troubleshooting and an overview of the other screens.

### Web layout defaults

On a fresh install, `Tools > Web Settings > Layout` starts with **Area 1 full screen** in both orientations: Area 1 is **Carrot Vision** and Area 2 is **Carrot Navi**. **Default** restores both orientations to Area 1 full screen with Carrot Vision in Area 1 and Carrot Navi in Area 2. See [Carrot Web layout](carrot-web.md#layout) for the per-screen details.

## Using the Settings screen

Carrot Web provides:

- **Category navigation** through category, group, and section
- **Search** by display title or parameter name
- **Favorites** by pressing and holding an item
- **Profiles** to save and restore named groups of values
- **Compare** to inspect a profile or backup before applying it
- **Reset** to restore the catalog defaults
- **File and QR backups** from the tools screen

A full reset or a backup from another vehicle can change many values at once. Save the current settings as both a profile and a file first.

### Per-setting detailed guides

Select the body of a setting, outside its value control, to open its detail screen. The information box below the existing setting box always shows **Description**, **Popular**, and **History** tabs, with **Description** selected by default. A tab remains available and shows an empty state when it has no data.

- **Description** first uses the setting guide for the current language from the validated GitHub published content.
- Content is shown only when both the published index and guide-page hashes match.
- If the request or validation fails, Carrot Web uses the last validated browser copy.
- If neither a validated remote version nor a validated browser copy exists, it shows an empty state instead of using an on-device document.
- **Popular** shows the collected value distribution as a reference, not as a recommendation.
- **History** shows the latest three changes first and offers the full history when more records exist.
- Network or documentation processing failures do not disable viewing or changing the setting.
- Korean and English Carrot Web sessions use their matching guide language.
- Until a Chinese guide exists, Chinese titles and short descriptions remain Chinese while the detailed panel explicitly falls back to English.
- In the GitHub Wiki sidebar, `User Guide > Understanding Settings > All Settings` follows
  Carrot Web's Korean menu hierarchy and order. The central settings catalog remains the
  multilingual Korean, English, and Chinese directory.

The information box stays vertically below the existing setting box in both portrait and landscape layouts. Multiplier and default actions keep their existing position and behavior inside the setting box.

Validated published guides are generated against the current setting code's defaults, ranges, and options. If a short catalog description and the detailed guide differ, diagnose the issue together with the current branch's setting code.

## Reading values and defaults

Each entry in `carrot_settings.json` contains:

| Field | Meaning |
|---|---|
| `name` | Unique name used by the code and Params |
| `title` | Title shown in Carrot Web |
| `descr` | Direction of adjustment, mode numbers, and cautions |
| `min` / `max` | Allowed range |
| `unit` | Step used by the `+` and `-` controls |
| `default` | Initial catalog value |

### The current value comes first

For a 2026 Staria EV, use `Hyundai Staria EV 2026` in vehicle selection. Existing Staria selections are not migrated automatically; check HDA2 and camera SCC settings separately against the vehicle wiring. See [vehicle selection](carrot-web.md#tools-page).

Persistent values stored on the device may remain after an update. Record the value currently shown by your device before relying on the JSON `default`, a number in a title, or another user's value.

A catalog default is not a recommended tune for every vehicle. Vehicle tuning, existing Params, and the branch from which the device was updated can all affect the starting value.

### Check the scale and unit

| Display example | Stored value | Effective value |
|---|---:|---:|
| `TFollowGap1 x0.01s` | `110` | 1.10 seconds |
| `SteerActuatorDelay` | `30` | 0.30 seconds |
| `LateralTorqueAccelFactor x0.001` | `2500` | 2.500 |
| `StopDistanceCarrot` | `600` | 6.00 m (600 cm) base value |
| Percentage `%` | `105` | 105% |

Ignoring `x0.01`, `x0.001`, `cm`, `km/h`, or `%` can make a value appear one hundred or one thousand times larger than it really is.

## Settings map

The current `carrot_settings.json` contains **189 parameters**. The driver-monitoring exception and `NNFF` are search only; the remaining entries appear in these menus:

| Category | Count | Groups |
|---|---:|---|
| Driving control | 125 | Startup and auto, buttons and presets, steering, speed and deceleration, cruise and following gap, experimental |
| Vehicle and hardware | 17 | Hyundai/Kia, CAN FD/HDA, radar, driver monitoring, vehicle assistance, device hardware |
| Display | 34 | Information, path, brightness/on-road view, external HUD |
| System | 12 | Recording/power, network/map, sound, software |

## Driving control

These 123 settings can affect vehicle motion. Change one item at a time.

<a id="start-auto"></a>
### Startup and auto — 9 settings

| Section | Parameters | Purpose |
|---|---|---|
| Startup | `AlwaysLateral`, `AutoEngage`, `DisableMinSteerSpeed` | Always-on lateral control, automatic engagement, and low-speed steering limits |
| Auto cruise | `AutoCruiseControl`, `AutoGasTokSpeed`, `AutoGasCancelSpeed`, `AutoGasSyncSpeed`, `CruiseOnDist` | Automatic cruise activation and accelerator-pedal behavior |

- `AlwaysLateral` permits lateral control even when cruise is not engaged. On supported Tesla vehicles it can also steer at true standstill in a forward-driving gear; moving below the minimum steering speed remains restricted. See [Tesla engagement](tesla.md#engagement-and-standstill).
- Always Lateral waits for the first confirmation of valid model, vehicle-parameter, pose, and other required inputs after system startup. Once confirmed, this startup check ends; later input interruptions or disengagement/reengagement do not rearm it. Existing runtime safety handling remains unchanged. Expiry of the startup waiting period alone does not pass this check.
- `AutoEngage`: `0` off, `1` lateral on, `2` lateral on with cruise ready.
- `AutoCruiseControl` covers Hyundai/Kia auto-cruise and soft-hold behavior.
- `DisableMinSteerSpeed` is vehicle-specific and relates to low-speed steering restrictions on SMDPS-equipped cars.

### Buttons and presets — 15 settings

Select a section title for the code-based state machine, units, and application conditions.

| Section | Parameters | Purpose |
|---|---|---|
| [Button modes](buttons-presets.md#button-modes) | `CruiseButtonMode`, `CancelButtonMode`, `LfaButtonMode`, `PaddleMode` | Cruise, cancel, LFA, and paddle-button behavior |
| [Speed units](buttons-presets.md#speed-units) | `CruiseSpeedUnit`, `CruiseSpeedUnitBasic`, `CruiseButtonLongDelay` | Short/long-press increments and long-press timing |
| [Button-message tests](buttons-presets.md#button-spam) | `CruiseButtonTest1`, `CruiseButtonTest2`, `CruiseButtonTest3` | Values used to synchronize the stock SCC set speed |
| [Speed presets](buttons-presets.md#speed-presets) | `CruiseSpeed1` through `CruiseSpeed5` | Speed table used by custom mode 3 |

The result depends heavily on whether the car uses stock SCC and which button message the vehicle accepts. Diagnose unexpected behavior with the normal `CruiseButtonMode=0` behavior first.

Volkswagen's separate `SET` button sets current speed and `RES` restores the previous set speed, while `+`/`-` follow the button mode, speed units, and long-press setting. Manual engagement with openpilot longitudinal control remains tied to the physical `SET`/`RES` buttons.

<a id="vehicle-steering"></a>
### Vehicle steering — 37 top-level + 5 ONNX detail + 1 search-only setting

| Section | Parameters | Purpose |
|---|---|---|
| ONNX Lane and BSD | `ShareData`, `OnnxLaneThreshold`, `OnnxLaneIntervalMs`, `OnnxBsdThreshold`, `OnnxBsdSmoothingMs`, `OnnxBsdIntervalMs` | On-device lane-type and camera BSD detection, source display, and tuning |
| Centering | `PathOffset`, `CameraYawTrimDeg` | Path position and camera-yaw trim |
| Steering feel | `SteerActuatorDelay`, `LatSmoothSec`, `LatSuspendAngleDeg`, `CustomSR`, `SteerRatioRate` | Timing, smoothing, suspension angle, and steering ratio |
| [Lane change](lane-change.md) and automatic turn | `LaneChangeNeedTorque`, `LaneChangeDelay`, `LaneChangeBsd`, `LaneLineCheck`, `BlinkerLatchedTurn`, `LaneChangeLeverWait`, `AutoTurnControl`, `AutoTurnControlSpeedTurn`, `AutoTurnControlTurnEnd`, `AutoTurnMapChange` | Lane-change entry conditions and ATC behavior |
| Lane mode | `LatMpcPathCost`, `LatMpcMotionCost`, `LatMpcAccelCost`, `LatMpcJerkCost`, `LatMpcSteeringRateCost`, `LatMpcInputOffset`, `UseLaneLineSpeed`, `UseLaneLineCurveSpeed`, `AdjustLaneOffset` | Lane-mode MPC weights and lane-line conditions |
| Advanced torque | `LateralTorqueCustom`, `LateralTorqueAccelFactor`, `LateralTorqueFriction`, `LateralTorqueKpV`, `LateralTorqueKiV`, `LateralTorqueKf`, `LateralTorqueKd` | Custom torque-control gains |
| Advanced torque, search only | `NNFF` | Neural-network feedforward for torque steering |
| Steering limits | `CustomSteerMax`, `CustomSteerDeltaUp`, `CustomSteerDeltaDown`, `CustomSteerDeltaUpLC`, `CustomSteerDeltaDownLC` | Maximum torque and torque-rate limits |

`ONNX Lane and BSD Detection` (`ShareData`) runs solid/dashed classification and camera BSD on the device. BSD alternates across configured sides whose model-estimated lane width is at least 2 m; speed and lane-change direction do not gate evaluation. The three-class model distinguishes a blind-spot threat from no nearby vehicle and a distant/rear vehicle. Its detail screen keeps the feature toggle at the top and shows the BSD detection-area editor in a separate card directly below it, with only side selection, image refresh, point undo/reset, and area save in the primary view. The basic rectangular point picker at the image's upper left uses `1(L)` and `1(R)` labels; tapping empty space adds a point whenever none is selected. Zoom, viewport panning, and whole-area dragging are omitted so only the selected point moves. Two equal-width buttons directly below the four editing actions open the road- and wide-camera images in pop-up dialogs. Runtime state, confidence, performance, diagnostics, and five persistent tuning values open from the lower-right **Expand/Collapse advanced settings** text and are collapsed by default. The editor retains one last camera frame and shows a dim default road example when no frame is available. An area can be saved only after receiving a real camera image in the current session. It defaults to off, and saved tuning values survive a service restart. The update includes OpenCV, prepared automatically during normal startup. When enabled, the service starts after onroad initialization and model/CAN/Panda readiness have settled for 0.5 seconds. Engaging cruise is not required. It stops offroad and waits again at the next onroad start. See [conditions and detailed values](lane-change.md#sharedata--onnx-lane-and-bsd-detection).

Even when lane mode is enabled through `UseLaneLineSpeed`, control temporarily switches to laneless if the model speed trajectory starts below 70% of measured vehicle speed or ends below 70% of its starting speed. Lane mode can resume after approximately one continuous second of acceptable speeds, provided the existing lane and speed conditions also pass.

A larger `SteerActuatorDelay` compensates by commanding earlier. A larger `LatSmoothSec` is smoother but may respond more slowly. Changing both together makes diagnosis difficult.

The default `SteerRatioRate` of `100%` applies the learned steering ratio without scaling. It is used when `CustomSR=0`; a stored rate outside the allowed range (`30–200%`) safely falls back to `100%`.

Manual steering ratio and learned-ratio scaling also apply to VW MEB, including ID.4. `CustomSR=159` selects a ratio of `15.9` and takes precedence over learned-ratio scaling. `CustomSR=0` with `SteerRatioRate=100%` uses the learned ratio unchanged; the control loop rereads settings while running. Selecting a manual ratio does not stop learning; it selects the value used for control.

Previously ignored ID.4 values now take effect. For example, with `CustomSR=0`, a saved rate of `30%` applies `0.3 times` the learned ratio. Set the rate to `100%` to use the learned value unchanged.

`LateralTorqueCustom` and `CustomSteer*` are advanced settings that can affect the vehicle tune and safety limits. Do not alter them without a vehicle-specific validated baseline and a recovery path.

#### NNFF torque control

`NNFF` is a neural-network feedforward option that defaults to off. Enabling selects torque-controller tuning on vehicles without angle control and uses NNFF when a matching model is available. Vehicles supported by comma's own NNFF retain that path's precedence; angle-control vehicles are unaffected.

Disabling turns off this NNFF option without changing the separate `NNFFLite` value. Changes apply at the next onroad start or after a device reboot, rather than immediately during a drive. An absent startup notice alone does not establish whether NNFF is active.

#### Steering recovery after driver intervention

Hyundai/Kia/Genesis angle-control vehicles use the former combined mode 3 as standard. The mode selector has been removed, and previously saved mode values are ignored. Torque-control vehicles are unaffected.

- Steering-error and driver-force levels and trends permit limited recovery as they converge. Unclear convergence pauses the increase or gently reduces it.
- A rapid force decline after sustained intervention takes priority. Once low force is confirmed, smaller error permits faster torque-ceiling recovery and larger error slows it. The two recovery increases are never added together.
- Strong renewed intervention yields quickly. Target steering angles and existing angle limits remain unchanged.

Force-sensor trends do not establish loss of hand contact or driver consent to handover. Existing `steeringPressed` and driver monitoring remain unchanged. Making this behavior standard does not establish the same steering feel on every vehicle.

### Speed and deceleration — 23 settings

| Section | Parameters | Purpose |
|---|---|---|
| [Speed cameras](speed-deceleration.md#speed-camera) | `AutoNaviSpeedCtrlMode`, `AutoNaviSpeedCtrlEnd`, `AutoNaviRearCameraHoldDistance`, `AutoNaviSpeedDecelRate`, `AutoNaviSpeedSafetyFactor`, `AutoNaviCountDownMode`, `VehicleNaviCanControl`, `VehicleNaviSchoolZoneControl`, `VehicleNaviDecelCancel`(`VehicleNaviDecelCancelBox`, `VehicleNaviDecelCancelMobileZone`, `VehicleNaviDecelCancelBump`, `VehicleNaviDecelCancelUnknown`, `VehicleNaviDecelCancelEarlyWarning`), `VehicleNaviSectionAvgControl`, `VehicleSpeedCameraControlMode`, `VehicleSpeedCameraDistanceTime` | Event types, rear-camera post-pass hold, stock camera distance matching and virtual distance, PV5 warning-confirmed camera retention/cancellation and section speed caps, deceleration start, and target speed |
| [Road speed limit](speed-deceleration.md#road-speed-limit) | `AutoRoadSpeedLimitOffset`, `AutoRoadSpeedAdjust`, `AutoSpeedUptoRoadSpeedLimit` | Desired-speed adjustment from the road limit |
| [Speed bumps](speed-deceleration.md#speed-bump) | `AutoNaviSpeedBumpTime`, `AutoNaviSpeedBumpSpeed`, `AutoNaviSpeedBumpEndDistance` | Completion time, crossing speed, and early-release distance |
| [Curves and turns](speed-deceleration.md#curve-turn) | `AutoCurveSpeedFactor`, `AutoCurveSpeedLowerLimit`, `TurnSpeedControlMode`, `MapTurnSpeedFactor`, `ApplyModelSpeed` | Curve slowing from curvature and distance, prompt recovery after confirmed easing, and route-turn speed |
| [Model stop](speed-deceleration.md#traffic-light) | `ModelStopPrediction`, `TrafficStopDistanceAdjust` | Model stop-prediction slowing, stop-position adjustment, and automatic stopped-vehicle alignment |

`AutoNaviSpeedCtrlMode` is `0` off, `1` fixed speed cameras, `2` cameras plus speed bumps, or `3` those events plus mobile-camera events.

While external navigation is connected, deceleration, countdowns, and navigation speed displays use it exclusively. Stock navigation remains excluded even without an external guidance item and resumes according to its settings after disconnection or receive timeout is detected.

`VehicleSpeedCameraControlMode=2` treats a new accelerator press after vehicle-received camera deceleration has actually begun as a request to ignore the current event. It keeps the highest speed reached while accelerating as the floor until the event ends; an accelerator held from before deceleration began does not start the override.

`AutoNaviSpeedDecelRate` defaults to `120` (1.20 m/s²); a lower value begins slowing farther away. Updating the software does not change an existing saved value. `AutoNaviSpeedSafetyFactor` applies a percentage of the event limit as the target. Before tuning either value, confirm that the event type, limit, and remaining distance are being received correctly.

`ModelStopPrediction` (Model Stop Prediction) is `0` off or `1` on (default). With no lead, it slows and stops when the driving model predicts a stop; the model may stop for traffic lights, stop lines, stop signs or obstacles and is not a traffic-light detector. There is no automatic departure after such a stop: press the accelerator (a lead moving away still departs through normal lead following). The setting replaces the retired `TrafficLightDetectMode`: its saved value is copied once at startup (`0`→`0`, `1`/`2`→`1`) unless `ModelStopPrediction` is already set. Setting `0` only stops using this slowing; it does not remove what the pretrained model has learned about signals. The driver must always verify signals and the road.

### Cruise and following gap — 29 settings overall, 26 on Hyundai/Kia/Genesis

| Section | Parameters | Purpose |
|---|---|---|
| [Driving mode](cruise-gap.md#driving-mode) | `MyDrivingMode`, `MyDrivingModeAuto` | Eco, safe, normal, high-speed modes and automatic selection |
| [Speed-based acceleration](cruise-gap.md#acceleration-table) | `CruiseMaxVals0` through `CruiseMaxVals6` | Maximum acceleration tendency by speed band |
| [Stopping and restarting](cruise-gap.md#stop-resume) | `StopDistanceCarrot`, `StoppingAccel`, `VEgoStopping`, `AChangeCostStarting` | Stop position, stop entry, and restart behavior |
| [Longitudinal tuning](cruise-gap.md#longitudinal-tuning) | `LongTuningKpV`, `LongTuningKiV`, `LongTuningKf`, `LongActuatorDelay` | Hyundai/Kia/Genesis fix/hide `100/0/100`; Toyota/Lexus fix/hide `0/0/100`; VW MEB, including ID.4, applies saved gains |
| [Following gap](cruise-gap.md#following-gap) | `TFollowGap1` through `TFollowGap4`, `DynamicTFollowLC`, `SpeedTFFactor`, `TFollowDecelBoost` | Gap times, lane-change relief using selected leads, and deceleration margin (default 0%) |
| [Following responsiveness](cruise-gap.md#lead-response) | `LeadAccelResponse`, `LeadAccelResponseTF1`–`LeadAccelResponseTF4` | Lead-start, acceleration and approach response at every following-distance level |
| [Carrot cruise](cruise-gap.md#carrot-cruise) | `CruiseEcoControl`, `CruiseCoastingPercent`, `CarrotCruiseDecel`, `CarrotCruiseAtcDecel` | Economy control, coasting margin with a fixed entry reference (default 0%: existing control), and cruise deceleration limits |

`MyDrivingMode` is `1` eco, `2` safe, `3` normal, or `4` high speed. High-speed mode disables model stop prediction and increases acceleration tendency, so read its behavior before selecting it.

Eco caps lead response at 2 and Safe at 3; Normal and High retain the selected value. Caps follow common/gap-specific selection and never raise lower choices or 0. Eco ×1.1 and Safe ×1.2 TF multipliers remain, with gradual release of mode allowance. Automatic selection uses Safe for stopping approaches and sustained slow following. Outside a stopping approach, lead acceleration above 1.0 m/s² for about 0.5 seconds restores Normal/Eco; otherwise, it requires three seconds of flow recovery or four seconds with no lead at 15 km/h or above.

`CruiseGapLevels` (Gap cycle levels) limits button cycling to 2 through the vehicle-supported maximum, which is the default. 2 uses TF1 and TF2; 3 uses TF1 through TF3. It applies on the next gap-button press and preserves unused TF and following responsiveness values. Applies with openpilot longitudinal control.

`TFollowGap1` through `TFollowGap4` are stored in hundredths of a second. Lower values reduce the time gap. Use `LeadAccelResponse` for acceleration response: levels 1–3 are gradual, 4 is quick, and 5 retains maximum response. Braking-onset distance is not separately remembered to enlarge the gap.

`LeadAccelResponse`: Adjusts how the car follows a lead vehicle as it starts or speeds up. Lower levels close the gap more gradually; higher levels follow more quickly. Level 0 turns off the acceleration boost, and level 5 is the most responsive test setting. See [Following responsiveness](cruise-gap.md#lead-response) for details. Extra-headroom hold conditions apply up to 1.2 times TF distance (speed × TF + stopping distance), ease progressively from 1.2 to 1.5 times, and no longer pause recovery at or above 1.5 times regardless of lead speed.

`SpeedTFFactor` applies a linear speed multiplier to the selected base TF: 10 is unchanged; 20 doubles it at 100 km/h. `LeadAccelResponseTF1`–`TF4` use the common response at -1 and a gap-specific response at 0–5. Levels 4–5 retain speed TF. The driving-screen bar shows the dynamically adjusted following target in metres.


Deceleration preview operates independently of the response level. During active control, remaining correction releases progressively when relative acceleration eases or the lead switches between radar and vision or disappears. Accelerator or brake intervention and longitudinal control exit clear it immediately.

`LongTuning*` and `LongActuatorDelay` are advanced settings that directly affect vehicles using openpilot longitudinal control. Hyundai, Kia, and Genesis fix `LongTuningKpV`, `LongTuningKiV`, and `LongTuningKf` at the safe `100/0/100` values and hide them from settings. Toyota/Lexus fix and hide common gains at `0/0/100` to avoid duplicate correction before their dedicated controller. Some parameters have no effect when stock ACC remains responsible for acceleration and braking.

`StoppingAccel` is adjustable again: default `-50`, range `-100 to -50`, step `10`. The stored value is multiplied by 0.01, with the same bounds enforced by control. Original stop-entry and braking behavior and vehicle-specific soft hold are restored; changes apply within about one second. See [Stopping and restarting](cruise-gap.md#stop-resume).

`VEgoStopping` has a range of `10–100` and a default of `50`; `10` means `0.10 m/s`. Previously stored values below `10` are automatically raised to `10`, and control enforces the same minimum during driving.

For Hyundai/Kia CANFD openpilot longitudinal control, aReqRaw and aReqValue become zero in the first StopReq frame, so this setting does not change transmitted acceleration while StopReq remains active. Automatic release/retry is removed.

On supported Tesla vehicles with the additional vehicle bus detected, the device's **alpha longitudinal** (`AlphaLongitudinalEnabled`) toggle also enables [automatic cruise set-speed adjustment](tesla.md#automatic-cruise-speed) to the vehicle-reported limit. Navigation/Carrot speed control takes precedence and pauses DAS-based adjustment until it ends and the limit is stable again; any manual pause remains in effect. Turning the right speed wheel pauses it; an opposite-direction wheel gesture within one second or disengaging and re-engaging resumes it. There is no separate Carrot Web setting for this feature.

<a id="experimental"></a>
### Experimental — 5 settings

Carrot Web path: **Driving control > Experimental** (Korean UI: **주행 제어 > 실험중**; internal group ID `EXPERIMENTAL`). Trial features whose effect and safety are not validated on a vehicle. Read the description and change one item at a time.

| Subsection | Parameters | Purpose |
|---|---|---|
| K9 stop method selector | `K9StopMode` | One-click single choice of 0, 1, 2 or 3 in the settings screen; methods never stack. The web settings screen shows only a short text; the details are here (web labels: 1 v10 (step braking), 2 v10 + light shudder fix, 3 v11). Kia K9 non-CAN-FD camera-SCC openpilot longitudinal only; other cars always behave as 0. 0: base braking (default). 1: v10 — a faithful port of creep_fix v10 from the same page (old-CAN long-control patch v10). Same structure as v9 (an independent re-implementation of creep_fix v9 from the same page: an independent re-implementation of creep_fix v9 published for a Niro EV (https://leegrt.org/oppo/creep_fix/). In the stopping state StopReq is withheld and the decel request is rate-limited toward a speed-scheduled target until real standstill, planned decel <= -1.0 m/s², weak decel, aEgo > 0.06 or a lead within 3 m; then StopReq is sent with at least -0.50 m/s². Low-speed pid braking gets a little extra decel; only on frames where it actually applies, the jerk upper limit is raised below 1.4 m/s and ComfortBand (how far the ESC may deviate from the request) is sent as 0. StopReq timing, stop distance and feel change.), but the stop_feather target is -0.26..-0.52 m/s² over 0.05..0.42 m/s (less braking below 0.25 m/s just before the wheels stop, weaker than v9), min_decel and the release-step speed condition (0.12 m/s) follow the page, and after the StopReq latch the decel request is pressed to -0.50 m/s² by 0.0035 per frame (0.35 m/s³; at once when the planned decel is <= -1.0 m/s²). The 3 m lead latch and the original command on disengage/override are also as in v9. It brakes less than v9 in places and is not checked on the vehicle yet. 2: v10 + light shudder fix — StopReq timing, fine stepping, the 0.35 m/s³ latch ramp, the 3 m lead latch and the jerk limit are those of 1 (v10); the decel request differs in one point (the driver's order 2026-10-24 20:26 "떨림구간 브레이크 아예 안떼게"): before the StopReq latch in the stopping state, and in pid below 3.0 m/s while the plan decelerates (aTarget below -0.05) and the command brakes, the decel request is never weaker than -0.26 m/s² (the end of 1's feather), so the brake is not released toward 0 just before the stop. Where 1 already sends -0.26 or stronger it is bit-identical to 1. Basis (log estimate): ESP12 CYL_PRES is the driver pedal pressure on the K9 and reads 0 under cruise braking, so TCS13 BrakeLight was used as the hydraulic-brake sign; below 3 m/s while decelerating, BrakeLight was on 96 % with no off/on at aReq -0.30..-0.26, on 88 % with 0.48 off/on per second at -0.26..-0.20, and on only 7 % (19.8 s, mostly released) at -0.20..-0.10. On the frames where 1 sends ComfortBand (how far the ESC may deviate from the request) as 0, upper and lower ComfortBand are sent as 0.60 / 0.50 m/s² at or below 2.5 m/s (9 km/h), as 0 like 1 at or above 3.0 m/s (about 11 km/h), and reduced in between. The value sits exactly on the 0.02 CAN grid and moves by at most 0.04 per step, so it never jumps. In the same 20 logged stops the shake below 2 m/s with band 0.5-0.7 (0.024-0.033 m/s²) was the same as with 0.7-0.9 (0.025-0.037), and above 2.5 m/s the band did not reduce the shake. So it aims at the shake reduction of the removed full-band old 4 with a smaller band (estimate). If the band changes the real deceleration, it stops closer to where 1 does (closed-loop sim, stop distance vs 1 median -0.18 / +0.10 m, the old 4: -0.26 / +0.13 m). The shake reduction is an estimate from logs; checked only by log replay and simulation. 3: v11 — the same page's creep_fix v11 (old-CAN long patch v11, written for the Niro EV) ported on top of 1 (v10). Taken from the page code as is: StopReq on a real stop (TCS13 StandStill, speed below 0.05 m/s) only after 12 frames (0.12 s) in a row (planned decel -1.0 m/s² or less, too little decel, aEgo>0.06 or a lead within 3 m still latch at once); feather floor -0.26 -> -0.28 m/s², too-little-decel line -0.02 -> -0.04; on a late entry with the request weaker than the target it presses 0.005 more per frame; release step below 0.12 m/s 0.0030 (0.0032); after StopReq it presses toward -0.50 m/s² at 0.0025 per frame (0.25 m/s³; v10 0.0035). Below 0.95 m/s, while already decelerating harder than -0.18 m/s², the request is not lowered further (not with a planned decel of -0.80 m/s² or less or a lead within 3 m). The jerk limit is raised only below 0.65 m/s (0.7 at 0.05 m/s, 1.8 at 0.14-0.45 m/s, 0.8 at 0.65 m/s). ComfortBand is 0 as in 1. Not taken on the K9: the page's smaller creep compensation below 1.5 m/s (-0.26 to 0) and its PID over-braking blend (up to 55%) are for the Niro EV regen -> hydraulic hand-over and both brake less; in the K9 closed-loop sim (20 stops x 72 conditions) they made the stop distance vs 1 +0.08 m median (max +0.40 m) longer and the min lead gap -0.08 m median (min -0.49 m) smaller, so the low-speed pid decel stays as in 1 (v10) (then distance median 0, lead gap min -0.01 m). The 3 m lead latch and the original command on disengage, override, pedal or invalid CAN are as in 1. The page's improvement claims are Niro EV logs, not K9 results. Checked only by log replay and simulation. Shared hard-brake ease-off (all values 0-3, 2026-10-24 evening): in pid above 3.0 m/s, from the moment a closing lead (leadRelSpeed below 0) brings the decel request to -1.0 m/s² or stronger until it is above -0.5, SCC14 JerkUpperLimit (the rate at which the ESC may release the brake) is kept at its floor 0.5 only while the planner eases off slowly (planner jerk 0..1.0 m/s³). The decel request, StopReq, JerkLowerLimit and ComfortBand are unchanged; a fast planned release (jerk 1.0 or more) or an opening lead keeps the base limit. Basis (log estimate): in the hard-brake clusters the release-then-regrab cycles (>= 0.25 m/s²) were 0.87 per 10 s of hard braking with ComfortBand 0 (the 1/2/3 family) against 0.23 with the base band, and when the planner eased off with JerkUpperLimit raised to 1.0-2.1 the measured release was about 1.9 times the command release (about 0.6-1.0 times with 0.5). Any other value (4 or more, negative) acts as 0. Pedal, override, disengage or invalid CAN return the original at once. Changes apply within about 0.5 s and start fresh. 2026-10-24 evening menu trim (the driver's order "브레이크 2.3.4 제거"): the old 2 (v9 + shudder fix), 3 (v10 + band 0.2) and 4 (v10 + shudder fix) were removed with their code and the old 5 and 6 moved to 2 and 3 (bit-identical behavior). Stored values are migrated once at the first start after the update (marker K9StopModeVer): old 5 -> 2, 6 -> 3, the removed 2, 3 and 4 -> 1 (v10), 0 and 1 unchanged, any other value is left as is (it acted as 0 before and still does). The stored value of the old 0-6 K9CreepMode menu is not carried over; this starts at 0. 1, 2 and 3 have not been checked on the vehicle yet. EV-origin values; effect, comfort and safety are not verified on the K9. |
| K9 lane-mode drift fix selector | `K9LaneFix` | Single choice of one number in the settings screen. Only lane mode (following the lane-line path) changes. `0` (default) off: same as before. `1` F1: (1) the lane-mode path starts at the car (model path start) and merges into the lane-line path over 2 s; a large initial offset extends the merge up to 4 s, but only as far as the merge curvature stays small, so at high speed it stays 2 s; when lane mode just turns on the correction is ramped in over 0.5 s. (2) From the moment lane mode turns off and the model path takes over, the steering command change is limited to 2.5 m/s³ lateral jerk (below the existing 5 m/s³ safety limit), released as soon as the normal change fits, at most 2 s. Not used during a lane change or while steering is off. Curvature/jerk/accel safety limits, the laneless path, lane-mode on/off conditions and stop functions are unchanged. Changes apply at once (within 0.05 s) and reset the transition state. Checked only by log replay and simulation; not vehicle-validated |
| K9 lane-line post-processing selector | `K9LRView` | Single choice of one number in the settings screen. `0` (default) off: the raw model lane-line output is used, identical to before. `1` on (lane-line tracker): modeld smooths the four lane lines per point with a Kalman filter driven by the model std. Before each update the previous lane lines are moved by the car's own speed and yaw rate taken from this frame's model output (ego-motion prediction), which reduces the lag on curve entry and bending lanes. Near points are smoothed more; far points, high yaw rate and large disagreements less; lane-line probability is averaged over about 0.15 s. A lane line that jumps more than 1 m in one frame (lane change etc.) is reset to the raw model value at once. Lane height, road edges, the model path and safety limits are unchanged; no extra GPU work and negligible CPU. Everything that reads lane lines (lane mode, lane-change logic, UI) sees the smoothed lines. In the video-replay validation set flicker dropped by about 61% vs raw; remaining: curve-entry lag about 1.3 frames (about 0.06 s), probability changes about 0.1-0.15 s later, and about 0.03 s extra lag on a lane line under the car. Applied within 5 s; switching off and on again restarts the filter. Checked only by low-resolution recorded-video replay simulation and device-log replay; not vehicle-validated. |
| External HUD screen select | `K9ExtHud` | Single choice of -1, 0 or 1. `-1` (default) not used: identical to before; `ClusterHud` (External HUD Display) and the external HUD settings decide as they do now. `0` existing external HUD screen: starts the same HUD as `ClusterHud`=1 from this menu; mirror, brightness, rotation, encoder, theme, screen mode, panel layout, radar and the other external HUD settings still apply. `1` taxi meter screen: the left driving screen is the same as 0 and the right information panel shows the Bonobono taxi meter, in the C1 look (the clam turns 1:1 with the steering angle; the carrot wheel icon is not drawn on this screen), computed from real distance and time (Gyeonggi regular (mid-size) taxi fare, Gyeonggi notice 2023-5572: 4,800 KRW for 1.6 km, 100 KRW per 131 m, 100 KRW per 30 s, distance and time counted together (time at or below 15 km/h), night 23-04 +30%, shown in 10 KRW steps; the 20% out-of-area surcharge is not applied); the passenger-seat occupant flag (CAN 0x500 bus0 bits 2-3) on for 5 s while stopped = passenger aboard, counted from the base fare once moving; the passenger getting out no longer switches to the pay screen. It is run from a separate Android app (택시미터 리모컨, for the Carlinkit box or a phone) with three big buttons: [시작 Start] reset and count from 0 (fare, distance, time), [일시정지/재개 Pause/Resume] stop counting (press again to go on), [정산 Settle] stop and show the separate settlement screen (total, base / distance / time / night surcharge lines, distance and time); [Start] there begins a new ride. The app finds the comma by a UDP 7720 broadcast on the same hotspot/LAN (or a typed address, e.g. a Tailscale IP); carrot_man opens 7720 only while this value is 1, hands presses to the HUD in the memory param `K9MeterCmd` and answers with `K9MeterState` (state and fare). After a reboot the meter starts empty. The badge under Bonobono shows only the lane icon (no LFA text) and the clam keeps its own colours whatever LFA or steering do. Season background: `K9MeterSeason`. Switching to `0` and back to `1` resets the meter (fare, distance and time to 0, base fare). Display only. No conflict: choosing `0`/`1` sets `ClusterHud` to `0`; switching `ClusterHud` to `1` returns this to `-1` (checked by the web save and by the HUD program). Only one HUD program runs; 0<->1 switches without a HUD restart within about 1 s. The meter design is the images in `cluster/assets/taxi_meter/`. Not vehicle-validated. |
| Taxi meter season background | `K9MeterSeason` | Sub-option used only when `K9ExtHud`=1 (taxi meter screen): picks the season of the meter background. `0` (default) auto: follows the month (Mar-May spring, Jun-Aug summer, Sep-Nov autumn, Dec-Feb winter). `1` spring, `2` summer, `3` autumn, `4` winter. Day (05:00), dusk (16:00) and night (19:00) still follow the clock for every value. Out-of-range values act as `0`. The HUD only reads this value; changes apply within about 1 s. No effect on the fare. Not vehicle-validated. |
| One-lane-line gap hold | `K9OneLaneHold` | Single choice of one number in the settings screen. `0` (default) off: identical to before. `1` on: Works only in laneless (the car follows the model path). While both lane lines are clear it measures the gap car left edge <-> left line and car right edge <-> right line (K9 width 1.915 m, filtered) and remembers the centre target gap (lane width / 2 - car width / 2) and the minimum gap (smaller of the two, at least 0.3 m). When only one line is visible afterwards, it adds a small steering curvature so the gap to that line returns to the remembered centre gap, and pushes harder when closer than the minimum gap. Two lines or none: nothing is added (laneless as before). Lane change, blinker, driver steering, turns (large curvature), low speed (below 29 km/h), a lane-width jump or width change ahead (merge, split), or a visible line that jumps, jitters or diverges from the model path: back to normal over 1 s. The memory is dropped after 15 s or 300 m without both lines, and at once when the visible line is more than 0.5 m off the remembered gap or keeps moving away for over 1 s while the hold already pushes (widening lane, merge, split). It also drops the memory at once when the visible line moves by itself, i.e. not explained by the car's own motion (previous frame's line moved along the model path), by more than 0.12 m summed over about 1.5 s (merge or branch line), and the push target then drops to 0 at once (the output falls at the jerk limit); the push already fades once that motion exceeds 0.03 m, and nothing is pushed in the first 0.6 s after only one line remains (a line dropout of 0.3 s or less does not restart that wait). While the model path itself turns quickly (lateral jerk above 0.1-0.3 m/s^3) the hold adds no jerk in the same direction (a push cut by the watch still falls). The damping (line rate) only takes back the lateral speed the hold itself added, so a jumpy line estimate never makes it push away from the centre target. The added amount is at most 0.25 m/s^2 lateral (0.6 inside the minimum gap), changes at most 0.15 m/s^3, with a 1 s ramp in/out; while ramping out the push only shrinks and never changes direction. A visible line with probability below 0.9 pushes proportionally less (0 at 0.45). Not used in lane mode. Checked only by device-log replay and closed-loop simulation; not vehicle-validated. |

<a id="vehicle-hardware"></a>
## Vehicle and hardware

These settings describe the car, harness, and device hardware configuration. Do not enable them merely as a display experiment.

| Group | Parameters | Purpose |
|---|---|---|
| Hyundai/Kia | `HyundaiCameraSCC`, `IsLdwsCar`, `HapticFeedbackWhenSpeedCamera` | SCC connection, LDWS behavior, and speed-event haptics |
| CAN FD/HDA | `CanfdHDA2`, `HyundaiCanfdClusterDirectTx`, `CanfdDebug`, `HDPuse` | HDA2 selection, cluster direct send, CAN FD diagnostics, and HDP |
| Radar | `EnableRadarTracks`, `RadarTrackFlip`, `EnableCornerRadar`, `CarrotRadarMode`, `CarrotRadarCutInSensitivity` | SCC radar, front-track orientation, corner radar, and Carrot Radar processing and cut-in sensitivity |
| Driver monitoring | `DriverMonitoringEnabled` (search only), `DriverMonitoringMode`, `CarrotVisionEnabled`, `MuteDoor`, `MuteSeatbelt` | Driver monitoring and selected vehicle alerts |
| Vehicle assistance | `MaxAngleFrames`, `SpeedFromPCM` | Steering-angle frames and stock-SCC speed control |
| Device hardware | `HardwareC3xLite` | Speakerless C3X Lite audio and process configuration |

> [!CAUTION]
> Incorrect `HyundaiCameraSCC`, `CanfdHDA2`, `EnableRadarTracks`, `CarrotRadarMode`, `CarrotRadarCutInSensitivity`, or `SpeedFromPCM` values can change vehicle identification, SCC, radar, or longitudinal behavior. Confirm the vehicle, model year, HDA generation, harness location, and whether stock ACC is retained.

`DriverMonitoringEnabled` defaults to ON and is available only through Carrot Web setting search. Search for the parameter name to find it. The OFF option is intended for an absent or failed DM camera; leaving the default ON unchanged is recommended. Turning it off stops normal-driving driver-monitoring alerts, monitoring-triggered force deceleration, and lockout, and may violate applicable laws or driving requirements depending on where and how the vehicle is used. Driver View keeps face preview with neutral enforcement. A Web OFF persists across drives and restarts until you manually turn the setting back ON. File and QR backups, restores and setting profiles exclude this value, so a backup from another device cannot turn monitoring off.

Regardless of gear or speed, including at standstill, three distinct physical CANCEL presses, each separated by a release, within three seconds turn monitoring off for the current ignition session without changing `DriverMonitoringEnabled`. Gear and speed changes do not reset the count. Automatic-control CANCEL echoes and BT CANCEL do not count. Any received non-CANCEL vehicle-button event, whether a press or release, resets the sequence. Invalid or stale state, an input-stream gap, or timeout also resets it; an indistinguishable stock-ACC speed-button echo may reset progress too. The next ignition-on or manager/device restart clears only the temporary off state, so monitoring resumes only if the saved setting is ON. `DriverMonitoringMode` and `CarrotVisionEnabled` remain unchanged.

`DriverMonitoringMode` applies while driver monitoring is on. It defaults to 0: stock comma camera monitoring criteria, or 15/30/45-second interaction monitoring when the camera is absent or failed. Mode 1 is for controlled experiments, with empty-road timing extensions and an interaction grace before camera warnings. A shared exception resets the usage restriction after one continuous second of valid Park, standstill and disengaged status. Mode changes apply live at roughly half-second intervals without rebooting. Switching preserves accumulated monitoring time, warning counts and lockout, and ends the previous interaction grace and forward-attention streak. See [driver monitoring and experimental-use conditions](driver-monitoring.md). `CarrotVisionEnabled` controls web road video independently. `DisableDM` remains migration-only; only its old value 2 video function is migrated once to `CarrotVisionEnabled`. Validated original `STEER_TOUCH_2AF` input on Hyundai/Kia/Genesis CAN-FD is supported without a vehicle-name whitelist, with held-contact and new-contact behavior depending on camera availability and mode.

See [Radar tracks and corner radar](radar.md) before changing radar modes.

**Cluster CAN Direct Send** (`HyundaiCanfdClusterDirectTx`) compares the original direct-send method on Hyundai/Kia/Genesis CAN-FD CameraSCC vehicles with intermittent cluster warnings. It defaults to OFF for every vehicle; enable it manually on an affected vehicle. Reboot the device after changing it. Updated Panda firmware supporting this setting is required. Resolution of vehicle warnings has not been validated.

With `HyundaiCameraSCC=0` and no camera-SCC configuration already applied to the vehicle, receiving `SCC_CONTROL` (CAN-FD) or `SCC12` (classic CAN) on the camera bus during the current onroad session adds **Enable CameraSCC** to the CAN error alert. This is bus2, or the corresponding camera bus with multiple Pandas. The hint uses reception history and does not establish the cause of every CAN error. It never changes the setting automatically; existing disengagement and engagement blocking remain active. Select the mode appropriate for the vehicle and wiring while stopped, then verify it in the next onroad session.

`SpeedFromPCM` defaults to `2` (curve/camera deceleration) and affects PCM speed selection and button spamming. With mode `3`, Toyota/Lexus use stock set speed without changing longitudinal-control mode. For `TOYOTA_RAV4_TSS2_2022` using PCM with direct longitudinal control, mode `0` also applies valid stock set-speed changes and cruise resume while preserving lower deceleration targets. See [button transmission details](buttons-presets.md#button-spam).

For dPath RadarD, `EnableRadarTracks=-2` is the vision-only experiment; `-1` always uses SCC without vision matching; `0` also always uses SCC; `1` matches front radar without SCC; `2` matches front radar plus low-speed SCC; and `3` uses SCC unconditionally after front-radar/vision matching fails. Matching modes use central vision at probability `0.40` or higher when matching fails. Modes `-1`, `0`, and `3` use vision only when SCC is absent. All SCC association ignores lateral position. SCC-only modes `-1` and `0` apply no additional dPath gate to vision fallback. Legacy Mando radar variants with 32 or 64 slots are handled automatically. A new stationary front lead requires vision or a matching corner detection; continuous front-radar observation alone cannot authorize it. When a separate measured moving target agrees with the visual position and speed, that vision cannot authorize or retain a different stationary reflection. Corner corroboration must match the selected stationary object itself. For a front candidate without corresponding corner corroboration, a vision-support interruption beyond the permitted brief hold resets both the pending object and its confirmation time before confirmation starts again. An already selected moving front can remain L1 within a bounded vision-uncertainty range while the same measured track stays physically continuous; a fixed 8 m difference alone no longer discards it. A new nearer match can replace it immediately, and gaps or physical jumps reset this allowance. A distant stopped front can qualify on a gentle curve through continuous measured radar history and repeated visual position agreement. Continuous corner position/speed evidence participates in that decision. Confirmed stationary fronts can bridge visual-range noise within distance and time limits while publishing radar distance and speed.

`RadarTrackFlip` offers `0: Normal` (default) and `1: Invert left/right`. It inverts both lateral position and lateral velocity of front radar tracks for lead selection and display, preserving SCC, corner radar and vision. There is no automatic inversion by vehicle model; use it only after confirming a left/right mismatch. Restart the vehicle or reboot the device to apply it at the next OnRoad start. See [radar left/right inversion](radar.md#radar-track-flip).

In versions supporting Bosch MRRevo14F reception, detecting the Bosch message group on a Bosch MRRevo14F candidate platform (vehicle-validated only on the Elantra AD) with `EnableRadarTracks=1`–`3` enables passive reception of tracks already being transmitted, skipping startup radar-activation UDS writes and SCC ECU communication disabling. `EnableRadarTracksResult` is not updated; check actual reception, CAN validity and radar errors. Passive reception does not mean all vehicle control is disabled. See the [Bosch passive reception exception](radar.md#bosch-passive-reception).

In modes `EnableRadarTracks=1`–`3`, a confirmed departing front lead can receive bounded future headway relief in ACC when outward radar motion and a visual switch to a farther vehicle agree. See [predicted CUT-OUT behavior](radar.md).

`CarrotRadarMode` continuously tracks vehicles with the front and corner radars to detect cut-ins, then matches camera and radar information in a new way to select the vehicle ahead. On vehicles with neither corner-radar nor radar-track support, it behaves the same as the existing mode. It can change acceleration and braking, so enable it only on the same vehicle after completing validation. The value is latched when the next OnRoad session starts, so end the current drive and restart the vehicle or reboot the device after changing it. The previous `RadarMotionMode` value is migrated to the new name once on the first startup after updating.

`CarrotRadarCutInSensitivity` controls only Carrot Radar Mode CUT-IN detection: `0` disables it, `1` is insensitive, `3` is normal (default), and `5` is very sensitive; `2` and `4` are the intermediate levels. Levels `1` through `5` require `0.50`, `0.40`, `0.35`, `0.25`, and `0.20 s` of continuing measured motion evidence, while the physical future prediction remains fixed at 5.0 seconds. A front-radar track with at least 0.50 m of strongly one-way progress in its recent measured history may receive at most one 20 Hz radar-frame credit so timestamp quantization does not discard a completed dwell; small adjacent drift does not. It does not affect conventional radar mode or `EnableCornerRadar`. The value is read at the next OnRoad start, so restart the vehicle or reboot the device after changing it.

`HardwareC3xLite` must remain off on standard C3 and C3X hardware. Enable it only on a C3X Lite, then reboot the device. The setting skips the unavailable amplifier so startup is not delayed by I2C retries, uses the GPIO buzzer for alerts, disables `micd`, `soundd`, and `loggerd`, and turns off `RecordAudio`. Normal route logging is unavailable while this hardware mode is enabled.

<a id="display"></a>
## Display

Display contains 34 settings. External-HUD settings control the layout and output of separate display hardware.

`ClusterHud` (External HUD Display) switches a **direct USB HUD** on or off. A HUD connected to Jetson starts automatically even when this value is `0`, and normally turns its display off with ignition. Use the existing always-on `ClusterHudDebug` modes to keep it visible offroad. Brightness and layout settings also apply to the Jetson HUD.

ESP32-S3 Wi-Fi display support has been removed. Connect the USB display to Jetson. On carrot-nav, comma sends HUD and navigation data over the dedicated Wi-Fi connection, without falling back to USB display transport. The comma-to-Jetson USB link carries model inference and connection provisioning. Both ends require the updated software.

The [NAS installer and setup guide](https://upload.shind0.synology.me/models/carrot-s3/index.html) provides the N16R8 board checklist, wiring, browser firmware installation, phone/C4 registration and firmware ZIP. Update an existing S3 once to 1.2.0 through the installer to enable H.264. Wi-Fi/C4 registration, shutdown retries and LED status are preserved.


The Jetson HUD shows the highest internal sensor temperature below `jetSON`, or in the upper-right corner in full-screen navigation/graph modes. If temperature data has not refreshed for three seconds, it shows `--°C`. A top-strip temperature warning starts 5°C below each sensor's configured thermal limit; reaching that limit shows an overheating warning. For example, a device configured to throttle at 99°C warns from 94°C. Check the fan and ventilation when warned. Vehicle driving alerts take precedence, and Jetson temperature remains separate from device temperature/memory statistics. This feature requires updated Jetson HUD software.

With `CarrotVisionEnabled` on, the external HUD and web camera view can be used together. Simultaneous video use may increase device load. See the [Carrot Web guide](carrot-web.md).

| Group | Parameters | Purpose |
|---|---|---|
| Information | `ShowDebugUI`, `ShowTpms`, `ShowDateTime`, `ShowPathEnd`, `ShowDeviceState`, `ShowLaneInfo`, `ShowRadarInfo`, `ShowRouteInfo`, `ShowPlotMode` | Debug, tire, time, lane, radar, and route information |
| Path | `ShowPathMode`, `ShowPathColor`, `ShowPathColorCruiseOff`, `ShowPathModeLane`, `ShowPathColorLane` | Path shape and color by driving state |
| Brightness/on-road view | `ShowCustomBrightness`, `ShowModelView`, `ShowCameraWithCluster` | Brightness, camera/model composition, and the on-device camera while the external HUD is connected |
| External HUD | `ClusterHud`, `ClusterHudBrightness`, `ClusterHudOrientation`, and related `ClusterHud*` settings | Supported TURZX HUD layout, live brightness, screen rotation, camera, radar, and encoder options |

For RAV4 2022 with the ARS display stream, `ShowRadarInfo=3` on the standard device
driving screen also shows adjacent-lane objects separately from control leads. Modes 1/2
and other receivers retain their behavior. A marker does not mean braking-target selection
or Toyota corner-radar support. See the [display conditions](radar.md).

USB external HUD output is fixed at **10 FPS**; while the eGPU is active, rendering, encoding, and USB output switch to **5 FPS**. Changes to eGPU activity apply automatically; changing the H.264 encoder rate restarts the HUD. `ClusterNaviMapFps` remains a separate setting for Android map input.

Onroad, the main UI uses CPU 6 and the external HUD uses CPU 7 with low-priority normal scheduling (nice 19). Offroad, both return to CPUs 0–3 so CPUs 4–7 can enter power save; always-on debug output follows the same rule. FPS selection, CPU selection, and realtime priority settings have been removed, and previously stored values are ignored. Actual refresh rates may fall below the output cap under load.

`ShowPlotMode` selects an on-road diagnostic graph; `0` turns it off. Modes `4` and `5` both use the primary lead vehicle (`radarState.leadOne`), and the mici device display also updates these graphs when the lead message values change.

| Color | `4`: Lead acceleration/relative speed | `5`: Lead acceleration/jerk |
|---|---|---|
| Yellow | First planned ego acceleration `longitudinalPlan.accels[0]` (m/s²) | Actual ego acceleration `carState.aEgo` (m/s²) |
| Green | Estimated lead acceleration `leadOne.aLeadK` (m/s²) | Lead acceleration `leadOne.aLead` (m/s²) |
| Orange | Relative lead speed `leadOne.vRel` (m/s) | Rate of change of lead acceleration `leadOne.jLead` (m/s³) |

Negative relative speed means the gap is closing; positive relative speed means it is opening. Lead values display as zero when no lead is detected, and this setting changes only the diagnostic display, with no effect on vehicle control.

An APN label remaining in the `ShowRouteInfo` description refers to route-input state. It is not an indication that CarrotMan or CarrotLink is supported.

`ShowCameraWithCluster=0` keeps the existing default: while the external HUD is connected, the on-device camera is hidden. Set it to `1` to show the on-device camera video.

`ShowModelView` (Driving Screen Layout) selects the C4 view: `0` camera + lanes/path, `1` camera only, `2` lanes/path on a black background, or `3` hide both. Lane/path graphics include lead-vehicle markers. It applies at every brightness setting, including automatic and 100%, with no additional wait after entering driving mode. Changes while driving appear within about five seconds without rebooting. Speed, alerts, and driver status stay visible; camera capture, driving decisions, and vehicle control are unchanged. This setting does not apply to C3/C3X. While an external HUD is connected, `ShowCameraWithCluster=0` hides both the on-device camera and lane/path graphics regardless of this choice.

`ClusterHudBrightness=0` follows camera exposure automatically; values `1` through `100` select fixed brightness. `ClusterHudOrientation` supports only `0` (0 degrees) and `2` (180 degrees); values `1` and `3` are ignored. The running TURZX process checks both stored settings every 100 ms. Brightness applies live; a managed H.264 orientation change automatically restarts the HUD and applies it through the capture-compatible stream setup.

`ClusterHudPanelLayout=0` places the driving view selected by `ClusterHudCameraViewMode` on the left and the information panel selected by the screen, debug, and navigation state on the right. `1` swaps them, placing information on the left and the driving view on the right. The running HUD applies the setting within about one second without a restart. Modes without two side regions, such as the full-screen graph and full-screen navigation, are unchanged. `ClusterHudDebug` forces always-on output and optional debug UI or navigation input; any resulting debug or navigation information panel follows the selected layout.

While the external HUD is connected over USB, `ShowCameraWithCluster=0` switches the on-device driving view on both regular C3/C3X hardware and mici to a black background and skips camera-video and model-path rendering. Setting it to `1` removes that connection-specific suppression and uses the normal on-device camera and on-road rendering path. A live change takes effect within about five seconds. Speed, speed limit, driver state, alerts, and the driving-state border remain visible with either value, and the option has no display effect after the external HUD disconnects. With value `0`, `camerad` and model input continue running; only duplicate rendering on the device display is reduced.

The final `ClusterHudScreenMode` layout is:

- `-1` uses the full width only in 3D camera views `0` and `1`, with no information panel or world shift. Left-side HUD items retain their margins, right-side gauges and TPMS align to the physical right edge, and the clock, world, and turn signals use the full-display center axis. In road-camera view `2`, it behaves exactly like mode `0`, including automatic navigation/report selection and `ClusterHudPanelLayout`.
- `0` is the default screen. It shows navigation while live navigation is received and automatically shows the driving report otherwise.
- `1` is the general live-debug screen, grouping delay, torque, steering, and lateral-plan state into four 2×2 cards. In 3D camera views, the world and driving HUD use the same 1124 px driving region as mode `0`, while the four cards fill the opposite 792 px information region. The acceleration, steering, fuel, and DEF gauges plus TPMS remain inside the driving region's right edge.
- `2` is the fixed system-debug screen. Its detail card shows the network address, output resolution/frame rate, camera state, memory capacity, and per-core CPU use. Its system-health card adds 2×2 circular gauges for CPU load, temperature, memory, and disk use, plus the device pitch/yaw target and numeric angles. In 3D camera views, the world and driving HUD use the same 1124 px driving region as mode `0`, while both cards use the opposite 792 px information region. It remains fixed regardless of connected, live, disconnected, or debug navigation state and never switches automatically to navigation, `NAVI DISCONNECTED`, the route overlay, or the driving report.
- `3` disables the driving scene and shows the `ShowPlotMode` graph at large size.
- `4` fits the 3D world and driving HUD into the same 1124 px driving region as mode `0` and uses the complete opposite 792 px information region for the graph. The acceleration, steering, fuel, and DEF gauges plus TPMS remain inside the driving region's right edge. Swapping the panel layout exchanges the graph and driving regions together.
- `5` always shows the driving report.

`ClusterHudScreenMode=5` shows a live driving report in the information panel. In default screen mode (`0`), the same report is shown automatically while no live navigation is being received, and the navigation panel returns when reception starts. The report background, cards, outlines, primary text, secondary text, and unavailable-value colors follow the active `ClusterHudTheme`, including Auto, Dark, and Light modes. Its large card summarizes driving time, distance, average and maximum speed, the automated-driving ratio, maximum acceleration/deceleration, and hard acceleration/braking/corner counts. The small card presents CPU load, temperature, memory, and disk use as a 2×2 set of circular gauges. Its lower target plots the stored device pitch (P) vertically and yaw (Y) horizontally relative to the calibrated center while retaining the numeric angles. The driving area retains the branch, network address, and frame-rate status; the core-usage text is omitted when it would overlap the report. In road-camera view, detected vehicles are enclosed by transparent rounded frames whose border retains the existing detection color; ungrouped radar detections use smaller transparent rounded markers in their source color. Vehicle frames use one lightweight outline, and frames that would be partially projected at the screen edge or stretched by a noisy radar heading are omitted.

The external HUD follows the device `LanguageSetting` and updates driving-report, driving-mode, and navigation status labels live in Korean (`ko`) or English (`en`). Other language values, including Chinese, fall back to English. With `IsMetric` enabled, vehicle/cruise/limit speeds, navigation, radar labels, and the driving report use `km/h`, `m`, and `km`; with it disabled they are converted to `mph`, `ft`, and `mi`. Acceleration and temperature remain `m/s²` and `°C`. Both settings are polled about once per second and do not require a HUD restart.

The enlarged driving-mode text beside the speed display has no background or border and remains above the final speed digit. Its text is green for Efficiency, orange for Safety, white for Normal, and red for High Speed. The gear badge also has a transparent center, retaining only its letter and outline. The upper-left steering/LFA, Wi-Fi, and clock row keeps extra outer and inter-icon spacing, and the speed-limit sign is offset slightly left.

`ClusterHudTheme=1` (Dark) renders the normal HUD's empty background in the same pure black used behind maps and while navigation is disconnected. Auto (`0`) uses the same dark palette from 18:00 to 06:00. The road, gauges, and regular information panels retain distinct dark shades for separation and readability.

On Hyundai/Kia CAN-FD hybrids, the external HUD's green `EV` indicator is enabled only when ECAN `0xFA` and `0x230` are both present with DLC32. It decodes the four-bit hybrid power-flow mode in `0x230` and shows `EV` for the observed motor/regen modes 1, 2, and 6. The normal HUD shows it between vehicle speed and cruise-set speed; full navigation intentionally omits it. The indicator remains hidden for other mode values and when the capability or sample is missing, invalid, or stale.

The normal external HUD also shows the current driving mode beside the traffic-state dot above vehicle speed. `MyDrivingMode` value `1` is a green Eco badge, `2` an orange Safe badge, `3` a white Normal badge, and `4` a red High badge. The badge is hidden for an unavailable, invalid, or stale `longitudinalPlan` and for values outside that range; full navigation omits it. The adjacent red or green dot is an independent model traffic-state indicator, not a driving-mode state.

The normal and road camera HUDs use the same fixed TPMS position below the acceleration, steering, fuel, and DEF gauges. The pressure font size is unchanged, with each value placed inside one of the enlarged tires of a simple toy-car diagram. The whole display is hidden only when all four pressure values are unavailable; an individually missing value shows `--`. Pressures below 31 psi are red, and no surrounding card or outline is drawn. When external navigation is active or its dashboard is connected, a green `NAV` appears below the Wi-Fi icon instead of the former lower-right `NAVI` label. The center clock, EV indicator, and fuel/DEF gauges remain unchanged.

### Carrot Vision AR and replay navigation events

Carrot Vision provides separate **Show AR** and **AR debug** controls outside the `carrot_settings.json` catalog. **Show AR** overlays driving guidance on the Vision video and requests the additional real-time data only while it is enabled. **AR debug** adds a troubleshooting panel with sign, anchor, and draw counts, the current blocking reason, and copyable history.

The Replay event timeline also identifies Carrot Navi connection and route-session changes, current and next maneuvers, lane guidance, road-safety alerts, average-speed zones, traffic signals, and intersection guidance. These entries are labels for reviewing transitions recorded in the replay.

<a id="system"></a>
## System

The 12 system settings cover recording, power, network, maps, sound, and software menus.

| Group | Parameters | Purpose |
|---|---|---|
| Recording and power | `RecordRoadCam`, `MaxTimeOffroadMin` | Road-camera storage and delayed shutdown |
| Network and map | `HotspotOnBoot`, `MapboxStyle` | Boot hotspot and map background style |
| Sound | `SoundLanguageSetting`, `SoundVolumeAdjust`, `SoundVolumeAdjustEngage` | Prompt language and volume |
| Software | `SoftwareMenu` | Carrot Web software-menu availability |

Check storage use for recording and network use, heat, and privacy before enabling live streaming.

`SoundVolumeAdjust` changes general sound volume. DM's first audible warning has a 70% minimum output gain; its final warning always uses 100%, regardless of volume settings or ambient-noise attenuation. The preceding visual-only notice remains silent. See [driver monitoring](driver-monitoring.md) for details.

Experimental DM switches to standard monitoring for 20 seconds only when moving traffic first appears after an absence. Additional vehicles do not extend it; experimental monitoring then resumes, with no empty-road bonus while traffic remains.

## Safe adjustment order

1. Record the current value in Carrot Web.
2. Save both a baseline profile and a file backup.
3. Write down the parameter, previous value, and reason for the change.
4. Change only one item by one `unit` step.
5. If a restart is required, restart only while parked and off-road.
6. Repeat the same test in a permitted, controlled environment.
7. Restore the previous value or baseline profile immediately if the result is worse or unclear.

See the Wiki [Tuning introduction](https://github.com/ajouatom/openpilot/wiki/Guide-Tuning) for the recommended steering and longitudinal adjustment order.
