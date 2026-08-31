# coasterMixer

## Project
- Blender add-on for authoring and driving rollercoasters.
- Target runtime: Blender 5.x and newer.
- Current milestone: single-file prototype add-on with a curve-driven segment editor in the 3D View N-panel.

## Current Scope
- The track is a graph of pieces: every curve object can be a piece, with connection lists at its start and end (`coaster_mixer_track.start_connections` / `end_connections`). Multiple connections at one end form a switch; the animatable `*_active_index` is the switch state.
- A coaster is anchored by the scene-level root piece. `resolve_track_route` walks the graph from the root through active switch states into a route (ordered pieces, cumulative arc lengths, cyclic flag). Pieces entered at their End are traversed reversed (forward axis and bank sign flip). Dead-end routes model storage/transfer tracks.
- Each piece owns its hardware zones and sensors (piece-local meters); `get_route_derived_data` assembles zones, block programs, and sensor points into route space for simulation (cached per root against the route key).
- Hardware zone types: `TRANSPORT` (drives toward target speed, can hold/move the train), `FRICTION_BRAKE` (decelerates, can stop/hold once at or below its target "controllable" speed, cannot push), `TRIM_BRAKE` (magnetic, caps speed only). Control resolution: transports drive, brakes cap — a stopped train under a station brake can still be pushed out.
- Blocks are sequencers: span derived from member zones (start of first to end of last, route space) plus an ordered action list — MOVE (to offset within the span at a controlled speed), WAIT (duration), TRIGGER (set a named channel value), DISPATCH (release). The train is captured on span entry; without a DISPATCH it stays parked (UI warns). Sensors are per-piece points that fire a TRIGGER/WAIT-only sequence when crossed, once per lap.
- Trigger channels are read in Blender via the `cm_trigger('channel')` driver function (deterministic, loops with the cycle); baking lays `cm:` timeline markers at trigger frames.
- The train front is `coaster_mixer_track.train_front_route_meters` on the root piece — meters along the route, deliberately not normalized so switch changes don't teleport the train.
- Train placement is arc-length accurate: follower empties carry location/rotation drivers calling the `cm_place` driver-namespace function, which samples the route's cached length tables (no Follow Path constraint — that evaluates parametrically and distorts speed). Followers trail the front by per-empty offsets in meters (`Object.coaster_mixer_follower`), banking taken from curve tilt.
- The N-panel edits zones/connections on the active viewport curve ("edit piece"), while train/simulation settings live on the root.
- Live playback is a deterministic function of the timeline: frame `0` is t0 (train at Start Position, speed 0), each frame advances one 1/fps step, and samples are memoized in a module-level trajectory cache. Once the quantized sim state recurs (stops and captures snap to frame boundaries to guarantee exact recurrence on circuits with blocks), playback loops on the detected cycle — scrubbing either direction and playing past the cycle end are O(1) lookups. Baking writes the same trajectory to keyframes.
- Start-at-station principle: the track conceptually starts at the station exit; the start cursor (`simulation_start_route_meters`, drawn as a green viewport arrow) is normally snapped to the last block's last move point via the Start at Station operator, so the train begins parked in the station at the end of the track and runs the remaining block actions before dispatch.
- Known limitation: switch keyframes are not sampled by the trajectory (animation writes don't fire property updates, so the cached route keeps the values it was resolved with). Throwing a switch as an edit rebuilds the preview from t0 with the new topology.
- Coast losses: rolling friction (coefficient on the root) plus aerodynamic drag (0.5 * rho * CdA * v^2, `drag_area_m2` on the root).

## Repo Conventions
- Keep the implementation in one script until the prototype has enough moving parts to justify a package split.
- `bench_sim.py` is a headless benchmark/regression harness (`Blender --background --factory-startup --python bench_sim.py -- <label>`); run it after touching the simulation or caching code and compare timings plus the physics assertions.
- Prefer storing authored coaster data on the selected curve object, not globally on the scene, except for UI state such as the active curve reference.
- Favor normalized curve positions for authoring data first; add derived world-space helpers on top.
- Treat the first spline of the selected curve as the working track until multi-spline support is added deliberately.

## Release Process
- Release from `main` only after the working tree is clean and the intended changes are committed.
- Keep prerelease packaging aligned with the Blender extension manifest in `coaster_mixer/blender_manifest.toml`; bump that `version` in its own commit after the feature/fix commit(s).
- Preserve the existing tag format: `v<manifest-version>` such as `v0.3.0-alpha.7`.
- Before publishing, run `python3 -m py_compile coaster_mixer/*.py` and rebuild the extension archive with `python3 scripts/build_addon.py`.
- The packaged asset must come from `dist/coaster_mixer-<manifest-version>.zip`, produced by `scripts/build_addon.py`; do not hand-roll a different ZIP layout.
- Push `main`, create the matching git tag on the manifest bump commit, and push that tag to `origin`.
- Publish prereleases with GitHub CLI so they match prior alpha drops: title `Coaster Mixer v<manifest-version>`, tag `v<manifest-version>`, `--prerelease`, and attach `dist/coaster_mixer-<manifest-version>.zip`.
- Release notes should be short and practical: one sentence describing the alpha focus, a short `Highlights:` list, and an install line mentioning the exact ZIP filename.
- The GitHub Actions workflow in `.github/workflows/build-addon.yml` only builds and uploads artifacts on `main`; it does not create the GitHub release. Creating the tag alone is not sufficient.

## Near-Term Next Steps
- Replace the prototype boundary gizmo with a curve-aware interface gizmo that sits directly on the segment seam.
- Introduce more segment types and per-type settings.
- Move simulation state per-train (off the scene) to allow multiple trains and block sections; switches under a stopped train should keep it piece-anchored rather than route-anchored.
- Add curvature-loaded friction to the coast model (aero drag is done).
- New block action kinds: set travel direction, actuate a track switch / animated track piece; backward MOVEs within a block.
- Convenience operators for connections ("connect selected piece ends", snap piece endpoints together) and a geometric mismatch warning when connected ends don't touch.
- Split the add-on into modules once operators, gizmos, and data models stabilize, and add a `blender_manifest.toml` for the extension platform.
