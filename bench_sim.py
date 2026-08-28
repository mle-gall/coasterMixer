# SPDX-FileCopyrightText: 2026 Coaster Mixer contributors
# SPDX-License-Identifier: GPL-3.0-or-later

"""Headless benchmark and regression harness for the Coaster Mixer add-on.

Run with:
    /Applications/Blender.app/Contents/MacOS/Blender --background --factory-startup \
        --python bench_sim.py -- <label>

Builds a two-piece circuit (second piece traversed reversed) with a lift
transport zone, a stop-capable station brake inside a block group, follower
empties, then measures:
  - live playback cost (frame_set loop with the simulation enabled)
  - bake operator wall time
  - how many times curve caches are rebuilt (thrash detector)
and asserts basic physics sanity (train moves, stops at the station, dwells).

Results are printed and appended to bench_results.json next to this script.
"""

import importlib
import json
import re
import sys
import time
from math import cos, pi, sin, tau
from pathlib import Path

import bpy
from mathutils import Vector

SCRIPT_DIR = Path(__file__).resolve().parent
ADDON_SOURCE_PATHS = tuple(sorted((SCRIPT_DIR / "coaster_mixer").glob("*.py")))
RESULTS_PATH = SCRIPT_DIR / "bench_results.json"

LIVE_FRAME_COUNT = 250
BAKE_FRAME_START = 1
BAKE_FRAME_END = 3200
FOLLOWER_COUNT = 12
STATION_DWELL_SECONDS = 5.0
MIDDLE_PIECE_COUNT = 6
MIDDLE_PIECE_POINTS = 25
MIDDLE_PIECE_LENGTH = 110.0


def load_addon():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    addon = importlib.import_module("coaster_mixer")
    addon.register()
    return addon


class CallCounter:
    def __init__(self, function):
        self.function = function
        self.count = 0

    def __call__(self, *args, **kwargs):
        self.count += 1
        return self.function(*args, **kwargs)


def make_bezier(name, points):
    curve = bpy.data.curves.new(name, type="CURVE")
    curve.dimensions = "3D"
    spline = curve.splines.new("BEZIER")
    spline.bezier_points.add(len(points) - 1)
    for bezier_point, (co, handle_left, handle_right) in zip(spline.bezier_points, points):
        bezier_point.co = co
        bezier_point.handle_left = handle_left
        bezier_point.handle_right = handle_right
        bezier_point.handle_left_type = "FREE"
        bezier_point.handle_right_type = "FREE"
    obj = bpy.data.objects.new(name, curve)
    bpy.context.collection.objects.link(obj)
    return obj


def make_wavy_piece(name, index):
    """Dense bezier piece (~MIDDLE_PIECE_LENGTH m) with gentle waves."""
    from math import sin, cos

    points = []
    count = MIDDLE_PIECE_POINTS
    spacing = MIDDLE_PIECE_LENGTH / (count - 1)
    for point_index in range(count):
        y = point_index * spacing
        x = 6.0 * sin(point_index * 0.7 + index)
        z = 2.0 * sin(point_index * 0.5) + 1.0 * cos(point_index * 0.9 + index)
        co = (x, y, z)
        tangent = (
            6.0 * 0.7 * cos(point_index * 0.7 + index) / spacing,
            1.0,
            (2.0 * 0.5 * cos(point_index * 0.5) - 0.9 * sin(point_index * 0.9 + index)) / spacing,
        )
        third = spacing / 3.0
        handle_left = (co[0] - tangent[0] * third, co[1] - third, co[2] - tangent[2] * third)
        handle_right = (co[0] + tangent[0] * third, co[1] + third, co[2] + tangent[2] * third)
        points.append((co, handle_left, handle_right))
    return make_bezier(name, points)


def build_scene(addon):
    scene = bpy.context.scene
    scene.frame_start = BAKE_FRAME_START
    scene.frame_end = BAKE_FRAME_END

    # Piece A: hill going +Y, ~70 m. Piece B: flat return path, authored
    # start-to-start with A so the route enters it at its END (reversed).
    piece_a = make_bezier(
        "PieceA",
        [
            ((0.0, 0.0, 0.0), (0.0, -10.0, 0.0), (0.0, 10.0, 5.0)),
            ((0.0, 30.0, 15.0), (0.0, 20.0, 15.0), (0.0, 40.0, 15.0)),
            ((0.0, 60.0, 0.0), (0.0, 50.0, 5.0), (0.0, 70.0, 0.0)),
        ],
    )
    # Slight fall toward B's start: the route traverses B reversed, so the
    # station sits on a gentle downhill and the train can creep out of the
    # block after its dwell (a flat station parks the train forever).
    piece_b = make_bezier(
        "PieceB",
        [
            ((0.0, 0.0, 0.0), (-10.0, -5.0, 0.0), (10.0, 5.0, 0.3)),
            ((30.0, 30.0, 1.0), (30.0, 15.0, 0.7), (30.0, 45.0, 1.3)),
            ((0.0, 60.0, 2.0), (10.0, 55.0, 1.7), (-10.0, 65.0, 2.3)),
        ],
    )

    settings_a = piece_a.coaster_mixer_track
    settings_b = piece_b.coaster_mixer_track

    # Dense middle pieces to make the benchmark representative of a real
    # layout: A -> M1 -> ... -> Mn -> B(end, reversed) -> back to A.
    middle_pieces = [make_wavy_piece(f"Middle{index:02d}", index) for index in range(MIDDLE_PIECE_COUNT)]

    chain = [piece_a] + middle_pieces
    for previous_piece, next_piece in zip(chain, chain[1:]):
        connection = previous_piece.coaster_mixer_track.end_connections.add()
        connection.target = next_piece
        connection.target_end = "START"
    connection = chain[-1].coaster_mixer_track.end_connections.add()
    connection.target = piece_b
    connection.target_end = "END"
    connection_back = settings_b.start_connections.add()
    connection_back.target = piece_a
    connection_back.target_end = "START"

    # Transport boosters keep the train moving briskly; one magnetic trim for
    # cap-path coverage mid-route.
    for index, middle_piece in enumerate(middle_pieces):
        zone = middle_piece.coaster_mixer_track.zones.add()
        if index == 2:
            zone.zone_type = "TRIM_BRAKE"
            zone.target_speed_mps = 12.0
        else:
            zone.zone_type = "TRANSPORT"
            zone.target_speed_mps = 15.0
        zone.start_meters = 20.0
        zone.length_meters = 30.0
        zone.max_acceleration_mps2 = 3.0
        zone.max_braking_mps2 = 3.0

    # Onride-photo sensor mid-route.
    sensor = middle_pieces[1].coaster_mixer_track.sensors.add()
    sensor.name = "Photo"
    sensor.position_meters = 50.0
    photo_on = sensor.actions.add()
    photo_on.kind = "TRIGGER"
    photo_on.channel = "photo"
    photo_on.value = 1.0
    photo_delay = sensor.actions.add()
    photo_delay.kind = "WAIT"
    photo_delay.duration_seconds = 0.5
    photo_off = sensor.actions.add()
    photo_off.kind = "TRIGGER"
    photo_off.channel = "photo"
    photo_off.value = 0.0

    # Lift on A from the start so the train launches from rest.
    lift = settings_a.zones.add()
    lift.zone_type = "TRANSPORT"
    lift.start_meters = 0.0
    lift.length_meters = 35.0
    lift.target_speed_mps = 6.0
    lift.max_acceleration_mps2 = 3.0
    lift.max_braking_mps2 = 3.0

    # Station on B (traversed reversed): a trim run slows the approach, then
    # an overlapping friction brake + drive tires hold and move the train.
    approach_trim = settings_b.zones.add()
    approach_trim.zone_type = "TRIM_BRAKE"
    approach_trim.start_meters = 25.0
    approach_trim.length_meters = 35.0
    approach_trim.target_speed_mps = 4.0
    approach_trim.max_braking_mps2 = 2.0

    station_brake = settings_b.zones.add()
    station_brake.zone_type = "FRICTION_BRAKE"
    station_brake.start_meters = 5.0
    station_brake.length_meters = 20.0
    station_brake.target_speed_mps = 1.5
    station_brake.max_braking_mps2 = 2.5

    station_drive = settings_b.zones.add()
    station_drive.zone_type = "TRANSPORT"
    station_drive.start_meters = 5.0
    station_drive.length_meters = 20.0
    station_drive.target_speed_mps = 2.0
    station_drive.max_acceleration_mps2 = 1.5
    station_drive.max_braking_mps2 = 1.5

    scene_settings = scene.coaster_mixer_scene
    scene_settings.track_object = piece_a

    block = settings_a.block_groups.add()
    block.name = "Station Block"
    for zone_index in (1, 2):  # friction brake + drive tires
        member = block.members.add()
        member.piece = piece_b
        member.zone_index = zone_index

    route = addon.get_resolved_route(piece_a)
    station_items = [
        item for item in addon.build_route_zones(route)
        if item["entry"]["object"] == piece_b and item["zone_index"] in {1, 2}
    ]
    block.start_route_meters = min(item["route_start"] for item in station_items)
    block.end_route_meters = max(item["route_end"] for item in station_items)

    tree = bpy.data.node_groups.new("Station Control", addon.CONTROL_TREE_IDNAME)
    block.control_tree = tree
    node_specs = [
        ("COASTERMIXER_ND_block_entered", {}),
        ("COASTERMIXER_ND_set_brake_hold", {"offset_meters": 4.0}),
        ("COASTERMIXER_ND_set_transport", {"speed_mps": 2.0}),
        ("COASTERMIXER_ND_wait_position", {"offset_meters": 4.0}),
        ("COASTERMIXER_ND_wait_speed", {"speed_mps": 0.05}),
        ("COASTERMIXER_ND_wait", {"duration_seconds": 4.0}),
        ("COASTERMIXER_ND_trigger", {"channel": "harness", "value": 1.0}),
        ("COASTERMIXER_ND_wait", {"duration_seconds": 2.0}),
        ("COASTERMIXER_ND_trigger", {"channel": "harness", "value": 0.0}),
        ("COASTERMIXER_ND_release_brake", {}),
        ("COASTERMIXER_ND_set_brake_hold", {"offset_meters": 8.0}),
        ("COASTERMIXER_ND_set_transport", {"speed_mps": 1.5}),
        ("COASTERMIXER_ND_wait_position", {"offset_meters": 8.0}),
        ("COASTERMIXER_ND_wait_speed", {"speed_mps": 0.05}),
        ("COASTERMIXER_ND_wait", {"duration_seconds": STATION_DWELL_SECONDS}),
        ("COASTERMIXER_ND_release_brake", {}),
        ("COASTERMIXER_ND_set_transport", {"speed_mps": 2.0}),
        ("COASTERMIXER_ND_dispatch", {}),
    ]
    previous = None
    for index, (node_type, fields) in enumerate(node_specs):
        node = tree.nodes.new(node_type)
        node.location = (index * 220.0, 0.0)
        for field_name, field_value in fields.items():
            setattr(node, field_name, field_value)
        if previous is not None:
            tree.links.new(previous.outputs["Then"], node.inputs["In"])
        previous = node

    # Start-at-station principle: the start cursor snaps to the last position
    # gate of the last block (the load position at the end of the track).
    result = bpy.ops.coaster_mixer.snap_start_to_station()
    assert result == {"FINISHED"}, "snap_start_to_station failed"

    driven_empty = bpy.data.objects.new("TrainFront", None)
    bpy.context.collection.objects.link(driven_empty)
    settings_a.driven_empty_object = driven_empty

    for index in range(FOLLOWER_COUNT):
        empty = bpy.data.objects.new(f"Car{index:02d}", None)
        bpy.context.collection.objects.link(empty)
        addon.ensure_follower_drivers(piece_a, empty, offset_meters=1.5 * index)

    result = bpy.ops.coaster_mixer.create_train_camera(
        "EXEC_DEFAULT",
        height_meters=1.6,
        look_ahead_meters=5.0,
        lens_millimeters=35.0,
        make_active=True,
    )
    assert result == {"FINISHED"}, f"ride camera creation failed: {result}"

    route = addon.get_resolved_route(piece_a)
    expected_pieces = MIDDLE_PIECE_COUNT + 2
    assert route["cyclic"], "expected a closed circuit"
    assert len(route["entries"]) == expected_pieces, (
        f"expected {expected_pieces} route pieces, got {len(route['entries'])}"
    )
    assert route["entries"][-1]["reversed"], "expected piece B to be traversed reversed"
    return piece_a, scene_settings


def check_camera_rig(addon, track_object, scene_settings):
    cameras = addon.collect_ride_cameras(track_object)
    assert len(cameras) == 1, f"expected one ride camera, got {len(cameras)}"
    camera_object = cameras[0]
    settings = camera_object.coaster_mixer_camera
    followers = addon.collect_track_followers(track_object)
    assert settings.mount_object == followers[1], "camera should initially mount to the second car"
    assert settings.target_object is not None, "camera target follower is missing"
    assert abs(settings.target_object.coaster_mixer_follower.vertical_offset_meters - 1.6) <= 1.0e-6

    settings.mount_object = followers[3]
    expected_target_offset = followers[3].coaster_mixer_follower.offset_meters - settings.look_ahead_meters
    actual_target_offset = settings.target_object.coaster_mixer_follower.offset_meters
    assert abs(actual_target_offset - expected_target_offset) <= 1.0e-6, (
        "camera target did not preserve its look-ahead distance after changing cars"
    )

    settings.offset_xyz = (0.25, -0.1, 1.7)
    settings.target_vertical_offset_meters = 1.7
    settings.shake_enabled = True
    scene_settings.simulation_current_speed_mps = 20.0
    addon.place_track_followers(track_object, 120.0)
    assert (camera_object.location - Vector(settings.offset_xyz)).length > 1.0e-7, (
        "enabled camera shake did not move the camera"
    )
    settings.shake_enabled = False
    addon.place_track_followers(track_object, 120.0)
    assert (camera_object.location - Vector(settings.offset_xyz)).length <= 1.0e-7, (
        "disabled camera shake should restore the authored offset"
    )


def check_vertical_safe_frames(addon):
    """Minimum-twist frames must remain continuous through a vertical tangent."""
    half_turn_seam = addon.unwrap_tilt_values([0.0, pi], cyclic=True)
    assert abs(half_turn_seam[-1] - pi) <= 1.0e-9, "automatic seam discarded a matching half turn"
    full_turn_seam = addon.unwrap_tilt_values([0.0, tau], cyclic=True)
    assert abs(full_turn_seam[-1] - tau) <= 1.0e-9, "automatic seam discarded a matching full turn"
    ordinary_seam = addon.unwrap_tilt_values([0.0, pi * 0.25], cyclic=True)
    assert abs(ordinary_seam[-1]) <= 1.0e-9, "automatic seam failed to unwind an ordinary tilt"

    sample_count = 33
    tangents = []
    distances = []
    for index in range(sample_count):
        angle = pi * index / (sample_count - 1)
        tangents.append(Vector((0.0, cos(angle), sin(angle))))
        distances.append(float(index))

    frames = addon.build_minimum_twist_frames(tangents, distances)
    assert len(frames) == sample_count
    previous_up = None
    for tangent, frame in zip(tangents, frames):
        forward = frame @ Vector((0.0, 1.0, 0.0))
        up = frame @ Vector((0.0, 0.0, 1.0))
        assert forward.dot(tangent) > 0.99999, "minimum-twist frame lost the track tangent"
        if previous_up is not None:
            assert previous_up.dot(up) > 0.98, "minimum-twist frame flipped near vertical"
        previous_up = up

    continuous_frames = addon.build_continuous_z_up_frames(tangents)
    previous_up = None
    for index, (tangent, frame) in enumerate(zip(tangents, continuous_frames)):
        forward = frame @ Vector((0.0, 1.0, 0.0))
        up = frame @ Vector((0.0, 0.0, 1.0))
        assert forward.dot(tangent) > 0.99999, "continuous Z-up frame lost the track tangent"
        if previous_up is not None:
            assert previous_up.dot(up) > 0.98, "continuous Z-up frame flipped near vertical"
        if index == 0:
            assert up.z > 0.99999, "continuous Z-up frame should begin upright"
        if index == sample_count - 1:
            assert up.z < -0.99999, "half-loop frame should finish inverted without a snap"
        previous_up = up


def run_live_loop(addon, scene_settings, per_frame_invalidation):
    """Time a frame_set loop with an explicit invalidation policy.

    Headless frame changes do not fire depsgraph_update_post, so the policies
    emulate what the handler does in the UI:
      - "none": warm caches (ideal / post-fix behavior)
      - "route": pop only the route caches (scoped invalidation fallback)
      - "all": clear_runtime_caches() every frame (current UI handler behavior)
    """
    scene = bpy.context.scene
    scene_settings.simulation_enabled = True
    bpy.ops.coaster_mixer.reset_simulation()

    curve_counter = CallCounter(addon.runtime.build_curve_cache_data)
    addon.runtime.build_curve_cache_data = curve_counter
    route_counter = CallCounter(addon.runtime.resolve_track_route)
    addon.runtime.resolve_track_route = route_counter

    started = time.perf_counter()
    for frame in range(1, LIVE_FRAME_COUNT + 1):
        if per_frame_invalidation == "all":
            addon.clear_runtime_caches()
        elif per_frame_invalidation == "route":
            addon.runtime.ROUTE_CACHE_BY_ROOT.clear()
        scene.frame_set(frame)
    elapsed = time.perf_counter() - started

    addon.runtime.build_curve_cache_data = curve_counter.function
    addon.runtime.resolve_track_route = route_counter.function
    scene_settings.simulation_enabled = False
    return {
        "seconds": round(elapsed, 4),
        "ms_per_frame": round(elapsed / LIVE_FRAME_COUNT * 1000.0, 3),
        "curve_cache_builds": curve_counter.count,
        "route_resolves": route_counter.count,
    }


def bench_live_playback(addon, scene_settings):
    results = {}
    for name, policy in (("warm", "none"), ("route_invalidate", "route"), ("thrash", "all")):
        stats = run_live_loop(addon, scene_settings, policy)
        for key, value in stats.items():
            results[f"live_{name}_{key}"] = value
    return results


def check_scrub_and_loop(addon, scene_settings, track_object):
    """Scrubbing must be deterministic and playback must loop on the cycle."""
    scene = bpy.context.scene
    track_settings = track_object.coaster_mixer_track
    scene_settings.simulation_enabled = True

    scene.frame_set(200)
    front_at_200 = track_settings.train_front_route_meters
    scene.frame_set(40)
    front_at_40 = track_settings.train_front_route_meters
    scene.frame_set(200)
    assert abs(track_settings.train_front_route_meters - front_at_200) <= 1.0e-9, "forward re-scrub mismatch"
    scene.frame_set(40)
    assert abs(track_settings.train_front_route_meters - front_at_40) <= 1.0e-9, "backward scrub mismatch"

    # Sampling far past the timeline forces cycle discovery.
    scene.frame_set(scene.frame_start + 12000)
    cache = addon.runtime.SIMULATION_TRAJECTORY_CACHE
    assert cache is not None and cache["cycle_length"], "no simulation cycle detected"
    cycle_start = cache["cycle_start"]
    cycle_length = cache["cycle_length"]
    assert cycle_length > 100, f"suspicious degenerate cycle of {cycle_length} frame(s)"

    probe_frame = scene.frame_start + cycle_start + 25
    scene.frame_set(probe_frame)
    front_probe = track_settings.train_front_route_meters
    for laps in (1, 3):
        scene.frame_set(probe_frame + laps * cycle_length)
        assert abs(track_settings.train_front_route_meters - front_probe) <= 1.0e-6, (
            f"loop mismatch after {laps} cycle(s)"
        )

    scene.frame_set(scene.frame_start)
    scene_settings.simulation_enabled = False
    return {
        "cycle_start_frame": cycle_start,
        "cycle_length_frames": cycle_length,
    }


def bench_bake(addon, track_object):
    curve_counter = CallCounter(addon.runtime.build_curve_cache_data)
    addon.runtime.build_curve_cache_data = curve_counter
    route_counter = CallCounter(addon.runtime.resolve_track_route)
    addon.runtime.resolve_track_route = route_counter

    started = time.perf_counter()
    result = bpy.ops.coaster_mixer.bake_simulation()
    elapsed = time.perf_counter() - started

    addon.runtime.build_curve_cache_data = curve_counter.function
    addon.runtime.resolve_track_route = route_counter.function
    assert result == {"FINISHED"}, f"bake failed: {result}"

    fcurve = addon.get_action_fcurve(track_object, addon.TRAIN_FRONT_METERS_DATA_PATH)
    assert fcurve is not None, "no baked fcurve found"
    values = [fcurve.evaluate(frame) for frame in range(BAKE_FRAME_START, BAKE_FRAME_END + 1)]
    return {
        "bake_seconds": round(elapsed, 4),
        "bake_ms_per_frame": round(elapsed / (BAKE_FRAME_END - BAKE_FRAME_START + 1) * 1000.0, 3),
        "bake_curve_cache_builds": curve_counter.count,
        "bake_route_resolves": route_counter.count,
    }, values


def check_ui_statics(addon):
    """Static checks for panel-draw-time errors headless runs can't hit.

    Panel draw() only executes with a real window, so an invalid icon name or
    a stale enum literal (e.g. assigning a removed zone type to an operator
    button) aborts the panel silently in the UI. Validate them from source.
    """
    source = "\n".join(path.read_text() for path in ADDON_SOURCE_PATHS)
    valid_icons = set(bpy.types.UILayout.bl_rna.functions["label"].parameters["icon"].enum_items.keys())

    used_icons = set(re.findall(r'icon="([A-Z_0-9]+)"', source))
    used_icons.update(addon.ZONE_TYPE_ICONS.values())
    used_icons.update(addon.ACTION_KIND_ICONS.values())
    unknown_icons = used_icons - valid_icons
    assert not unknown_icons, f"invalid icon names: {sorted(unknown_icons)}"

    zone_types = {identifier for identifier, _label, _description in addon.ZONE_TYPE_ITEMS}
    for literal in re.findall(r'\.zone_type = "([A-Z_]+)"', source):
        assert literal in zone_types, f"stale zone_type literal in addon source: {literal}"

    action_kinds = {identifier for identifier, _label, _description in addon.ACTION_KIND_ITEMS}
    for literal in re.findall(r'\.kind = "([A-Z_]+)"', source):
        assert literal in action_kinds, f"stale action kind literal in addon source: {literal}"

    for literal in re.findall(r'\.owner = "([A-Z_]+)"', source):
        assert literal in {"BLOCK", "SENSOR"}, f"stale action owner literal in addon source: {literal}"


def find_plateaus(values):
    plateaus = []
    run_start = 0
    for index in range(1, len(values)):
        if abs(values[index] - values[index - 1]) > 1.0e-5:
            if index - run_start >= 3:
                plateaus.append((values[run_start], index - run_start))
            run_start = index
    if len(values) - run_start >= 3:
        plateaus.append((values[run_start], len(values) - run_start))
    return plateaus


def check_physics(addon, track_object, values):
    scene = bpy.context.scene
    fps = scene.render.fps / scene.render.fps_base

    route = addon.get_resolved_route(track_object)
    derived = addon.get_route_derived_data(route)
    assert len(derived["programs"]) == 1, f"expected one block program, got {len(derived['programs'])}"
    program = derived["programs"][0]
    assert program["warnings"] == [], f"unexpected block warnings: {program['warnings']}"
    span_start = program["span"][0]
    unload_target = span_start + 4.0
    load_target = span_start + 8.0

    # t0: parked at the load point, waits the load dwell, departs on dispatch.
    assert abs(values[0] - load_target) <= 1.0e-3, (
        f"train does not start at the load point ({values[0]:.2f} vs {load_target:.2f})"
    )
    load_frames = int(STATION_DWELL_SECONDS * fps)
    assert abs(values[load_frames - 2] - load_target) <= 1.0e-3, "train left before the load wait ended"
    assert values[load_frames + 15] > load_target + 0.05, "train did not depart after dispatch"
    assert max(values) > load_target + 1.0, "train barely moved"

    # Station stops land on the exact action points with their full durations
    # (unload wait + harness open wait share the unload point).
    plateaus = find_plateaus(values)

    def has_plateau(position, min_frames):
        return any(
            abs(value - position) <= 1.0e-2 and length >= min_frames for value, length in plateaus
        )

    assert has_plateau(unload_target, 6.0 * fps - 3), (
        f"missing unload plateau at {unload_target:.2f} m; plateaus: {plateaus}"
    )
    assert has_plateau(load_target, STATION_DWELL_SECONDS * fps - 3), (
        f"missing load plateau at {load_target:.2f} m; plateaus: {plateaus}"
    )

    # Trigger channels recorded by block actions and the trackside sensor.
    cache = addon.runtime.SIMULATION_TRAJECTORY_CACHE
    assert cache is not None, "trajectory cache missing"
    channels = cache["channels"]
    assert "harness" in channels and "photo" in channels, f"missing channels: {sorted(channels)}"

    harness_frames, harness_values = channels["harness"]
    assert harness_values[:2] == [1.0, 0.0], f"harness sequence wrong: {harness_values[:4]}"
    harness_gap = harness_frames[1] - harness_frames[0]
    assert abs(harness_gap - 2.0 * fps) <= 2, f"harness open lasted {harness_gap} frames"

    photo_frames, photo_values = channels["photo"]
    assert photo_values[:2] == [1.0, 0.0], f"photo sequence wrong: {photo_values[:4]}"
    photo_gap = photo_frames[1] - photo_frames[0]
    assert abs(photo_gap - 0.5 * fps) <= 2, f"photo pulse lasted {photo_gap} frames"

    # Sensor fires exactly once (two events) per simulation cycle.
    assert cache["cycle_length"], "no cycle available for the per-lap sensor check"
    cycle_start, cycle_length = cache["cycle_start"], cache["cycle_length"]
    photo_in_cycle = [frame for frame in photo_frames if cycle_start <= frame < cycle_start + cycle_length]
    assert len(photo_in_cycle) == 2, f"expected 2 photo events per cycle, got {len(photo_in_cycle)}"

    # cm_trigger driver lookup matches the recorded channel timeline.
    scene.frame_set(scene.frame_start + harness_frames[0])
    assert addon.coaster_mixer_trigger_driver("harness") == 1.0, "cm_trigger should read harness=1"
    scene.frame_set(scene.frame_start + harness_frames[1])
    assert addon.coaster_mixer_trigger_driver("harness") == 0.0, "cm_trigger should read harness=0"

    # Followers must be placed by the batched frame updater (off origin).
    scene.frame_set(BAKE_FRAME_START + 50)
    depsgraph = bpy.context.evaluated_depsgraph_get()
    placed = 0
    for obj in bpy.data.objects:
        if obj.type == "EMPTY" and obj.coaster_mixer_follower.track_object is not None:
            if obj.evaluated_get(depsgraph).matrix_world.translation.length > 0.1:
                placed += 1
    assert placed >= FOLLOWER_COUNT, f"only {placed} followers placed by batched placement"

    return {
        "block_span_m": [round(program["span"][0], 3), round(program["span"][1], 3)],
        "unload_stop_m": round(unload_target, 3),
        "load_stop_m": round(load_target, 3),
        "harness_open_frames": harness_gap,
        "photo_pulse_frames": photo_gap,
        "route_length_m": round(route["total_length"], 3),
        "final_front_m": round(float(values[-1]), 3),
    }


def main():
    label = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else "unlabeled"
    addon = load_addon()
    track_object, scene_settings = build_scene(addon)

    results = {"label": label}
    check_ui_statics(addon)
    check_camera_rig(addon, track_object, scene_settings)
    check_vertical_safe_frames(addon)
    results.update(bench_live_playback(addon, scene_settings))
    results.update(check_scrub_and_loop(addon, scene_settings, track_object))
    bake_stats, values = bench_bake(addon, track_object)
    results.update(bake_stats)
    results.update(check_physics(addon, track_object, values))
    results["baked_values_sample"] = [round(float(values[frame]), 4) for frame in range(0, len(values), 25)]

    print("BENCH_RESULT " + json.dumps(results))
    history = []
    if RESULTS_PATH.exists():
        history = json.loads(RESULTS_PATH.read_text())
    history.append(results)
    RESULTS_PATH.write_text(json.dumps(history, indent=2) + "\n")


if __name__ == "__main__":
    main()
