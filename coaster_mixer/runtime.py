# SPDX-FileCopyrightText: 2026 Coaster Mixer contributors
# SPDX-License-Identifier: GPL-3.0-or-later

import bpy
import gpu
from bisect import bisect_left, bisect_right
from math import atan2, log1p, pi, sin, tau
from time import perf_counter
from mathutils import Matrix, Quaternion, Vector
from mathutils.geometry import interpolate_bezier
from gpu_extras.batch import batch_for_shader
from bpy.app.handlers import persistent


DEFAULT_ZONE_LENGTH_METERS = 10.0
DEFAULT_STATION_DISPATCH_SPEED_MPS = 4.0
DEFAULT_STATION_ACCELERATION_MPS2 = 1.5
DEFAULT_STATION_BRAKING_MPS2 = 2.0
PROPERTY_COMPARE_EPSILON = 1.0e-5
SIMULATION_STOP_EPSILON = 1.0e-4
GRAVITY_ACCELERATION = 9.81
AIR_DENSITY_KG_M3 = 1.225
TRACK_LINE_COLOR = (0.22, 0.38, 0.52, 0.30)
ACTIVE_ZONE_COLOR = (1.0, 0.62, 0.18, 0.95)
TRANSPORT_ZONE_COLOR = (0.18, 0.78, 0.42, 0.95)
BRAKE_ZONE_COLOR = (0.92, 0.24, 0.18, 0.95)
TRIM_ZONE_COLOR = (0.95, 0.82, 0.2, 0.95)
BRAKE_SLOWDOWN_COLOR = (0.55, 0.16, 0.12, 0.6)
START_MARKER_COLOR = (0.4, 1.0, 0.55, 1.0)
SENSOR_MARKER_COLOR = (0.95, 0.3, 0.85, 1.0)
BLOCK_GROUP_COLOR = (0.82, 0.58, 1.0, 0.75)
SEAM_MARKER_COLOR = (1.0, 0.84, 0.36, 1.0)
HOLD_POINT_COLOR = (0.12, 0.92, 0.96, 1.0)
CONTROL_TRANSPORT_COLOR = (0.12, 0.55, 1.0, 0.95)
CONTROL_BRAKE_COLOR = (1.0, 0.28, 0.08, 0.95)
CONTROL_LOGIC_COLOR = (0.72, 0.42, 1.0, 0.95)
CONTROL_POSITION_COLOR = (1.0, 1.0, 1.0, 1.0)
VIEWPORT_DRAW_HANDLER = None
CONTROL_SELECTION_MSGBUS_OWNER = object()
PROPERTY_SYNC_GUARD = False
SIMULATION_ENABLED_UPDATE_GUARD = False
# Legacy constraint name kept only so older files can be cleaned up on attach.
DRIVEN_EMPTY_CONSTRAINT_NAME = "CoasterMixerFollowPath"
TRAIN_FRONT_METERS_DATA_PATH = "coaster_mixer_track.train_front_route_meters"
FOLLOWER_OFFSET_DATA_PATH = "coaster_mixer_follower.offset_meters"
ROUTE_MAX_PIECES = 64
CONNECTION_END_ITEMS = [
    ("START", "Start", "Enter the target piece at its start and traverse it forward"),
    ("END", "End", "Enter the target piece at its end and traverse it reversed"),
]
PLACEMENT_DRIVER_FUNCTION_NAME = "cm_place"
TRIGGER_DRIVER_FUNCTION_NAME = "cm_trigger"
PLACEMENT_SAMPLE_CACHE = {}
PLACEMENT_CHANNEL_CACHE = {}
PLACEMENT_SAMPLE_CACHE_LIMIT = 4096
CURVE_CACHE_REVISION_COUNTER = 0
CURVE_CACHE_BY_OBJECT = {}
ROUTE_CACHE_BY_ROOT = {}
ROUTE_ZONE_CACHE_BY_ROOT = {}
OVERLAY_DRAW_CACHE_BY_OBJECT = {}
# Per-frame simulation trajectory (front/speed/stop time), deterministic in
# the timeline: frame_start is t0, one 1/fps step per frame. Self-validating
# via its key (route key + physics scalars), so scrubbing and looping are
# plain lookups.
SIMULATION_TRAJECTORY_CACHE = None
TRAJECTORY_FRAME_LIMIT = 20000
TRAJECTORY_STATE_DECIMALS = 3

ZONE_TYPE_ITEMS = [
    ("TRIM_BRAKE", "Trim Brake", "Magnetic trim: caps the train speed toward the target, can never stop or hold"),
    ("TRANSPORT", "Transport", "Drive tires, LSM, or chain: drives the train toward the target speed and can stop, hold, and move it precisely"),
    ("FRICTION_BRAKE", "Friction Brake", "Friction brake run: decelerates the train and can stop or hold it once at or below its controllable (target) speed, cannot push"),
]
ZONE_TYPE_LABELS = {identifier: label for identifier, label, _description in ZONE_TYPE_ITEMS}
ZONE_TYPE_ICONS = {
    "TRIM_BRAKE": "MODIFIER",
    "TRANSPORT": "MODIFIER",
    "FRICTION_BRAKE": "MODIFIER",
}
STOP_CAPABLE_ZONE_TYPES = {"TRANSPORT", "FRICTION_BRAKE"}
ACTION_KIND_ITEMS = [
    ("MOVE", "Move To", "Move the captured train to an offset within the block span at a controlled speed and stop there"),
    ("WAIT", "Wait", "Keep the train (or the sensor sequence) waiting for a duration"),
    ("TRIGGER", "Trigger", "Set a named trigger channel to a value; read it in Blender drivers via cm_trigger('channel')"),
    ("DISPATCH", "Release Block", "End block sequencing and return the train to track hardware and physics"),
]
ACTION_KIND_ICONS = {
    "MOVE": "FORWARD",
    "WAIT": "PAUSE",
    "TRIGGER": "OUTLINER_OB_LIGHT",
    "DISPATCH": "PLAY",
}
SENSOR_ACTION_KINDS = {"WAIT", "TRIGGER"}
CONTROL_TREE_IDNAME = "CoasterMixerControlTree"
CONTROL_SOCKET_IDNAME = "CoasterMixerControlSocket"
BANK_SEAM_MODE_ITEMS = [
    ("AUTO", "Automatic Seam", "Preserve authored point tilts and choose the equivalent start angle that makes only the cyclic closing seam shortest"),
    ("AUTHORED", "Exact Authored Values", "Interpolate the raw Blender tilt values exactly, including any roll across the cyclic seam"),
    ("MANUAL", "Manual Seam Winding", "Keep interior banking continuous and explicitly choose the accumulated full turns at the cyclic seam"),
]
ORIENTATION_FRAME_ITEMS = [
    (
        "Z_UP",
        "Z Up (Legacy)",
        "Orient each sample against world Z; compatible with existing tracks but singular at vertical tangents",
    ),
    (
        "MINIMUM_TWIST",
        "Minimum Twist (Vertical Safe)",
        "Parallel-transport orientation along the curve without flipping when the track becomes vertical",
    ),
]
CONTROL_RESPONSE_CURVE_ITEMS = [
    ("LINEAR", "Linear", "Apply a constant acceleration or deceleration limit toward the target speed"),
    ("SMOOTH", "Smooth", "Ease in toward the target speed with a gentler finish"),
    ("LOG", "Logarithmic", "Respond strongly at larger speed errors, then taper near the target"),
]
CONTROL_TEMPLATE_ITEMS = [
    ("CUSTOM", "Custom", "Keep and edit the assigned node tree manually"),
    ("STOPPED_LAUNCH", "Stopped Launch", "Stop and hold at a precise point, then launch under transport control"),
    ("ROLLING_LAUNCH", "Rolling Launch", "Apply a transport target without stopping before release"),
    ("STANDARD_LIFT", "Standard Lift", "Carry the train through the block at a fixed configured transport speed"),
    ("TRIM_BRAKE", "Trimmed Brake Zone", "Cap speed through the block without stopping the train"),
    ("LOAD_STATION", "Load Station", "Hold for loading, release the brake, and move onward under transport control"),
    ("UNLOAD_STATION", "Unload Station", "Stop and hold for unloading, then move the train onward"),
]
CONTROL_RESPONSE_CURVE_LABELS = {
    identifier: label for identifier, label, _description in CONTROL_RESPONSE_CURVE_ITEMS
}


def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def values_differ(value_a, value_b, epsilon=PROPERTY_COMPARE_EPSILON):
    return abs(value_a - value_b) > epsilon


def assign_rna_property(owner, attribute, value):
    global PROPERTY_SYNC_GUARD

    previous_guard = PROPERTY_SYNC_GUARD
    PROPERTY_SYNC_GUARD = True
    try:
        setattr(owner, attribute, value)
    finally:
        PROPERTY_SYNC_GUARD = previous_guard


def refresh_view_layer():
    context = bpy.context
    view_layer = getattr(context, "view_layer", None)
    if view_layer is not None:
        view_layer.update()


def assign_simulation_enabled(scene_settings, value):
    global SIMULATION_ENABLED_UPDATE_GUARD

    previous_guard = SIMULATION_ENABLED_UPDATE_GUARD
    SIMULATION_ENABLED_UPDATE_GUARD = True
    try:
        setattr(scene_settings, "simulation_enabled", value)
    finally:
        SIMULATION_ENABLED_UPDATE_GUARD = previous_guard


def is_curve_object(_self, obj):
    return obj is not None and obj.type == "CURVE"


def is_empty_object(_self, obj):
    return obj is not None and obj.type == "EMPTY"


def tag_redraw_view3d():
    context = bpy.context
    window_manager = getattr(context, "window_manager", None)
    if window_manager is None:
        return

    for window in window_manager.windows:
        screen = window.screen
        if screen is None:
            continue

        for area in screen.areas:
            if area.type == "VIEW_3D":
                area.tag_redraw()


def ensure_control_selection_subscription():
    bpy.msgbus.clear_by_owner(CONTROL_SELECTION_MSGBUS_OWNER)
    bpy.msgbus.subscribe_rna(
        key=(bpy.types.Node, "select"),
        owner=CONTROL_SELECTION_MSGBUS_OWNER,
        args=(),
        notify=tag_redraw_view3d,
    )


def remove_control_selection_subscription():
    bpy.msgbus.clear_by_owner(CONTROL_SELECTION_MSGBUS_OWNER)


def tag_track_placement_update(track_object, refresh=True):
    # Follower placement is driven by depsgraph drivers reading the path
    # factor, so a property write plus an update tag is all that is needed.
    if track_object is None:
        return

    track_object.update_tag()
    if refresh:
        refresh_view_layer()


def get_primary_spline(track_object):
    if track_object is None or track_object.type != "CURVE":
        return None

    curve_data = getattr(track_object, "data", None)
    if curve_data is None or not curve_data.splines:
        return None

    return curve_data.splines[0]


def is_track_cyclic(track_object):
    spline = get_primary_spline(track_object)
    return bool(spline is not None and spline.use_cyclic_u)


def get_empty_curve_cache():
    return {
        "points": [],
        "distances": [],
        "horizontal_distances": [],
        "segment_directions": [],
        "point_tangents": [],
        "point_frames": [],
        "tilts": [],
        "total_length": 0.0,
        "revision": 0,
    }


def get_curve_cache_signature(track_object):
    spline = get_primary_spline(track_object)
    if spline is None:
        return None

    matrix_key = tuple(component for row in track_object.matrix_world for component in row)
    track_settings = track_object.coaster_mixer_track

    if spline.type == "BEZIER":
        point_key = tuple(
            (
                tuple(point.co),
                tuple(point.handle_left),
                tuple(point.handle_right),
                point.handle_left_type,
                point.handle_right_type,
                point.tilt,
            )
            for point in spline.bezier_points
        )
    else:
        point_key = tuple((tuple(point.co[:4]), point.tilt) for point in spline.points)

    return (
        matrix_key,
        spline.type,
        bool(spline.use_cyclic_u),
        int(getattr(spline, "resolution_u", 0)),
        track_settings.bank_seam_mode,
        track_settings.bank_seam_turns,
        track_settings.orientation_frame_mode,
        point_key,
    )


def unwrap_tilt_values(values, cyclic=False, seam_mode="AUTO", seam_turns=0):
    """Build tilt values, optionally choosing a cyclic seam endpoint.

    Blender permits a cyclic spline to begin at 0 and end at 2*pi. Those
    angles are the same orientation. Automatic mode changes only the
    generated closing endpoint; interior values stay exactly as authored so
    follower banking agrees with Blender, including intentional inversions.
    """
    if not values:
        return []
    authored = list(values)
    if seam_mode == "AUTHORED":
        if cyclic and len(authored) > 1:
            authored.append(authored[0])
        return authored
    if cyclic and len(authored) > 1:
        if seam_mode == "MANUAL":
            authored.append(authored[0] + seam_turns * tau)
        else:
            previous = authored[-1]
            closing_delta = (authored[0] - previous + pi) % tau - pi
            authored.append(previous + closing_delta)
    return authored


def build_curve_world_points(track_object):
    """Sample the primary spline into world points with parallel tilt values."""
    spline = get_primary_spline(track_object)
    if spline is None:
        return [], []

    matrix_world = track_object.matrix_world
    track_settings = track_object.coaster_mixer_track
    world_points = []
    tilts = []

    if spline.type == "BEZIER":
        bezier_points = spline.bezier_points
        count = len(bezier_points)
        if count == 0:
            return [], []
        if count == 1:
            return [matrix_world @ bezier_points[0].co], [bezier_points[0].tilt]

        segment_count = count if spline.use_cyclic_u else count - 1
        resolution = max(int(spline.resolution_u) * 2, 16)
        control_tilts = unwrap_tilt_values(
            [point.tilt for point in bezier_points],
            cyclic=spline.use_cyclic_u,
            seam_mode=track_settings.bank_seam_mode,
            seam_turns=track_settings.bank_seam_turns,
        )

        for index in range(segment_count):
            point_a = bezier_points[index]
            point_b = bezier_points[(index + 1) % count]
            tilt_a = control_tilts[index]
            tilt_b = control_tilts[index + 1]
            segment_points = interpolate_bezier(
                point_a.co,
                point_a.handle_right,
                point_b.handle_left,
                point_b.co,
                resolution,
            )
            denominator = max(len(segment_points) - 1, 1)
            start_sample = 1 if index > 0 else 0
            for sample_index in range(start_sample, len(segment_points)):
                sample_factor = sample_index / denominator
                world_points.append(matrix_world @ segment_points[sample_index])
                tilts.append(tilt_a + (tilt_b - tilt_a) * sample_factor)

        return world_points, tilts

    control_tilts = unwrap_tilt_values(
        [point.tilt for point in spline.points],
        cyclic=spline.use_cyclic_u,
        seam_mode=track_settings.bank_seam_mode,
        seam_turns=track_settings.bank_seam_turns,
    )
    for index, point in enumerate(spline.points):
        world_points.append(matrix_world @ Vector(point.co[:3]))
        tilts.append(control_tilts[index])

    if spline.use_cyclic_u and len(world_points) > 1:
        world_points.append(world_points[0].copy())
        tilts.append(control_tilts[-1])

    return world_points, tilts


def build_point_tangents(points, segment_directions):
    if len(points) <= 1 or not segment_directions:
        return [Vector((0.0, 0.0, 1.0)) for _point in points]

    point_tangents = []
    for index in range(len(points)):
        if index == 0:
            tangent = segment_directions[0].copy()
        elif index == len(points) - 1:
            tangent = segment_directions[-1].copy()
        else:
            tangent = segment_directions[index - 1] + segment_directions[index]
            if tangent.length <= 1.0e-8:
                tangent = segment_directions[index - 1].copy()
        tangent.normalize()
        point_tangents.append(tangent)

    # Smooth the seam tangent for closed tracks so the train does not snap.
    if len(segment_directions) >= 2 and (points[0] - points[-1]).length <= 1.0e-6:
        wrap_tangent = segment_directions[0] + segment_directions[-1]
        if wrap_tangent.length > 1.0e-8:
            wrap_tangent.normalize()
            point_tangents[0] = wrap_tangent
            point_tangents[-1] = wrap_tangent.copy()

    return point_tangents


def frame_from_tangent_and_up(tangent, up_hint):
    """Build a local +Y-forward, +Z-up frame from orthogonal world vectors."""
    forward = tangent.normalized()
    up = up_hint - forward * up_hint.dot(forward)
    if up.length <= 1.0e-8:
        fallback = Vector((1.0, 0.0, 0.0))
        if abs(forward.dot(fallback)) > 0.95:
            fallback = Vector((0.0, 1.0, 0.0))
        up = fallback - forward * fallback.dot(forward)
    up.normalize()
    right = forward.cross(up).normalized()
    up = right.cross(forward).normalized()
    return Matrix((right, forward, up)).transposed().to_quaternion()


def build_minimum_twist_frames(point_tangents, distances, cyclic=False):
    """Build rotation-minimizing frames by parallel-transporting the up axis."""
    if not point_tangents:
        return []

    first_tangent = point_tangents[0].normalized()
    first_frame = frame_from_tangent_and_up(first_tangent, Vector((0.0, 0.0, 1.0)))
    frames = [first_frame]
    previous_tangent = first_tangent
    previous_up = first_frame @ Vector((0.0, 0.0, 1.0))

    for tangent_value in point_tangents[1:]:
        tangent = tangent_value.normalized()
        transport = previous_tangent.rotation_difference(tangent)
        transported_up = transport @ previous_up
        frame = frame_from_tangent_and_up(tangent, transported_up)
        frames.append(frame)
        previous_tangent = tangent
        previous_up = frame @ Vector((0.0, 0.0, 1.0))

    if cyclic and len(frames) > 1 and distances and distances[-1] > 1.0e-8:
        tangent = point_tangents[0].normalized()
        first_up = frames[0] @ Vector((0.0, 0.0, 1.0))
        last_up = frames[-1] @ Vector((0.0, 0.0, 1.0))
        closure_angle = atan2(tangent.dot(last_up.cross(first_up)), last_up.dot(first_up))
        if abs(closure_angle) > 1.0e-9:
            total_length = distances[-1]
            frames = [
                frame @ Quaternion((0.0, 1.0, 0.0), closure_angle * distance / total_length)
                for frame, distance in zip(frames, distances)
            ]

    return frames


def build_curve_cache_data(track_object):
    global CURVE_CACHE_REVISION_COUNTER

    raw_points, raw_tilts = build_curve_world_points(track_object)
    if not raw_points:
        return get_empty_curve_cache()

    points = [raw_points[0].copy()]
    tilts = [raw_tilts[0]]
    distances = [0.0]
    # Prefix sum of XY-projected length: cos(slope) integrated over arc
    # length, so span averages come from two lookups instead of sampling.
    horizontal_distances = [0.0]
    segment_directions = []

    for point, tilt in zip(raw_points[1:], raw_tilts[1:]):
        segment_vector = point - points[-1]
        step = segment_vector.length
        if step <= 1.0e-6:
            continue
        segment_directions.append(segment_vector / step)
        points.append(point.copy())
        tilts.append(tilt)
        distances.append(distances[-1] + step)
        horizontal_step = (segment_vector.x * segment_vector.x + segment_vector.y * segment_vector.y) ** 0.5
        horizontal_distances.append(horizontal_distances[-1] + horizontal_step)

    point_tangents = build_point_tangents(points, segment_directions)
    point_frames = []
    if track_object.coaster_mixer_track.orientation_frame_mode == "MINIMUM_TWIST":
        point_frames = build_minimum_twist_frames(
            point_tangents,
            distances,
            cyclic=(len(points) > 1 and (points[0] - points[-1]).length <= 1.0e-6),
        )

    CURVE_CACHE_REVISION_COUNTER += 1
    return {
        "points": points,
        "distances": distances,
        "horizontal_distances": horizontal_distances,
        "segment_directions": segment_directions,
        "point_tangents": point_tangents,
        "point_frames": point_frames,
        "tilts": tilts,
        "total_length": distances[-1] if distances else 0.0,
        "revision": CURVE_CACHE_REVISION_COUNTER,
    }


def build_curve_cache(track_object):
    if track_object is None or track_object.type != "CURVE":
        return get_empty_curve_cache()

    object_key = track_object.as_pointer()
    signature = get_curve_cache_signature(track_object)
    if signature is None:
        CURVE_CACHE_BY_OBJECT.pop(object_key, None)
        return get_empty_curve_cache()

    cached_entry = CURVE_CACHE_BY_OBJECT.get(object_key)
    if cached_entry is not None and cached_entry["signature"] == signature:
        return cached_entry["cache"]

    cache = build_curve_cache_data(track_object)
    CURVE_CACHE_BY_OBJECT[object_key] = {
        "signature": signature,
        "cache": cache,
    }
    return cache


def sample_curve_cache_at_distance(cache, distance):
    points = cache["points"]
    distances = cache["distances"]
    segment_directions = cache.get("segment_directions", [])
    total_length = cache["total_length"]

    if not points:
        return None, None, cache

    if len(points) == 1 or total_length <= 1.0e-8:
        return points[0], Vector((0.0, 0.0, 1.0)), cache

    target_distance = clamp(distance, 0.0, total_length)

    index = bisect_left(distances, target_distance, lo=1)
    if index >= len(points):
        index = len(points) - 1

    segment_end = distances[index]
    segment_start = distances[index - 1]
    segment_length = segment_end - segment_start
    factor = 0.0 if segment_length <= 1.0e-8 else (target_distance - segment_start) / segment_length
    location = points[index - 1].lerp(points[index], factor)
    tangent = segment_directions[index - 1].copy() if index - 1 < len(segment_directions) else points[index] - points[index - 1]
    if tangent.length <= 1.0e-8:
        tangent = Vector((0.0, 0.0, 1.0))
    else:
        tangent.normalize()
    return location, tangent, cache


def sample_curve_placement(cache, distance, reverse=False):
    """Arc-length sample returning a world location and banked orientation.

    `reverse` flips travel direction for pieces entered from their end: the
    forward axis inverts and the bank sign flips with it.
    """
    points = cache["points"]
    distances = cache["distances"]
    point_tangents = cache["point_tangents"]
    point_frames = cache.get("point_frames", [])
    tilts = cache["tilts"]
    total_length = cache["total_length"]

    if not points:
        return None, None

    if len(points) == 1 or total_length <= 1.0e-8:
        return points[0].copy(), Quaternion()

    target_distance = clamp(distance, 0.0, total_length)

    index = bisect_left(distances, target_distance, lo=1)
    if index >= len(points):
        index = len(points) - 1

    segment_start = distances[index - 1]
    segment_length = distances[index] - segment_start
    factor = 0.0 if segment_length <= 1.0e-8 else (target_distance - segment_start) / segment_length

    location = points[index - 1].lerp(points[index], factor)
    tangent = point_tangents[index - 1].lerp(point_tangents[index], factor)
    if tangent.length <= 1.0e-8:
        tangent = Vector((0.0, 1.0, 0.0))
    else:
        tangent.normalize()

    tilt = tilts[index - 1] + (tilts[index] - tilts[index - 1]) * factor

    if point_frames:
        rotation = point_frames[index - 1].slerp(point_frames[index], factor)
        if reverse:
            # Preserve physical up while local +Y changes travel direction.
            rotation = rotation @ Quaternion((0.0, 0.0, 1.0), pi)
            tilt = -tilt
    else:
        if reverse:
            tangent = tangent * -1.0
            tilt = -tilt
        # Legacy mode: world-Z reference is singular when tangent is vertical.
        rotation = tangent.to_track_quat("Y", "Z")
    if abs(tilt) > 1.0e-9:
        rotation = rotation @ Quaternion((0.0, 1.0, 0.0), tilt)

    return location, rotation


def get_connection_list(track_settings, end_identifier):
    if end_identifier == "END":
        return track_settings.end_connections
    return track_settings.start_connections


def get_connection_active_index(track_settings, end_identifier):
    if end_identifier == "END":
        return track_settings.end_active_index
    return track_settings.start_active_index


def get_active_connection(track_settings, end_identifier):
    connections = get_connection_list(track_settings, end_identifier)
    if len(connections) == 0:
        return None

    index = clamp(get_connection_active_index(track_settings, end_identifier), 0, len(connections) - 1)
    connection = connections[index]
    target = connection.target
    if target is None or target.type != "CURVE":
        return None

    return connection


def resolve_track_route(root_object):
    """Walk the piece graph from the root through active switch states.

    Returns a route dict: ordered piece entries with cumulative start
    distances, the total arc length, whether the route closes back on the
    root, and a key that changes whenever any input of the route changes.
    """
    if root_object is None or root_object.type != "CURVE":
        return {"entries": [], "starts": [], "total_length": 0.0, "cyclic": False, "key": ()}

    entries = []
    key_parts = []
    total_length = 0.0
    cyclic = False
    visited = set()
    current_object = root_object
    traversed_reversed = False

    while current_object is not None and len(entries) < ROUTE_MAX_PIECES:
        visit_key = (current_object.as_pointer(), traversed_reversed)
        if visit_key in visited:
            cyclic = current_object == root_object and not traversed_reversed
            break
        visited.add(visit_key)

        cache = build_curve_cache(current_object)
        track_settings = current_object.coaster_mixer_track
        entries.append({
            "object": current_object,
            "settings": track_settings,
            "cache": cache,
            "reversed": traversed_reversed,
            "start": total_length,
            "length": cache["total_length"],
        })
        key_parts.append((
            current_object.as_pointer(),
            traversed_reversed,
            cache["revision"],
            track_settings.start_active_index,
            track_settings.end_active_index,
        ))
        total_length += cache["total_length"]

        if is_track_cyclic(current_object):
            # A closed spline is a complete circuit on its own.
            cyclic = len(entries) == 1
            break

        exit_end = "START" if traversed_reversed else "END"
        connection = get_active_connection(track_settings, exit_end)
        if connection is None:
            break

        traversed_reversed = connection.target_end == "END"
        current_object = connection.target

    return {
        "entries": entries,
        "starts": [entry["start"] for entry in entries],
        "total_length": total_length,
        "cyclic": cyclic,
        "key": (tuple(key_parts), cyclic),
    }


def invalidate_simulation_trajectory():
    global SIMULATION_TRAJECTORY_CACHE
    SIMULATION_TRAJECTORY_CACHE = None


def invalidate_route_cache(root_object=None):
    PLACEMENT_SAMPLE_CACHE.clear()
    PLACEMENT_CHANNEL_CACHE.clear()
    # Zone and block parameters feed the simulation but are not part of the
    # trajectory key, so any route-level edit drops the trajectory too.
    invalidate_simulation_trajectory()
    if root_object is None:
        ROUTE_CACHE_BY_ROOT.clear()
        ROUTE_ZONE_CACHE_BY_ROOT.clear()
        OVERLAY_DRAW_CACHE_BY_OBJECT.clear()
        return

    object_key = root_object.as_pointer()
    ROUTE_CACHE_BY_ROOT.pop(object_key, None)
    ROUTE_ZONE_CACHE_BY_ROOT.pop(object_key, None)
    OVERLAY_DRAW_CACHE_BY_OBJECT.pop(object_key, None)


def get_resolved_route(root_object):
    if root_object is None or root_object.type != "CURVE":
        return resolve_track_route(root_object)

    object_key = root_object.as_pointer()
    cached_entry = ROUTE_CACHE_BY_ROOT.get(object_key)
    if cached_entry is not None:
        return cached_entry

    route = resolve_track_route(root_object)
    ROUTE_CACHE_BY_ROOT[object_key] = route
    return route


def wrap_route_distance(route, distance):
    total_length = route["total_length"]
    if total_length <= 1.0e-8:
        return 0.0
    if route["cyclic"]:
        return distance % total_length
    return clamp(distance, 0.0, total_length)


def find_route_entry(route, distance):
    entries = route["entries"]
    if not entries:
        return None

    index = clamp(bisect_right(route["starts"], distance) - 1, 0, len(entries) - 1)
    while index > 0 and entries[index]["length"] <= 1.0e-8:
        index -= 1
    return entries[index]


def route_to_piece_distance(entry, route_distance):
    local_distance = clamp(route_distance - entry["start"], 0.0, entry["length"])
    if entry["reversed"]:
        return entry["length"] - local_distance
    return local_distance


def piece_to_route_distance(entry, piece_distance):
    local_distance = clamp(piece_distance, 0.0, entry["length"])
    if entry["reversed"]:
        return entry["start"] + (entry["length"] - local_distance)
    return entry["start"] + local_distance


def map_piece_span_to_route(entry, span):
    distance_a = piece_to_route_distance(entry, span[0])
    distance_b = piece_to_route_distance(entry, span[1])
    return (min(distance_a, distance_b), max(distance_a, distance_b))


def sample_route_placement(route, distance):
    wrapped_distance = wrap_route_distance(route, distance)
    entry = find_route_entry(route, wrapped_distance)
    if entry is None:
        return None, None

    # The cache captured at route-resolve time is valid as long as the route
    # is: the route key embeds every piece's cache revision.
    return sample_curve_placement(
        entry["cache"],
        route_to_piece_distance(entry, wrapped_distance),
        reverse=entry["reversed"],
    )


def sample_route_tangent_z(route, distance):
    wrapped_distance = wrap_route_distance(route, distance)
    entry = find_route_entry(route, wrapped_distance)
    if entry is None:
        return 0.0

    _location, tangent, _cache = sample_curve_cache_at_distance(
        entry["cache"], route_to_piece_distance(entry, wrapped_distance)
    )
    if tangent is None:
        return 0.0
    return -tangent.z if entry["reversed"] else tangent.z


def sample_route_tangent(route, distance):
    """Return the unit travel tangent at a route-space distance."""
    _location, rotation = sample_route_placement(route, distance)
    if rotation is None:
        return Vector((0.0, 1.0, 0.0))
    tangent = rotation @ Vector((0.0, 1.0, 0.0))
    if tangent.length <= 1.0e-8:
        return Vector((0.0, 1.0, 0.0))
    tangent.normalize()
    return tangent


def sample_route_curvature_vector(route, distance, sample_radius=0.5):
    """Estimate dT/ds, the signed 3D curvature vector, in inverse meters."""
    total_length = route["total_length"]
    if total_length <= 1.0e-8:
        return Vector((0.0, 0.0, 0.0))

    radius = min(max(sample_radius, 0.05), total_length * 0.25)
    if route["cyclic"]:
        tangent_before = sample_route_tangent(route, distance - radius)
        tangent_after = sample_route_tangent(route, distance + radius)
        denominator = 2.0 * radius
    else:
        before = max(distance - radius, 0.0)
        after = min(distance + radius, total_length)
        denominator = after - before
        if denominator <= 1.0e-8:
            return Vector((0.0, 0.0, 0.0))
        tangent_before = sample_route_tangent(route, before)
        tangent_after = sample_route_tangent(route, after)
    return (tangent_after - tangent_before) / denominator


def sample_curve_scalars_at_distance(cache, distance):
    """Return (elevation, cumulative horizontal length) at an arc distance."""
    points = cache["points"]
    distances = cache["distances"]
    horizontal_distances = cache["horizontal_distances"]
    total_length = cache["total_length"]

    if not points:
        return 0.0, 0.0

    if len(points) == 1 or total_length <= 1.0e-8:
        return points[0].z, 0.0

    target_distance = clamp(distance, 0.0, total_length)
    index = bisect_left(distances, target_distance, lo=1)
    if index >= len(points):
        index = len(points) - 1

    segment_start = distances[index - 1]
    segment_length = distances[index] - segment_start
    factor = 0.0 if segment_length <= 1.0e-8 else (target_distance - segment_start) / segment_length
    elevation = points[index - 1].z + (points[index].z - points[index - 1].z) * factor
    horizontal = (
        horizontal_distances[index - 1]
        + (horizontal_distances[index] - horizontal_distances[index - 1]) * factor
    )
    return elevation, horizontal


def get_route_span_profile(route, start_distance, end_distance):
    """Exact elevation gain and horizontal length over a route span.

    sin(slope) = dz/ds and cos(slope) = d(horizontal)/ds, so the span
    averages are (delta_z / span) and (horizontal / span) — no footprint
    sampling needed. Accumulated per piece so geometric gaps between
    connected pieces are not counted as slope.
    """
    delta_z = 0.0
    horizontal = 0.0
    for entry in route["entries"]:
        span_start = max(start_distance, entry["start"])
        span_end = min(end_distance, entry["start"] + entry["length"])
        if span_end - span_start <= 1.0e-9:
            continue
        cache = entry["cache"]
        elevation_a, horizontal_a = sample_curve_scalars_at_distance(
            cache, route_to_piece_distance(entry, span_start)
        )
        elevation_b, horizontal_b = sample_curve_scalars_at_distance(
            cache, route_to_piece_distance(entry, span_end)
        )
        delta_z += elevation_b - elevation_a
        horizontal += abs(horizontal_b - horizontal_a)
    return delta_z, horizontal


def build_route_zones(route):
    """Assemble every piece's authored hardware zones into route-space intervals."""
    items = []
    for entry in route["entries"]:
        track_settings = entry["settings"]
        for zone_index in range(len(track_settings.zones)):
            zone = track_settings.zones[zone_index]
            local_start, local_end = resolve_zone_span(zone)
            if local_end - local_start <= 1.0e-8:
                continue
            route_span = map_piece_span_to_route(entry, (local_start, local_end))
            items.append({
                "zone": zone,
                "zone_index": zone_index,
                "entry": entry,
                "local_start": local_start,
                "local_end": local_end,
                "route_start": route_span[0],
                "route_end": route_span[1],
            })

    items.sort(key=lambda item: item["route_start"])
    return items


def resolve_block_action(action, span_start, span_end, stop_capable_spans, previous_target, warnings):
    resolved = {"kind": action.kind, "label": action.label}
    if action.kind == "MOVE":
        target = span_start + clamp(action.offset_meters, 0.0, max(span_end - span_start, 0.0))
        resolved["target"] = target
        resolved["speed"] = max(action.speed_mps, 0.0)
        if not any(
            span[0] - SIMULATION_STOP_EPSILON <= target <= span[1] + SIMULATION_STOP_EPSILON
            for span in stop_capable_spans
        ):
            warnings.append(f"Move point at +{action.offset_meters:.1f} m is outside stop-capable hardware")
        if previous_target is not None and target < previous_target - SIMULATION_STOP_EPSILON:
            warnings.append("Move points go backward; the train only moves forward")
        return resolved, target
    if action.kind == "WAIT":
        resolved["duration"] = max(action.duration_seconds, 0.0)
    elif action.kind == "TRIGGER":
        resolved["channel"] = action.channel.strip()
        resolved["value"] = action.value
    return resolved, previous_target


def resolve_block_route_span(route, block_group):
    """Map signed authored block positions into forward route space."""
    total_length = route["total_length"]
    authored_start = block_group.start_route_meters
    authored_end = block_group.end_route_meters
    if route["cyclic"] and authored_start < 0.0 and authored_end <= 0.0:
        return (
            clamp(total_length + authored_start, 0.0, total_length),
            clamp(total_length + authored_end, 0.0, total_length),
        )
    return (
        clamp(authored_start, 0.0, total_length),
        clamp(authored_end, max(authored_start, 0.0), total_length),
    )


def build_route_block_programs(route, route_zones):
    """Resolve every block group into a route-space capture-and-run program."""
    if not route["entries"]:
        return []

    root_track_settings = route["entries"][0]["settings"]
    zone_items_by_key = {
        (item["entry"]["object"].as_pointer(), item["zone_index"]): item for item in route_zones
    }

    programs = []
    for block_group in root_track_settings.block_groups:
        member_items = []
        for member in block_group.members:
            piece = member.piece
            if piece is None or piece.type != "CURVE":
                continue
            item = zone_items_by_key.get((piece.as_pointer(), member.zone_index))
            if item is not None:
                member_items.append(item)
        # A hardware-backed block follows its assigned actuators. This keeps
        # occupancy aligned when a zone is moved after "Create Block From
        # Zone" and prevents a stale block at route zero from capturing the
        # initial train ahead of the Station. Manual authored spans remain as
        # the fallback for blocks with no valid members.
        if member_items:
            span_start = min(item["route_start"] for item in member_items)
            span_end = max(item["route_end"] for item in member_items)
        else:
            span_start, span_end = resolve_block_route_span(route, block_group)
        stop_capable_spans = [
            (item["route_start"], item["route_end"])
            for item in member_items
            if item["zone"].zone_type in STOP_CAPABLE_ZONE_TYPES
        ]
        friction_brake_spans = [
            (item["route_start"], item["route_end"])
            for item in member_items
            if item["zone"].zone_type == "FRICTION_BRAKE"
        ]
        transport_items = [item for item in member_items if item["zone"].zone_type == "TRANSPORT"]
        brake_items = [
            item for item in member_items if item["zone"].zone_type != "TRANSPORT"
        ]
        max_acceleration = max(
            (max(item["zone"].max_acceleration_mps2, 0.0) for item in transport_items), default=0.0
        )
        max_braking = max(
            (max(item["zone"].max_braking_mps2, 0.0) for item in member_items), default=0.0
        )
        drive_min_speed = min(
            (max(item["zone"].minimum_speed_mps, 0.0) for item in transport_items),
            default=0.0,
        )
        drive_max_speed = max(
            (max(item["zone"].target_speed_mps, 0.0) for item in transport_items),
            default=0.0,
        )
        controllable_speed = max(
            (
                max(item["zone"].target_speed_mps, 0.0)
                for item in member_items
                if item["zone"].zone_type != "TRANSPORT"
            ),
            default=0.0,
        )

        warnings = []
        actions = compile_control_tree(
            block_group.control_tree,
            span_start,
            span_end,
            stop_capable_spans,
            friction_brake_spans,
            [get_route_zone_span(item) for item in transport_items],
            [get_route_zone_span(item) for item in brake_items],
            drive_min_speed,
            drive_max_speed,
            max_acceleration,
            max_braking,
            controllable_speed,
            warnings,
        )
        has_dispatch = any(action["kind"] == "DISPATCH" for action in actions)
        if any(action["kind"] == "SET_BRAKE_STOP" for action in actions) and not friction_brake_spans:
            warnings.append("Hold-point control requires an assigned Friction Brake actuator")
        if actions and not has_dispatch:
            warnings.append("No Release Block action: the train stays captured by this block")

        programs.append({
            "key": f"block:{block_group.as_pointer()}",
            "name": block_group.name,
            "span": (span_start, span_end),
            "actuator_spans": stop_capable_spans,
            "transport_spans": [get_route_zone_span(item) for item in transport_items],
            "brake_spans": [get_route_zone_span(item) for item in brake_items],
            "friction_brake_spans": friction_brake_spans,
            "control_tree": block_group.control_tree,
            "max_acceleration": max_acceleration,
            "max_braking": max_braking,
            "controllable_speed": controllable_speed,
            "drive_min_speed": drive_min_speed,
            "drive_max_speed": drive_max_speed,
            "actions": actions,
            "has_dispatch": has_dispatch,
            "warnings": warnings,
        })

    programs.sort(key=lambda program: program["span"][0])
    return programs


def build_route_sensors(route):
    """Resolve per-piece sensors into route-space trigger points."""
    sensors = []
    for entry in route["entries"]:
        track_settings = entry["settings"]
        for sensor in track_settings.sensors:
            actions = []
            for action in sensor.actions:
                if action.kind == "WAIT":
                    actions.append({"kind": "WAIT", "duration": max(action.duration_seconds, 0.0)})
                elif action.kind == "TRIGGER" and action.channel.strip():
                    actions.append({"kind": "TRIGGER", "channel": action.channel.strip(), "value": action.value})
            sensors.append({
                "key": f"sensor:{sensor.as_pointer()}",
                "name": sensor.name,
                "sensor": sensor,
                "entry": entry,
                "position": piece_to_route_distance(entry, max(sensor.position_meters, 0.0)),
                "actions": actions,
            })

    sensors.sort(key=lambda sensor: sensor["position"])
    return sensors


def get_route_derived_data(route):
    """Zones, block programs, and sensors resolved into route space, cached
    per root against the route key (invalidated on any zone/block edit)."""
    if not route["entries"]:
        return {"zones": [], "programs": [], "programs_by_key": {}, "sensors": []}

    root_object = route["entries"][0]["object"]
    object_key = root_object.as_pointer()
    route_key = route["key"]
    cached_entry = ROUTE_ZONE_CACHE_BY_ROOT.get(object_key)
    if cached_entry is not None and cached_entry["route_key"] == route_key:
        return cached_entry["derived"]

    zones = build_route_zones(route)
    programs = build_route_block_programs(route, zones)
    derived = {
        "zones": zones,
        "programs": programs,
        "programs_by_key": {program["key"]: program for program in programs},
        "sensors": build_route_sensors(route),
    }
    ROUTE_ZONE_CACHE_BY_ROOT[object_key] = {
        "route_key": route_key,
        "derived": derived,
    }
    return derived


def get_resolved_route_zones(route):
    return get_route_derived_data(route)["zones"]


def get_route_zone_span(item):
    return item["route_start"], item["route_end"]


def coaster_mixer_placement_driver(driven_object, front_meters, offset_meters, channel_index):
    """Driver-namespace entry point placing follower empties along the route.

    Channels 0-2 are world location XYZ, channels 3-5 rotation_euler XYZ.
    Must never raise: drivers evaluate inside the depsgraph.
    """
    try:
        driven_key = driven_object.as_pointer()
        channel_entry = PLACEMENT_CHANNEL_CACHE.get(driven_key)
        if (
            channel_entry is not None
            and channel_entry[0] == front_meters
            and channel_entry[1] == offset_meters
        ):
            return channel_entry[2][channel_index]

        follower_settings = getattr(driven_object, "coaster_mixer_follower", None)
        track_object = follower_settings.track_object if follower_settings is not None else None
        if track_object is None or track_object.type != "CURVE":
            return 0.0

        route = get_resolved_route(track_object)
        if route["total_length"] <= 1.0e-8:
            return 0.0

        # Follower offsets are signed: positive values trail the train while
        # negative values lead it.  Camera look-ahead targets rely on the
        # latter, including wrapping naturally across a cyclic route seam.
        distance = wrap_route_distance(route, front_meters - offset_meters)

        sample_key = (
            track_object.as_pointer(),
            driven_object.as_pointer(),
            route["key"],
            round(front_meters, 5),
            round(offset_meters, 5),
        )
        entry = PLACEMENT_SAMPLE_CACHE.get(sample_key)
        if entry is None or entry[0] != route["key"]:
            location, rotation = sample_route_placement(route, distance)
            if location is None:
                return 0.0
            euler = rotation.to_euler("XYZ")
            if len(PLACEMENT_SAMPLE_CACHE) >= PLACEMENT_SAMPLE_CACHE_LIMIT:
                PLACEMENT_SAMPLE_CACHE.clear()
            entry = (
                route["key"],
                (location.x, location.y, location.z),
                (euler.x, euler.y, euler.z),
            )
            PLACEMENT_SAMPLE_CACHE[sample_key] = entry

        channels = entry[1] + entry[2]
        PLACEMENT_CHANNEL_CACHE[driven_key] = (front_meters, offset_meters, channels)
        return channels[channel_index]
    except Exception:
        return 0.0


def coaster_mixer_trigger_driver(channel):
    """Driver-namespace lookup of a trigger channel at the current frame.

    Returns the last value a block or sensor TRIGGER action set on the
    channel (0.0 before any trigger). Deterministic, loops with the
    simulation cycle. Must never raise: drivers evaluate in the depsgraph.
    """
    try:
        context = bpy.context
        scene = getattr(context, "scene", None)
        scene_settings = getattr(scene, "coaster_mixer_scene", None)
        if scene_settings is None:
            return 0.0
        track_object = scene_settings.track_object
        if track_object is None or track_object.type != "CURVE":
            return 0.0

        frame = scene.frame_current
        track_settings = track_object.coaster_mixer_track
        if sample_simulation_trajectory(scene, track_object, track_settings, frame) is None:
            return 0.0

        cache = SIMULATION_TRAJECTORY_CACHE
        if cache is None:
            return 0.0
        channel_data = cache["channels"].get(channel)
        if not channel_data:
            return 0.0

        index = resolve_trajectory_index(cache, max(int(frame) - scene.frame_start, 0))
        frames, values = channel_data
        position = bisect_right(frames, index)
        if position == 0:
            return 0.0
        return values[position - 1]
    except Exception:
        return 0.0


def ensure_driver_namespace():
    bpy.app.driver_namespace[PLACEMENT_DRIVER_FUNCTION_NAME] = coaster_mixer_placement_driver
    bpy.app.driver_namespace[TRIGGER_DRIVER_FUNCTION_NAME] = coaster_mixer_trigger_driver


def remove_driver_namespace():
    bpy.app.driver_namespace.pop(PLACEMENT_DRIVER_FUNCTION_NAME, None)
    bpy.app.driver_namespace.pop(TRIGGER_DRIVER_FUNCTION_NAME, None)


def build_segment_draw_points(cache, start_distance, end_distance):
    points = cache["points"]
    distances = cache["distances"]
    total_length = cache["total_length"]

    if not points:
        return []

    if len(points) == 1 or total_length <= 1.0e-8:
        return [points[0].copy()]

    start_distance = clamp(start_distance, 0.0, total_length)
    end_distance = clamp(end_distance, 0.0, total_length)
    if end_distance < start_distance:
        start_distance, end_distance = end_distance, start_distance

    start_location, _tangent, _cache = sample_curve_cache_at_distance(cache, start_distance)
    end_location, _tangent, _cache = sample_curve_cache_at_distance(cache, end_distance)
    if start_location is None or end_location is None:
        return []

    draw_points = [start_location]

    for index, point in enumerate(points[1:-1], start=1):
        distance = distances[index]
        if start_distance < distance < end_distance:
            draw_points.append(point.copy())

    if (end_location - draw_points[-1]).length > 1.0e-6 or len(draw_points) == 1:
        draw_points.append(end_location)

    return draw_points


def draw_polyline(points, color, width):
    if len(points) < 2:
        return

    shader = gpu.shader.from_builtin("POLYLINE_UNIFORM_COLOR")
    batch = batch_for_shader(shader, "LINE_STRIP", {"pos": [tuple(point) for point in points]})
    viewport = gpu.state.viewport_get()
    shader.bind()
    shader.uniform_float("viewportSize", (viewport[2], viewport[3]))
    shader.uniform_float("lineWidth", width)
    shader.uniform_float("color", color)
    batch.draw(shader)


def draw_points(points, color, size):
    if not points:
        return

    shader = gpu.shader.from_builtin("UNIFORM_COLOR")
    batch = batch_for_shader(shader, "POINTS", {"pos": [tuple(point) for point in points]})
    gpu.state.point_size_set(size)
    shader.bind()
    shader.uniform_float("color", color)
    batch.draw(shader)
    gpu.state.point_size_set(1.0)


def clamp_zone_span(track_object, zone):
    total_length = get_track_total_length(track_object)
    start_distance = clamp(zone.start_meters, 0.0, max(total_length, 0.0))
    max_length = max(total_length - start_distance, 0.0)
    zone_length = clamp(zone.length_meters, 0.0, max_length)
    return start_distance, zone_length


def sync_zone_to_curve(track_object, zone):
    start_distance, zone_length = clamp_zone_span(track_object, zone)
    if values_differ(start_distance, zone.start_meters):
        assign_rna_property(zone, "start_meters", start_distance)
    if values_differ(zone_length, zone.length_meters):
        assign_rna_property(zone, "length_meters", zone_length)


def resolve_zone_span(zone):
    start_distance = max(zone.start_meters, 0.0)
    end_distance = start_distance + max(zone.length_meters, 0.0)
    return start_distance, end_distance


def zone_label(zone, index):
    if zone.name:
        return zone.name
    return f"{ZONE_TYPE_LABELS.get(zone.zone_type, zone.zone_type)} {index + 1:02d}"


def evaluate_control_response_factor(curve_mode, error_ratio):
    clamped_ratio = clamp(error_ratio, 0.0, 1.0)
    if curve_mode == "SMOOTH":
        return clamped_ratio * clamped_ratio * (3.0 - 2.0 * clamped_ratio)
    if curve_mode == "LOG":
        return log1p(9.0 * clamped_ratio) / log1p(10.0)
    return 1.0


def step_speed_toward_target(current_speed, target_speed, max_rate_mps2, delta_seconds, curve_mode):
    speed_delta = target_speed - current_speed
    if abs(speed_delta) <= SIMULATION_STOP_EPSILON:
        return target_speed

    if delta_seconds <= 0.0:
        return current_speed

    max_rate = max(max_rate_mps2, 0.0)
    if max_rate <= 1.0e-8:
        return current_speed

    error_reference = max(abs(target_speed), abs(current_speed), 0.25)
    error_ratio = clamp(abs(speed_delta) / error_reference, 0.0, 1.0)
    response_factor = evaluate_control_response_factor(curve_mode, error_ratio)
    allowed_change = max_rate * response_factor * delta_seconds
    if allowed_change <= 1.0e-8:
        return current_speed

    clamped_change = min(abs(speed_delta), allowed_change)
    return current_speed + clamped_change * (1.0 if speed_delta > 0.0 else -1.0)


def get_zone_influence(route, track_settings, front_distance, zone_span):
    """Exact fraction of the train footprint covered by the zone span."""
    if route["cyclic"] and route["total_length"] > 1.0e-8:
        total_length = route["total_length"]
        front = front_distance % total_length
        train_length = min(max(track_settings.train_length_meters, 0.0), total_length)
        if train_length <= 1.0e-6:
            return 1.0 if is_distance_in_span(front, zone_span) else 0.0
        occupied_spans = []
        rear = front - train_length
        if rear >= 0.0:
            occupied_spans.append((rear, front))
        else:
            occupied_spans.append((0.0, front))
            occupied_spans.append((total_length + rear, total_length))
        overlap = sum(
            max(min(span_end, zone_span[1]) - max(span_start, zone_span[0]), 0.0)
            for span_start, span_end in occupied_spans
        )
        return clamp(overlap / train_length, 0.0, 1.0)
    rear_distance, clamped_front_distance = get_train_occupied_span(route, track_settings, front_distance)
    footprint_length = clamped_front_distance - rear_distance
    if footprint_length <= 1.0e-6:
        return 1.0 if is_distance_in_span(clamped_front_distance, zone_span) else 0.0

    overlap = min(clamped_front_distance, zone_span[1]) - max(rear_distance, zone_span[0])
    return clamp(overlap / footprint_length, 0.0, 1.0)


def get_route_control_state(route, track_settings, route_zones, front_distance):
    """Resolve track hardware into a drive spec and a cap spec.

    Transports drive the train toward their target (either direction);
    brakes only cap speed and can never push. The strongest-influence
    transport drives; the most restrictive influencing brake caps.
    """
    drive_spec = None
    cap_spec = None

    for item in route_zones:
        zone = item["zone"]
        influence = get_zone_influence(route, track_settings, front_distance, get_route_zone_span(item))
        if influence <= 0.0:
            continue

        spec = {
            "kind": zone.zone_type,
            "target_speed": max(zone.target_speed_mps, 0.0),
            "acceleration_mps2": max(zone.max_acceleration_mps2, 0.0) * influence,
            "braking_mps2": max(zone.max_braking_mps2, 0.0) * influence,
            "curve_mode": "LINEAR",
            "influence": influence,
        }
        if zone.zone_type == "TRANSPORT":
            if drive_spec is None or (spec["influence"], spec["target_speed"]) > (
                drive_spec["influence"],
                drive_spec["target_speed"],
            ):
                drive_spec = spec
        else:
            if cap_spec is None or (spec["target_speed"], -spec["influence"]) < (
                cap_spec["target_speed"],
                -cap_spec["influence"],
            ):
                cap_spec = spec

    return {"drive": drive_spec, "cap": cap_spec}


def get_active_control_selection(context, route):
    """Return (compiled program, selected node) for a visible control editor."""
    screen = getattr(context, "screen", None)
    if screen is None:
        return None, None
    derived = get_route_derived_data(route)
    programs_by_tree = {
        program["control_tree"].as_pointer(): program
        for program in derived["programs"]
        if program.get("control_tree") is not None
    }
    for area in screen.areas:
        if area.type != "NODE_EDITOR":
            continue
        space = area.spaces.active
        tree = getattr(space, "edit_tree", None) or getattr(space, "node_tree", None)
        if tree is None or tree.bl_idname != CONTROL_TREE_IDNAME:
            continue
        node = tree.nodes.active
        if node is None or not node.select:
            continue
        program = programs_by_tree.get(tree.as_pointer())
        if program is not None:
            return program, node
    return None, None


def get_control_selection_signature(program, node):
    if program is None or node is None:
        return None
    values = [program["key"], node.as_pointer(), node.bl_idname]
    for attribute in (
        "offset_meters", "speed_mps", "acceleration_mps2", "braking_mps2",
        "duration_seconds", "comparison", "channel", "value",
    ):
        if hasattr(node, attribute):
            value = getattr(node, attribute)
            values.append(round(value, 6) if isinstance(value, float) else value)
    return tuple(values)


def get_overlay_route_signature(route, scene):
    """Route-level overlay inputs: blocks, sensors, start cursor, trajectory."""
    derived = get_route_derived_data(route)
    program_signature = tuple(
        (
            program["key"],
            round(program["span"][0], 4),
            round(program["span"][1], 4),
            tuple(
                (action["kind"], round(action.get("target", 0.0), 4))
                for action in program["actions"]
            ),
        )
        for program in derived["programs"]
    )
    sensor_signature = tuple((sensor["key"], round(sensor["position"], 4)) for sensor in derived["sensors"])

    trajectory = SIMULATION_TRAJECTORY_CACHE
    trajectory_signature = None
    if trajectory is not None and route["entries"]:
        if trajectory["key"][0] == route["entries"][0]["object"].as_pointer():
            trajectory_signature = (trajectory["key"], trajectory["cycle_length"] is not None)

    scene_settings = getattr(scene, "coaster_mixer_scene", None)
    start_meters = round(scene_settings.simulation_start_route_meters, 4) if scene_settings else 0.0
    return (program_signature, sensor_signature, trajectory_signature, start_meters)


def get_overlay_draw_cache_signature(route, edit_piece, edit_settings, scene, control_selection=None):
    if edit_piece is None or edit_settings is None:
        return None

    if not route["entries"] and len(edit_settings.zones) == 0:
        return None

    zone_signature = []
    for zone in edit_settings.zones:
        zone_signature.append((
            zone.zone_type,
            zone.name,
            round(zone.start_meters, 6),
            round(zone.length_meters, 6),
            round(zone.minimum_speed_mps, 6),
            round(zone.target_speed_mps, 6),
            round(zone.max_acceleration_mps2, 6),
            round(zone.max_braking_mps2, 6),
        ))

    return (
        route["key"],
        edit_piece.as_pointer(),
        get_curve_cache_signature(edit_piece),
        edit_settings.active_zone_index,
        tuple(zone_signature),
        get_overlay_route_signature(route, scene),
        get_control_selection_signature(*(control_selection or (None, None))),
    )


def build_route_span_polylines(route, start_distance, end_distance):
    """World-space polylines covering a route-space span across pieces."""
    polylines = []
    for entry in route["entries"]:
        overlap_start = max(start_distance, entry["start"])
        overlap_end = min(end_distance, entry["start"] + entry["length"])
        if overlap_end - overlap_start <= 1.0e-6:
            continue
        local_a = route_to_piece_distance(entry, overlap_start)
        local_b = route_to_piece_distance(entry, overlap_end)
        points = build_segment_draw_points(entry["cache"], min(local_a, local_b), max(local_a, local_b))
        if len(points) >= 2:
            polylines.append(points)
    return polylines


def sample_route_location(route, distance):
    location, _rotation = sample_route_placement(route, wrap_route_distance(route, distance))
    return location


def build_start_arrow_lines(route, distance):
    """A lifted chevron at the start cursor pointing along travel direction."""
    location, rotation = sample_route_placement(route, wrap_route_distance(route, distance))
    if location is None or rotation is None:
        return []

    forward = rotation @ Vector((0.0, 1.0, 0.0))
    right = rotation @ Vector((1.0, 0.0, 0.0))
    up = rotation @ Vector((0.0, 0.0, 1.0))
    base = location + up * 0.4
    tip = base + forward * 1.6
    return [
        [base, tip],
        [tip - forward * 0.6 + right * 0.45, tip],
        [tip - forward * 0.6 - right * 0.45, tip],
    ]


def find_trajectory_zone_entry_speed(route, route_start):
    """Speed at which the simulated train first crosses a route distance.

    Only answered once the trajectory cycle is known, so the result (and the
    overlay cache built from it) is stable.
    """
    trajectory = SIMULATION_TRAJECTORY_CACHE
    if trajectory is None or trajectory["cycle_length"] is None or not route["entries"]:
        return None
    if trajectory["key"][0] != route["entries"][0]["object"].as_pointer():
        return None

    fronts = trajectory["fronts"]
    speeds = trajectory["speeds"]
    for index in range(1, len(fronts)):
        if fronts[index - 1] < route_start <= fronts[index]:
            return max(speeds[index - 1], speeds[index])
    return None


def build_friction_slowdown_polylines(route, route_zones):
    """Dynamic slowdown portion of each friction brake (entry speed dependent)."""
    polylines = []
    for item in route_zones:
        zone = item["zone"]
        if zone.zone_type != "FRICTION_BRAKE":
            continue
        rate = max(zone.max_braking_mps2, 1.0e-3)
        controllable_speed = max(zone.target_speed_mps, 0.0)
        entry_speed = find_trajectory_zone_entry_speed(route, item["route_start"])
        if entry_speed is None or entry_speed <= controllable_speed:
            continue
        slowdown_length = (entry_speed * entry_speed - controllable_speed * controllable_speed) / (2.0 * rate)
        slowdown_end = min(item["route_start"] + slowdown_length, item["route_end"])
        polylines.extend(build_route_span_polylines(route, item["route_start"], slowdown_end))
    return polylines


def build_overlay_draw_data(route, edit_piece, edit_settings, scene, control_selection=None):
    route_polylines = [entry["cache"]["points"] for entry in route["entries"]]
    if all(entry["object"] != edit_piece for entry in route["entries"]):
        # The edited piece is not on the active route (e.g. a storage track
        # behind a thrown switch); still show its line for context.
        route_polylines.append(build_curve_cache(edit_piece)["points"])

    draw_data = {
        "route_polylines": route_polylines,
        "active_points": [],
        "transport_polylines": [],
        "brake_polylines": [],
        "trim_polylines": [],
        "seam_points": [],
        "hold_points": [],
        "block_span_polylines": [],
        "slowdown_polylines": [],
        "sensor_points": [],
        "start_arrow_lines": [],
        "control_transport_polylines": [],
        "control_brake_polylines": [],
        "control_logic_polylines": [],
        "control_position_points": [],
        "control_hold_points": [],
    }

    # Route-level indicators: block spans + move points, sensors, friction
    # slowdown segments, and the start cursor arrow.
    if route["entries"]:
        derived = get_route_derived_data(route)
        for program in derived["programs"]:
            draw_data["block_span_polylines"].extend(
                build_route_span_polylines(route, program["span"][0], program["span"][1])
            )
            for action in program["actions"]:
                if action["kind"] == "WAIT_POSITION":
                    location = sample_route_location(route, action["target"])
                    if location is not None:
                        draw_data["hold_points"].append(location)
        for sensor in derived["sensors"]:
            location = sample_route_location(route, sensor["position"])
            if location is not None:
                draw_data["sensor_points"].append(location)
        draw_data["slowdown_polylines"] = build_friction_slowdown_polylines(route, derived["zones"])

        scene_settings = getattr(scene, "coaster_mixer_scene", None)
        if scene_settings is not None:
            draw_data["start_arrow_lines"] = build_start_arrow_lines(
                route, scene_settings.simulation_start_route_meters
            )

        program, node = control_selection or (None, None)
        if program is not None and node is not None:
            if node.bl_idname == "COASTERMIXER_ND_set_transport":
                for span in program["transport_spans"]:
                    draw_data["control_transport_polylines"].extend(
                        build_route_span_polylines(route, span[0], span[1])
                    )
            elif node.bl_idname in {
                "COASTERMIXER_ND_set_brake", "COASTERMIXER_ND_set_brake_hold",
                "COASTERMIXER_ND_release_brake",
            }:
                for span in program["brake_spans"]:
                    draw_data["control_brake_polylines"].extend(
                        build_route_span_polylines(route, span[0], span[1])
                    )
                if node.bl_idname == "COASTERMIXER_ND_set_brake_hold":
                    target = clamp(
                        program["span"][0] + node.offset_meters,
                        program["span"][0],
                        program["span"][1],
                    )
                    location = sample_route_location(route, target)
                    if location is not None:
                        draw_data["control_hold_points"].append(location)
            elif node.bl_idname == "COASTERMIXER_ND_wait_position":
                target = clamp(
                    program["span"][0] + node.offset_meters,
                    program["span"][0],
                    program["span"][1],
                )
                location = sample_route_location(route, target)
                if location is not None:
                    draw_data["control_position_points"].append(location)
            else:
                draw_data["control_logic_polylines"].extend(
                    build_route_span_polylines(route, program["span"][0], program["span"][1])
                )
                if node.bl_idname == "COASTERMIXER_ND_dispatch":
                    location = sample_route_location(route, program["span"][1])
                    if location is not None:
                        draw_data["control_position_points"].append(location)

    if len(edit_settings.zones) == 0:
        return draw_data

    active_index = get_clamped_active_zone_index(edit_settings)
    cache = build_curve_cache(edit_piece)
    for zone_index, zone in enumerate(edit_settings.zones):
        start_distance, end_distance = resolve_zone_span(zone)
        points = build_segment_draw_points(cache, start_distance, end_distance)
        if zone.zone_type == "TRANSPORT":
            draw_data["transport_polylines"].append(points)
        elif zone.zone_type == "TRIM_BRAKE":
            draw_data["trim_polylines"].append(points)
        else:
            draw_data["brake_polylines"].append(points)

        if zone_index == active_index:
            draw_data["active_points"] = points
            start_location, _tangent, _cache = sample_curve_cache_at_distance(cache, start_distance)
            end_location, _tangent, _cache = sample_curve_cache_at_distance(cache, end_distance)
            draw_data["seam_points"] = [point for point in (start_location, end_location) if point is not None]

    return draw_data


def get_overlay_draw_data(track_object, edit_piece, edit_settings, scene):
    if track_object is None or edit_piece is None or edit_settings is None:
        return None

    object_key = track_object.as_pointer()
    route = get_resolved_route(track_object)
    control_selection = get_active_control_selection(bpy.context, route)
    signature = get_overlay_draw_cache_signature(
        route, edit_piece, edit_settings, scene, control_selection
    )
    if signature is None:
        OVERLAY_DRAW_CACHE_BY_OBJECT.pop(object_key, None)
        return None

    cached_entry = OVERLAY_DRAW_CACHE_BY_OBJECT.get(object_key)
    if cached_entry is not None and cached_entry["signature"] == signature:
        return cached_entry["draw_data"]

    draw_data = build_overlay_draw_data(route, edit_piece, edit_settings, scene, control_selection)
    OVERLAY_DRAW_CACHE_BY_OBJECT[object_key] = {
        "signature": signature,
        "draw_data": draw_data,
    }
    return draw_data


def draw_viewport_overlay():
    context = bpy.context
    scene = getattr(context, "scene", None)
    scene_settings = getattr(scene, "coaster_mixer_scene", None)
    screen = getattr(context, "screen", None)
    if (
        scene_settings is not None
        and scene_settings.hide_overlays_while_playing
        and screen is not None
        and getattr(screen, "is_animation_playing", False)
    ):
        return

    track_object = resolve_active_track_object(context)
    if track_object is None:
        return

    edit_piece, edit_settings = resolve_edit_track_settings(context)
    draw_data = get_overlay_draw_data(track_object, edit_piece, edit_settings, scene)
    if draw_data is None:
        return

    gpu.state.blend_set("ALPHA")
    for polyline in draw_data["route_polylines"]:
        draw_polyline(polyline, TRACK_LINE_COLOR, 2.0)
    if scene_settings is None or scene_settings.show_block_overlays:
        for polyline in draw_data["block_span_polylines"]:
            draw_polyline(polyline, BLOCK_GROUP_COLOR, 12.0)
        draw_points(draw_data["hold_points"], HOLD_POINT_COLOR, 14.0)
    if scene_settings is None or scene_settings.show_hardware_overlays:
        for polyline in draw_data["transport_polylines"]:
            draw_polyline(polyline, TRANSPORT_ZONE_COLOR, 8.0)
        for polyline in draw_data["brake_polylines"]:
            draw_polyline(polyline, BRAKE_ZONE_COLOR, 8.0)
        for polyline in draw_data["trim_polylines"]:
            draw_polyline(polyline, TRIM_ZONE_COLOR, 8.0)
        for polyline in draw_data["slowdown_polylines"]:
            draw_polyline(polyline, BRAKE_SLOWDOWN_COLOR, 8.0)
    if scene_settings is None or scene_settings.show_control_overlays:
        for polyline in draw_data["control_logic_polylines"]:
            draw_polyline(polyline, CONTROL_LOGIC_COLOR, 22.0)
        for polyline in draw_data["control_transport_polylines"]:
            draw_polyline(polyline, CONTROL_TRANSPORT_COLOR, 18.0)
        for polyline in draw_data["control_brake_polylines"]:
            draw_polyline(polyline, CONTROL_BRAKE_COLOR, 18.0)
        draw_points(draw_data["control_position_points"], CONTROL_POSITION_COLOR, 20.0)
        draw_points(draw_data["control_hold_points"], CONTROL_BRAKE_COLOR, 22.0)
    draw_polyline(draw_data["active_points"], ACTIVE_ZONE_COLOR, 12.0)
    for line in draw_data["start_arrow_lines"]:
        draw_polyline(line, START_MARKER_COLOR, 4.0)
    draw_points(draw_data["seam_points"], SEAM_MARKER_COLOR, 10.0)
    draw_points(draw_data["sensor_points"], SENSOR_MARKER_COLOR, 12.0)
    gpu.state.blend_set("NONE")


def ensure_viewport_draw_handler():
    global VIEWPORT_DRAW_HANDLER
    if VIEWPORT_DRAW_HANDLER is not None:
        return

    VIEWPORT_DRAW_HANDLER = bpy.types.SpaceView3D.draw_handler_add(
        draw_viewport_overlay,
        (),
        "WINDOW",
        "POST_VIEW",
    )


def remove_viewport_draw_handler():
    global VIEWPORT_DRAW_HANDLER
    if VIEWPORT_DRAW_HANDLER is None:
        return

    bpy.types.SpaceView3D.draw_handler_remove(VIEWPORT_DRAW_HANDLER, "WINDOW")
    VIEWPORT_DRAW_HANDLER = None


def resolve_active_track_object(context):
    scene_settings = getattr(context.scene, "coaster_mixer_scene", None)
    if scene_settings is None:
        return None

    track_object = scene_settings.track_object
    if track_object is None or track_object.type != "CURVE":
        return None

    return track_object


def resolve_active_track_settings(context):
    track_object = resolve_active_track_object(context)
    if track_object is None:
        return None, None

    track_settings = track_object.coaster_mixer_track
    return track_object, track_settings


def resolve_edit_piece(context):
    # Zones and connections are edited on the active viewport curve, falling
    # back to the coaster root when no curve is selected.
    active_object = getattr(context, "object", None)
    if active_object is not None and active_object.type == "CURVE":
        return active_object
    return resolve_active_track_object(context)


def resolve_edit_track_settings(context):
    edit_piece = resolve_edit_piece(context)
    if edit_piece is None:
        return None, None
    return edit_piece, edit_piece.coaster_mixer_track


def remove_legacy_follow_path_constraint(track_object, empty_object):
    # The prototype drove empties with a Follow Path constraint, which
    # evaluates the curve parametrically instead of by arc length. Remove it
    # so it cannot fight the placement drivers.
    for constraint in list(empty_object.constraints):
        if constraint.type != "FOLLOW_PATH":
            continue
        if constraint.name == DRIVEN_EMPTY_CONSTRAINT_NAME or constraint.target == track_object:
            empty_object.constraints.remove(constraint)


def get_driver_fcurve_indexed(owner, data_path, array_index):
    animation_data = getattr(owner, "animation_data", None)
    drivers = getattr(animation_data, "drivers", None)
    if drivers is None:
        return None

    for fcurve in drivers:
        if fcurve.data_path == data_path and fcurve.array_index == array_index:
            return fcurve

    return None


def get_action_fcurve(owner, data_path):
    animation_data = getattr(owner, "animation_data", None)
    action = getattr(animation_data, "action", None)
    if action is None:
        return None
    fcurves = getattr(action, "fcurves", None)
    if fcurves is not None:
        return fcurves.find(data_path)

    action_slot = getattr(animation_data, "action_slot", None)
    if action_slot is None:
        return None

    layers = getattr(action, "layers", None)
    if layers is None:
        return None

    for layer in layers:
        strips = getattr(layer, "strips", None)
        if strips is None:
            continue

        for strip in strips:
            channelbag_fn = getattr(strip, "channelbag", None)
            if channelbag_fn is None:
                continue

            channelbag = channelbag_fn(action_slot, ensure=False)
            if channelbag is None:
                continue

            channelbag_fcurves = getattr(channelbag, "fcurves", None)
            if channelbag_fcurves is None:
                continue

            fcurve = channelbag_fcurves.find(data_path)
            if fcurve is not None:
                return fcurve

    return None


def get_action_fcurve_owner(owner, data_path):
    animation_data = getattr(owner, "animation_data", None)
    action = getattr(animation_data, "action", None)
    if action is None:
        return None, None

    fcurves = getattr(action, "fcurves", None)
    if fcurves is not None:
        return fcurves.find(data_path), fcurves

    action_slot = getattr(animation_data, "action_slot", None)
    if action_slot is None:
        return None, None

    layers = getattr(action, "layers", None)
    if layers is None:
        return None, None

    for layer in layers:
        strips = getattr(layer, "strips", None)
        if strips is None:
            continue

        for strip in strips:
            channelbag_fn = getattr(strip, "channelbag", None)
            if channelbag_fn is None:
                continue

            channelbag = channelbag_fn(action_slot, ensure=False)
            if channelbag is None:
                continue

            channelbag_fcurves = getattr(channelbag, "fcurves", None)
            if channelbag_fcurves is None:
                continue

            fcurve = channelbag_fcurves.find(data_path)
            if fcurve is not None:
                return fcurve, channelbag_fcurves

    return None, None


def clear_action_fcurve(owner, data_path):
    fcurve, fcurve_collection = get_action_fcurve_owner(owner, data_path)
    if fcurve is None or fcurve_collection is None:
        return False

    fcurve_collection.remove(fcurve)
    return True


def insert_dense_fcurve_keyframes(owner, data_path, frame_values):
    if not frame_values:
        return None

    animation_data = owner.animation_data_create()
    action = getattr(animation_data, "action", None)
    if action is None:
        action = bpy.data.actions.new(name=f"{owner.name}Action")
        animation_data.action = action

    fcurves = getattr(action, "fcurves", None)
    if fcurves is None:
        return None

    fcurve = fcurves.find(data_path)
    if fcurve is None:
        fcurve = fcurves.new(data_path=data_path)

    try:
        keyframe_points = fcurve.keyframe_points
        keyframe_points.add(len(frame_values))
        if len(keyframe_points) != len(frame_values):
            return None

        coordinates = []
        for frame, value in frame_values:
            coordinates.extend((float(frame), float(value)))
        keyframe_points.foreach_set("co", coordinates)
        set_fcurve_linear(fcurve)
        keyframe_points.update()
        fcurve.update()
    except Exception:
        clear_action_fcurve(owner, data_path)
        return None

    return fcurve


def set_fcurve_linear(fcurve):
    if fcurve is None:
        return

    for keyframe in fcurve.keyframe_points:
        keyframe.interpolation = "LINEAR"


def has_baked_path_animation(track_object):
    return get_action_fcurve(track_object, TRAIN_FRONT_METERS_DATA_PATH) is not None


def configure_placement_driver(fcurve, track_object, empty_object, channel_index):
    driver = fcurve.driver
    driver.type = "SCRIPTED"
    driver.use_self = True
    driver.expression = f"{PLACEMENT_DRIVER_FUNCTION_NAME}(self, front, offset, {channel_index})"

    while driver.variables:
        driver.variables.remove(driver.variables[0])

    front_variable = driver.variables.new()
    front_variable.name = "front"
    front_variable.type = "SINGLE_PROP"
    front_target = front_variable.targets[0]
    front_target.id = track_object
    front_target.data_path = TRAIN_FRONT_METERS_DATA_PATH

    offset_variable = driver.variables.new()
    offset_variable.name = "offset"
    offset_variable.type = "SINGLE_PROP"
    offset_target = offset_variable.targets[0]
    offset_target.id = empty_object
    offset_target.data_path = FOLLOWER_OFFSET_DATA_PATH


def placement_driver_is_valid(fcurve):
    driver = getattr(fcurve, "driver", None)
    if driver is None or driver.type != "SCRIPTED" or not driver.use_self:
        return False
    return driver.expression.startswith(f"{PLACEMENT_DRIVER_FUNCTION_NAME}(self")


def remove_placement_channel_drivers(empty_object):
    """Remove only Coaster Mixer legacy drivers, preserving user animation."""
    for data_path in ("location", "rotation_euler"):
        for axis_index in range(3):
            fcurve = get_driver_fcurve_indexed(empty_object, data_path, axis_index)
            if fcurve is not None and placement_driver_is_valid(fcurve):
                try:
                    empty_object.driver_remove(data_path, axis_index)
                except (TypeError, ValueError, RuntimeError):
                    pass


def ensure_follower_drivers(track_object, empty_object, offset_meters=None):
    """Attach an empty to batched arc-length placement."""
    follower_settings = empty_object.coaster_mixer_follower
    if follower_settings.track_object != track_object:
        assign_rna_property(follower_settings, "track_object", track_object)
    if offset_meters is not None and values_differ(follower_settings.offset_meters, offset_meters):
        assign_rna_property(follower_settings, "offset_meters", offset_meters)

    remove_legacy_follow_path_constraint(track_object, empty_object)

    if empty_object.rotation_mode != "XYZ":
        empty_object.rotation_mode = "XYZ"

    # Older setups used six scripted Python drivers per follower. Placement
    # is now batched once per frame, so remove those channel drivers.
    remove_placement_channel_drivers(empty_object)
    empty_object["coaster_mixer_batched_placement"] = True
    place_track_followers(track_object)


def remove_follower_drivers(empty_object):
    remove_placement_channel_drivers(empty_object)

    follower_settings = empty_object.coaster_mixer_follower
    if follower_settings.track_object is not None:
        assign_rna_property(follower_settings, "track_object", None)
    if "coaster_mixer_batched_placement" in empty_object:
        del empty_object["coaster_mixer_batched_placement"]


def collect_track_followers(track_object):
    driven_empty_object = track_object.coaster_mixer_track.driven_empty_object
    followers = [
        object_ref
        for object_ref in bpy.data.objects
        if (
            object_ref.type == "EMPTY"
            and object_ref.coaster_mixer_follower.track_object == track_object
            and object_ref != driven_empty_object
            and not object_ref.get("coaster_mixer_camera_target", False)
        )
    ]
    followers.sort(key=lambda object_ref: (object_ref.coaster_mixer_follower.offset_meters, object_ref.name))
    return followers


def collect_track_placement_objects(track_object):
    """All batched followers, including camera look-ahead helpers."""
    return [
        object_ref
        for object_ref in bpy.data.objects
        if (
            object_ref.type == "EMPTY"
            and object_ref.coaster_mixer_follower.track_object == track_object
        )
    ]


def collect_ride_cameras(track_object):
    """Return cameras authored by Coaster Mixer for this root track."""
    cameras = []
    for camera_object in bpy.data.objects:
        if camera_object.type != "CAMERA":
            continue
        camera_settings = getattr(camera_object, "coaster_mixer_camera", None)
        if camera_settings is not None and camera_settings.track_object == track_object:
            cameras.append(camera_object)
            continue
        if camera_object.parent is None or camera_object.parent.type != "EMPTY":
            continue
        aim_constraint = camera_object.constraints.get("Coaster Mixer Look Ahead")
        if aim_constraint is not None and camera_object.parent.coaster_mixer_follower.track_object == track_object:
            # Migrate cameras created before dedicated camera settings existed.
            if camera_settings is not None and camera_settings.track_object is None:
                target_object = aim_constraint.target
                mount_offset = camera_object.parent.coaster_mixer_follower.offset_meters
                target_offset = (
                    target_object.coaster_mixer_follower.offset_meters
                    if target_object is not None and target_object.type == "EMPTY"
                    else mount_offset - 5.0
                )
                assign_rna_property(camera_settings, "track_object", track_object)
                assign_rna_property(camera_settings, "mount_object", camera_object.parent)
                if target_object is not None and target_object.type == "EMPTY":
                    assign_rna_property(camera_settings, "target_object", target_object)
                assign_rna_property(camera_settings, "offset_xyz", tuple(camera_object.location))
                assign_rna_property(
                    camera_settings,
                    "look_ahead_meters",
                    max(mount_offset - target_offset, 0.1),
                )
                assign_rna_property(
                    camera_settings,
                    "target_vertical_offset_meters",
                    camera_object.location.z,
                )
            cameras.append(camera_object)
    cameras.sort(key=lambda camera_object: camera_object.name)
    return cameras


def get_camera_shake_offset(track_object, camera_object, mount_rotation, route, front_meters):
    """Return deterministic, local camera jitter driven by speed and track load."""
    camera_settings = getattr(camera_object, "coaster_mixer_camera", None)
    if camera_settings is None or not camera_settings.shake_enabled:
        return Vector((0.0, 0.0, 0.0))

    scene = getattr(bpy.context, "scene", None)
    scene_settings = getattr(scene, "coaster_mixer_scene", None)
    speed = 0.0
    if scene_settings is not None and scene_settings.track_object == track_object:
        speed = max(scene_settings.simulation_current_speed_mps, 0.0)
    if speed <= SIMULATION_STOP_EPSILON:
        return Vector((0.0, 0.0, 0.0))

    curvature_acceleration = sample_route_curvature_vector(route, front_meters) * (speed * speed)
    mount_side = mount_rotation @ Vector((1.0, 0.0, 0.0))
    mount_up = mount_rotation @ Vector((0.0, 0.0, 1.0))
    lateral_g = abs(curvature_acceleration.dot(mount_side)) / GRAVITY_ACCELERATION
    apparent_acceleration = curvature_acceleration - Vector((0.0, 0.0, -GRAVITY_ACCELERATION))
    vertical_g = abs(apparent_acceleration.dot(mount_up)) / GRAVITY_ACCELERATION
    load_change = abs(vertical_g - 1.0)

    amplitude = camera_settings.shake_factor * (
        0.006 * clamp(speed / 25.0, 0.0, 2.0)
        + 0.004 * clamp(lateral_g, 0.0, 3.0)
        + 0.003 * clamp(load_change, 0.0, 3.0)
    )
    amplitude = min(amplitude, 0.08)
    phase = front_meters
    return Vector(
        (
            amplitude * (0.65 * sin(phase * 19.7) + 0.35 * sin(phase * 43.1 + 0.8)),
            amplitude * 0.35 * sin(phase * 31.3 + 2.1),
            amplitude * (0.7 * sin(phase * 23.9 + 1.4) + 0.3 * sin(phase * 52.7)),
        )
    )


def place_ride_cameras(track_object, placement_by_object, route, front_meters):
    """Aim cameras at their look-ahead target while preserving mount bank."""
    for camera_object in collect_ride_cameras(track_object):
        if camera_object.parent is None:
            continue
        aim_constraint = camera_object.constraints.get("Coaster Mixer Look Ahead")
        if aim_constraint is None or aim_constraint.target is None:
            continue
        mount_sample = placement_by_object.get(camera_object.parent.as_pointer())
        target_sample = placement_by_object.get(aim_constraint.target.as_pointer())
        if mount_sample is None or target_sample is None:
            continue

        # The old Track To constraint used the target empty's Z axis for its
        # roll. Keep it only as a durable target reference; orientation is
        # solved here from target position + the mounted car's banked up axis.
        aim_constraint.influence = 0.0
        mount_location, mount_rotation = mount_sample
        target_location, _target_rotation = target_sample
        camera_settings = getattr(camera_object, "coaster_mixer_camera", None)
        if camera_settings is not None and camera_settings.track_object == track_object:
            base_offset = Vector(camera_settings.offset_xyz)
            camera_object.location = base_offset + get_camera_shake_offset(
                track_object,
                camera_object,
                mount_rotation,
                route,
                front_meters,
            )
        camera_location = mount_location + mount_rotation @ camera_object.location
        forward = target_location - camera_location
        if forward.length_squared <= 1.0e-10:
            continue
        forward.normalize()

        mount_up = mount_rotation @ Vector((0.0, 0.0, 1.0))
        camera_up = mount_up - forward * mount_up.dot(forward)
        if camera_up.length_squared <= 1.0e-10:
            mount_side = mount_rotation @ Vector((1.0, 0.0, 0.0))
            camera_up = mount_side.cross(forward)
        camera_up.normalize()
        camera_back = -forward  # Blender cameras look along local -Z.
        camera_right = camera_up.cross(camera_back).normalized()
        camera_up = camera_back.cross(camera_right).normalized()
        world_rotation = Matrix((camera_right, camera_up, camera_back)).transposed().to_quaternion()
        local_rotation = mount_rotation.inverted() @ world_rotation
        camera_object.rotation_euler = local_rotation.to_euler("XYZ")


def place_track_followers(track_object, front_meters=None):
    """Resolve the route once and place every follower once for this frame."""
    if track_object is None or track_object.type != "CURVE":
        return
    route = get_resolved_route(track_object)
    if route["total_length"] <= 1.0e-8:
        return
    if front_meters is None:
        front_meters = track_object.coaster_mixer_track.train_front_route_meters

    samples_by_offset = {}
    placement_by_object = {}
    for follower_object in collect_track_placement_objects(track_object):
        # Lazily migrate followers from files saved with the driver-based rig.
        if not follower_object.get("coaster_mixer_batched_placement", False):
            remove_placement_channel_drivers(follower_object)
            follower_object["coaster_mixer_batched_placement"] = True
        offset = follower_object.coaster_mixer_follower.offset_meters
        sample = samples_by_offset.get(offset)
        if sample is None:
            distance = wrap_route_distance(route, front_meters - offset)
            location, rotation = sample_route_placement(route, distance)
            if location is None or rotation is None:
                continue
            sample = (location, rotation, rotation.to_euler("XYZ"))
            samples_by_offset[offset] = sample
        if follower_object.rotation_mode != "XYZ":
            follower_object.rotation_mode = "XYZ"
        vertical_offset = follower_object.coaster_mixer_follower.vertical_offset_meters
        follower_location = sample[0] + sample[1] @ Vector((0.0, 0.0, vertical_offset))
        follower_object.location = follower_location
        follower_object.rotation_euler = sample[2]
        placement_by_object[follower_object.as_pointer()] = (follower_location, sample[1])
    place_ride_cameras(track_object, placement_by_object, route, front_meters)


def ensure_driven_empty_path_setup(track_object, track_settings):
    if track_object is None or track_settings is None:
        return None, "No active track"

    driven_empty_object = track_settings.driven_empty_object
    if driven_empty_object is None:
        return None, "No driven empty selected"

    if driven_empty_object.type != "EMPTY":
        return None, "Driven object must be an empty"

    ensure_follower_drivers(track_object, driven_empty_object, offset_meters=0.0)
    tag_track_placement_update(track_object)
    return True, "Driven empty placed by arc-length drivers"


def get_follower_setup_status(track_object, empty_object):
    if track_object is None:
        return "No active track"

    if empty_object is None:
        return "Select a driven empty"

    follower_settings = empty_object.coaster_mixer_follower
    if follower_settings.track_object != track_object:
        return "Empty is not attached to this track"

    if empty_object.parent is not None:
        return "Batched placement active (warning: parent offsets placement)"

    return "Batched placement active"


def get_scene_fps(scene):
    render = scene.render
    fps_base = render.fps_base if render.fps_base > 1.0e-8 else 1.0
    fps = render.fps / fps_base if render.fps > 0 else 24.0
    return fps if fps > 1.0e-8 else 24.0


def get_simulation_bake_frame_range(scene):
    if scene.use_preview_range:
        return scene.frame_preview_start, scene.frame_preview_end, "Preview"
    return scene.frame_start, scene.frame_end, "Scene"


def is_distance_in_span(distance, span):
    return span is not None and span[0] - SIMULATION_STOP_EPSILON <= distance <= span[1] + SIMULATION_STOP_EPSILON


def do_spans_overlap(span_a, span_b):
    if span_a is None or span_b is None:
        return False
    return span_a[0] <= span_b[1] + SIMULATION_STOP_EPSILON and span_b[0] <= span_a[1] + SIMULATION_STOP_EPSILON


def get_train_occupied_span(route, track_settings, front_distance):
    total_length = route["total_length"]
    clamped_front_distance = clamp(front_distance, 0.0, total_length)
    rear_distance = max(clamped_front_distance - max(track_settings.train_length_meters, 0.0), 0.0)
    return rear_distance, clamped_front_distance


def get_train_force_state(route, track_settings, front_distance, current_speed=0.0):
    total_length = route["total_length"]
    if total_length <= 1.0e-8:
        return {
            "mass": max(track_settings.train_weight_kilograms, 1.0),
            "drive_force": 0.0,
            "rolling_force": 0.0,
            "average_sine": 0.0,
            "average_cosine": 1.0,
            "average_normal_g": 1.0,
        }

    rear_distance, clamped_front_distance = get_train_occupied_span(route, track_settings, front_distance)
    footprint_length = clamped_front_distance - rear_distance

    if footprint_length <= 1.0e-6:
        average_sine = clamp(sample_route_tangent_z(route, clamped_front_distance), -1.0, 1.0)
        average_cosine = max(1.0 - average_sine * average_sine, 0.0) ** 0.5
    else:
        delta_z, horizontal_length = get_route_span_profile(route, rear_distance, clamped_front_distance)
        average_sine = clamp(delta_z / footprint_length, -1.0, 1.0)
        average_cosine = clamp(horizontal_length / footprint_length, 0.0, 1.0)

    mass = max(track_settings.train_weight_kilograms, 1.0)
    weight_force = mass * GRAVITY_ACCELERATION
    drive_force = -weight_force * average_sine

    # Rolling losses scale with wheel load, not merely the gravity component
    # perpendicular to the rail. At speed, track curvature adds (or removes)
    # centripetal load. Sampling along the train also captures simultaneous
    # loading of cars in different parts of a valley, crest, or inversion.
    sample_count = 7
    train_length = max(track_settings.train_length_meters, 0.0)
    gravity_vector = Vector((0.0, 0.0, -GRAVITY_ACCELERATION))
    normal_acceleration_sum = 0.0
    for sample_index in range(sample_count):
        factor = sample_index / (sample_count - 1) if sample_count > 1 else 1.0
        sample_distance = clamped_front_distance - train_length * factor
        sample_distance = wrap_route_distance(route, sample_distance)
        tangent = sample_route_tangent(route, sample_distance)
        curvature = sample_route_curvature_vector(route, sample_distance)
        gravity_perpendicular = gravity_vector - tangent * gravity_vector.dot(tangent)
        constraint_acceleration = curvature * (current_speed * current_speed) - gravity_perpendicular
        normal_acceleration_sum += constraint_acceleration.length

    average_normal_acceleration = normal_acceleration_sum / sample_count
    average_normal_g = average_normal_acceleration / GRAVITY_ACCELERATION
    normal_force = mass * average_normal_acceleration
    rolling_force = max(track_settings.friction_coefficient, 0.0) * normal_force

    return {
        "mass": mass,
        "drive_force": drive_force,
        "rolling_force": rolling_force,
        "average_sine": average_sine,
        "average_cosine": average_cosine,
        "average_normal_g": average_normal_g,
    }


def get_passive_acceleration(route, track_settings, front_distance, current_speed):
    force_state = get_train_force_state(route, track_settings, front_distance, current_speed)
    mass = force_state["mass"]
    drive_force = force_state["drive_force"]
    rolling_force = force_state["rolling_force"]

    if abs(current_speed) > SIMULATION_STOP_EPSILON:
        direction = 1.0 if current_speed > 0.0 else -1.0
        drag_force = (
            0.5
            * AIR_DENSITY_KG_M3
            * max(track_settings.drag_coefficient, 0.0)
            * max(track_settings.frontal_area_m2, 0.0)
            * current_speed
            * current_speed
        )
        net_force = drive_force - direction * (rolling_force + drag_force)
        return net_force / mass

    if abs(drive_force) > rolling_force:
        direction = 1.0 if drive_force > 0.0 else -1.0
        return (drive_force - direction * rolling_force) / mass

    return 0.0


def get_controlled_next_speed(route, track_settings, control_state, front_distance, current_speed, delta_seconds):
    """Combine drive and cap hardware into the next free-running speed.

    The drive (or gravity, when no transport acts) proposes a speed; an
    active brake then caps it. Brakes only remove speed: a stopped train
    under a station brake can still be pushed out by drive tires.
    """
    drive_spec = control_state["drive"]
    cap_spec = control_state["cap"]

    if drive_spec is not None:
        drive_rate = (
            drive_spec["acceleration_mps2"]
            if drive_spec["target_speed"] >= current_speed
            else drive_spec["braking_mps2"]
        )
        next_speed = step_speed_toward_target(
            current_speed,
            drive_spec["target_speed"],
            drive_rate,
            delta_seconds,
            drive_spec["curve_mode"],
        )
    else:
        passive_acceleration = get_passive_acceleration(route, track_settings, front_distance, current_speed)
        next_speed = current_speed + passive_acceleration * delta_seconds

    if cap_spec is not None and abs(next_speed) > cap_spec["target_speed"] + SIMULATION_STOP_EPSILON:
        cap_target = cap_spec["target_speed"] if next_speed >= 0.0 else -cap_spec["target_speed"]
        capped_speed = step_speed_toward_target(
            current_speed,
            cap_target,
            cap_spec["braking_mps2"],
            delta_seconds,
            cap_spec["curve_mode"],
        )
        if next_speed >= 0.0:
            next_speed = min(next_speed, max(capped_speed, cap_target))
        else:
            next_speed = max(next_speed, min(capped_speed, cap_target))

    # A friction-brake hold is a positional restraint, not merely a
    # zero-speed cap.  Once its envelope has brought the train to rest, keep
    # it from rolling backward under gravity until Release Brake executes.
    # This is essential for stations and stopped launches on an incline.
    if (
        cap_spec is not None
        and cap_spec.get("kind") == "SET_BRAKE_STOP"
        and front_distance >= cap_spec["stop_target"] - SIMULATION_STOP_EPSILON
        and next_speed < 0.0
    ):
        next_speed = 0.0

    return next_speed


def resolve_control_command_influence(route, track_settings, front_distance, command):
    if command is None:
        return None
    spans = command.get("spans", ())
    influence = min(sum(
        get_zone_influence(route, track_settings, front_distance, span) for span in spans
    ), 1.0)
    if influence <= 0.0:
        return None
    resolved = dict(command)
    resolved["acceleration_mps2"] = command.get("acceleration_mps2", 0.0) * influence
    resolved["braking_mps2"] = command.get("braking_mps2", 0.0) * influence
    resolved["influence"] = influence
    return resolved


def resolve_block_control_state(route, track_settings, state, front_distance):
    """Resolve persistent block commands, including a position-aware brake envelope."""
    drive_spec = resolve_control_command_influence(
        route, track_settings, front_distance, state.get("block_drive")
    )
    brake_spec = resolve_control_command_influence(
        route, track_settings, front_distance, state.get("block_brake")
    )
    if brake_spec is not None and brake_spec.get("kind") == "SET_BRAKE_STOP":
        brake_spec = dict(brake_spec)
        distance_left = max(brake_spec["stop_target"] - front_distance, 0.0)
        braking = max(brake_spec["braking_mps2"], 0.0)
        brake_spec["target_speed"] = (2.0 * braking * distance_left) ** 0.5
    return {"drive": drive_spec, "cap": brake_spec}


def emit_sensor_crossings(sensors, completed_keys, segment_start, segment_end, events_out):
    """Queue sensor sequences whose trigger point lies on the travelled segment.

    Each sensor fires once per lap (its key re-arms when the seam wrap clears
    the completed set). WAIT actions delay subsequent TRIGGERs; nothing here
    affects train motion.
    """
    if events_out is None or not sensors or segment_end - segment_start <= 0.0:
        return

    for sensor in sensors:
        if sensor["key"] in completed_keys:
            continue
        position = sensor["position"]
        if segment_start - SIMULATION_STOP_EPSILON < position <= segment_end + SIMULATION_STOP_EPSILON:
            completed_keys.add(sensor["key"])
            delay_seconds = 0.0
            for action in sensor["actions"]:
                if action["kind"] == "WAIT":
                    delay_seconds += action["duration"]
                elif action["kind"] == "TRIGGER":
                    events_out.append((delay_seconds, action["channel"], action["value"]))


def enter_block_action(state, program, completed_keys, events_out):
    """Apply instantaneous commands and stop at the next wait condition."""
    actions = program["actions"]
    while state["action_index"] < len(actions):
        action = actions[state["action_index"]]
        kind = action["kind"]
        if kind == "TRIGGER":
            if action["channel"] and events_out is not None:
                events_out.append((0.0, action["channel"], action["value"]))
            state["action_index"] += 1
            continue
        if kind == "SET_TRANSPORT":
            state["block_drive"] = action
            state["action_index"] += 1
            continue
        if kind in {"SET_BRAKE", "SET_BRAKE_STOP"}:
            state["block_brake"] = action
            state["action_index"] += 1
            continue
        if kind == "RELEASE_BRAKE":
            state["block_brake"] = None
            state["action_index"] += 1
            continue
        if kind == "DISPATCH":
            state["override_drive"] = state["block_drive"]
            state["override_brake"] = state["block_brake"]
            state["override_span"] = program["span"]
            completed_keys.add(program["key"])
            state["block"] = None
            state["action_index"] = 0
            state["wait"] = 0.0
            state["block_drive"] = None
            state["block_brake"] = None
            return False
        if kind == "WAIT":
            if state["wait"] <= 1.0e-8:
                # Arm only when not already counting down (a finished wait
                # advances the index immediately, so wait==0 here means
                # "not started yet").
                state["wait"] = max(action["duration"], 0.0)
                if state["wait"] <= 1.0e-8:
                    state["action_index"] += 1
                    continue
            return True
        if kind in {"WAIT_POSITION", "WAIT_SPEED"}:
            return True
        return True
    return True


def find_block_capture(programs, completed_keys, segment_start, segment_end):
    """First non-completed program whose span start lies on the travelled segment."""
    if not programs or segment_end - segment_start <= 0.0:
        return None

    best_program = None
    for program in programs:
        if not program["actions"] or program["key"] in completed_keys:
            continue
        span_start = program["span"][0]
        if segment_start + SIMULATION_STOP_EPSILON < span_start <= segment_end + SIMULATION_STOP_EPSILON:
            if best_program is None or span_start < best_program["span"][0]:
                best_program = program
    return best_program


def build_initial_simulation_state(route, derived, start_meters, events_out=None):
    state = {
        "front": wrap_route_distance(route, start_meters),
        "speed": 0.0,
        "wait": 0.0,
        "block": None,
        "action_index": 0,
        "block_drive": None,
        "block_brake": None,
        "override_drive": None,
        "override_brake": None,
        "override_span": None,
        "completed": set(),
    }

    # Starting inside a block span means the train is parked there under
    # hardware control: skip the move actions already behind the start point
    # and resume the program from that spot (waits, triggers, dispatch).
    for program in derived["programs"]:
        span_start, span_end = program["span"]
        if not program["actions"]:
            continue
        at_cyclic_station_exit = (
            route["cyclic"]
            and state["front"] <= 1.0e-3
            and abs(span_end - route["total_length"]) <= 1.0e-3
        )
        if span_start - 1.0e-3 <= state["front"] <= span_end + 1.0e-3 or at_cyclic_station_exit:
            state["block"] = program["key"]
            comparison_front = route["total_length"] if at_cyclic_station_exit else state["front"]
            state["action_index"] = 0
            last_initial_target = None
            for index, action in enumerate(program["actions"]):
                if (
                    action["kind"] == "WAIT_POSITION"
                    and action["comparison"] == "AT_OR_AFTER"
                    and action["target"] <= comparison_front + 1.0e-3
                    and (
                        last_initial_target is None
                        or action["target"] > last_initial_target + SIMULATION_STOP_EPSILON
                    )
                ):
                    state["action_index"] = index + 1
                    last_initial_target = action["target"]
            for action in program["actions"][:state["action_index"]]:
                if action["kind"] == "SET_TRANSPORT":
                    state["block_drive"] = action
                elif action["kind"] in {"SET_BRAKE", "SET_BRAKE_STOP"}:
                    state["block_brake"] = action
                elif action["kind"] == "RELEASE_BRAKE":
                    state["block_brake"] = None
            enter_block_action(state, program, state["completed"], events_out)
            break

    return state


def advance_simulation_state(state, route, derived, track_settings, delta_seconds, events_out=None):
    """Advance a pure simulation state dict by delta_seconds (no RNA access).

    `events_out` collects (delay_seconds, channel, value) trigger events fired
    by block actions and sensors during the step. Stops and captures absorb
    the rest of their frame (frame-boundary snap) so lap states recur exactly
    and trajectory cycle detection stays reliable.
    """
    if delta_seconds <= 0.0:
        return

    remaining_time = delta_seconds
    current_speed = state["speed"]
    completed_keys = state["completed"]
    total_length = route["total_length"]
    track_is_cyclic = route["cyclic"]
    front_distance = wrap_route_distance(route, state["front"])
    route_zones = derived["zones"]
    programs = derived["programs"]
    sensors = derived["sensors"]

    while remaining_time > 1.0e-8:
        if total_length <= 1.0e-8:
            current_speed = 0.0
            break

        # --- Captured by a block program -------------------------------
        if state["block"] is not None:
            program = derived["programs_by_key"].get(state["block"])
            if program is None:
                state["block"] = None
                state["action_index"] = 0
                state["wait"] = 0.0
                continue

            if (
                track_is_cyclic
                and front_distance <= SIMULATION_STOP_EPSILON
                and abs(program["span"][1] - total_length) <= SIMULATION_STOP_EPSILON
            ):
                front_distance = total_length

            if not enter_block_action(state, program, completed_keys, events_out):
                continue  # dispatched: physics resumes with the leftover time

            actions = program["actions"]
            if state["action_index"] >= len(actions):
                # Program exhausted without a dispatch: parked.
                current_speed = 0.0
                remaining_time = 0.0
                break

            action = actions[state["action_index"]]
            if action["kind"] == "WAIT_POSITION":
                reached = (
                    front_distance >= action["target"] - SIMULATION_STOP_EPSILON
                    if action["comparison"] == "AT_OR_AFTER"
                    else front_distance <= action["target"] + SIMULATION_STOP_EPSILON
                )
                if reached:
                    state["action_index"] += 1
                    continue
            elif action["kind"] == "WAIT_SPEED":
                reached = (
                    abs(current_speed) <= action["speed"] + SIMULATION_STOP_EPSILON
                    if action["comparison"] == "AT_OR_BELOW"
                    else abs(current_speed) >= action["speed"] - SIMULATION_STOP_EPSILON
                )
                if reached:
                    state["action_index"] += 1
                    continue

            time_slice = min(remaining_time, state["wait"]) if action["kind"] == "WAIT" else remaining_time
            control_state = resolve_block_control_state(route, track_settings, state, front_distance)
            next_speed = get_controlled_next_speed(
                route, track_settings, control_state, front_distance, current_speed, time_slice
            )
            proposed_distance = front_distance + (current_speed + next_speed) * 0.5 * time_slice
            active_brake = state.get("block_brake")
            if (
                active_brake is not None
                and control_state["cap"] is not None
                and active_brake.get("kind") == "SET_BRAKE_STOP"
                and front_distance <= active_brake["stop_target"] <= proposed_distance
            ):
                proposed_distance = active_brake["stop_target"]
                next_speed = 0.0
            emit_sensor_crossings(sensors, completed_keys, front_distance, proposed_distance, events_out)
            front_distance = proposed_distance
            current_speed = next_speed
            if action["kind"] == "WAIT":
                remaining_time -= time_slice
                state["wait"] -= time_slice
                if state["wait"] <= 1.0e-8:
                    state["wait"] = 0.0
                    state["action_index"] += 1
                    continue
            elif action["kind"] == "WAIT_POSITION":
                crossed = (
                    front_distance >= action["target"] - SIMULATION_STOP_EPSILON
                    if action["comparison"] == "AT_OR_AFTER"
                    else front_distance <= action["target"] + SIMULATION_STOP_EPSILON
                )
                if crossed:
                    front_distance = action["target"]
                    state["action_index"] += 1
            elif action["kind"] == "WAIT_SPEED":
                crossed = (
                    abs(current_speed) <= action["speed"] + SIMULATION_STOP_EPSILON
                    if action["comparison"] == "AT_OR_BELOW"
                    else abs(current_speed) >= action["speed"] - SIMULATION_STOP_EPSILON
                )
                if crossed:
                    state["action_index"] += 1
            remaining_time = 0.0
            break

        # --- Free running ----------------------------------------------
        if not track_is_cyclic and front_distance >= total_length - SIMULATION_STOP_EPSILON and current_speed >= 0.0:
            front_distance = total_length
            current_speed = 0.0
            break
        if not track_is_cyclic and front_distance <= SIMULATION_STOP_EPSILON and current_speed < 0.0:
            front_distance = 0.0
            current_speed = 0.0
            break

        control_state = get_route_control_state(route, track_settings, route_zones, front_distance)
        override_span = state.get("override_span")
        if override_span is not None:
            override_influence = get_zone_influence(route, track_settings, front_distance, override_span)
            if override_influence > 0.0:
                override_drive = resolve_control_command_influence(
                    route, track_settings, front_distance, state.get("override_drive")
                )
                override_brake = resolve_control_command_influence(
                    route, track_settings, front_distance, state.get("override_brake")
                )
                if override_drive is not None:
                    control_state["drive"] = override_drive
                if override_brake is not None:
                    control_state["cap"] = override_brake
            else:
                state["override_drive"] = None
                state["override_brake"] = None
                state["override_span"] = None
        next_speed = get_controlled_next_speed(
            route, track_settings, control_state, front_distance, current_speed, remaining_time
        )
        proposed_distance = front_distance + (current_speed + next_speed) * 0.5 * remaining_time

        unclamped_proposed_distance = proposed_distance
        if not track_is_cyclic:
            proposed_distance = clamp(proposed_distance, 0.0, total_length)

        capture_program = find_block_capture(programs, completed_keys, front_distance, proposed_distance)
        if capture_program is not None:
            span_start = capture_program["span"][0]
            emit_sensor_crossings(sensors, completed_keys, front_distance, span_start, events_out)
            front_distance = span_start
            current_speed = next_speed
            state["block"] = capture_program["key"]
            state["action_index"] = 0
            state["wait"] = 0.0
            state["block_drive"] = None
            state["block_brake"] = None
            state["override_drive"] = None
            state["override_brake"] = None
            state["override_span"] = None
            enter_block_action(state, capture_program, completed_keys, events_out)
            # Frame-boundary snap at capture for exact lap recurrence.
            break

        if track_is_cyclic and unclamped_proposed_distance >= total_length:
            distance_to_wrap = max(total_length - front_distance, 0.0)
            average_speed = max((current_speed + next_speed) * 0.5, SIMULATION_STOP_EPSILON)
            time_to_wrap = distance_to_wrap / average_speed if distance_to_wrap > SIMULATION_STOP_EPSILON else 0.0
            time_to_wrap = clamp(time_to_wrap, 0.0, remaining_time)
            emit_sensor_crossings(sensors, completed_keys, front_distance, total_length, events_out)
            remaining_time -= time_to_wrap
            front_distance = 0.0
            current_speed = next_speed
            # Re-arm blocks and sensors for the next lap.
            completed_keys.clear()
            continue

        if track_is_cyclic and unclamped_proposed_distance < 0.0:
            distance_to_wrap = max(front_distance, 0.0)
            average_speed = max(abs((current_speed + next_speed) * 0.5), SIMULATION_STOP_EPSILON)
            time_to_wrap = distance_to_wrap / average_speed if distance_to_wrap > SIMULATION_STOP_EPSILON else 0.0
            time_to_wrap = clamp(time_to_wrap, 0.0, remaining_time)
            remaining_time -= time_to_wrap
            front_distance = total_length
            current_speed = next_speed
            completed_keys.clear()
            continue

        emit_sensor_crossings(sensors, completed_keys, front_distance, proposed_distance, events_out)
        front_distance = proposed_distance
        current_speed = next_speed
        remaining_time = 0.0

    state["front"] = wrap_route_distance(route, front_distance)
    state["speed"] = current_speed
    state["wait"] = max(state["wait"], 0.0)


def quantize_simulation_state(state):
    def command_signature(command):
        if command is None:
            return None
        return (
            command.get("kind"),
            round(command.get("target_speed", 0.0), TRAJECTORY_STATE_DECIMALS),
            round(command.get("acceleration_mps2", 0.0), TRAJECTORY_STATE_DECIMALS),
            round(command.get("braking_mps2", 0.0), TRAJECTORY_STATE_DECIMALS),
            round(command.get("stop_target", 0.0), TRAJECTORY_STATE_DECIMALS),
            command.get("curve_mode", "LINEAR"),
        )

    return (
        round(state["front"], TRAJECTORY_STATE_DECIMALS),
        round(state["speed"], TRAJECTORY_STATE_DECIMALS),
        round(state["wait"], TRAJECTORY_STATE_DECIMALS),
        state["block"],
        state["action_index"],
        command_signature(state.get("block_drive")),
        command_signature(state.get("block_brake")),
        command_signature(state.get("override_drive")),
        command_signature(state.get("override_brake")),
        tuple(state["override_span"]) if state.get("override_span") is not None else None,
        frozenset(state["completed"]),
    )


def get_trajectory_key(scene, track_object, track_settings, route):
    scene_settings = scene.coaster_mixer_scene
    return (
        track_object.as_pointer(),
        route["key"],
        round(get_scene_fps(scene), 6),
        scene.frame_start,
        round(scene_settings.simulation_start_route_meters, 6),
        round(track_settings.train_length_meters, 6),
        round(track_settings.train_weight_kilograms, 6),
        round(track_settings.friction_coefficient, 9),
        round(track_settings.drag_coefficient, 6),
        round(track_settings.frontal_area_m2, 6),
    )


def record_trajectory_events(cache, frame_index, fps, step_events):
    """Convert (delay_seconds, channel, value) step events into channel timelines."""
    for delay_seconds, channel, value in step_events:
        event_frame = frame_index + int(round(max(delay_seconds, 0.0) * fps))
        cache["events"].append((event_frame, channel, value))
        frames, values = cache["channels"].setdefault(channel, ([], []))
        insert_at = bisect_right(frames, event_frame)
        frames.insert(insert_at, event_frame)
        values.insert(insert_at, value)


def get_simulation_trajectory_cache(scene, track_object, track_settings):
    global SIMULATION_TRAJECTORY_CACHE

    route = get_resolved_route(track_object)
    if not route["entries"] or route["total_length"] <= 1.0e-8:
        return None, route

    key = get_trajectory_key(scene, track_object, track_settings, route)
    cache = SIMULATION_TRAJECTORY_CACHE
    if cache is None or cache["key"] != key:
        derived = get_route_derived_data(route)
        initial_events = []
        state = build_initial_simulation_state(
            route, derived, scene.coaster_mixer_scene.simulation_start_route_meters, initial_events
        )
        cache = {
            "key": key,
            "fronts": [state["front"]],
            "speeds": [state["speed"]],
            "stops": [state["wait"]],
            "state": state,
            "seen_states": {quantize_simulation_state(state): 0},
            "wrap_indices": [],
            "cycle_start": None,
            "cycle_length": None,
            "events": [],
            "channels": {},
        }
        record_trajectory_events(cache, 0, get_scene_fps(scene), initial_events)
        SIMULATION_TRAJECTORY_CACHE = cache
    return cache, route


def resolve_trajectory_index(cache, index):
    cycle_length = cache["cycle_length"]
    if cycle_length:
        cycle_start = cache["cycle_start"]
        if index >= cycle_start:
            index = cycle_start + (index - cycle_start) % cycle_length
    return index


def sample_simulation_trajectory(scene, track_object, track_settings, frame):
    """Deterministic (front, speed, stop_remaining) for a timeline frame.

    frame_start is simulation time zero; each frame advances one 1/fps step.
    Frames are memoized, and once the quantized state recurs the trajectory
    loops, so scrubbing in either direction and playing past the cycle end
    are O(1) lookups.
    """
    cache, route = get_simulation_trajectory_cache(scene, track_object, track_settings)
    if cache is None:
        return None

    index = resolve_trajectory_index(cache, max(int(frame) - scene.frame_start, 0))
    fronts = cache["fronts"]

    if index >= len(fronts) and cache["cycle_length"] is None:
        derived = get_route_derived_data(route)
        fps = get_scene_fps(scene)
        delta_seconds = 1.0 / fps
        state = cache["state"]
        seen_states = cache["seen_states"]
        half_length = route["total_length"] * 0.5
        while len(fronts) <= index and len(fronts) <= TRAJECTORY_FRAME_LIMIT:
            previous_front = state["front"]
            step_events = []
            advance_simulation_state(state, route, derived, track_settings, delta_seconds, step_events)
            quantized = quantize_simulation_state(state)
            seen_index = seen_states.get(quantized)
            if seen_index is not None:
                cache["cycle_start"] = seen_index
                cache["cycle_length"] = len(fronts) - seen_index
                index = resolve_trajectory_index(cache, index)
                break
            if step_events:
                record_trajectory_events(cache, len(fronts), fps, step_events)
            if state["front"] < previous_front - half_length:
                cache["wrap_indices"].append(len(fronts))
            seen_states[quantized] = len(fronts)
            fronts.append(state["front"])
            cache["speeds"].append(state["speed"])
            cache["stops"].append(state["wait"])

        if cache["cycle_length"] is None and len(fronts) > TRAJECTORY_FRAME_LIMIT:
            # No exact recurrence (e.g. a stopless free-running circuit):
            # approximate the loop with the last two seam crossings so
            # playback keeps looping instead of freezing.
            wrap_indices = cache["wrap_indices"]
            if len(wrap_indices) >= 2:
                cache["cycle_start"] = wrap_indices[-2]
                cache["cycle_length"] = wrap_indices[-1] - wrap_indices[-2]
            else:
                cache["cycle_start"] = len(fronts) - 1
                cache["cycle_length"] = 1
            index = resolve_trajectory_index(cache, index)

    index = min(index, len(fronts) - 1)
    return fronts[index], cache["speeds"][index], cache["stops"][index]


def apply_simulation_frame(scene, track_object=None, track_settings=None):
    """Place the train at the trajectory sample for the current frame."""
    scene_settings = getattr(scene, "coaster_mixer_scene", None)
    if scene_settings is None or not scene_settings.simulation_enabled:
        return

    if track_object is None:
        track_object = scene_settings.track_object
    if track_object is None or track_object.type != "CURVE":
        return
    if track_settings is None:
        track_settings = track_object.coaster_mixer_track

    sample = sample_simulation_trajectory(scene, track_object, track_settings, scene.frame_current)
    if sample is None:
        return

    front_distance, speed, stop_remaining = sample
    assign_rna_property(track_settings, "train_front_route_meters", front_distance)
    assign_rna_property(scene_settings, "simulation_current_speed_mps", speed)
    assign_rna_property(scene_settings, "simulation_stop_remaining_seconds", stop_remaining)
    place_track_followers(track_object, front_distance)


def get_clamped_active_zone_index(track_settings):
    count = len(track_settings.zones)
    if count == 0:
        return 0

    return clamp(track_settings.active_zone_index, 0, count - 1)


def ensure_active_zone_index(track_settings):
    clamped_index = get_clamped_active_zone_index(track_settings)
    if clamped_index != track_settings.active_zone_index:
        assign_rna_property(track_settings, "active_zone_index", clamped_index)


def get_zone_index(track_settings, zone):
    zone_pointer = zone.as_pointer()
    for index, candidate in enumerate(track_settings.zones):
        if candidate.as_pointer() == zone_pointer:
            return index
    return -1


def get_track_total_length(track_object):
    return build_curve_cache(track_object)["total_length"]


def sync_track_zones_to_curve(track_object, track_settings):
    global PROPERTY_SYNC_GUARD

    if PROPERTY_SYNC_GUARD or track_object is None or track_settings is None or len(track_settings.zones) == 0:
        return

    PROPERTY_SYNC_GUARD = True
    try:
        for zone in track_settings.zones:
            sync_zone_to_curve(track_object, zone)
    finally:
        PROPERTY_SYNC_GUARD = False


def zone_span_update(zone, _context):
    owner = getattr(zone, "id_data", None)
    if PROPERTY_SYNC_GUARD or owner is None or owner.type != "CURVE":
        return

    track_settings = owner.coaster_mixer_track
    index = get_zone_index(track_settings, zone)
    if index < 0:
        return

    sync_zone_to_curve(owner, zone)
    # Invalidate every root: this piece can sit on any coaster's route, and
    # the route-zone caches are keyed by root, not by the edited piece.
    invalidate_route_cache()
    tag_redraw_view3d()


def zone_details_update(zone, _context):
    owner = getattr(zone, "id_data", None)
    if PROPERTY_SYNC_GUARD or owner is None or owner.type != "CURVE":
        return

    sync_zone_to_curve(owner, zone)
    invalidate_route_cache()
    tag_redraw_view3d()


def active_zone_index_update(_settings, _context):
    tag_redraw_view3d()


def track_settings_update(_settings, _context):
    if PROPERTY_SYNC_GUARD:
        return

    owner = getattr(_settings, "id_data", None)
    if owner is not None and owner.type == "CURVE":
        tag_track_placement_update(owner)
    tag_redraw_view3d()


def scene_display_update(_settings, _context):
    tag_redraw_view3d()


def driven_empty_object_update(settings, _context):
    owner = getattr(settings, "id_data", None)
    if owner is not None and owner.type == "CURVE" and settings.driven_empty_object is not None:
        ensure_driven_empty_path_setup(owner, settings)
    if bpy.context is not None:
        apply_simulation_frame(bpy.context.scene)
    tag_redraw_view3d()


def simulation_enabled_update(settings, _context):
    if SIMULATION_ENABLED_UPDATE_GUARD:
        return

    scene = getattr(settings, "id_data", None)
    if scene is None:
        return

    apply_simulation_frame(scene)
    tag_redraw_view3d()


def simulation_start_update(settings, _context):
    # The start position is part of the trajectory key; re-place the train
    # immediately so the change is visible without scrubbing.
    scene = getattr(settings, "id_data", None)
    if scene is not None:
        apply_simulation_frame(scene)
    tag_redraw_view3d()


def track_object_update(_settings, _context):
    invalidate_route_cache()
    track_object = resolve_active_track_object(bpy.context)
    if track_object is not None:
        track_settings = track_object.coaster_mixer_track
        seed_default_station(bpy.context, track_object)
        sync_track_zones_to_curve(track_object, track_settings)
        if track_settings.driven_empty_object is not None:
            ensure_driven_empty_path_setup(track_object, track_settings)
        apply_simulation_frame(bpy.context.scene, track_object, track_settings)
    tag_redraw_view3d()


def block_group_update(_settings, _context):
    invalidate_route_cache()
    track_object = resolve_active_track_object(bpy.context)
    if track_object is not None:
        tag_track_placement_update(track_object)
    tag_redraw_view3d()


def is_control_tree(_self, node_tree):
    return node_tree is not None and node_tree.bl_idname == CONTROL_TREE_IDNAME


def get_block_hardware_types(block_group):
    hardware_types = set()
    if block_group is None:
        return hardware_types
    for member in block_group.members:
        piece = member.piece
        if piece is None or piece.type != "CURVE":
            continue
        zones = piece.coaster_mixer_track.zones
        if 0 <= member.zone_index < len(zones):
            hardware_types.add(zones[member.zone_index].zone_type)
    return hardware_types


def get_context_active_block(context):
    if context is None:
        return None
    _track_object, track_settings = resolve_active_track_settings(context)
    if track_settings is None or len(track_settings.block_groups) == 0:
        return None
    return track_settings.block_groups[get_clamped_block_group_index(track_settings)]


def control_template_items_callback(owner, context):
    block_group = owner if hasattr(owner, "members") else get_context_active_block(context)
    hardware_types = get_block_hardware_types(block_group)
    has_transport = "TRANSPORT" in hardware_types
    has_friction_brake = "FRICTION_BRAKE" in hardware_types
    has_any_brake = bool(hardware_types & {"FRICTION_BRAKE", "TRIM_BRAKE"})
    allowed = {"CUSTOM"}
    if has_transport:
        allowed.update({"ROLLING_LAUNCH", "STANDARD_LIFT"})
    if has_transport and has_friction_brake:
        allowed.update({"STOPPED_LAUNCH", "LOAD_STATION", "UNLOAD_STATION"})
    if has_any_brake:
        allowed.add("TRIM_BRAKE")
    return [item for item in CONTROL_TEMPLATE_ITEMS if item[0] in allowed]
