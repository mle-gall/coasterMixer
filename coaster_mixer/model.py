# SPDX-FileCopyrightText: 2026 Coaster Mixer contributors
# SPDX-License-Identifier: GPL-3.0-or-later

"""Blender RNA data model and control-graph node types."""

from .runtime import *

class CoasterMixerControlSocket(bpy.types.NodeSocket):
    bl_idname = CONTROL_SOCKET_IDNAME
    bl_label = "Flow"

    def draw(self, _context, layout, _node, text):
        layout.label(text=text or self.name)

    def draw_color(self, _context, _node):
        return (0.82, 0.58, 1.0, 1.0)


class CoasterMixerControlTree(bpy.types.NodeTree):
    bl_idname = CONTROL_TREE_IDNAME
    bl_label = "Coaster Control"
    bl_icon = "NODETREE"


class CoasterMixerControlNodeMixin:
    @classmethod
    def poll(cls, node_tree):
        return node_tree.bl_idname == CONTROL_TREE_IDNAME

    def draw_label(self):
        return f"⚠ {self.bl_label}" if get_control_node_error(self) else self.bl_label


def get_control_node_program(node):
    context = bpy.context
    track_object = resolve_active_track_object(context) if context is not None else None
    if track_object is None:
        return None
    derived = get_route_derived_data(get_resolved_route(track_object))
    tree = getattr(node, "id_data", None)
    return next(
        (program for program in derived["programs"] if program.get("control_tree") == tree),
        None,
    )


def get_control_node_error(node):
    program = get_control_node_program(node)
    if program is None:
        return "Node tree is not assigned to an active-route block"
    node_type = node.bl_idname
    if node_type == "COASTERMIXER_ND_set_transport" and not program["transport_spans"]:
        return "No Transport actuator is assigned to this block"
    if node_type == "COASTERMIXER_ND_set_transport":
        hold_action = next(
            (action for action in program["actions"] if action["kind"] == "SET_BRAKE_STOP"),
            None,
        )
        context = bpy.context
        track_object, track_settings = resolve_active_track_settings(context)
        route = get_resolved_route(track_object) if track_object is not None else None
        if hold_action is not None and route is not None and not any(
            get_zone_influence(route, track_settings, hold_action["stop_target"], span) > 0.0
            for span in program["transport_spans"]
        ):
            return "Hold position has no Transport influence for departure"
    if node_type in {"COASTERMIXER_ND_set_brake", "COASTERMIXER_ND_release_brake"} and not program["brake_spans"]:
        return "No brake actuator is assigned to this block"
    if node_type == "COASTERMIXER_ND_set_brake_hold":
        if not program["friction_brake_spans"]:
            return "No Friction Brake actuator is assigned to this block"
        target = program["span"][0] + node.offset_meters
        if not any(
            span_start - SIMULATION_STOP_EPSILON <= target <= span_end + SIMULATION_STOP_EPSILON
            for span_start, span_end in program["friction_brake_spans"]
        ):
            return "Hold position is outside the assigned Friction Brake influence"
    if node_type == "COASTERMIXER_ND_wait_position":
        block_length = max(program["span"][1] - program["span"][0], 0.0)
        if node.offset_meters > block_length + SIMULATION_STOP_EPSILON:
            return "Position is outside the block span"
    return None


def draw_control_node_error(layout, node):
    error = get_control_node_error(node)
    if error:
        layout.label(text=error, icon="ERROR")


class COASTERMIXER_ND_block_entered(CoasterMixerControlNodeMixin, bpy.types.Node):
    bl_idname = "COASTERMIXER_ND_block_entered"
    bl_label = "Block Entered"
    bl_icon = "TRACKING"

    def init(self, _context):
        self.outputs.new(CONTROL_SOCKET_IDNAME, "Then")


class COASTERMIXER_ND_set_transport(CoasterMixerControlNodeMixin, bpy.types.Node):
    bl_idname = "COASTERMIXER_ND_set_transport"
    bl_label = "Set Transport Target"
    bl_icon = "FORWARD"

    speed_mps: bpy.props.FloatProperty(name="Target Speed", description="Requested transport speed in meters per second (m/s)", min=0.0, default=2.0, update=block_group_update)
    acceleration_mps2: bpy.props.FloatProperty(name="Acceleration", description="Requested acceleration in meters per second squared (m/s²)", min=0.0, default=1.0, update=block_group_update)
    braking_mps2: bpy.props.FloatProperty(name="Braking", description="Requested transport deceleration in meters per second squared (m/s²)", min=0.0, default=1.5, update=block_group_update)
    response_curve: bpy.props.EnumProperty(name="Response", items=CONTROL_RESPONSE_CURVE_ITEMS, default="LINEAR", update=block_group_update)

    def init(self, _context):
        self.inputs.new(CONTROL_SOCKET_IDNAME, "In")
        self.outputs.new(CONTROL_SOCKET_IDNAME, "Then")

    def draw_buttons(self, _context, layout):
        layout.prop(self, "speed_mps")
        layout.prop(self, "acceleration_mps2")
        layout.prop(self, "braking_mps2")
        layout.prop(self, "response_curve")
        draw_control_node_error(layout, self)


class COASTERMIXER_ND_set_brake(CoasterMixerControlNodeMixin, bpy.types.Node):
    bl_idname = "COASTERMIXER_ND_set_brake"
    bl_label = "Set Brake Target"
    bl_icon = "FREEZE"

    speed_mps: bpy.props.FloatProperty(name="Target Speed", description="Requested brake target speed in meters per second (m/s); zero holds the train", min=0.0, default=0.0, update=block_group_update)
    braking_mps2: bpy.props.FloatProperty(name="Braking", description="Requested braking deceleration in meters per second squared (m/s²)", min=0.0, default=1.5, update=block_group_update)
    response_curve: bpy.props.EnumProperty(name="Response", items=CONTROL_RESPONSE_CURVE_ITEMS, default="LINEAR", update=block_group_update)

    def init(self, _context):
        self.inputs.new(CONTROL_SOCKET_IDNAME, "In")
        self.outputs.new(CONTROL_SOCKET_IDNAME, "Then")

    def draw_buttons(self, _context, layout):
        layout.prop(self, "speed_mps")
        layout.prop(self, "braking_mps2")
        layout.prop(self, "response_curve")
        draw_control_node_error(layout, self)


class COASTERMIXER_ND_set_brake_hold(CoasterMixerControlNodeMixin, bpy.types.Node):
    bl_idname = "COASTERMIXER_ND_set_brake_hold"
    bl_label = "Brake to Hold Point"
    bl_icon = "PIVOT_CURSOR"

    offset_meters: bpy.props.FloatProperty(name="Block Offset", description="Exact stopping point in meters from the start of the block", min=0.0, subtype="DISTANCE", default=5.0, update=block_group_update)
    braking_mps2: bpy.props.FloatProperty(name="Braking", description="Requested braking deceleration in meters per second squared (m/s²)", min=0.0, default=1.5, update=block_group_update)
    response_curve: bpy.props.EnumProperty(name="Response", items=CONTROL_RESPONSE_CURVE_ITEMS, default="LINEAR", update=block_group_update)

    def init(self, _context):
        self.inputs.new(CONTROL_SOCKET_IDNAME, "In")
        self.outputs.new(CONTROL_SOCKET_IDNAME, "Then")

    def draw_buttons(self, _context, layout):
        layout.prop(self, "offset_meters")
        layout.prop(self, "braking_mps2")
        layout.prop(self, "response_curve")
        draw_control_node_error(layout, self)


class COASTERMIXER_ND_release_brake(CoasterMixerControlNodeMixin, bpy.types.Node):
    bl_idname = "COASTERMIXER_ND_release_brake"
    bl_label = "Release Brake"
    bl_icon = "UNLOCKED"

    def init(self, _context):
        self.inputs.new(CONTROL_SOCKET_IDNAME, "In")
        self.outputs.new(CONTROL_SOCKET_IDNAME, "Then")

    def draw_buttons(self, _context, layout):
        draw_control_node_error(layout, self)


class COASTERMIXER_ND_wait(CoasterMixerControlNodeMixin, bpy.types.Node):
    bl_idname = "COASTERMIXER_ND_wait"
    bl_label = "Wait"
    bl_icon = "PAUSE"

    duration_seconds: bpy.props.FloatProperty(name="Seconds", min=0.0, default=5.0, update=block_group_update)

    def init(self, _context):
        self.inputs.new(CONTROL_SOCKET_IDNAME, "In")
        self.outputs.new(CONTROL_SOCKET_IDNAME, "Then")

    def draw_buttons(self, _context, layout):
        layout.prop(self, "duration_seconds")


class COASTERMIXER_ND_wait_position(CoasterMixerControlNodeMixin, bpy.types.Node):
    bl_idname = "COASTERMIXER_ND_wait_position"
    bl_label = "Wait for Position"
    bl_icon = "EMPTY_SINGLE_ARROW"

    offset_meters: bpy.props.FloatProperty(name="Block Offset", description="Position in meters from the start of the block", min=0.0, subtype="DISTANCE", default=5.0, update=block_group_update)
    comparison: bpy.props.EnumProperty(
        name="Condition",
        items=[
            ("AT_OR_AFTER", "At or After", "Continue when the train reaches or passes this position in the forward direction"),
            ("AT_OR_BEFORE", "At or Before", "Continue when the train reaches or passes this position in the reverse direction"),
        ],
        default="AT_OR_AFTER",
        update=block_group_update,
    )

    def init(self, _context):
        self.inputs.new(CONTROL_SOCKET_IDNAME, "In")
        self.outputs.new(CONTROL_SOCKET_IDNAME, "Then")

    def draw_buttons(self, _context, layout):
        layout.prop(self, "offset_meters")
        layout.prop(self, "comparison")
        draw_control_node_error(layout, self)


class COASTERMIXER_ND_wait_speed(CoasterMixerControlNodeMixin, bpy.types.Node):
    bl_idname = "COASTERMIXER_ND_wait_speed"
    bl_label = "Wait for Speed"
    bl_icon = "DRIVER"

    speed_mps: bpy.props.FloatProperty(name="Speed", description="Speed threshold in meters per second (m/s)", min=0.0, default=0.1, update=block_group_update)
    comparison: bpy.props.EnumProperty(
        name="Condition",
        items=[
            ("AT_OR_BELOW", "At or Below", "Continue when absolute train speed is at or below the threshold"),
            ("AT_OR_ABOVE", "At or Above", "Continue when absolute train speed is at or above the threshold"),
        ],
        default="AT_OR_BELOW",
        update=block_group_update,
    )

    def init(self, _context):
        self.inputs.new(CONTROL_SOCKET_IDNAME, "In")
        self.outputs.new(CONTROL_SOCKET_IDNAME, "Then")

    def draw_buttons(self, _context, layout):
        layout.prop(self, "speed_mps")
        layout.prop(self, "comparison")


class COASTERMIXER_ND_trigger(CoasterMixerControlNodeMixin, bpy.types.Node):
    bl_idname = "COASTERMIXER_ND_trigger"
    bl_label = "Set Trigger"
    bl_icon = "OUTLINER_OB_LIGHT"

    channel: bpy.props.StringProperty(name="Channel", default="", update=block_group_update)
    value: bpy.props.FloatProperty(name="Value", default=1.0, update=block_group_update)

    def init(self, _context):
        self.inputs.new(CONTROL_SOCKET_IDNAME, "In")
        self.outputs.new(CONTROL_SOCKET_IDNAME, "Then")

    def draw_buttons(self, _context, layout):
        layout.prop(self, "channel")
        layout.prop(self, "value")


class COASTERMIXER_ND_dispatch(CoasterMixerControlNodeMixin, bpy.types.Node):
    bl_idname = "COASTERMIXER_ND_dispatch"
    bl_label = "Release Block"
    bl_icon = "PLAY"

    def init(self, _context):
        self.inputs.new(CONTROL_SOCKET_IDNAME, "In")


def compile_control_tree(
    node_tree, span_start, span_end, stop_capable_spans, friction_brake_spans,
    transport_spans, brake_spans, drive_min_speed, drive_max_speed,
    max_acceleration, max_braking, controllable_speed, warnings
):
    if node_tree is None or node_tree.bl_idname != CONTROL_TREE_IDNAME:
        warnings.append("No control graph assigned")
        return []
    starts = [node for node in node_tree.nodes if node.bl_idname == "COASTERMIXER_ND_block_entered"]
    if not starts:
        warnings.append("Control graph has no Block Entered node")
        return []
    actions = []
    node = starts[0]
    visited = set()
    while node is not None:
        pointer = node.as_pointer()
        if pointer in visited:
            warnings.append("Control graph contains a cycle")
            break
        visited.add(pointer)
        if node.bl_idname == "COASTERMIXER_ND_set_transport":
            requested_speed = max(node.speed_mps, 0.0)
            if drive_max_speed <= SIMULATION_STOP_EPSILON:
                resolved_speed = 0.0
            elif requested_speed <= SIMULATION_STOP_EPSILON:
                resolved_speed = 0.0
            else:
                resolved_speed = clamp(requested_speed, drive_min_speed, drive_max_speed)
                if abs(resolved_speed - requested_speed) > SIMULATION_STOP_EPSILON:
                    warnings.append(
                        f"Transport target {requested_speed:.1f} m/s is clamped to {resolved_speed:.1f} m/s"
                    )
            resolved_acceleration = min(max(node.acceleration_mps2, 0.0), max_acceleration)
            resolved_braking = min(max(node.braking_mps2, 0.0), max_braking)
            if node.acceleration_mps2 > max_acceleration + SIMULATION_STOP_EPSILON:
                warnings.append(f"{node.bl_label} acceleration is clamped to {max_acceleration:.1f} m/s²")
            if node.braking_mps2 > max_braking + SIMULATION_STOP_EPSILON:
                warnings.append(f"{node.bl_label} braking is clamped to {max_braking:.1f} m/s²")
            actions.append({
                "kind": "SET_TRANSPORT", "target_speed": resolved_speed,
                "acceleration_mps2": resolved_acceleration, "braking_mps2": resolved_braking,
                "curve_mode": node.response_curve, "spans": tuple(transport_spans),
            })
        elif node.bl_idname == "COASTERMIXER_ND_set_brake":
            requested_speed = max(node.speed_mps, 0.0)
            resolved_speed = min(requested_speed, controllable_speed)
            resolved_braking = min(max(node.braking_mps2, 0.0), max_braking)
            if requested_speed > controllable_speed + SIMULATION_STOP_EPSILON:
                warnings.append(f"Brake target is clamped to {controllable_speed:.1f} m/s")
            if node.braking_mps2 > max_braking + SIMULATION_STOP_EPSILON:
                warnings.append(f"Brake command is clamped to {max_braking:.1f} m/s²")
            actions.append({
                "kind": "SET_BRAKE", "target_speed": resolved_speed,
                "braking_mps2": resolved_braking, "curve_mode": node.response_curve,
                "spans": tuple(brake_spans),
            })
        elif node.bl_idname == "COASTERMIXER_ND_set_brake_hold":
            resolved_braking = min(max(node.braking_mps2, 0.0), max_braking)
            if node.braking_mps2 > max_braking + SIMULATION_STOP_EPSILON:
                warnings.append(f"Hold-point braking is clamped to {max_braking:.1f} m/s²")
            target = clamp(span_start + node.offset_meters, span_start, span_end)
            if not any(start - 1.0e-4 <= target <= end + 1.0e-4 for start, end in friction_brake_spans):
                warnings.append(f"Hold point {target:.1f} m is outside assigned Friction Brake hardware")
            actions.append({
                "kind": "SET_BRAKE_STOP", "target_speed": 0.0,
                "stop_target": target, "braking_mps2": resolved_braking,
                "curve_mode": node.response_curve, "spans": tuple(friction_brake_spans),
            })
            # Holding is a blocking operation: downstream logic must not run
            # until the train has reached the point and settled there.
            actions.append({
                "kind": "WAIT_POSITION", "target": target,
                "comparison": "AT_OR_AFTER",
            })
            actions.append({
                "kind": "WAIT_SPEED", "speed": 0.05,
                "comparison": "AT_OR_BELOW",
            })
        elif node.bl_idname == "COASTERMIXER_ND_release_brake":
            actions.append({"kind": "RELEASE_BRAKE"})
        elif node.bl_idname == "COASTERMIXER_ND_wait":
            actions.append({"kind": "WAIT", "duration": max(node.duration_seconds, 0.0)})
        elif node.bl_idname == "COASTERMIXER_ND_wait_position":
            actions.append({
                "kind": "WAIT_POSITION",
                "target": clamp(span_start + node.offset_meters, span_start, span_end),
                "comparison": node.comparison,
            })
        elif node.bl_idname == "COASTERMIXER_ND_wait_speed":
            actions.append({
                "kind": "WAIT_SPEED", "speed": max(node.speed_mps, 0.0),
                "comparison": node.comparison,
            })
        elif node.bl_idname == "COASTERMIXER_ND_trigger":
            actions.append({"kind": "TRIGGER", "channel": node.channel.strip(), "value": node.value})
        elif node.bl_idname == "COASTERMIXER_ND_dispatch":
            actions.append({"kind": "DISPATCH"})
            break
        output = node.outputs.get("Then")
        if output is None or not output.is_linked:
            break
        node = output.links[0].to_node
    return actions


class CoasterMixerZone(bpy.types.PropertyGroup):
    zone_type: bpy.props.EnumProperty(
        name="Type",
        items=ZONE_TYPE_ITEMS,
        default="FRICTION_BRAKE",
        update=zone_details_update,
    )
    name: bpy.props.StringProperty(
        name="Name",
        description="Optional label for this hardware zone",
        default="",
        update=zone_details_update,
    )
    start_meters: bpy.props.FloatProperty(
        name="Start",
        description="Distance from the start of the piece to this hardware zone",
        min=0.0,
        subtype="DISTANCE",
        default=0.0,
        update=zone_span_update,
    )
    length_meters: bpy.props.FloatProperty(
        name="Length",
        description="Hardware zone length measured along the curve",
        min=0.0,
        subtype="DISTANCE",
        default=DEFAULT_ZONE_LENGTH_METERS,
        update=zone_span_update,
    )
    minimum_speed_mps: bpy.props.FloatProperty(
        name="Minimum Speed",
        description="Lowest non-zero commanded speed supported by this actuator, in meters per second (m/s)",
        min=0.0,
        default=0.0,
        precision=3,
        update=zone_details_update,
    )
    target_speed_mps: bpy.props.FloatProperty(
        name="Maximum Speed",
        description="Maximum speed this actuator can command or safely control, in meters per second (m/s)",
        min=0.0,
        default=10.0,
        precision=3,
        update=zone_details_update,
    )
    max_acceleration_mps2: bpy.props.FloatProperty(
        name="Maximum Acceleration",
        description="Maximum acceleration this hardware can apply, in meters per second squared (m/s²)",
        min=0.0,
        default=2.0,
        precision=3,
        update=zone_details_update,
    )
    max_braking_mps2: bpy.props.FloatProperty(
        name="Maximum Braking",
        description="Maximum braking deceleration this hardware can apply, in meters per second squared (m/s²)",
        min=0.0,
        default=2.5,
        precision=3,
        update=zone_details_update,
    )


class CoasterMixerAction(bpy.types.PropertyGroup):
    """One step of a block or sensor sequence."""

    kind: bpy.props.EnumProperty(
        name="Action",
        items=ACTION_KIND_ITEMS,
        default="WAIT",
        update=block_group_update,
    )
    label: bpy.props.StringProperty(
        name="Label",
        description="Optional label (e.g. Unload, Load, Hold)",
        default="",
        update=block_group_update,
    )
    offset_meters: bpy.props.FloatProperty(
        name="Offset",
        description="Move target measured from the block span start, along the travel direction",
        min=0.0,
        subtype="DISTANCE",
        default=0.0,
        update=block_group_update,
    )
    speed_mps: bpy.props.FloatProperty(
        name="Speed",
        description="Controlled speed for this move, in meters per second (m/s)",
        min=0.0,
        default=1.5,
        precision=3,
        update=block_group_update,
    )
    duration_seconds: bpy.props.FloatProperty(
        name="Duration",
        description="How long to wait",
        min=0.0,
        subtype="TIME",
        default=5.0,
        update=block_group_update,
    )
    channel: bpy.props.StringProperty(
        name="Channel",
        description="Trigger channel name; read it in drivers via cm_trigger('channel')",
        default="",
        update=block_group_update,
    )
    value: bpy.props.FloatProperty(
        name="Value",
        description="Value the trigger channel is set to",
        default=1.0,
        precision=3,
        update=block_group_update,
    )


class CoasterMixerSensor(bpy.types.PropertyGroup):
    """A trackside point that fires a trigger sequence when the train front crosses it."""

    name: bpy.props.StringProperty(
        name="Name",
        default="Sensor",
        update=block_group_update,
    )
    position_meters: bpy.props.FloatProperty(
        name="Position",
        description="Distance from the start of the piece to the sensor point",
        min=0.0,
        subtype="DISTANCE",
        default=0.0,
        update=block_group_update,
    )
    actions: bpy.props.CollectionProperty(type=CoasterMixerAction)
    active_action_index: bpy.props.IntProperty(
        name="Active Action",
        min=0,
        default=0,
    )


class CoasterMixerBlockMember(bpy.types.PropertyGroup):
    piece: bpy.props.PointerProperty(
        name="Piece",
        description="Track piece containing the stop-capable zone",
        type=bpy.types.Object,
        poll=is_curve_object,
        update=block_group_update,
    )
    zone_index: bpy.props.IntProperty(
        name="Zone Index",
        description="Index of the stop-capable zone on the selected piece",
        min=0,
        default=0,
        update=block_group_update,
    )


class CoasterMixerBlockGroup(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(
        name="Name",
        default="Block",
        update=block_group_update,
    )
    start_route_meters: bpy.props.FloatProperty(
        name="Entry",
        description="Block entry relative to the route start; negative values lie behind the start arrow on a cyclic route",
        subtype="DISTANCE",
        default=-DEFAULT_ZONE_LENGTH_METERS,
        update=block_group_update,
    )
    end_route_meters: bpy.props.FloatProperty(
        name="Exit",
        description="Block exit relative to the route start; zero is the station-exit arrow",
        subtype="DISTANCE",
        default=0.0,
        update=block_group_update,
    )
    members: bpy.props.CollectionProperty(type=CoasterMixerBlockMember)
    active_member_index: bpy.props.IntProperty(
        name="Active Member",
        min=0,
        default=0,
        update=block_group_update,
    )
    control_tree: bpy.props.PointerProperty(
        name="Control Logic",
        description="Node graph executed when the train enters this occupancy block",
        type=bpy.types.NodeTree,
        poll=is_control_tree,
        update=block_group_update,
    )
    control_template: bpy.props.EnumProperty(
        name="Template",
        description="Recipe used to generate the editable control node tree",
        items=control_template_items_callback,
    )


def route_topology_update(_settings, _context):
    # A connection or switch change can reroute any coaster; retag the scene
    # root so its placement drivers re-evaluate.
    context = bpy.context
    track_object = resolve_active_track_object(context) if context is not None else None
    invalidate_route_cache()
    if track_object is not None:
        tag_track_placement_update(track_object)
    tag_redraw_view3d()


class CoasterMixerConnection(bpy.types.PropertyGroup):
    target: bpy.props.PointerProperty(
        name="Target Piece",
        description="Track piece the train continues onto",
        type=bpy.types.Object,
        poll=is_curve_object,
        update=route_topology_update,
    )
    target_end: bpy.props.EnumProperty(
        name="Enter At",
        description="Which end of the target piece the train enters; entering at End traverses it reversed",
        items=CONNECTION_END_ITEMS,
        default="START",
        update=route_topology_update,
    )


def follower_settings_update(settings, _context):
    PLACEMENT_CHANNEL_CACHE.clear()
    track_object = settings.track_object
    if track_object is not None and track_object.type == "CURVE":
        place_track_followers(track_object)
        refresh_view_layer()
    tag_redraw_view3d()


class CoasterMixerFollowerSettings(bpy.types.PropertyGroup):
    track_object: bpy.props.PointerProperty(
        name="Track",
        description="Coaster track this empty follows; placement drivers sample it by arc length",
        type=bpy.types.Object,
        poll=is_curve_object,
        update=follower_settings_update,
    )
    offset_meters: bpy.props.FloatProperty(
        name="Offset",
        description="Signed distance behind the train front in meters; negative values place helpers ahead of the train",
        subtype="DISTANCE",
        default=0.0,
        update=follower_settings_update,
    )
    vertical_offset_meters: bpy.props.FloatProperty(
        name="Vertical Offset",
        description="Local height above the sampled track position",
        subtype="DISTANCE",
        default=0.0,
        update=follower_settings_update,
    )


def is_camera_mount_object(settings, obj):
    return bool(
        obj is not None
        and obj.type == "EMPTY"
        and settings.track_object is not None
        and obj.coaster_mixer_follower.track_object == settings.track_object
        and obj != settings.track_object.coaster_mixer_track.driven_empty_object
        and not obj.get("coaster_mixer_camera_target", False)
    )


def camera_settings_update(settings, _context):
    camera_object = settings.id_data
    if camera_object is None or camera_object.type != "CAMERA":
        return
    mount_object = settings.mount_object
    if (
        mount_object is not None
        and mount_object.type == "EMPTY"
        and mount_object.coaster_mixer_follower.track_object == settings.track_object
    ):
        camera_object.parent = mount_object
    target_object = settings.target_object
    if target_object is not None and target_object.type == "EMPTY":
        target_settings = target_object.coaster_mixer_follower
        if target_settings.track_object != settings.track_object:
            assign_rna_property(target_settings, "track_object", settings.track_object)
        mount_offset = (
            mount_object.coaster_mixer_follower.offset_meters
            if mount_object is not None
            and mount_object.type == "EMPTY"
            and mount_object.coaster_mixer_follower.track_object == settings.track_object
            else 0.0
        )
        desired_offset = mount_offset - settings.look_ahead_meters
        if values_differ(target_settings.offset_meters, desired_offset):
            assign_rna_property(target_settings, "offset_meters", desired_offset)
        if values_differ(target_settings.vertical_offset_meters, settings.target_vertical_offset_meters):
            assign_rna_property(
                target_settings,
                "vertical_offset_meters",
                settings.target_vertical_offset_meters,
            )
    track_object = settings.track_object
    if track_object is not None and track_object.type == "CURVE":
        place_track_followers(track_object)
        refresh_view_layer()
    tag_redraw_view3d()


class CoasterMixerCameraSettings(bpy.types.PropertyGroup):
    track_object: bpy.props.PointerProperty(
        name="Track",
        description="Root coaster track driven by this ride camera",
        type=bpy.types.Object,
        poll=is_curve_object,
        update=camera_settings_update,
    )
    target_object: bpy.props.PointerProperty(
        name="Look Ahead Target",
        description="Track follower used as the camera aim target",
        type=bpy.types.Object,
        poll=is_empty_object,
        update=camera_settings_update,
    )
    mount_object: bpy.props.PointerProperty(
        name="Mounted Car",
        description="Train follower empty carrying this ride camera",
        type=bpy.types.Object,
        poll=is_camera_mount_object,
        update=camera_settings_update,
    )
    offset_xyz: bpy.props.FloatVectorProperty(
        name="Camera Offset",
        description="Seat-relative camera offset in the mounted follower's local XYZ axes",
        subtype="TRANSLATION",
        size=3,
        default=(0.0, 0.0, 1.6),
        update=camera_settings_update,
    )
    look_ahead_meters: bpy.props.FloatProperty(
        name="Look Ahead",
        description="Route distance ahead of the mounted car maintained by the camera target follower",
        min=0.1,
        subtype="DISTANCE",
        default=5.0,
        update=camera_settings_update,
    )
    target_vertical_offset_meters: bpy.props.FloatProperty(
        name="Target Height",
        description="Local height of the look-ahead follower so the camera aims horizontally at equal offsets",
        subtype="DISTANCE",
        default=1.6,
        update=camera_settings_update,
    )
    shake_enabled: bpy.props.BoolProperty(
        name="Camera Shake",
        description="Add deterministic camera movement driven by speed and lateral and vertical track load",
        default=False,
        update=camera_settings_update,
    )
    shake_factor: bpy.props.FloatProperty(
        name="Shake Factor",
        description="Multiplier for speed- and G-driven camera shake",
        min=0.0,
        max=10.0,
        soft_max=3.0,
        default=1.0,
        update=camera_settings_update,
    )


class CoasterMixerTrackSettings(bpy.types.PropertyGroup):
    zones: bpy.props.CollectionProperty(type=CoasterMixerZone)
    sensors: bpy.props.CollectionProperty(type=CoasterMixerSensor)
    block_groups: bpy.props.CollectionProperty(type=CoasterMixerBlockGroup)
    orientation_frame_mode: bpy.props.EnumProperty(
        name="Orientation Frame",
        description="How follower up orientation is carried along this curve piece",
        items=ORIENTATION_FRAME_ITEMS,
        default="Z_UP",
        update=track_settings_update,
    )
    bank_seam_mode: bpy.props.EnumProperty(
        name="Bank Continuity",
        description="How equivalent curve tilt angles are interpreted while sampling this piece",
        items=BANK_SEAM_MODE_ITEMS,
        default="AUTO",
        update=track_settings_update,
    )
    bank_seam_half_turns: bpy.props.IntProperty(
        name="Seam Half Turns",
        description="Manual bank winding at the cyclic seam; 1 adds 180 degrees and 2 adds 360 degrees",
        min=-200,
        max=200,
        default=0,
        update=track_settings_update,
    )
    train_length_meters: bpy.props.FloatProperty(
        name="Train Length",
        description="Overall train length used to interpret controlled-area timing",
        min=0.0,
        subtype="DISTANCE",
        default=10.0,
        update=track_settings_update,
    )
    train_weight_kilograms: bpy.props.FloatProperty(
        name="Train Weight",
        description="Approximate full train mass used by gravity sections",
        min=0.0,
        default=5000.0,
        update=track_settings_update,
    )
    driven_empty_object: bpy.props.PointerProperty(
        name="Driven Empty",
        description="Empty object that follows the ride curve and drives the train rig",
        type=bpy.types.Object,
        poll=is_empty_object,
        update=driven_empty_object_update,
    )
    train_front_route_meters: bpy.props.FloatProperty(
        name="Train Front",
        description="Position of the train front along the resolved route, in meters from the route start",
        min=0.0,
        subtype="DISTANCE",
        default=0.0,
        precision=3,
        update=track_settings_update,
    )
    start_connections: bpy.props.CollectionProperty(type=CoasterMixerConnection)
    start_active_index: bpy.props.IntProperty(
        name="Start Switch Position",
        description="Active exit used when the train leaves through the start of this piece; animate it to throw the switch",
        min=0,
        default=0,
        update=route_topology_update,
    )
    end_connections: bpy.props.CollectionProperty(type=CoasterMixerConnection)
    end_active_index: bpy.props.IntProperty(
        name="End Switch Position",
        description="Active exit used when the train leaves through the end of this piece; animate it to throw the switch",
        min=0,
        default=0,
        update=route_topology_update,
    )
    friction_coefficient: bpy.props.FloatProperty(
        name="Friction Coefficient",
        description="Rolling resistance coefficient applied to gravity and curvature-loaded wheel force in unactuated sections",
        min=0.0,
        default=0.015,
        precision=4,
        update=track_settings_update,
    )
    drag_coefficient: bpy.props.FloatProperty(
        name="Drag Coefficient (Cx)",
        description="Dimensionless aerodynamic drag coefficient Cx (Cd)",
        min=0.0,
        default=1.0,
        precision=3,
        update=track_settings_update,
    )
    frontal_area_m2: bpy.props.FloatProperty(
        name="Frontal Area",
        description="Train frontal area in square meters used to calculate aerodynamic drag",
        min=0.0,
        default=4.5,
        precision=2,
        update=track_settings_update,
    )
    active_zone_index: bpy.props.IntProperty(
        name="Active Zone",
        default=0,
        min=0,
        update=active_zone_index_update,
    )
    active_sensor_index: bpy.props.IntProperty(
        name="Active Sensor",
        default=0,
        min=0,
        update=active_zone_index_update,
    )
    active_block_group_index: bpy.props.IntProperty(
        name="Active Block",
        default=0,
        min=0,
        update=active_zone_index_update,
    )


class CoasterMixerSceneSettings(bpy.types.PropertyGroup):
    show_hardware_overlays: bpy.props.BoolProperty(
        name="Hardware",
        description="Show transport, friction-brake, trim-brake, and braking-influence overlays",
        default=True,
        update=scene_display_update,
    )
    show_block_overlays: bpy.props.BoolProperty(
        name="Blocks",
        description="Show block occupancy spans and their hold points",
        default=False,
        update=scene_display_update,
    )
    show_control_overlays: bpy.props.BoolProperty(
        name="Selected Control Node",
        description="Show the influence and target feedback for the selected control node",
        default=True,
        update=scene_display_update,
    )
    hide_overlays_while_playing: bpy.props.BoolProperty(
        name="Hide Overlays While Playing",
        description="Skip track overlay drawing during playback for better viewport FPS",
        default=True,
        update=scene_display_update,
    )
    simulation_enabled: bpy.props.BoolProperty(
        name="Enable Simulation",
        description="Advance the train simulation from the current timeline frame",
        default=False,
        update=simulation_enabled_update,
    )
    simulation_start_route_meters: bpy.props.FloatProperty(
        name="Start Position",
        description="Train front position along the route at the first timeline frame, in meters",
        min=0.0,
        subtype="DISTANCE",
        default=0.0,
        precision=3,
        update=simulation_start_update,
    )
    simulation_current_speed_mps: bpy.props.FloatProperty(
        name="Current Speed",
        description="Runtime signed train speed in meters per second (m/s)",
        default=0.0,
        precision=4,
    )
    simulation_stop_remaining_seconds: bpy.props.FloatProperty(
        name="Stop Time Remaining",
        description="Runtime time left for the current stop or block dwell",
        min=0.0,
        default=0.0,
        precision=4,
    )
    track_object: bpy.props.PointerProperty(
        name="Main Curve",
        description="Curve object that acts as the coaster guide track",
        type=bpy.types.Object,
        poll=is_curve_object,
        update=track_object_update,
    )
