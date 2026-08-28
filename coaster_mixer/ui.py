# SPDX-FileCopyrightText: 2026 Coaster Mixer contributors
# SPDX-License-Identifier: GPL-3.0-or-later

"""3D View panels, UI helpers, and Blender application handlers."""

from .operators import *

def draw_zone_controls(layout, zone, total_length):
    _start_distance, end_distance = resolve_zone_span(zone)

    column = layout.column()
    column.use_property_split = True
    column.use_property_decorate = False

    column.prop(zone, "name")
    column.prop(zone, "zone_type")

    span_column = column.column(align=True)
    span_column.prop(zone, "start_meters")
    span_column.prop(zone, "length_meters")
    column.label(text=f"Ends at {end_distance:.2f} m / piece {total_length:.2f} m")

    speed_hints = {
        "TRIM_BRAKE": "Hardware envelope; ride control chooses the requested speed.",
        "TRANSPORT": "Hardware envelope; control-node speeds are clamped to this range.",
        "FRICTION_BRAKE": "Maximum controllable speed; this actuator cannot push the train.",
    }
    control_column = column.column(align=True, heading="Control")
    if zone.zone_type == "TRANSPORT":
        control_column.prop(zone, "minimum_speed_mps")
    control_column.prop(zone, "target_speed_mps", text="Maximum Speed")
    if zone.zone_type == "TRANSPORT":
        control_column.prop(zone, "max_acceleration_mps2")
    control_column.prop(zone, "max_braking_mps2")
    column.label(text=speed_hints.get(zone.zone_type, ""), icon="INFO")

def action_summary(action):
    if action.kind == "MOVE":
        return f"Move to +{action.offset_meters:.1f} m @ {action.speed_mps:.1f} m/s"
    if action.kind == "WAIT":
        return f"Wait {action.duration_seconds:.1f} s"
    if action.kind == "TRIGGER":
        channel = action.channel.strip() or "?"
        return f"Trigger {channel} = {action.value:g}"
    return "Release Block"


class COASTERMIXER_UL_actions(bpy.types.UIList):
    def draw_item(self, _context, layout, _data, item, _icon, _active_data, _active_propname, index=0, _flt_flag=0):
        row = layout.row(align=True)
        label = item.label or action_summary(item)
        row.label(text=label, icon=ACTION_KIND_ICONS.get(item.kind, "DOT"))
        if item.label:
            row.label(text=action_summary(item))


def draw_action_sequence(layout, action_owner, owner_kind):
    """Actions UIList + editing controls, shared by blocks and sensors."""
    row = layout.row()
    row.template_list(
        "COASTERMIXER_UL_actions",
        f"coaster_mixer_actions_{owner_kind.lower()}",
        action_owner,
        "actions",
        action_owner,
        "active_action_index",
        rows=3,
    )
    buttons = row.column(align=True)
    kinds = ACTION_KIND_ITEMS if owner_kind == "BLOCK" else [
        item for item in ACTION_KIND_ITEMS if item[0] in SENSOR_ACTION_KINDS
    ]
    for kind_identifier, _kind_label, _kind_description in kinds:
        add_operator = buttons.operator(
            "coaster_mixer.add_action", text="", icon=ACTION_KIND_ICONS[kind_identifier]
        )
        add_operator.owner = owner_kind
        add_operator.kind = kind_identifier
    buttons.separator()
    move_up = buttons.operator("coaster_mixer.move_action", text="", icon="TRIA_UP")
    move_up.owner = owner_kind
    move_up.direction = -1
    move_down = buttons.operator("coaster_mixer.move_action", text="", icon="TRIA_DOWN")
    move_down.owner = owner_kind
    move_down.direction = 1
    remove_operator = buttons.operator("coaster_mixer.remove_action", text="", icon="REMOVE")
    remove_operator.owner = owner_kind

    if len(action_owner.actions) == 0:
        return

    action = action_owner.actions[clamp(action_owner.active_action_index, 0, len(action_owner.actions) - 1)]
    fields = layout.column()
    fields.use_property_split = True
    fields.use_property_decorate = False
    fields.prop(action, "kind")
    fields.prop(action, "label")
    if action.kind == "MOVE":
        fields.prop(action, "offset_meters")
        fields.prop(action, "speed_mps")
    elif action.kind == "WAIT":
        fields.prop(action, "duration_seconds")
    elif action.kind == "TRIGGER":
        fields.prop(action, "channel")
        fields.prop(action, "value")


def draw_block_groups(layout, context, track_settings):
    edit_object, edit_settings = resolve_edit_track_settings(context)
    can_create_from_zone = False
    if edit_object is not None and edit_settings is not None and len(edit_settings.zones) > 0:
        active_zone = edit_settings.zones[get_clamped_active_zone_index(edit_settings)]
        can_create_from_zone = active_zone.zone_type in STOP_CAPABLE_ZONE_TYPES
    create_row = layout.row()
    create_row.enabled = can_create_from_zone
    create_row.operator(
        "coaster_mixer.create_block_from_active_zone",
        text="Create from Selected Actuator",
        icon="ADD",
    )

    row = layout.row()
    row.template_list(
        "UI_UL_list",
        "coaster_mixer_blocks",
        track_settings,
        "block_groups",
        track_settings,
        "active_block_group_index",
        rows=3,
    )
    buttons = row.column(align=True)
    buttons.operator("coaster_mixer.add_block_group", text="", icon="ADD")
    buttons.operator("coaster_mixer.remove_block_group", text="", icon="REMOVE")

    if len(track_settings.block_groups) == 0:
        layout.label(text="Create an occupancy block, then assign its control graph.")
        return

    block_group = track_settings.block_groups[get_clamped_block_group_index(track_settings)]
    details = layout.box()
    name_column = details.column()
    name_column.use_property_split = True
    name_column.use_property_decorate = False
    name_column.prop(block_group, "name")
    name_column.prop(block_group, "start_route_meters")
    name_column.prop(block_group, "end_route_meters")

    # Members + resolved program state (span, warnings) from the route cache.
    program = None
    track_object = resolve_active_track_object(context)
    if track_object is not None:
        derived = get_route_derived_data(get_resolved_route(track_object))
        program = derived["programs_by_key"].get(f"block:{block_group.as_pointer()}")
    if program is not None:
        details.label(
            text=f"Span: {program['span'][0]:.1f} m - {program['span'][1]:.1f} m",
            icon="ARROW_LEFTRIGHT",
        )
        details.label(
            text=(
                f"Actuator limits: {program['drive_min_speed']:.1f}–{program['drive_max_speed']:.1f} m/s, "
                f"accel {program['max_acceleration']:.1f}, brake {program['max_braking']:.1f} m/s²"
            ),
            icon="MOD_PHYSICS",
        )
        for warning in program["warnings"]:
            details.label(text=warning, icon="ERROR")
    else:
        details.label(text="Not on the active route (no resolved span).", icon="INFO")

    details.label(text="Assigned Actuators", icon="LINKED")
    details.operator("coaster_mixer.add_active_zone_to_block", text="Assign Selected Actuator", icon="LINKED")
    for member_index, member in enumerate(block_group.members):
        row = details.row(align=True)
        piece = member.piece
        zone_name = "Missing zone"
        if piece is not None and piece.type == "CURVE":
            zones = piece.coaster_mixer_track.zones
            if 0 <= member.zone_index < len(zones):
                zone_name = f"{piece.name}: {zone_label(zones[member.zone_index], member.zone_index)}"
        row.label(text=zone_name, icon="LINKED")
        remove = row.operator("coaster_mixer.remove_block_member", text="", icon="X")
        remove.index = member_index

    logic_box = layout.box()
    logic_box.label(text="Ride Control Logic", icon="NODETREE")
    template_row = logic_box.row(align=True)
    template_row.prop(block_group, "control_template", text="Template")
    apply_template = template_row.operator(
        "coaster_mixer.apply_control_template", text="Apply…", icon="PRESET"
    )
    available_template_ids = {
        item[0] for item in control_template_items_callback(block_group, context)
    }
    apply_template.template = (
        block_group.control_template
        if block_group.control_template in available_template_ids
        else "CUSTOM"
    )
    logic_box.prop(block_group, "control_tree", text="Graph")
    logic_box.operator("coaster_mixer.edit_control_graph", icon="NODETREE")
    logic_box.label(text="Block occupancy, track actuators, and control logic are independent layers.", icon="INFO")


def get_startup_diagnostics(scene, track_object, track_settings):
    route = get_resolved_route(track_object)
    derived = get_route_derived_data(route)
    start = wrap_route_distance(route, scene.coaster_mixer_scene.simulation_start_route_meters)
    messages = []
    if not scene.coaster_mixer_scene.simulation_enabled:
        messages.append("Simulation is disabled; enable the checkbox in the Simulation panel header")
    if route["total_length"] <= 1.0e-8:
        return ["Route has no usable length"]
    active_program = next(
        (
            program for program in derived["programs"]
            if program["span"][0] - 1.0e-3 <= start <= program["span"][1] + 1.0e-3
            or (
                route["cyclic"] and start <= 1.0e-3
                and abs(program["span"][1] - route["total_length"]) <= 1.0e-3
            )
        ),
        None,
    )
    mismatched_program = next(
        (
            program for program in derived["programs"]
            if program is not active_program
            and any(span_start - 1.0e-3 <= start <= span_end + 1.0e-3 for span_start, span_end in program["actuator_spans"])
        ),
        None,
    )
    if mismatched_program is not None:
        messages.append(
            f"Start is on an actuator assigned to {mismatched_program['name']}, but outside that block's occupancy span"
        )
    can_move_in_program = active_program is not None and any(
        action["kind"] == "SET_TRANSPORT" and action["target_speed"] > SIMULATION_STOP_EPSILON
        for action in active_program["actions"]
    )
    has_power = any(
        item["zone"].zone_type == "TRANSPORT"
        and item["zone"].target_speed_mps > SIMULATION_STOP_EPSILON
        and get_zone_influence(route, track_settings, start, get_route_zone_span(item)) > 0.0
        for item in derived["zones"]
    )
    if active_program is not None:
        messages.extend(active_program["warnings"])
    if not can_move_in_program and not has_power:
        messages.append("Train cannot depart from rest: add a powered Transport actuator and Set Transport Target node")
    return messages


def draw_control_node_add_menu(self, context):
    space = context.space_data
    if getattr(space, "tree_type", "") != CONTROL_TREE_IDNAME:
        return
    layout = self.layout
    layout.separator()
    layout.label(text="Coaster Control")
    for node_type, label, icon in (
        ("COASTERMIXER_ND_block_entered", "Block Entered", "TRACKING"),
        ("COASTERMIXER_ND_set_transport", "Set Transport Target", "FORWARD"),
        ("COASTERMIXER_ND_set_brake", "Set Brake Target", "FREEZE"),
        ("COASTERMIXER_ND_set_brake_hold", "Brake to Hold Point", "PIVOT_CURSOR"),
        ("COASTERMIXER_ND_release_brake", "Release Brake", "UNLOCKED"),
        ("COASTERMIXER_ND_wait", "Wait", "PAUSE"),
        ("COASTERMIXER_ND_wait_position", "Wait for Position", "EMPTY_SINGLE_ARROW"),
        ("COASTERMIXER_ND_wait_speed", "Wait for Speed", "DRIVER"),
        ("COASTERMIXER_ND_trigger", "Set Trigger", "OUTLINER_OB_LIGHT"),
        ("COASTERMIXER_ND_dispatch", "Release Block", "PLAY"),
    ):
        operator = layout.operator("node.add_node", text=label, icon=icon)
        operator.type = node_type
        operator.use_transform = True


@persistent
def coaster_mixer_frame_change_handler(scene, _depsgraph=None):
    # Playback is a pure timeline lookup: scrubbing forward, backward, or
    # jumping frames all land on the same deterministic trajectory.
    scene_settings = getattr(scene, "coaster_mixer_scene", None)
    if scene_settings is None:
        return
    if scene_settings.simulation_enabled:
        apply_simulation_frame(scene)
    else:
        track_object = scene_settings.track_object
        if track_object is not None and track_object.type == "CURVE":
            # Baked playback animates the same front-distance property.
            place_track_followers(track_object)
    tag_redraw_view3d()


def ensure_frame_change_handler():
    handlers = bpy.app.handlers.frame_change_post
    if coaster_mixer_frame_change_handler not in handlers:
        handlers.append(coaster_mixer_frame_change_handler)


def remove_frame_change_handler():
    handlers = bpy.app.handlers.frame_change_post
    if coaster_mixer_frame_change_handler in handlers:
        handlers.remove(coaster_mixer_frame_change_handler)


@persistent
def coaster_mixer_depsgraph_update_handler(_scene, depsgraph):
    # Only the resolved-route cache needs eager invalidation: the curve,
    # route-zone, overlay, and placement caches all validate themselves
    # against curve signatures or the route key on access. Clearing
    # everything here used to wipe all caches on every simulation frame
    # (the sim's own update_tag lands in this handler), forcing a full
    # Python re-sample of every piece per frame.
    for update in depsgraph.updates:
        update_id = update.id
        if isinstance(update_id, bpy.types.Curve):
            # Curve Tools and Edit Mode can reorder spline points without
            # touching any Coaster Mixer property. Invalidate simulation and
            # all route-derived data so the new first point becomes meter 0.
            invalidate_route_cache()
            return
        if isinstance(update_id, bpy.types.Object) and update_id.type == "CURVE":
            if getattr(update, "is_updated_geometry", False) or getattr(update, "is_updated_transform", False):
                ROUTE_CACHE_BY_ROOT.clear()
                PLACEMENT_SAMPLE_CACHE.clear()
                PLACEMENT_CHANNEL_CACHE.clear()
                return


def ensure_depsgraph_update_handler():
    handlers = bpy.app.handlers.depsgraph_update_post
    if coaster_mixer_depsgraph_update_handler not in handlers:
        handlers.append(coaster_mixer_depsgraph_update_handler)


def remove_depsgraph_update_handler():
    handlers = bpy.app.handlers.depsgraph_update_post
    if coaster_mixer_depsgraph_update_handler in handlers:
        handlers.remove(coaster_mixer_depsgraph_update_handler)


def clear_runtime_caches():
    CURVE_CACHE_BY_OBJECT.clear()
    ROUTE_CACHE_BY_ROOT.clear()
    ROUTE_ZONE_CACHE_BY_ROOT.clear()
    OVERLAY_DRAW_CACHE_BY_OBJECT.clear()
    PLACEMENT_SAMPLE_CACHE.clear()
    PLACEMENT_CHANNEL_CACHE.clear()
    invalidate_simulation_trajectory()


@persistent
def coaster_mixer_undo_redo_handler(_scene=None, _depsgraph=None):
    # Undo/redo restore property values without firing update callbacks, so
    # every derived cache (route zones, stop specs, trajectory) may be stale.
    invalidate_route_cache()
    tag_redraw_view3d()


def ensure_undo_redo_handlers():
    for handlers in (bpy.app.handlers.undo_post, bpy.app.handlers.redo_post):
        if coaster_mixer_undo_redo_handler not in handlers:
            handlers.append(coaster_mixer_undo_redo_handler)


def remove_undo_redo_handlers():
    for handlers in (bpy.app.handlers.undo_post, bpy.app.handlers.redo_post):
        if coaster_mixer_undo_redo_handler in handlers:
            handlers.remove(coaster_mixer_undo_redo_handler)


@persistent
def coaster_mixer_load_post_handler(_filepath=None):
    # Pointers from the previous file may be reused by the new one, so the
    # caches must not survive a file load.
    clear_runtime_caches()
    ensure_driver_namespace()


def ensure_load_post_handler():
    handlers = bpy.app.handlers.load_post
    if coaster_mixer_load_post_handler not in handlers:
        handlers.append(coaster_mixer_load_post_handler)


def remove_load_post_handler():
    handlers = bpy.app.handlers.load_post
    if coaster_mixer_load_post_handler in handlers:
        handlers.remove(coaster_mixer_load_post_handler)


class CoasterMixerPanelMixin:
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Coaster"


class COASTERMIXER_PT_coaster(CoasterMixerPanelMixin, bpy.types.Panel):
    bl_label = "Coaster"
    bl_idname = "COASTERMIXER_PT_coaster"

    def draw(self, context):
        layout = self.layout
        scene_settings = context.scene.coaster_mixer_scene

        row = layout.row(align=True)
        row.prop(scene_settings, "track_object", text="Root")
        row.operator("coaster_mixer.set_root_from_active", text="", icon="EYEDROPPER")

        track_object = resolve_active_track_object(context)
        if track_object is None:
            layout.label(text="Pick a curve as the coaster root to start.", icon="INFO")
            return

        route = get_resolved_route(track_object)
        shape_label = "closed circuit" if route["cyclic"] else "open route"
        layout.label(
            text=f"{len(route['entries'])} piece(s), {route['total_length']:.1f} m, {shape_label}",
            icon="TRACKING" if route["cyclic"] else "INFO",
        )


class COASTERMIXER_PT_route(CoasterMixerPanelMixin, bpy.types.Panel):
    bl_label = "Route Pieces"
    bl_idname = "COASTERMIXER_PT_route"
    bl_parent_id = "COASTERMIXER_PT_coaster"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return resolve_active_track_object(context) is not None

    def draw(self, context):
        layout = self.layout
        track_object = resolve_active_track_object(context)
        route = get_resolved_route(track_object)
        if not route["entries"]:
            layout.label(text="No pieces on the route yet.")
            return

        edit_piece = resolve_edit_piece(context)
        column = layout.column(align=True)
        for entry in route["entries"]:
            piece = entry["object"]
            row = column.row(align=True)
            select_operator = row.operator(
                "coaster_mixer.select_piece",
                text=piece.name,
                icon="CURVE_DATA",
                depress=piece == edit_piece,
            )
            select_operator.piece_name = piece.name
            direction_label = "reversed" if entry["reversed"] else "forward"
            row.label(text=f"{direction_label}, {entry['length']:.1f} m")

        if not route["cyclic"]:
            layout.label(text="Open route: the train stops at the last piece.", icon="INFO")


class COASTERMIXER_PT_piece(CoasterMixerPanelMixin, bpy.types.Panel):
    bl_label = "Track Hardware"
    bl_idname = "COASTERMIXER_PT_piece"

    @classmethod
    def poll(cls, context):
        return resolve_edit_piece(context) is not None

    def draw(self, context):
        layout = self.layout
        piece_object, piece_settings = resolve_edit_track_settings(context)
        if piece_object is None or piece_settings is None:
            return

        track_object = resolve_active_track_object(context)
        suffix = "  (root)" if piece_object == track_object else ""
        layout.label(text=f"{piece_object.name}{suffix}", icon="CURVE_DATA")

        spline = get_primary_spline(piece_object)
        if spline is None:
            layout.label(text="This curve has no spline data.", icon="ERROR")
            return

        banking_box = layout.box()
        banking_box.label(text="Curve Banking", icon="DRIVER_ROTATIONAL_DIFFERENCE")
        banking_box.prop(piece_settings, "orientation_frame_mode")
        if piece_settings.orientation_frame_mode == "CONTINUOUS_Z_UP":
            banking_box.label(text="Matches Z Up without flipping at vertical tangents.", icon="INFO")
        elif piece_settings.orientation_frame_mode == "MINIMUM_TWIST":
            banking_box.label(text="Vertical-safe: tilt is applied as physical banking.", icon="INFO")
        banking_box.prop(piece_settings, "bank_seam_mode")
        if piece_settings.bank_seam_mode == "MANUAL":
            banking_box.prop(piece_settings, "bank_seam_half_turns")
        if spline.use_cyclic_u:
            if piece_settings.bank_seam_mode == "AUTO":
                banking_box.label(text="Matching 180°/360° winding is preserved automatically.", icon="INFO")
            elif piece_settings.bank_seam_mode == "MANUAL":
                seam_degrees = piece_settings.bank_seam_half_turns * 180.0
                banking_box.label(text=f"Seam closes at start tilt + {seam_degrees:g}°.", icon="INFO")

        layout.label(text="Actuators", icon="MODIFIER")
        row = layout.row()
        row.template_list(
            "COASTERMIXER_UL_zones",
            "",
            piece_settings,
            "zones",
            piece_settings,
            "active_zone_index",
            rows=4,
        )
        buttons = row.column(align=True)
        buttons.operator("coaster_mixer.add_zone", text="", icon="ADD")
        buttons.operator("coaster_mixer.duplicate_zone", text="", icon="DUPLICATE")
        buttons.operator("coaster_mixer.remove_zone", text="", icon="REMOVE")

        if len(piece_settings.zones) == 0:
            layout.label(text="Add a transport, friction brake, or trim brake.", icon="INFO")
        else:
            active_index = get_clamped_active_zone_index(piece_settings)
            zone = piece_settings.zones[active_index]
            total_length = build_curve_cache(piece_object)["total_length"]

            details = layout.box()
            details.label(
                text=zone_label(zone, active_index),
                icon=ZONE_TYPE_ICONS.get(zone.zone_type, "FORCE_HARMONIC"),
            )
            draw_zone_controls(details, zone, total_length)

        sensor_box = layout.box()
        sensor_box.label(text="Sensors", icon="ANTIALIASED")
        row = sensor_box.row()
        row.template_list(
            "UI_UL_list",
            "coaster_mixer_sensors",
            piece_settings,
            "sensors",
            piece_settings,
            "active_sensor_index",
            rows=2,
        )
        sensor_buttons = row.column(align=True)
        sensor_buttons.operator("coaster_mixer.add_sensor", text="", icon="ADD")
        sensor_buttons.operator("coaster_mixer.remove_sensor", text="", icon="REMOVE")
        if len(piece_settings.sensors) > 0:
            sensor = piece_settings.sensors[
                clamp(piece_settings.active_sensor_index, 0, len(piece_settings.sensors) - 1)
            ]
            sensor_column = sensor_box.column()
            sensor_column.use_property_split = True
            sensor_column.use_property_decorate = False
            sensor_column.prop(sensor, "name")
            sensor_column.prop(sensor, "position_meters")
            draw_action_sequence(sensor_box, sensor, "SENSOR")


class COASTERMIXER_PT_connections(CoasterMixerPanelMixin, bpy.types.Panel):
    bl_label = "Connections & Switches"
    bl_idname = "COASTERMIXER_PT_connections"
    bl_parent_id = "COASTERMIXER_PT_piece"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        _piece_object, piece_settings = resolve_edit_track_settings(context)
        if piece_settings is None:
            return

        for end_identifier, end_label in (("START", "Piece Start"), ("END", "Piece End")):
            end_box = layout.box()
            end_box.label(text=end_label)
            connections = get_connection_list(piece_settings, end_identifier)
            for connection_index, connection in enumerate(connections):
                row = end_box.row(align=True)
                row.prop(connection, "target", text="")
                row.prop(connection, "target_end", text="")
                remove_operator = row.operator("coaster_mixer.remove_connection", text="", icon="X")
                remove_operator.end = end_identifier
                remove_operator.index = connection_index
            add_operator = end_box.operator("coaster_mixer.add_connection", icon="ADD")
            add_operator.end = end_identifier
            if len(connections) > 1:
                index_property = "end_active_index" if end_identifier == "END" else "start_active_index"
                end_box.prop(piece_settings, index_property)
                active_connection = get_active_connection(piece_settings, end_identifier)
                if active_connection is not None and active_connection.target is not None:
                    end_box.label(text=f"Active exit: {active_connection.target.name}")
                else:
                    end_box.label(text="Active exit: none (dead end)", icon="ERROR")


class COASTERMIXER_PT_train(CoasterMixerPanelMixin, bpy.types.Panel):
    bl_label = "Train"
    bl_idname = "COASTERMIXER_PT_train"

    @classmethod
    def poll(cls, context):
        return resolve_active_track_object(context) is not None

    def draw(self, context):
        layout = self.layout
        track_object, track_settings = resolve_active_track_settings(context)
        if track_settings is None:
            return

        physics_box = layout.box()
        physics_box.label(text="Physical Model", icon="PHYSICS")
        column = physics_box.column()
        column.use_property_split = True
        column.use_property_decorate = False
        column.prop(track_settings, "train_length_meters")
        column.prop(track_settings, "train_weight_kilograms")
        column.prop(track_settings, "friction_coefficient")
        column.prop(track_settings, "drag_coefficient")
        column.prop(track_settings, "frontal_area_m2")
        rig_box = layout.box()
        rig_box.label(text="Train Rig", icon="CONSTRAINT_BONE")
        rig_column = rig_box.column()
        rig_column.use_property_split = True
        rig_column.use_property_decorate = False
        rig_column.prop(track_settings, "driven_empty_object")

        if track_settings.driven_empty_object is not None:
            rig_box.operator("coaster_mixer.setup_driven_empty", text="Attach Driven Empty", icon="DRIVER")
        rig_box.label(text=get_follower_setup_status(track_object, track_settings.driven_empty_object))

        follower_box = layout.box()
        follower_box.label(text="Followers", icon="LINKED")
        followers = collect_track_followers(track_object)
        if followers:
            for follower_object in followers:
                row = follower_box.row(align=True)
                row.label(text=follower_object.name, icon="EMPTY_AXIS")
                row.prop(follower_object.coaster_mixer_follower, "offset_meters", text="")
                detach = row.operator("coaster_mixer.detach_follower", text="", icon="X")
                detach.empty_name = follower_object.name
        else:
            follower_box.label(text="No follower empties attached yet.")
        button_row = follower_box.row(align=True)
        button_row.operator("coaster_mixer.create_train_followers", text="Create Cars", icon="OUTLINER_OB_EMPTY")
        button_row.operator("coaster_mixer.attach_selected_followers", text="Attach Selected", icon="LINKED")


class COASTERMIXER_PT_camera(CoasterMixerPanelMixin, bpy.types.Panel):
    bl_label = "Ride Cameras"
    bl_idname = "COASTERMIXER_PT_camera"
    bl_parent_id = "COASTERMIXER_PT_train"

    @classmethod
    def poll(cls, context):
        return resolve_active_track_object(context) is not None

    def draw(self, context):
        layout = self.layout
        track_object = resolve_active_track_object(context)
        cameras = collect_ride_cameras(track_object)
        if not cameras:
            layout.label(text="No ride camera created yet.")
        for camera_object in cameras:
            camera_box = layout.box()
            camera_box.label(text=camera_object.name, icon="OUTLINER_OB_CAMERA")
            settings = camera_object.coaster_mixer_camera
            column = camera_box.column()
            column.use_property_split = True
            column.use_property_decorate = False
            column.prop(settings, "mount_object")
            column.prop(settings, "offset_xyz")
            column.prop(camera_object.data, "lens")
            column.prop(settings, "look_ahead_meters")
            column.prop(settings, "target_vertical_offset_meters")
            column.prop(settings, "shake_enabled")
            shake_column = column.column()
            shake_column.enabled = settings.shake_enabled
            shake_column.prop(settings, "shake_factor")
        layout.operator("coaster_mixer.create_train_camera", text="Create Ride Camera", icon="OUTLINER_OB_CAMERA")


class COASTERMIXER_PT_simulation(CoasterMixerPanelMixin, bpy.types.Panel):
    bl_label = "Simulation"
    bl_idname = "COASTERMIXER_PT_simulation"

    @classmethod
    def poll(cls, context):
        return resolve_active_track_object(context) is not None

    def draw_header(self, context):
        self.layout.prop(context.scene.coaster_mixer_scene, "simulation_enabled", text="")

    def draw(self, context):
        layout = self.layout
        scene_settings = context.scene.coaster_mixer_scene
        track_object, track_settings = resolve_active_track_settings(context)
        if track_settings is None:
            return

        column = layout.column()
        column.use_property_split = True
        column.use_property_decorate = False
        start_row = column.row(align=True)
        start_row.prop(scene_settings, "simulation_start_route_meters")
        start_row.operator("coaster_mixer.snap_start_to_station", text="", icon="HOME")
        column.prop(track_settings, "train_front_route_meters")
        overlay_box = layout.box()
        overlay_box.label(text="Viewport Overlays", icon="OVERLAY")
        overlay_row = overlay_box.row(align=True)
        overlay_row.prop(scene_settings, "show_hardware_overlays", toggle=True)
        overlay_row.prop(scene_settings, "show_block_overlays", toggle=True)
        overlay_box.prop(scene_settings, "show_control_overlays")
        overlay_box.prop(scene_settings, "hide_overlays_while_playing")

        layout.operator("coaster_mixer.reset_simulation", icon="FILE_REFRESH")
        layout.label(text=f"Speed: {scene_settings.simulation_current_speed_mps:.2f} m/s")
        diagnostics = get_startup_diagnostics(context.scene, track_object, track_settings)
        if diagnostics:
            diagnostic_box = layout.box()
            diagnostic_box.label(text="Startup Check", icon="ERROR")
            for message in diagnostics:
                diagnostic_box.label(text=message, icon="DOT")
        else:
            layout.label(text="Startup check passed", icon="CHECKMARK")
        if scene_settings.simulation_stop_remaining_seconds > 0.0:
            layout.label(text=f"Stop remaining: {scene_settings.simulation_stop_remaining_seconds:.2f} s")

        trajectory_cache = SIMULATION_TRAJECTORY_CACHE
        if (
            trajectory_cache is not None
            and trajectory_cache["key"][0] == track_object.as_pointer()
            and trajectory_cache["cycle_length"]
        ):
            cycle_frames = trajectory_cache["cycle_length"]
            cycle_seconds = cycle_frames / get_scene_fps(context.scene)
            layout.label(
                text=f"Loop: {cycle_frames} frames ({cycle_seconds:.1f} s)",
                icon="FILE_REFRESH",
            )

        bake_box = layout.box()
        bake_start_frame, bake_end_frame, bake_range_label = get_simulation_bake_frame_range(context.scene)
        bake_box.label(text=f"{bake_range_label} range: {bake_start_frame} - {bake_end_frame}", icon="ACTION")
        bake_box.operator("coaster_mixer.bake_simulation", icon="ACTION")
        if has_baked_path_animation(track_object):
            bake_box.label(text="Baked train-front keys are active", icon="KEYFRAME_HLT")
            bake_box.operator("coaster_mixer.clear_baked_simulation", icon="TRASH")


class COASTERMIXER_PT_blocks(CoasterMixerPanelMixin, bpy.types.Panel):
    bl_label = "Blocks"
    bl_idname = "COASTERMIXER_PT_blocks"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        return resolve_active_track_object(context) is not None

    def draw(self, context):
        _track_object, track_settings = resolve_active_track_settings(context)
        if track_settings is None:
            return
        draw_block_groups(self.layout, context, track_settings)


class COASTERMIXER_PT_setup_utilities(CoasterMixerPanelMixin, bpy.types.Panel):
    bl_label = "Reset & Utilities"
    bl_idname = "COASTERMIXER_PT_setup_utilities"
    bl_parent_id = "COASTERMIXER_PT_coaster"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(cls, context):
        active_object = getattr(context, "object", None)
        return (
            active_object is not None and active_object.type == "CURVE"
        ) or resolve_active_track_object(context) is not None

    def draw(self, context):
        layout = self.layout
        active_object = getattr(context, "object", None)
        target = (
            active_object
            if active_object is not None and active_object.type == "CURVE"
            else resolve_active_track_object(context)
        )
        if target is None:
            return
        layout.label(text=f"Target: {target.name}", icon="CURVE_DATA")
        layout.operator(
            "coaster_mixer.clear_curve_setup",
            text="Clear / Reset Curve Setup…",
            icon="FILE_REFRESH",
        )
