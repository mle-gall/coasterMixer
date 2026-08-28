# SPDX-FileCopyrightText: 2026 Coaster Mixer contributors
# SPDX-License-Identifier: GPL-3.0-or-later

"""Authoring, setup, simulation, and bake operators."""

from .model import *

class COASTERMIXER_UL_zones(bpy.types.UIList):
    def draw_item(self, _context, layout, data, item, _icon, _active_data, _active_propname, index=0, _flt_flag=0):
        row = layout.row(align=True)
        row.label(text=zone_label(item, index), icon=ZONE_TYPE_ICONS.get(item.zone_type, "FORCE_HARMONIC"))
        row.label(text=f"{item.start_meters:.2f} m | {item.length_meters:.2f} m")


def create_zone(track_object, track_settings, zone_type, source_zone=None):
    sync_track_zones_to_curve(track_object, track_settings)
    total_length = get_track_total_length(track_object)
    next_start_meters = 0.0
    if source_zone is None:
        if len(track_settings.zones) > 0:
            active_index = get_clamped_active_zone_index(track_settings)
            active_zone = track_settings.zones[active_index]
            next_start_meters = resolve_zone_span(active_zone)[1]
            if is_track_cyclic(track_object) and total_length > 1.0e-8:
                # The initial station ends at the cyclic seam. Sequential
                # authoring should continue from route zero, not create a
                # zero-length actuator at total_length.
                if abs(next_start_meters - total_length) <= 1.0e-3:
                    next_start_meters = 0.0
                else:
                    next_start_meters %= total_length
        elif is_track_cyclic(track_object):
            # Route zero is the station exit: seed the first hardware run on
            # the final segment, immediately behind the start arrow.
            next_start_meters = max(total_length - DEFAULT_ZONE_LENGTH_METERS, 0.0)
    source_values = None
    if source_zone is not None:
        source_values = {
            "name": source_zone.name,
            "start_meters": source_zone.start_meters,
            "length_meters": source_zone.length_meters,
            "minimum_speed_mps": source_zone.minimum_speed_mps,
            "target_speed_mps": source_zone.target_speed_mps,
            "max_acceleration_mps2": source_zone.max_acceleration_mps2,
            "max_braking_mps2": source_zone.max_braking_mps2,
        }
    zone = track_settings.zones.add()
    zone.zone_type = zone_type
    if source_values is not None:
        for attribute, value in source_values.items():
            setattr(zone, attribute, value)
    else:
        zone.start_meters = clamp(next_start_meters, 0.0, total_length)
        zone.length_meters = min(DEFAULT_ZONE_LENGTH_METERS, max(total_length - zone.start_meters, 0.0))
        if zone_type == "TRANSPORT":
            zone.minimum_speed_mps = 0.5
            zone.target_speed_mps = 10.0
            zone.max_acceleration_mps2 = 2.0
            zone.max_braking_mps2 = 2.5
        elif zone_type == "FRICTION_BRAKE":
            zone.max_acceleration_mps2 = 0.0
            zone.target_speed_mps = 8.0
            zone.max_braking_mps2 = 3.0
        else:
            zone.max_acceleration_mps2 = 0.0
            zone.target_speed_mps = 15.0
            zone.max_braking_mps2 = 1.5
    track_settings.active_zone_index = len(track_settings.zones) - 1
    sync_zone_to_curve(track_object, zone)
    invalidate_route_cache()
    tag_redraw_view3d()
    return zone


class COASTERMIXER_OT_add_zone(bpy.types.Operator):
    bl_idname = "coaster_mixer.add_zone"
    bl_label = "Add Track Actuator"
    bl_description = "Add a physical brake, drive, launch, or lift actuator to the active curve"
    bl_options = {"REGISTER", "UNDO"}

    zone_type: bpy.props.EnumProperty(
        name="Type",
        items=ZONE_TYPE_ITEMS,
        default="FRICTION_BRAKE",
    )

    @classmethod
    def poll(cls, context):
        _track_object, track_settings = resolve_edit_track_settings(context)
        return track_settings is not None

    def execute(self, context):
        track_object, track_settings = resolve_edit_track_settings(context)
        if track_object is None or track_settings is None:
            self.report({"WARNING"}, "Select a curve object first")
            return {"CANCELLED"}

        create_zone(track_object, track_settings, self.zone_type)
        return {"FINISHED"}

    def draw(self, _context):
        self.layout.prop(self, "zone_type", text="Type")

    def invoke(self, context, _event):
        return context.window_manager.invoke_props_dialog(self, width=300)


class COASTERMIXER_OT_duplicate_zone(bpy.types.Operator):
    bl_idname = "coaster_mixer.duplicate_zone"
    bl_label = "Duplicate Zone"
    bl_description = "Duplicate the active hardware zone"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        _track_object, track_settings = resolve_edit_track_settings(context)
        return track_settings is not None and len(track_settings.zones) > 0

    def execute(self, context):
        track_object, track_settings = resolve_edit_track_settings(context)
        if track_object is None or track_settings is None or len(track_settings.zones) == 0:
            self.report({"WARNING"}, "No zone to duplicate")
            return {"CANCELLED"}

        ensure_active_zone_index(track_settings)
        source_zone = track_settings.zones[track_settings.active_zone_index]
        create_zone(track_object, track_settings, source_zone.zone_type, source_zone=source_zone)
        return {"FINISHED"}


class COASTERMIXER_OT_remove_zone(bpy.types.Operator):
    bl_idname = "coaster_mixer.remove_zone"
    bl_label = "Remove Zone"
    bl_description = "Remove the active hardware zone"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        _track_object, track_settings = resolve_edit_track_settings(context)
        return track_settings is not None and len(track_settings.zones) > 0

    def execute(self, context):
        track_object, track_settings = resolve_edit_track_settings(context)
        if track_settings is None or len(track_settings.zones) == 0:
            self.report({"WARNING"}, "No zone to remove")
            return {"CANCELLED"}

        ensure_active_zone_index(track_settings)
        active_index = track_settings.active_zone_index
        track_settings.zones.remove(active_index)
        ensure_active_zone_index(track_settings)
        if track_object is not None:
            sync_track_zones_to_curve(track_object, track_settings)
            invalidate_route_cache()
        tag_redraw_view3d()
        return {"FINISHED"}


def get_clamped_block_group_index(track_settings):
    count = len(track_settings.block_groups)
    if count == 0:
        return 0
    return clamp(track_settings.active_block_group_index, 0, count - 1)


def create_control_template_tree(
    name, template, block_length, hold_offset, hold_duration, target_speed,
    acceleration, transport_braking, brake_target_speed, braking, response_curve,
):
    tree = bpy.data.node_groups.new(name=f"{name} Control", type=CONTROL_TREE_IDNAME)
    specs = [("COASTERMIXER_ND_block_entered", {})]
    if template in {"STOPPED_LAUNCH", "LOAD_STATION", "UNLOAD_STATION"}:
        specs.extend([
            ("COASTERMIXER_ND_set_brake_hold", {
                "offset_meters": hold_offset, "braking_mps2": braking,
                "response_curve": response_curve,
            }),
            ("COASTERMIXER_ND_wait", {"duration_seconds": hold_duration}),
            ("COASTERMIXER_ND_release_brake", {}),
            ("COASTERMIXER_ND_set_transport", {
                "speed_mps": target_speed, "acceleration_mps2": acceleration,
                "braking_mps2": transport_braking, "response_curve": response_curve,
            }),
            ("COASTERMIXER_ND_wait_position", {"offset_meters": block_length}),
            ("COASTERMIXER_ND_dispatch", {}),
        ])
    elif template in {"ROLLING_LAUNCH", "STANDARD_LIFT"}:
        specs.extend([
            ("COASTERMIXER_ND_set_transport", {
                "speed_mps": target_speed, "acceleration_mps2": acceleration,
                "braking_mps2": transport_braking, "response_curve": response_curve,
            }),
            ("COASTERMIXER_ND_wait_position", {"offset_meters": block_length}),
            ("COASTERMIXER_ND_dispatch", {}),
        ])
    elif template == "TRIM_BRAKE":
        specs.extend([
            ("COASTERMIXER_ND_set_brake", {
                "speed_mps": brake_target_speed, "braking_mps2": braking,
                "response_curve": response_curve,
            }),
            ("COASTERMIXER_ND_wait_position", {"offset_meters": block_length}),
            ("COASTERMIXER_ND_release_brake", {}),
            ("COASTERMIXER_ND_dispatch", {}),
        ])

    previous = None
    for index, (node_type, fields) in enumerate(specs):
        node = tree.nodes.new(node_type)
        node.location = (index * 220.0, 0.0)
        for field_name, value in fields.items():
            setattr(node, field_name, value)
        if previous is not None:
            tree.links.new(previous.outputs["Then"], node.inputs["In"])
        previous = node
    return tree


def create_default_control_tree(name, move_offset=None):
    hold_offset = max(move_offset if move_offset is not None else DEFAULT_ZONE_LENGTH_METERS, 0.0)
    return create_control_template_tree(
        name, "LOAD_STATION", max(hold_offset, DEFAULT_ZONE_LENGTH_METERS), hold_offset,
        5.0, DEFAULT_STATION_DISPATCH_SPEED_MPS,
        DEFAULT_STATION_ACCELERATION_MPS2, DEFAULT_STATION_BRAKING_MPS2,
        4.0, 1.5, "LINEAR",
    )


def create_custom_control_tree(name):
    tree = bpy.data.node_groups.new(name=f"{name} Control", type=CONTROL_TREE_IDNAME)
    tree.nodes.new("COASTERMIXER_ND_block_entered")
    return tree


class COASTERMIXER_OT_apply_control_template(bpy.types.Operator):
    bl_idname = "coaster_mixer.apply_control_template"
    bl_label = "Apply Control Template"
    bl_description = "Generate an editable control node tree from a parameterized ride-control template"
    bl_options = {"REGISTER", "UNDO"}

    template: bpy.props.EnumProperty(name="Template", items=control_template_items_callback)
    hold_offset_meters: bpy.props.FloatProperty(name="Hold Position", description="Hold position in meters from the block entry", min=0.0, subtype="DISTANCE", default=10.0)
    hold_duration_seconds: bpy.props.FloatProperty(name="Hold Duration", description="Time held at the stop in seconds", min=0.0, default=5.0)
    target_speed_mps: bpy.props.FloatProperty(name="Target Speed", description="Requested transport target in meters per second (m/s)", min=0.0, default=2.0)
    acceleration_mps2: bpy.props.FloatProperty(name="Acceleration", description="Requested transport acceleration in meters per second squared (m/s²)", min=0.0, default=1.0)
    transport_braking_mps2: bpy.props.FloatProperty(name="Transport Braking", description="Requested transport deceleration in meters per second squared (m/s²)", min=0.0, default=1.5)
    brake_target_speed_mps: bpy.props.FloatProperty(name="Trim Speed", description="Non-stopping brake speed cap in meters per second (m/s)", min=0.0, default=8.0)
    braking_mps2: bpy.props.FloatProperty(name="Brake Deceleration", description="Requested brake deceleration in meters per second squared (m/s²)", min=0.0, default=1.5)
    response_curve: bpy.props.EnumProperty(name="Response", items=CONTROL_RESPONSE_CURVE_ITEMS, default="LINEAR")

    @classmethod
    def poll(cls, context):
        track_object, track_settings = resolve_active_track_settings(context)
        return track_settings is not None and len(track_settings.block_groups) > 0

    def draw(self, _context):
        layout = self.layout
        layout.use_property_split = True
        layout.prop(self, "template")
        if self.template in {"STOPPED_LAUNCH", "LOAD_STATION", "UNLOAD_STATION"}:
            layout.prop(self, "hold_offset_meters")
            layout.prop(self, "hold_duration_seconds")
            layout.prop(self, "target_speed_mps")
            layout.prop(self, "acceleration_mps2")
            layout.prop(self, "transport_braking_mps2")
            layout.prop(self, "braking_mps2")
            layout.prop(self, "response_curve")
        elif self.template in {"ROLLING_LAUNCH", "STANDARD_LIFT"}:
            layout.prop(self, "target_speed_mps")
            layout.prop(self, "acceleration_mps2")
            layout.prop(self, "transport_braking_mps2")
            layout.prop(self, "response_curve")
        elif self.template == "TRIM_BRAKE":
            layout.prop(self, "brake_target_speed_mps")
            layout.prop(self, "braking_mps2")
            layout.prop(self, "response_curve")

    def invoke(self, context, _event):
        track_object, track_settings = resolve_active_track_settings(context)
        block = track_settings.block_groups[get_clamped_block_group_index(track_settings)]
        available_ids = {item[0] for item in control_template_items_callback(self, context)}
        if block.control_template in available_ids and block.control_template != "CUSTOM":
            self.template = block.control_template
        block_length = abs(block.end_route_meters - block.start_route_meters)
        self.hold_offset_meters = block_length
        tree = block.control_tree
        wait_node = None
        if tree is not None:
            hold_node = next(
                (node for node in tree.nodes if node.bl_idname == "COASTERMIXER_ND_set_brake_hold"),
                None,
            )
            wait_node = next(
                (node for node in tree.nodes if node.bl_idname == "COASTERMIXER_ND_wait"),
                None,
            )
            transport_nodes = sorted(
                (node for node in tree.nodes if node.bl_idname == "COASTERMIXER_ND_set_transport"),
                key=lambda node: node.location.x,
            )
            brake_node = next(
                (node for node in tree.nodes if node.bl_idname == "COASTERMIXER_ND_set_brake"),
                None,
            )
            if hold_node is not None:
                self.hold_offset_meters = hold_node.offset_meters
                self.braking_mps2 = hold_node.braking_mps2
                self.response_curve = hold_node.response_curve
            if wait_node is not None:
                self.hold_duration_seconds = wait_node.duration_seconds
            if transport_nodes:
                departure_node = transport_nodes[-1]
                self.target_speed_mps = departure_node.speed_mps
                self.acceleration_mps2 = departure_node.acceleration_mps2
                self.transport_braking_mps2 = departure_node.braking_mps2
                self.response_curve = departure_node.response_curve
            if brake_node is not None:
                self.brake_target_speed_mps = brake_node.speed_mps
                self.braking_mps2 = brake_node.braking_mps2
                self.response_curve = brake_node.response_curve
        if track_object is not None:
            route = get_resolved_route(track_object)
            program = get_route_derived_data(route)["programs_by_key"].get(
                f"block:{block.as_pointer()}"
            )
            if program is not None and program["friction_brake_spans"]:
                self.hold_offset_meters = clamp(
                    max(span[1] for span in program["friction_brake_spans"]) - program["span"][0],
                    0.0,
                    block_length,
                )
        if self.template == "UNLOAD_STATION":
            if wait_node is None:
                self.hold_duration_seconds = 2.0
        return context.window_manager.invoke_props_dialog(self, width=430)

    def execute(self, context):
        _track_object, track_settings = resolve_active_track_settings(context)
        if track_settings is None or len(track_settings.block_groups) == 0:
            return {"CANCELLED"}
        if self.template == "CUSTOM":
            self.report({"WARNING"}, "Custom keeps the current graph; choose a generated template")
            return {"CANCELLED"}
        block = track_settings.block_groups[get_clamped_block_group_index(track_settings)]
        block_length = max(abs(block.end_route_meters - block.start_route_meters), 0.0)
        old_tree = block.control_tree
        new_tree = create_control_template_tree(
            block.name, self.template, block_length,
            clamp(self.hold_offset_meters, 0.0, block_length), self.hold_duration_seconds,
            self.target_speed_mps, self.acceleration_mps2, self.transport_braking_mps2,
            self.brake_target_speed_mps, self.braking_mps2, self.response_curve,
        )
        block.control_tree = new_tree
        block.control_template = self.template
        if old_tree is not None and old_tree.users == 0:
            bpy.data.node_groups.remove(old_tree)
        block_group_update(track_settings, context)
        self.report({"INFO"}, f"Applied {dict((key, label) for key, label, _ in CONTROL_TEMPLATE_ITEMS)[self.template]}")
        return {"FINISHED"}


class COASTERMIXER_OT_add_block_group(bpy.types.Operator):
    bl_idname = "coaster_mixer.add_block_group"
    bl_label = "Add Block"
    bl_description = "Create a manual block group on the coaster root"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        _track_object, track_settings = resolve_active_track_settings(context)
        return track_settings is not None

    def execute(self, context):
        track_object, track_settings = resolve_active_track_settings(context)
        block_group = track_settings.block_groups.add()
        block_group.name = f"Block {len(track_settings.block_groups):02d}"
        route = get_resolved_route(track_object)
        route_length = route["total_length"]
        first_block = len(track_settings.block_groups) == 1
        if route["cyclic"] and first_block:
            block_length = min(DEFAULT_ZONE_LENGTH_METERS, route_length)
            block_group.start_route_meters = -block_length
            block_group.end_route_meters = 0.0
            context.scene.coaster_mixer_scene.simulation_start_route_meters = max(
                route_length - block_length * 0.5, 0.0
            )
        else:
            block_group.start_route_meters = 0.0
            block_group.end_route_meters = min(DEFAULT_ZONE_LENGTH_METERS, route_length)
        block_group.control_tree = create_custom_control_tree(block_group.name)
        block_group.control_template = "CUSTOM"
        track_settings.active_block_group_index = len(track_settings.block_groups) - 1
        block_group_update(track_settings, context)
        return {"FINISHED"}


class COASTERMIXER_OT_remove_block_group(bpy.types.Operator):
    bl_idname = "coaster_mixer.remove_block_group"
    bl_label = "Remove Block"
    bl_description = "Remove the active manual block group"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        _track_object, track_settings = resolve_active_track_settings(context)
        return track_settings is not None and len(track_settings.block_groups) > 0

    def execute(self, context):
        _track_object, track_settings = resolve_active_track_settings(context)
        index = get_clamped_block_group_index(track_settings)
        track_settings.block_groups.remove(index)
        if len(track_settings.block_groups) > 0:
            track_settings.active_block_group_index = get_clamped_block_group_index(track_settings)
        block_group_update(track_settings, context)
        return {"FINISHED"}


class COASTERMIXER_OT_edit_control_graph(bpy.types.Operator):
    bl_idname = "coaster_mixer.edit_control_graph"
    bl_label = "Edit Control Graph"
    bl_description = "Open the active block's ride-control graph in this area"

    @classmethod
    def poll(cls, context):
        _track_object, settings = resolve_active_track_settings(context)
        return settings is not None and len(settings.block_groups) > 0

    def execute(self, context):
        _track_object, settings = resolve_active_track_settings(context)
        block = settings.block_groups[get_clamped_block_group_index(settings)]
        if block.control_tree is None:
            block.control_tree = create_custom_control_tree(block.name)
            block.control_template = "CUSTOM"
        context.area.type = "NODE_EDITOR"
        context.area.ui_type = CONTROL_TREE_IDNAME
        context.space_data.pin = True
        context.space_data.node_tree = block.control_tree
        return {"FINISHED"}


class COASTERMIXER_OT_add_active_zone_to_block(bpy.types.Operator):
    bl_idname = "coaster_mixer.add_active_zone_to_block"
    bl_label = "Add Active Zone"
    bl_description = "Add the selected stop-capable hardware zone to the active block group"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        _root_object, root_settings = resolve_active_track_settings(context)
        _piece_object, piece_settings = resolve_edit_track_settings(context)
        return (
            root_settings is not None
            and piece_settings is not None
            and len(root_settings.block_groups) > 0
            and len(piece_settings.zones) > 0
        )

    def execute(self, context):
        root_object, root_settings = resolve_active_track_settings(context)
        piece_object, piece_settings = resolve_edit_track_settings(context)
        ensure_active_zone_index(piece_settings)
        zone_index = piece_settings.active_zone_index
        zone = piece_settings.zones[zone_index]
        if zone.zone_type not in STOP_CAPABLE_ZONE_TYPES:
            self.report({"WARNING"}, "Only transport or friction-brake zones can hold a train in a block")
            return {"CANCELLED"}

        block_group = root_settings.block_groups[get_clamped_block_group_index(root_settings)]
        for member in block_group.members:
            if member.piece == piece_object and member.zone_index == zone_index:
                self.report({"INFO"}, "Zone is already in this block")
                return {"FINISHED"}

        member = block_group.members.add()
        member.piece = piece_object
        member.zone_index = zone_index
        block_group.active_member_index = len(block_group.members) - 1
        block_group_update(root_settings, context)
        return {"FINISHED"}


class COASTERMIXER_OT_create_block_from_active_zone(bpy.types.Operator):
    bl_idname = "coaster_mixer.create_block_from_active_zone"
    bl_label = "Create Block From Active Zone"
    bl_description = "Create a block stop from the selected hardware zone"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        root_object, root_settings = resolve_active_track_settings(context)
        _piece_object, piece_settings = resolve_edit_track_settings(context)
        return root_settings is not None and piece_settings is not None and len(piece_settings.zones) > 0

    def execute(self, context):
        root_object, root_settings = resolve_active_track_settings(context)
        piece_object, piece_settings = resolve_edit_track_settings(context)
        ensure_active_zone_index(piece_settings)
        zone_index = piece_settings.active_zone_index
        zone = piece_settings.zones[zone_index]
        if zone.zone_type not in STOP_CAPABLE_ZONE_TYPES:
            self.report({"WARNING"}, "Only transport or friction-brake zones can hold a train in a block")
            return {"CANCELLED"}

        block_group = root_settings.block_groups.add()
        block_group.name = f"{zone_label(zone, zone_index)} Block"
        member = block_group.members.add()
        member.piece = piece_object
        member.zone_index = zone_index
        block_group.active_member_index = 0
        route = get_resolved_route(root_object)
        zone_item = next(
            (
                item for item in build_route_zones(route)
                if item["entry"]["object"] == piece_object and item["zone_index"] == zone_index
            ),
            None,
        )
        if zone_item is not None:
            route_start = zone_item["route_start"]
            route_end = zone_item["route_end"]
            if route["cyclic"] and abs(route_end - route["total_length"]) <= 1.0e-3:
                block_group.start_route_meters = route_start - route["total_length"]
                block_group.end_route_meters = 0.0
            else:
                block_group.start_route_meters = route_start
                block_group.end_route_meters = route_end
            if len(root_settings.block_groups) == 1:
                context.scene.coaster_mixer_scene.simulation_start_route_meters = (
                    route_start + (route_end - route_start) * 0.5
                )
        template = "ROLLING_LAUNCH" if zone.zone_type == "TRANSPORT" else "TRIM_BRAKE"
        block_group.control_tree = create_control_template_tree(
            block_group.name, template, zone.length_meters,
            max(zone.length_meters * 0.5, 0.0), 0.0,
            zone.target_speed_mps, zone.max_acceleration_mps2, zone.max_braking_mps2,
            zone.target_speed_mps, zone.max_braking_mps2, "LINEAR",
        )
        block_group.control_template = template

        root_settings.active_block_group_index = len(root_settings.block_groups) - 1
        block_group_update(root_settings, context)
        return {"FINISHED"}


class COASTERMIXER_OT_remove_block_member(bpy.types.Operator):
    bl_idname = "coaster_mixer.remove_block_member"
    bl_label = "Remove Block Member"
    bl_description = "Remove a zone from the active block group"
    bl_options = {"REGISTER", "UNDO"}

    index: bpy.props.IntProperty(default=0, min=0)

    @classmethod
    def poll(cls, context):
        _track_object, track_settings = resolve_active_track_settings(context)
        return track_settings is not None and len(track_settings.block_groups) > 0

    def execute(self, context):
        _track_object, track_settings = resolve_active_track_settings(context)
        block_group = track_settings.block_groups[get_clamped_block_group_index(track_settings)]
        if self.index < 0 or self.index >= len(block_group.members):
            self.report({"WARNING"}, "No block member to remove")
            return {"CANCELLED"}

        block_group.members.remove(self.index)
        block_group.active_member_index = clamp(block_group.active_member_index, 0, max(len(block_group.members) - 1, 0))
        block_group_update(track_settings, context)
        return {"FINISHED"}


ACTION_OWNER_ITEMS = [("SENSOR", "Sensor", "Actions fired by the active sensor")]


def resolve_action_owner(context, owner):
    """Return the property group whose `actions` collection is edited."""
    if owner == "SENSOR":
        _piece_object, piece_settings = resolve_edit_track_settings(context)
        if piece_settings is None or len(piece_settings.sensors) == 0:
            return None
        sensor_index = clamp(piece_settings.active_sensor_index, 0, len(piece_settings.sensors) - 1)
        return piece_settings.sensors[sensor_index]

    return None


class COASTERMIXER_OT_add_action(bpy.types.Operator):
    bl_idname = "coaster_mixer.add_action"
    bl_label = "Add Action"
    bl_description = "Append an action to the sequence"
    bl_options = {"REGISTER", "UNDO"}

    owner: bpy.props.EnumProperty(name="Owner", items=ACTION_OWNER_ITEMS, default="SENSOR")
    kind: bpy.props.EnumProperty(name="Action", items=ACTION_KIND_ITEMS, default="WAIT")

    def execute(self, context):
        action_owner = resolve_action_owner(context, self.owner)
        if action_owner is None:
            self.report({"WARNING"}, "No active block or sensor to add an action to")
            return {"CANCELLED"}
        if self.owner == "SENSOR" and self.kind not in SENSOR_ACTION_KINDS:
            self.report({"WARNING"}, "Sensors only run Wait and Trigger actions")
            return {"CANCELLED"}

        action = action_owner.actions.add()
        action.kind = self.kind
        action_owner.active_action_index = len(action_owner.actions) - 1
        block_group_update(action_owner, context)
        return {"FINISHED"}


class COASTERMIXER_OT_remove_action(bpy.types.Operator):
    bl_idname = "coaster_mixer.remove_action"
    bl_label = "Remove Action"
    bl_description = "Remove the selected action from the sequence"
    bl_options = {"REGISTER", "UNDO"}

    owner: bpy.props.EnumProperty(name="Owner", items=ACTION_OWNER_ITEMS, default="SENSOR")

    def execute(self, context):
        action_owner = resolve_action_owner(context, self.owner)
        if action_owner is None or len(action_owner.actions) == 0:
            self.report({"WARNING"}, "No action to remove")
            return {"CANCELLED"}

        index = clamp(action_owner.active_action_index, 0, len(action_owner.actions) - 1)
        action_owner.actions.remove(index)
        action_owner.active_action_index = clamp(index, 0, max(len(action_owner.actions) - 1, 0))
        block_group_update(action_owner, context)
        return {"FINISHED"}


class COASTERMIXER_OT_move_action(bpy.types.Operator):
    bl_idname = "coaster_mixer.move_action"
    bl_label = "Move Action"
    bl_description = "Move the selected action up or down in the sequence"
    bl_options = {"REGISTER", "UNDO"}

    owner: bpy.props.EnumProperty(name="Owner", items=ACTION_OWNER_ITEMS, default="SENSOR")
    direction: bpy.props.IntProperty(name="Direction", default=1, min=-1, max=1)

    def execute(self, context):
        action_owner = resolve_action_owner(context, self.owner)
        if action_owner is None or len(action_owner.actions) < 2:
            return {"CANCELLED"}

        index = clamp(action_owner.active_action_index, 0, len(action_owner.actions) - 1)
        new_index = clamp(index + self.direction, 0, len(action_owner.actions) - 1)
        if new_index == index:
            return {"CANCELLED"}

        action_owner.actions.move(index, new_index)
        action_owner.active_action_index = new_index
        block_group_update(action_owner, context)
        return {"FINISHED"}


class COASTERMIXER_OT_add_sensor(bpy.types.Operator):
    bl_idname = "coaster_mixer.add_sensor"
    bl_label = "Add Sensor"
    bl_description = "Add a trackside sensor point to the active curve"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        _piece_object, piece_settings = resolve_edit_track_settings(context)
        return piece_settings is not None

    def execute(self, context):
        piece_object, piece_settings = resolve_edit_track_settings(context)
        if piece_object is None or piece_settings is None:
            return {"CANCELLED"}

        sensor = piece_settings.sensors.add()
        sensor.name = f"Sensor {len(piece_settings.sensors):02d}"
        sensor.position_meters = clamp(
            get_track_total_length(piece_object) * 0.5, 0.0, get_track_total_length(piece_object)
        )
        piece_settings.active_sensor_index = len(piece_settings.sensors) - 1
        block_group_update(piece_settings, context)
        return {"FINISHED"}


class COASTERMIXER_OT_remove_sensor(bpy.types.Operator):
    bl_idname = "coaster_mixer.remove_sensor"
    bl_label = "Remove Sensor"
    bl_description = "Remove the selected sensor from the active curve"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        _piece_object, piece_settings = resolve_edit_track_settings(context)
        return piece_settings is not None and len(piece_settings.sensors) > 0

    def execute(self, context):
        _piece_object, piece_settings = resolve_edit_track_settings(context)
        index = clamp(piece_settings.active_sensor_index, 0, len(piece_settings.sensors) - 1)
        piece_settings.sensors.remove(index)
        piece_settings.active_sensor_index = clamp(index, 0, max(len(piece_settings.sensors) - 1, 0))
        block_group_update(piece_settings, context)
        return {"FINISHED"}


class COASTERMIXER_OT_snap_start_to_station(bpy.types.Operator):
    bl_idname = "coaster_mixer.snap_start_to_station"
    bl_label = "Start at Station"
    bl_description = "Place the simulation start at the Station exit (route zero), or at the last authored stop when no seam station exists"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return resolve_active_track_object(context) is not None

    def execute(self, context):
        track_object = resolve_active_track_object(context)
        route = get_resolved_route(track_object)
        derived = get_route_derived_data(route)

        seam_station = next(
            (
                program for program in derived["programs"]
                if program["name"].strip().lower() == "station"
                and route["cyclic"]
                and abs(program["span"][1] - route["total_length"]) <= 1.0e-3
            ),
            None,
        )
        if seam_station is not None:
            context.scene.coaster_mixer_scene.simulation_start_route_meters = 0.0
            self.report({"INFO"}, "Simulation starts stopped at the Station exit (0 m)")
            return {"FINISHED"}

        last_target = None
        for program in derived["programs"]:
            for action in program["actions"]:
                if action["kind"] == "WAIT_POSITION":
                    last_target = action["target"]
        if last_target is None:
            self.report({"WARNING"}, "No station block or Wait for Position node found on the route")
            return {"CANCELLED"}

        context.scene.coaster_mixer_scene.simulation_start_route_meters = last_target
        self.report({"INFO"}, f"Simulation starts at {last_target:.2f} m (station stop point)")
        return {"FINISHED"}


class COASTERMIXER_OT_setup_driven_empty(bpy.types.Operator):
    bl_idname = "coaster_mixer.setup_driven_empty"
    bl_label = "Attach Driven Empty"
    bl_description = "Place the driven empty with arc-length placement drivers (location and banked rotation) at the train front"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        track_object, track_settings = resolve_active_track_settings(context)
        return track_object is not None and track_settings is not None and track_settings.driven_empty_object is not None

    def execute(self, context):
        track_object, track_settings = resolve_active_track_settings(context)
        success, message = ensure_driven_empty_path_setup(track_object, track_settings)
        if success is None:
            self.report({"WARNING"}, message)
            return {"CANCELLED"}

        self.report({"INFO"}, message)
        tag_redraw_view3d()
        return {"FINISHED"}


class COASTERMIXER_OT_attach_selected_followers(bpy.types.Operator):
    bl_idname = "coaster_mixer.attach_selected_followers"
    bl_label = "Attach Selected"
    bl_description = "Attach the selected empties to the active track as follower empties with placement drivers"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        return track_object is not None

    def execute(self, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        selected_empties = [
            object_ref
            for object_ref in context.selected_objects
            if object_ref.type == "EMPTY"
        ]
        if not selected_empties:
            self.report({"WARNING"}, "Select one or more empty objects first")
            return {"CANCELLED"}

        for empty_object in selected_empties:
            ensure_follower_drivers(track_object, empty_object)

        tag_track_placement_update(track_object)
        tag_redraw_view3d()
        self.report({"INFO"}, f"Attached {len(selected_empties)} follower empties")
        return {"FINISHED"}


class COASTERMIXER_OT_create_train_followers(bpy.types.Operator):
    bl_idname = "coaster_mixer.create_train_followers"
    bl_label = "Create Car Empties"
    bl_description = "Create a chain of follower empties spaced in meters behind the train front, ready to parent train cars to"
    bl_options = {"REGISTER", "UNDO"}

    car_count: bpy.props.IntProperty(
        name="Car Count",
        description="Number of follower empties to create",
        min=1,
        max=64,
        default=4,
    )
    car_spacing_meters: bpy.props.FloatProperty(
        name="Car Spacing",
        description="Arc-length distance between consecutive followers",
        min=0.01,
        subtype="DISTANCE",
        default=2.4,
    )
    start_offset_meters: bpy.props.FloatProperty(
        name="Start Offset",
        description="Arc-length distance from the train front to the first follower",
        min=0.0,
        subtype="DISTANCE",
        default=0.0,
    )

    @classmethod
    def poll(cls, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        return track_object is not None

    def invoke(self, context, _event):
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        if track_object is None:
            self.report({"WARNING"}, "Select a curve object first")
            return {"CANCELLED"}

        for car_index in range(self.car_count):
            offset_meters = self.start_offset_meters + self.car_spacing_meters * car_index
            empty_object = bpy.data.objects.new(f"{track_object.name} Car {car_index + 1:02d}", None)
            empty_object.empty_display_type = "PLAIN_AXES"
            empty_object.empty_display_size = 0.5
            context.collection.objects.link(empty_object)
            ensure_follower_drivers(track_object, empty_object, offset_meters=offset_meters)

        tag_track_placement_update(track_object)
        tag_redraw_view3d()
        self.report({"INFO"}, f"Created {self.car_count} follower empties")
        return {"FINISHED"}


class COASTERMIXER_OT_create_train_camera(bpy.types.Operator):
    bl_idname = "coaster_mixer.create_train_camera"
    bl_label = "Create Ride Camera"
    bl_description = "Create a bank-following camera above the second train follower, aimed at a track-driven look-ahead target"
    bl_options = {"REGISTER", "UNDO"}

    height_meters: bpy.props.FloatProperty(
        name="Height",
        description="Camera height above the second follower in meters",
        min=0.0,
        subtype="DISTANCE",
        default=1.6,
    )
    look_ahead_meters: bpy.props.FloatProperty(
        name="Look Ahead",
        description="Distance in meters ahead of the train front used as the camera aim target",
        min=0.1,
        subtype="DISTANCE",
        default=5.0,
    )
    lens_millimeters: bpy.props.FloatProperty(
        name="Lens",
        description="Camera focal length in millimeters",
        min=1.0,
        max=500.0,
        default=35.0,
    )
    make_active: bpy.props.BoolProperty(
        name="Make Active Camera",
        description="Use the new ride camera as the scene camera",
        default=True,
    )

    @classmethod
    def poll(cls, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        return track_object is not None

    def invoke(self, context, _event):
        return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        followers = collect_track_followers(track_object)
        if len(followers) < 2:
            self.report({"WARNING"}, "Create at least two car empties before creating the ride camera")
            return {"CANCELLED"}

        mount_object = followers[1]
        target_object = bpy.data.objects.new(f"{track_object.name} Camera Look Ahead", None)
        target_object.empty_display_type = "PLAIN_AXES"
        target_object.empty_display_size = 0.25
        target_object["coaster_mixer_camera_target"] = True
        context.collection.objects.link(target_object)
        ensure_follower_drivers(track_object, target_object, offset_meters=-self.look_ahead_meters)

        camera_data = bpy.data.cameras.new(f"{track_object.name} Ride Camera")
        camera_data.lens = self.lens_millimeters
        camera_object = bpy.data.objects.new(camera_data.name, camera_data)
        context.collection.objects.link(camera_object)
        camera_object.parent = mount_object
        camera_object.location = (0.0, 0.0, self.height_meters)
        camera_object.rotation_euler = (0.0, 0.0, 0.0)

        aim_constraint = camera_object.constraints.new(type="TRACK_TO")
        aim_constraint.name = "Coaster Mixer Look Ahead"
        aim_constraint.target = target_object
        aim_constraint.track_axis = "TRACK_NEGATIVE_Z"
        aim_constraint.up_axis = "UP_Y"
        aim_constraint.influence = 0.0

        if self.make_active:
            context.scene.camera = camera_object

        place_track_followers(track_object)
        refresh_view_layer()
        tag_redraw_view3d()
        self.report({"INFO"}, f"Created ride camera above {mount_object.name}")
        return {"FINISHED"}


class COASTERMIXER_OT_detach_follower(bpy.types.Operator):
    bl_idname = "coaster_mixer.detach_follower"
    bl_label = "Detach Follower"
    bl_description = "Remove the placement drivers from this follower empty and detach it from the track"
    bl_options = {"REGISTER", "UNDO"}

    empty_name: bpy.props.StringProperty(name="Empty Name")

    def execute(self, _context):
        empty_object = bpy.data.objects.get(self.empty_name)
        if empty_object is None:
            self.report({"WARNING"}, f"Object '{self.empty_name}' not found")
            return {"CANCELLED"}

        remove_follower_drivers(empty_object)
        tag_redraw_view3d()
        self.report({"INFO"}, f"Detached {empty_object.name}")
        return {"FINISHED"}


class COASTERMIXER_OT_add_connection(bpy.types.Operator):
    bl_idname = "coaster_mixer.add_connection"
    bl_label = "Add Connection"
    bl_description = "Add an exit connection at this end of the piece; multiple connections make this end a switch"
    bl_options = {"REGISTER", "UNDO"}

    end: bpy.props.EnumProperty(name="Piece End", items=CONNECTION_END_ITEMS, default="END")

    @classmethod
    def poll(cls, context):
        _piece, piece_settings = resolve_edit_track_settings(context)
        return piece_settings is not None

    def execute(self, context):
        _piece, piece_settings = resolve_edit_track_settings(context)
        if piece_settings is None:
            return {"CANCELLED"}

        get_connection_list(piece_settings, self.end).add()
        tag_redraw_view3d()
        return {"FINISHED"}


class COASTERMIXER_OT_remove_connection(bpy.types.Operator):
    bl_idname = "coaster_mixer.remove_connection"
    bl_label = "Remove Connection"
    bl_description = "Remove this exit connection from the piece"
    bl_options = {"REGISTER", "UNDO"}

    end: bpy.props.EnumProperty(name="Piece End", items=CONNECTION_END_ITEMS, default="END")
    index: bpy.props.IntProperty(name="Index", min=0, default=0)

    def execute(self, context):
        _piece, piece_settings = resolve_edit_track_settings(context)
        if piece_settings is None:
            return {"CANCELLED"}

        connections = get_connection_list(piece_settings, self.end)
        if self.index >= len(connections):
            return {"CANCELLED"}

        connections.remove(self.index)
        index_property = "end_active_index" if self.end == "END" else "start_active_index"
        clamped_index = clamp(getattr(piece_settings, index_property), 0, max(len(connections) - 1, 0))
        if clamped_index != getattr(piece_settings, index_property):
            assign_rna_property(piece_settings, index_property, clamped_index)
        route_topology_update(piece_settings, context)
        return {"FINISHED"}


class COASTERMIXER_OT_reset_simulation(bpy.types.Operator):
    bl_idname = "coaster_mixer.reset_simulation"
    bl_label = "Recompute Simulation"
    bl_description = "Discard the cached simulation trajectory and recompute it from the timeline start"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        invalidate_simulation_trajectory()
        apply_simulation_frame(context.scene)
        self.report({"INFO"}, "Simulation trajectory recomputed")
        tag_redraw_view3d()
        return {"FINISHED"}


class COASTERMIXER_OT_bake_simulation(bpy.types.Operator):
    bl_idname = "coaster_mixer.bake_simulation"
    bl_label = "Bake Simulation"
    bl_description = "Bake the simulated path factor to keyframes on the active track"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        _track_object, track_settings = resolve_active_track_settings(context)
        return track_settings is not None

    def execute(self, context):
        scene = context.scene
        scene_settings = getattr(scene, "coaster_mixer_scene", None)
        track_object, track_settings = resolve_active_track_settings(context)
        if scene_settings is None or track_object is None or track_settings is None:
            self.report({"WARNING"}, "Select a curve object first")
            return {"CANCELLED"}

        frame_start, frame_end, range_label = get_simulation_bake_frame_range(scene)
        if frame_end < frame_start:
            self.report({"WARNING"}, f"{range_label} frame range is invalid")
            return {"CANCELLED"}

        window_manager = context.window_manager
        bake_started = perf_counter()
        try:
            # The trajectory is read-only, so baking is just writing the same
            # samples live playback shows to keyframes.
            window_manager.progress_begin(frame_start, frame_end)
            frame_values = []
            for frame in range(frame_start, frame_end + 1):
                sample = sample_simulation_trajectory(scene, track_object, track_settings, frame)
                if sample is None:
                    self.report({"WARNING"}, "The route is empty; nothing to bake")
                    return {"CANCELLED"}
                frame_values.append((frame, sample[0]))
                if frame % 32 == 0:
                    window_manager.progress_update(frame)

            clear_action_fcurve(track_object, TRAIN_FRONT_METERS_DATA_PATH)
            baked_curve = insert_dense_fcurve_keyframes(track_object, TRAIN_FRONT_METERS_DATA_PATH, frame_values)
            if baked_curve is None:
                clear_action_fcurve(track_object, TRAIN_FRONT_METERS_DATA_PATH)
                for frame, front_meters in frame_values:
                    assign_rna_property(track_settings, "train_front_route_meters", front_meters)
                    track_object.keyframe_insert(data_path=TRAIN_FRONT_METERS_DATA_PATH, frame=frame)

                baked_curve = get_action_fcurve(track_object, TRAIN_FRONT_METERS_DATA_PATH)
                set_fcurve_linear(baked_curve)

            self.place_trigger_markers(scene, frame_start, frame_end)
            assign_simulation_enabled(scene_settings, False)
            scene.frame_set(scene.frame_current)
        except Exception as exc:
            self.report({"ERROR"}, f"Simulation bake failed: {exc}")
            return {"CANCELLED"}
        finally:
            window_manager.progress_end()

        baked_frame_count = frame_end - frame_start + 1
        bake_seconds = perf_counter() - bake_started
        self.report(
            {"INFO"},
            f"Baked {baked_frame_count} frames in {bake_seconds:.2f}s and disabled runtime simulation",
        )
        tag_redraw_view3d()
        return {"FINISHED"}

    @staticmethod
    def place_trigger_markers(scene, frame_start, frame_end):
        """Lay timeline markers at trigger events inside the baked range."""
        cache = SIMULATION_TRAJECTORY_CACHE
        if cache is None or not cache["events"]:
            return

        events_by_index = {}
        for event_frame, channel, value in cache["events"]:
            events_by_index.setdefault(event_frame, []).append((channel, value))

        marker_prefix = "cm:"
        for marker in [m for m in scene.timeline_markers if m.name.startswith(marker_prefix)]:
            scene.timeline_markers.remove(marker)

        for frame in range(frame_start, frame_end + 1):
            mapped_index = resolve_trajectory_index(cache, max(frame - scene.frame_start, 0))
            for channel, value in events_by_index.get(mapped_index, ()):
                scene.timeline_markers.new(f"{marker_prefix}{channel}={value:g}", frame=frame)


class COASTERMIXER_OT_clear_baked_simulation(bpy.types.Operator):
    bl_idname = "coaster_mixer.clear_baked_simulation"
    bl_label = "Clear Baked Keys"
    bl_description = "Remove baked path-factor keyframes from the active track"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        return track_object is not None and has_baked_path_animation(track_object)

    def execute(self, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        if track_object is None:
            self.report({"WARNING"}, "Select a curve object first")
            return {"CANCELLED"}

        if not clear_action_fcurve(track_object, TRAIN_FRONT_METERS_DATA_PATH):
            self.report({"WARNING"}, "No baked path-factor keys found")
            return {"CANCELLED"}

        context.scene.frame_set(context.scene.frame_current)
        self.report({"INFO"}, "Cleared baked path-factor keys")
        tag_redraw_view3d()
        return {"FINISHED"}


def seed_default_station(context, track_object):
    settings = track_object.coaster_mixer_track
    if len(settings.zones) > 0 or len(settings.block_groups) > 0:
        return False
    route = get_resolved_route(track_object)
    total_length = route["total_length"]
    if total_length <= 1.0e-6:
        return False
    station_length = min(DEFAULT_ZONE_LENGTH_METERS, total_length)
    hardware_start = max(total_length - station_length, 0.0) if route["cyclic"] else 0.0

    for name, zone_type in (("Station Brake", "FRICTION_BRAKE"), ("Station Drive", "TRANSPORT")):
        zone = settings.zones.add()
        zone.name = name
        zone.zone_type = zone_type
        zone.start_meters = hardware_start
        zone.length_meters = station_length
        zone.target_speed_mps = DEFAULT_STATION_DISPATCH_SPEED_MPS
        if zone_type == "TRANSPORT":
            zone.minimum_speed_mps = 0.5
            zone.max_acceleration_mps2 = DEFAULT_STATION_ACCELERATION_MPS2
            zone.max_braking_mps2 = DEFAULT_STATION_BRAKING_MPS2
        else:
            zone.max_acceleration_mps2 = 0.0
            zone.max_braking_mps2 = 2.5

    block = settings.block_groups.add()
    block.name = "Station"
    block.start_route_meters = -station_length if route["cyclic"] else 0.0
    block.end_route_meters = 0.0 if route["cyclic"] else station_length
    for zone_index in (0, 1):
        member = block.members.add()
        member.piece = track_object
        member.zone_index = zone_index
    block.control_tree = create_default_control_tree("Station")
    block.control_template = "LOAD_STATION"
    settings.active_zone_index = 1
    settings.active_block_group_index = 0
    context.scene.coaster_mixer_scene.simulation_start_route_meters = 0.0
    block_group_update(settings, context)
    context.scene.frame_set(context.scene.frame_start)
    assign_simulation_enabled(context.scene.coaster_mixer_scene, True)
    return True


class COASTERMIXER_OT_set_root_from_active(bpy.types.Operator):
    bl_idname = "coaster_mixer.set_root_from_active"
    bl_label = "Use Active Curve"
    bl_description = "Set the active viewport curve as the coaster root"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        active_object = getattr(context, "object", None)
        return active_object is not None and active_object.type == "CURVE"

    def execute(self, context):
        # Assigning through the property runs track_object_update, which
        # resyncs zones, drivers, and the simulation state.
        track_object = context.object
        settings = track_object.coaster_mixer_track
        was_empty = len(settings.zones) == 0 and len(settings.block_groups) == 0
        context.scene.coaster_mixer_scene.track_object = track_object
        seeded_station = was_empty and len(settings.block_groups) == 1 and settings.block_groups[0].name == "Station"

        message = f"Coaster root set to {track_object.name}"
        if seeded_station:
            message += " with an initial Station block"
        self.report({"INFO"}, message)
        return {"FINISHED"}


class COASTERMIXER_OT_clear_curve_setup(bpy.types.Operator):
    bl_idname = "coaster_mixer.clear_curve_setup"
    bl_label = "Clear Curve Setup"
    bl_description = "Remove Coaster Mixer actuators, sensors, blocks, control graphs, and outgoing connections from the active curve"
    bl_options = {"REGISTER", "UNDO"}

    create_default_station: bpy.props.BoolProperty(
        name="Recreate Default Station",
        description="After clearing, create the standard brake, drive, Station block, and control graph",
        default=True,
    )

    @classmethod
    def poll(cls, context):
        active_object = getattr(context, "object", None)
        return (
            active_object is not None and active_object.type == "CURVE"
        ) or resolve_active_track_object(context) is not None

    def draw(self, _context):
        layout = self.layout
        layout.label(text="The curve geometry and train followers will be preserved.", icon="INFO")
        layout.prop(self, "create_default_station")

    def invoke(self, context, _event):
        return context.window_manager.invoke_props_dialog(self, width=420)

    def execute(self, context):
        active_object = getattr(context, "object", None)
        track_object = (
            active_object
            if active_object is not None and active_object.type == "CURVE"
            else resolve_active_track_object(context)
        )
        if track_object is None:
            self.report({"WARNING"}, "Select a curve or choose a coaster root first")
            return {"CANCELLED"}
        settings = track_object.coaster_mixer_track
        control_trees = [block.control_tree for block in settings.block_groups if block.control_tree is not None]

        root_object, root_settings = resolve_active_track_settings(context)
        if root_settings is not None and root_object != track_object:
            for block in root_settings.block_groups:
                for member_index in reversed(range(len(block.members))):
                    if block.members[member_index].piece == track_object:
                        block.members.remove(member_index)

        settings.zones.clear()
        settings.sensors.clear()
        settings.block_groups.clear()
        settings.start_connections.clear()
        settings.end_connections.clear()
        settings.active_zone_index = 0
        settings.active_sensor_index = 0
        settings.active_block_group_index = 0
        settings.start_active_index = 0
        settings.end_active_index = 0
        settings.train_front_route_meters = 0.0

        for node_tree in control_trees:
            if node_tree is not None and node_tree.users == 0:
                bpy.data.node_groups.remove(node_tree)

        scene_settings = context.scene.coaster_mixer_scene
        if scene_settings.track_object == track_object:
            assign_simulation_enabled(scene_settings, False)
            scene_settings.simulation_start_route_meters = 0.0

        clear_runtime_caches()
        tag_redraw_view3d()

        if self.create_default_station:
            context.scene.coaster_mixer_scene.track_object = track_object
            if not seed_default_station(context, track_object) and len(settings.block_groups) == 0:
                self.report({"WARNING"}, "Setup cleared, but the default station could not be created")
                return {"FINISHED"}
            self.report({"INFO"}, f"Reset {track_object.name} with a default Station")
        else:
            self.report({"INFO"}, f"Cleared Coaster Mixer setup from {track_object.name}")
        return {"FINISHED"}


class COASTERMIXER_OT_select_piece(bpy.types.Operator):
    bl_idname = "coaster_mixer.select_piece"
    bl_label = "Select Piece"
    bl_description = "Select this piece in the viewport and edit its zones and connections"
    bl_options = {"REGISTER", "UNDO"}

    piece_name: bpy.props.StringProperty(name="Piece Name")

    def execute(self, context):
        piece = bpy.data.objects.get(self.piece_name)
        if piece is None:
            self.report({"WARNING"}, f"Object '{self.piece_name}' not found")
            return {"CANCELLED"}

        for selected_object in context.selected_objects:
            selected_object.select_set(False)
        piece.select_set(True)
        context.view_layer.objects.active = piece
        tag_redraw_view3d()
        return {"FINISHED"}

