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
    bl_label = "Attach Train Front Empty"
    bl_description = "Place the train front empty with arc-length placement at the train front"
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
    bl_label = "Attach Selected Empties"
    bl_description = "Attach the selected empties to the active track as train car mounts, preserving their current spacing"
    bl_options = {"REGISTER", "UNDO"}

    flip_train: bpy.props.BoolProperty(
        name="Flip Train",
        description="Reverse the inferred front/back direction of the selected empty chain",
        default=False,
    )
    adjust_train_length: bpy.props.BoolProperty(
        name="Set Train Length",
        description="Update the physical train length to match the resulting full mount layout",
        default=True,
    )
    adjust_station_length: bpy.props.BoolProperty(
        name="Resize Station to Train",
        description="Match the seam Station to the resulting train length by moving its entry backward while keeping its exit at the coaster start",
        default=True,
    )

    @classmethod
    def poll(cls, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        return track_object is not None

    def invoke(self, context, _event):
        self._preview_state = capture_attach_preview_state(context)
        self._apply_preview(context)
        return context.window_manager.invoke_props_dialog(self, width=420)

    def check(self, context):
        self._apply_preview(context)
        return True

    def cancel(self, context):
        restore_attach_preview_state(self._preview_state)
        tag_redraw_view3d()

    def draw(self, context):
        layout = self.layout
        track_object, track_settings = resolve_active_track_settings(context)
        selected_empties = [object_ref for object_ref in context.selected_objects if object_ref.type == "EMPTY"]
        import_layout = infer_selected_mount_layout(selected_empties, flip_train=self.flip_train)
        front_empty = import_layout["front_empty"]
        ordered_empties = import_layout["ordered_empties"]
        mount_lengths = import_layout["mount_lengths"]
        added_length = import_layout["total_length"]

        current_total_length = (
            self._preview_state["mount_total_length"]
            if getattr(self, "_preview_state", None) is not None
            else (get_train_mount_total_length_meters(collect_track_followers(track_object)) if track_object is not None else 0.0)
        )
        proposed_total_length = added_length

        column = layout.column()
        column.use_property_split = True
        column.use_property_decorate = False
        column.prop(self, "flip_train")

        summary = layout.box()
        summary.label(text=f"Selected empties: {len(ordered_empties)}", icon="OUTLINER_OB_EMPTY")
        if front_empty is not None:
            summary.label(text=f"Inferred front: {front_empty.name}", icon="EMPTY_AXIS")
        if track_settings is not None:
            axis_label = next(
                (label for identifier, label, _description in TRAIN_MOUNT_AXIS_PRESET_ITEMS if identifier == import_layout["axis_preset"]),
                import_layout["axis_preset"],
            )
            summary.label(text=f"Detected empty axes: {axis_label}", icon="ORIENTATION_GLOBAL")
        if mount_lengths:
            summary.label(text="Imported lengths: " + ", ".join(f"{length:.2f} m" for length in mount_lengths[:6]))
            if len(mount_lengths) > 6:
                summary.label(text=f"... and {len(mount_lengths) - 6} more")
        summary.label(text=f"Train length: {current_total_length:.2f} m -> {proposed_total_length:.2f} m", icon="DRIVER_DISTANCE")
        summary.prop(self, "adjust_train_length")

        station = get_adjustable_seam_station(track_object) if track_object is not None else None
        station_column = summary.column()
        station_column.enabled = station is not None
        station_column.prop(self, "adjust_station_length")
        if station is not None:
            station_length = min(proposed_total_length, station["maximum_length"])
            station_column.label(text=f"Station: {station['current_length']:.2f} m -> {station_length:.2f} m")
            station_column.label(text="Exit stays at coaster start; entry moves backward.", icon="INFO")
            if proposed_total_length > station["maximum_length"] + 1.0e-3:
                station_column.label(text="Limited by the final track piece length.", icon="ERROR")
        else:
            summary.label(text="No adjustable Station ending at the coaster start.", icon="INFO")

    def _apply_preview(self, context):
        track_object, track_settings = resolve_active_track_settings(context)
        if track_object is None or track_settings is None:
            return
        selected_empties = [object_ref for object_ref in context.selected_objects if object_ref.type == "EMPTY"]
        import_layout = infer_selected_mount_layout(selected_empties, flip_train=self.flip_train)
        apply_inferred_train_attach(
            track_object,
            track_settings,
            import_layout,
            reverse_facing=self.flip_train,
            rig_mode="STANDARD",
        )
        tag_redraw_view3d()

    def execute(self, context):
        track_object, track_settings = resolve_active_track_settings(context)
        selected_empties = [
            object_ref
            for object_ref in context.selected_objects
            if object_ref.type == "EMPTY"
        ]
        if not selected_empties:
            self.report({"WARNING"}, "Select one or more empty objects first")
            return {"CANCELLED"}

        import_layout = infer_selected_mount_layout(selected_empties, flip_train=self.flip_train)
        front_empty = import_layout["front_empty"]
        ordered_empties = import_layout["ordered_empties"]
        mount_lengths = import_layout["mount_lengths"]
        if front_empty is None:
            self.report({"WARNING"}, "Could not infer a train line from the selected empties")
            return {"CANCELLED"}
        assign_rna_property(track_settings, "train_mount_axis_preset", import_layout["axis_preset"])

        previous_front = track_settings.driven_empty_object
        if previous_front is not None and previous_front != front_empty:
            remove_follower_drivers(previous_front)
        running_offset = apply_inferred_train_attach(
            track_object,
            track_settings,
            import_layout,
            reverse_facing=self.flip_train,
            rig_mode="STANDARD",
        )

        proposed_total_length = running_offset
        if self.adjust_train_length:
            track_settings.train_length_meters = proposed_total_length

        station_message = ""
        if self.adjust_station_length:
            station = get_adjustable_seam_station(track_object)
            if station is not None:
                station_length = resize_seam_station(station, proposed_total_length)
                station_message = f"; Station resized backward to {station_length:.2f} m"

        tag_track_placement_update(track_object)
        tag_redraw_view3d()
        train_message = f"; train length set to {proposed_total_length:.2f} m" if self.adjust_train_length else ""
        reverse_message = "; train flipped" if self.flip_train else ""
        self.report({"INFO"}, f"Attached {len(ordered_empties)} train mounts behind {front_empty.name}{reverse_message}{train_message}{station_message}")
        self._preview_state = None
        return {"FINISHED"}


class COASTERMIXER_OT_attach_selected_ik_chain(bpy.types.Operator):
    bl_idname = "coaster_mixer.attach_selected_ik_chain"
    bl_label = "Attach IK Chain Train"
    bl_description = "Attach the active empty as the train leader and the other selected empties as ordered IK targets"
    bl_options = {"REGISTER", "UNDO"}

    flip_train: bpy.props.BoolProperty(
        name="Flip Train",
        description="Reverse the interpreted train direction while keeping the active empty as the imported leader",
        default=False,
    )
    adjust_train_length: bpy.props.BoolProperty(
        name="Set Train Length",
        description="Update the physical train length to match the imported IK chain",
        default=True,
    )
    adjust_station_length: bpy.props.BoolProperty(
        name="Resize Station to Train",
        description="Match the seam Station to the resulting train length by moving its entry backward while keeping its exit at the coaster start",
        default=True,
    )

    @classmethod
    def poll(cls, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        active_object = getattr(context.view_layer.objects, "active", None)
        return (
            track_object is not None
            and active_object is not None
            and active_object.type == "EMPTY"
            and sum(1 for object_ref in context.selected_objects if object_ref.type == "EMPTY") >= 2
        )

    def invoke(self, context, _event):
        self._preview_state = capture_attach_preview_state(context)
        self._apply_preview(context)
        return context.window_manager.invoke_props_dialog(self, width=440)

    def check(self, context):
        self._apply_preview(context)
        return True

    def cancel(self, context):
        restore_attach_preview_state(self._preview_state)
        tag_redraw_view3d()

    def draw(self, context):
        layout = self.layout
        track_object, _track_settings = resolve_active_track_settings(context)
        selected_empties = [object_ref for object_ref in context.selected_objects if object_ref.type == "EMPTY"]
        import_layout = infer_selected_ik_chain_layout(context, selected_empties, flip_train=self.flip_train)
        leader_empty = import_layout["leader_empty"]
        ik_targets = import_layout["ordered_empties"]
        mount_lengths = import_layout["mount_lengths"]
        orientation_check = import_layout["orientation_check"]

        current_total_length = (
            self._preview_state["mount_total_length"]
            if getattr(self, "_preview_state", None) is not None
            else (get_train_mount_total_length_meters(collect_track_followers(track_object)) if track_object is not None else 0.0)
        )
        proposed_total_length = import_layout["total_length"]

        column = layout.column()
        column.use_property_split = True
        column.use_property_decorate = False
        column.prop(self, "flip_train")

        summary = layout.box()
        if leader_empty is not None:
            summary.label(text=f"Leader: {leader_empty.name}", icon="EMPTY_AXIS")
        summary.label(text=f"IK targets: {len(ik_targets)}", icon="CONSTRAINT_BONE")
        axis_label = next(
            (label for identifier, label, _description in TRAIN_MOUNT_AXIS_PRESET_ITEMS if identifier == import_layout["axis_preset"]),
            import_layout["axis_preset"],
        )
        summary.label(text=f"Detected empty axes: {axis_label}", icon="ORIENTATION_GLOBAL")
        if orientation_check["mismatch_count"] > 0:
            summary.label(
                text=f"Orientation mismatch on {orientation_check['mismatch_count']} empties; leader and targets should share axes.",
                icon="ERROR",
            )
        if mount_lengths:
            summary.label(text="Target gaps: " + ", ".join(f"{length:.2f} m" for length in mount_lengths[:6]))
            if len(mount_lengths) > 6:
                summary.label(text=f"... and {len(mount_lengths) - 6} more")
        summary.label(text=f"Train length: {current_total_length:.2f} m -> {proposed_total_length:.2f} m", icon="DRIVER_DISTANCE")
        summary.prop(self, "adjust_train_length")

        station = get_adjustable_seam_station(track_object) if track_object is not None else None
        station_column = summary.column()
        station_column.enabled = station is not None
        station_column.prop(self, "adjust_station_length")
        if station is not None:
            station_length = min(proposed_total_length, station["maximum_length"])
            station_column.label(text=f"Station: {station['current_length']:.2f} m -> {station_length:.2f} m")

    def _apply_preview(self, context):
        track_object, track_settings = resolve_active_track_settings(context)
        if track_object is None or track_settings is None:
            return
        selected_empties = [object_ref for object_ref in context.selected_objects if object_ref.type == "EMPTY"]
        import_layout = infer_selected_ik_chain_layout(context, selected_empties, flip_train=self.flip_train)
        apply_inferred_train_attach(
            track_object,
            track_settings,
            import_layout,
            reverse_facing=self.flip_train,
            rig_mode="IK_CHAIN",
        )
        tag_redraw_view3d()

    def execute(self, context):
        track_object, track_settings = resolve_active_track_settings(context)
        selected_empties = [object_ref for object_ref in context.selected_objects if object_ref.type == "EMPTY"]
        if len(selected_empties) < 2:
            self.report({"WARNING"}, "Select the leader empty plus at least one IK target empty")
            return {"CANCELLED"}
        import_layout = infer_selected_ik_chain_layout(context, selected_empties, flip_train=self.flip_train)
        leader_empty = import_layout["leader_empty"]
        if leader_empty is None:
            self.report({"WARNING"}, "The active empty must be the IK chain leader")
            return {"CANCELLED"}
        orientation_check = import_layout["orientation_check"]
        if orientation_check["mismatch_count"] > 0:
            self.report({"WARNING"}, "Leader and IK target empties do not share a consistent axis orientation")
            restore_attach_preview_state(self._preview_state)
            return {"CANCELLED"}

        running_offset = apply_inferred_train_attach(
            track_object,
            track_settings,
            import_layout,
            reverse_facing=self.flip_train,
            rig_mode="IK_CHAIN",
        )
        if self.adjust_train_length:
            track_settings.train_length_meters = running_offset

        station_message = ""
        if self.adjust_station_length:
            station = get_adjustable_seam_station(track_object)
            if station is not None:
                station_length = resize_seam_station(station, running_offset)
                station_message = f"; Station resized backward to {station_length:.2f} m"

        tag_track_placement_update(track_object)
        tag_redraw_view3d()
        reverse_message = "; train flipped" if self.flip_train else ""
        self.report({"INFO"}, f"Attached IK chain with leader {leader_empty.name} and {len(import_layout['ordered_empties'])} targets{reverse_message}{station_message}")
        self._preview_state = None
        return {"FINISHED"}


def get_adjustable_seam_station(track_object):
    """Return the default seam station and its route-backed member zones.

    Resizing is intentionally limited to a Station whose hardware ends at the
    cyclic route seam and lives on the final route piece. That lets us preserve
    the station exit exactly while moving only its upstream edge.
    """
    route = get_resolved_route(track_object)
    if not route["cyclic"] or not route["entries"]:
        return None

    derived = get_route_derived_data(route)
    programs_by_key = {program["key"]: program for program in derived["programs"]}
    zones_by_key = {
        (item["entry"]["object"].as_pointer(), item["zone_index"]): item
        for item in derived["zones"]
    }
    route_end = route["total_length"]
    final_entry = route["entries"][-1]
    root_settings = track_object.coaster_mixer_track

    for block in root_settings.block_groups:
        if block.name.strip().lower() != "station":
            continue
        program = programs_by_key.get(f"block:{block.as_pointer()}")
        if program is None or abs(program["span"][1] - route_end) > 1.0e-3:
            continue
        member_items = []
        for member in block.members:
            if member.piece is None:
                continue
            item = zones_by_key.get((member.piece.as_pointer(), member.zone_index))
            if item is not None:
                member_items.append(item)
        if not member_items or any(
            item["entry"]["object"] != final_entry["object"]
            or item["entry"]["reversed"] != final_entry["reversed"]
            or abs(item["route_end"] - route_end) > 1.0e-3
            for item in member_items
        ):
            continue
        return {
            "route": route,
            "block": block,
            "member_items": member_items,
            "current_length": max(route_end - program["span"][0], 0.0),
            "maximum_length": final_entry["length"],
        }
    return None


def resize_seam_station(station, requested_length):
    """Resize a seam station upstream, leaving its route-space exit fixed."""
    route = station["route"]
    block = station["block"]
    old_length = station["current_length"]
    new_length = clamp(requested_length, 0.0, station["maximum_length"])
    route_end = route["total_length"]
    route_start = route_end - new_length

    for item in station["member_items"]:
        entry = item["entry"]
        local_a = route_to_piece_distance(entry, route_start)
        local_b = route_to_piece_distance(entry, route_end)
        zone = item["zone"]
        zone.start_meters = min(local_a, local_b)
        zone.length_meters = abs(local_b - local_a)
        sync_zone_to_curve(entry["object"], zone)

    # Signed seam coordinates make the anchoring explicit: the exit remains
    # zero and only the entry moves backward as the station grows.
    block.start_route_meters = -new_length
    block.end_route_meters = 0.0

    # The generated Station graph places its hold and dispatch gates at the
    # old downstream edge. Keep those edge-bound gates at the new edge while
    # leaving deliberately authored intermediate positions untouched.
    tree = block.control_tree
    if tree is not None:
        for node in tree.nodes:
            if hasattr(node, "offset_meters") and abs(node.offset_meters - old_length) <= 1.0e-3:
                node.offset_meters = new_length

    block_group_update(route["entries"][0]["settings"], bpy.context)
    return new_length


def capture_attach_preview_object_state(object_ref):
    if object_ref is None:
        return None
    follower_settings = object_ref.coaster_mixer_follower
    return {
        "object": object_ref,
        "track_object": follower_settings.track_object,
        "offset_meters": follower_settings.offset_meters,
        "vertical_offset_meters": follower_settings.vertical_offset_meters,
        "reverse_forward_axis": follower_settings.reverse_forward_axis,
        "train_role": follower_settings.train_role,
        "source_mount_object": follower_settings.source_mount_object,
        "rotation_mode": object_ref.rotation_mode,
        "location": object_ref.location.copy(),
        "rotation_euler": object_ref.rotation_euler.copy(),
        "batched_placement": bool(object_ref.get("coaster_mixer_batched_placement", False)),
        "train_mount": bool(object_ref.get("coaster_mixer_train_mount", False)),
    }


def restore_attach_preview_object_state(snapshot):
    if not snapshot:
        return
    object_ref = snapshot["object"]
    if object_ref is None or bpy.data.objects.get(object_ref.name) is None:
        return
    follower_settings = object_ref.coaster_mixer_follower
    assign_rna_property(follower_settings, "track_object", snapshot["track_object"])
    assign_rna_property(follower_settings, "offset_meters", snapshot["offset_meters"])
    assign_rna_property(follower_settings, "vertical_offset_meters", snapshot["vertical_offset_meters"])
    assign_rna_property(follower_settings, "reverse_forward_axis", snapshot["reverse_forward_axis"])
    assign_rna_property(follower_settings, "train_role", snapshot["train_role"])
    assign_rna_property(follower_settings, "source_mount_object", snapshot["source_mount_object"])
    object_ref.rotation_mode = snapshot["rotation_mode"]
    object_ref.location = snapshot["location"]
    object_ref.rotation_euler = snapshot["rotation_euler"]
    if snapshot["batched_placement"]:
        object_ref["coaster_mixer_batched_placement"] = True
    else:
        object_ref.pop("coaster_mixer_batched_placement", None)
    mark_train_mount(object_ref, enabled=snapshot["train_mount"])


def capture_attach_preview_state(context):
    track_object, track_settings = resolve_active_track_settings(context)
    if track_object is None or track_settings is None:
        return None
    selected_empties = [object_ref for object_ref in context.selected_objects if object_ref.type == "EMPTY"]
    tracked_objects = list(selected_empties)
    if track_settings.driven_empty_object is not None:
        tracked_objects.append(track_settings.driven_empty_object)
    unique_objects = []
    seen = set()
    for object_ref in tracked_objects:
        if object_ref is None:
            continue
        object_key = object_ref.as_pointer()
        if object_key in seen:
            continue
        seen.add(object_key)
        unique_objects.append(object_ref)
    return {
        "track_object": track_object,
        "driven_empty_object": track_settings.driven_empty_object,
        "train_rig_mode": track_settings.train_rig_mode,
        "train_mount_axis_preset": track_settings.train_mount_axis_preset,
        "train_mounts_reversed": track_settings.train_mounts_reversed,
        "train_length_meters": track_settings.train_length_meters,
        "mount_total_length": get_train_mount_total_length_meters(collect_track_followers(track_object)),
        "objects": [capture_attach_preview_object_state(object_ref) for object_ref in unique_objects],
    }


def restore_attach_preview_state(snapshot):
    if not snapshot:
        return
    track_object = snapshot["track_object"]
    if track_object is None or bpy.data.objects.get(track_object.name) is None:
        return
    track_settings = track_object.coaster_mixer_track
    assign_rna_property(track_settings, "driven_empty_object", snapshot["driven_empty_object"])
    assign_rna_property(track_settings, "train_rig_mode", snapshot["train_rig_mode"])
    assign_rna_property(track_settings, "train_mount_axis_preset", snapshot["train_mount_axis_preset"])
    assign_rna_property(track_settings, "train_mounts_reversed", snapshot["train_mounts_reversed"])
    assign_rna_property(track_settings, "train_length_meters", snapshot["train_length_meters"])
    for object_snapshot in snapshot["objects"]:
        restore_attach_preview_object_state(object_snapshot)
    place_track_followers(track_object)
    refresh_view_layer()


def apply_inferred_train_attach(track_object, track_settings, import_layout, reverse_facing=False, rig_mode="STANDARD"):
    front_empty = import_layout.get("front_empty")
    ordered_empties = import_layout.get("ordered_empties", [])
    mount_lengths = import_layout.get("mount_lengths", [])
    if track_object is None or track_settings is None or front_empty is None:
        return 0.0
    previous_front = track_settings.driven_empty_object
    if previous_front is not None and previous_front != front_empty:
        remove_follower_drivers(previous_front)
    assign_rna_property(track_settings, "train_rig_mode", rig_mode)
    assign_rna_property(track_settings, "train_mount_axis_preset", import_layout.get("axis_preset", "Y_FORWARD_Z_UP"))
    assign_rna_property(track_settings, "train_mounts_reversed", reverse_facing)
    assign_rna_property(track_settings, "driven_empty_object", front_empty)
    ensure_follower_drivers(track_object, front_empty, offset_meters=0.0)
    front_empty.coaster_mixer_follower.reverse_forward_axis = False
    set_train_follower_role(front_empty, "IK_LEADER" if rig_mode == "IK_CHAIN" else "MOUNT")
    mark_train_mount(front_empty, enabled=False)
    running_offset = 0.0
    for empty_object, length_meters in zip(ordered_empties, mount_lengths):
        running_offset += length_meters
        ensure_follower_drivers(track_object, empty_object, offset_meters=running_offset)
        empty_object.coaster_mixer_follower.reverse_forward_axis = False
        set_train_follower_role(empty_object, "IK_TARGET" if rig_mode == "IK_CHAIN" else "MOUNT")
        mark_train_mount(empty_object)
    place_track_followers(track_object)
    refresh_view_layer()
    return running_offset


def get_selected_train_line_axis(empties):
    if len(empties) < 2:
        return Vector((1.0, 0.0, 0.0))
    endpoint_a = empties[0]
    endpoint_b = empties[1]
    farthest_distance_squared = -1.0
    for index, object_a in enumerate(empties[:-1]):
        position_a = object_a.matrix_world.translation
        for object_b in empties[index + 1:]:
            delta = object_b.matrix_world.translation - position_a
            distance_squared = delta.length_squared
            if distance_squared > farthest_distance_squared:
                farthest_distance_squared = distance_squared
                endpoint_a = object_a
                endpoint_b = object_b
    axis = endpoint_b.matrix_world.translation - endpoint_a.matrix_world.translation
    if axis.length <= 1.0e-8:
        return Vector((1.0, 0.0, 0.0))
    axis.normalize()
    return axis


def infer_empty_axis_preset(empties, line_axis):
    world_up = Vector((0.0, 0.0, 1.0))
    best_preset = "Y_FORWARD_Z_UP"
    best_score = float("-inf")
    best_forward_alignment = 0.0
    for preset_identifier, _label, _description in TRAIN_MOUNT_AXIS_PRESET_ITEMS:
        _right_axis, forward_axis, up_axis = get_train_mount_axis_basis(preset_identifier)
        forward_alignment_total = 0.0
        up_alignment_total = 0.0
        for empty_object in empties:
            rotation = empty_object.matrix_world.to_quaternion()
            forward_alignment_total += (rotation @ forward_axis).dot(line_axis)
            up_alignment_total += (rotation @ up_axis).dot(world_up)
        count = max(len(empties), 1)
        average_forward_alignment = forward_alignment_total / count
        average_up_alignment = up_alignment_total / count
        score = abs(average_forward_alignment) * 2.0 + average_up_alignment
        if score > best_score:
            best_score = score
            best_preset = preset_identifier
            best_forward_alignment = average_forward_alignment
    return best_preset, best_forward_alignment


def measure_empty_orientation_consistency(empties, axis_preset, line_axis):
    if not empties:
        return {"mismatch_count": 0, "worst_forward_alignment": 1.0, "worst_up_alignment": 1.0}
    _right_axis, forward_axis, up_axis = get_train_mount_axis_basis(axis_preset)
    mismatch_count = 0
    worst_forward_alignment = 1.0
    worst_up_alignment = 1.0
    world_up = Vector((0.0, 0.0, 1.0))
    for empty_object in empties:
        rotation = empty_object.matrix_world.to_quaternion()
        forward_alignment = abs((rotation @ forward_axis).dot(line_axis))
        up_alignment = (rotation @ up_axis).dot(world_up)
        worst_forward_alignment = min(worst_forward_alignment, forward_alignment)
        worst_up_alignment = min(worst_up_alignment, up_alignment)
        if forward_alignment < 0.75 or up_alignment < 0.25:
            mismatch_count += 1
    return {
        "mismatch_count": mismatch_count,
        "worst_forward_alignment": worst_forward_alignment,
        "worst_up_alignment": worst_up_alignment,
    }


def infer_selected_mount_layout(selected_empties, flip_train=False):
    empties = [object_ref for object_ref in selected_empties if object_ref is not None and object_ref.type == "EMPTY"]
    if not empties:
        return {"front_empty": None, "ordered_empties": [], "mount_lengths": [], "total_length": 0.0, "axis_preset": "Y_FORWARD_Z_UP"}
    if len(empties) == 1:
        return {
            "front_empty": empties[0],
            "ordered_empties": [],
            "mount_lengths": [],
            "total_length": 0.0,
            "axis_preset": "Y_FORWARD_Z_UP",
        }

    line_axis = get_selected_train_line_axis(empties)
    axis_preset, average_forward_alignment = infer_empty_axis_preset(empties, line_axis)
    sorted_empties = sorted(
        empties,
        key=lambda object_ref: (object_ref.matrix_world.translation.dot(line_axis), object_ref.name),
    )
    front_to_back = list(reversed(sorted_empties)) if average_forward_alignment >= 0.0 else list(sorted_empties)
    if flip_train:
        front_to_back.reverse()

    front_empty = front_to_back[0]
    ordered_empties = front_to_back[1:]
    gaps = []
    previous_object = front_empty
    for empty_object in ordered_empties:
        gap = (empty_object.matrix_world.translation - previous_object.matrix_world.translation).length
        gaps.append(max(gap, 0.01))
        previous_object = empty_object

    total_length = sum(gaps)
    return {
        "front_empty": front_empty,
        "ordered_empties": ordered_empties,
        "mount_lengths": gaps,
        "total_length": total_length,
        "axis_preset": axis_preset,
    }


def infer_selected_ik_chain_layout(context, selected_empties, flip_train=False):
    empties = [object_ref for object_ref in selected_empties if object_ref is not None and object_ref.type == "EMPTY"]
    active_object = getattr(context.view_layer.objects, "active", None)
    if active_object is None or active_object.type != "EMPTY" or active_object not in empties:
        active_object = empties[0] if empties else None
    if active_object is None:
        return {
            "front_empty": None,
            "leader_empty": None,
            "ordered_empties": [],
            "mount_lengths": [],
            "total_length": 0.0,
            "axis_preset": "Y_FORWARD_Z_UP",
            "orientation_check": {"mismatch_count": 0, "worst_forward_alignment": 1.0, "worst_up_alignment": 1.0},
        }

    target_empties = [object_ref for object_ref in empties if object_ref != active_object]
    if not target_empties:
        return {
            "front_empty": active_object,
            "leader_empty": active_object,
            "ordered_empties": [],
            "mount_lengths": [],
            "total_length": 0.0,
            "axis_preset": "Y_FORWARD_Z_UP",
            "orientation_check": {"mismatch_count": 0, "worst_forward_alignment": 1.0, "worst_up_alignment": 1.0},
        }

    line_axis = get_selected_train_line_axis(empties)
    axis_preset, _average_forward_alignment = infer_empty_axis_preset(empties, line_axis)
    leader_position = active_object.matrix_world.translation
    ordered_targets = sorted(
        target_empties,
        key=lambda object_ref: (
            (object_ref.matrix_world.translation - leader_position).dot(line_axis),
            object_ref.name,
        ),
    )
    if ordered_targets and (ordered_targets[0].matrix_world.translation - leader_position).dot(line_axis) < 0.0:
        line_axis.negate()
        ordered_targets = sorted(
            target_empties,
            key=lambda object_ref: (
                (object_ref.matrix_world.translation - leader_position).dot(line_axis),
                object_ref.name,
            ),
        )
    orientation_check = measure_empty_orientation_consistency(empties, axis_preset, line_axis)
    if flip_train:
        ordered_targets = list(reversed(ordered_targets))

    gaps = []
    previous_object = active_object
    for empty_object in ordered_targets:
        gap = (empty_object.matrix_world.translation - previous_object.matrix_world.translation).length
        gaps.append(max(gap, 0.01))
        previous_object = empty_object

    total_length = sum(gaps)
    return {
        "front_empty": active_object,
        "leader_empty": active_object,
        "ordered_empties": ordered_targets,
        "mount_lengths": gaps,
        "total_length": total_length,
        "axis_preset": axis_preset,
        "orientation_check": orientation_check,
    }


class COASTERMIXER_OT_edit_train_mount_length(bpy.types.Operator):
    bl_idname = "coaster_mixer.edit_train_mount_length"
    bl_label = "Edit Car Length"
    bl_description = "Change the gap to this car mount and shift this mount and everything behind it"
    bl_options = {"REGISTER", "UNDO"}

    empty_name: bpy.props.StringProperty(name="Empty Name")
    length_meters: bpy.props.FloatProperty(
        name="Length",
        description="Gap from the previous train anchor in meters; changing it shifts this mount and every mount behind it",
        min=0.01,
        subtype="DISTANCE",
        default=2.4,
    )
    adjust_train_length: bpy.props.BoolProperty(
        name="Set Train Length",
        description="Update the physical train length to match the resulting total mount layout",
        default=True,
    )
    adjust_station_length: bpy.props.BoolProperty(
        name="Resize Station to Train",
        description="Match the seam Station to the resulting train length by moving its entry backward while keeping its exit at the coaster start",
        default=True,
    )

    def _resolve_mount_context(self, context):
        empty_object = bpy.data.objects.get(self.empty_name)
        if empty_object is None or empty_object.type != "EMPTY":
            return None, None, None, None
        track_object = empty_object.coaster_mixer_follower.track_object
        if track_object is None or track_object.type != "CURVE":
            return empty_object, None, None, None
        mounts = collect_track_followers(track_object)
        try:
            mount_index = next(index for index, mount in enumerate(mounts) if mount == empty_object)
        except StopIteration:
            return empty_object, track_object, track_object.coaster_mixer_track, None
        return empty_object, track_object, track_object.coaster_mixer_track, mount_index

    def invoke(self, context, _event):
        _empty_object, _track_object, _track_settings, mount_index = self._resolve_mount_context(context)
        if mount_index is None:
            self.report({"WARNING"}, f"Train mount '{self.empty_name}' is no longer attached")
            return {"CANCELLED"}
        mounts = collect_track_followers(bpy.data.objects[self.empty_name].coaster_mixer_follower.track_object)
        self.length_meters = max(get_train_mount_length_meters(mounts, mount_index), 0.01)
        return context.window_manager.invoke_props_dialog(self, width=420)

    def draw(self, context):
        layout = self.layout
        empty_object, track_object, track_settings, mount_index = self._resolve_mount_context(context)
        if empty_object is None or track_object is None or track_settings is None or mount_index is None:
            layout.label(text="Train mount is no longer available.", icon="ERROR")
            return

        mounts = collect_track_followers(track_object)
        current_length = get_train_mount_length_meters(mounts, mount_index)
        current_total_length = get_train_mount_total_length_meters(mounts)
        delta = self.length_meters - current_length
        proposed_total_length = max(current_total_length + delta, 0.0)

        column = layout.column()
        column.use_property_split = True
        column.use_property_decorate = False
        column.prop(self, "length_meters")

        summary = layout.box()
        summary.label(text=f"Train length: {current_total_length:.2f} m -> {proposed_total_length:.2f} m", icon="DRIVER_DISTANCE")
        summary.prop(self, "adjust_train_length")

        station = get_adjustable_seam_station(track_object)
        station_column = summary.column()
        station_column.enabled = station is not None
        station_column.prop(self, "adjust_station_length")
        if station is not None:
            station_length = min(proposed_total_length, station["maximum_length"])
            station_column.label(text=f"Station: {station['current_length']:.2f} m -> {station_length:.2f} m")
            station_column.label(text="Exit stays at coaster start; entry moves backward.", icon="INFO")
            if proposed_total_length > station["maximum_length"] + 1.0e-3:
                station_column.label(text="Limited by the final track piece length.", icon="ERROR")
        else:
            summary.label(text="No adjustable Station ending at the coaster start.", icon="INFO")

    def execute(self, context):
        empty_object, track_object, track_settings, mount_index = self._resolve_mount_context(context)
        if empty_object is None or track_object is None or track_settings is None or mount_index is None:
            self.report({"WARNING"}, f"Train mount '{self.empty_name}' is no longer attached")
            return {"CANCELLED"}

        mounts = collect_track_followers(track_object)
        current_length = get_train_mount_length_meters(mounts, mount_index)
        delta = self.length_meters - current_length
        if abs(delta) <= 1.0e-5:
            return {"FINISHED"}

        for mount in mounts[mount_index:]:
            settings = mount.coaster_mixer_follower
            settings.offset_meters = max(settings.offset_meters + delta, 0.0)

        proposed_total_length = get_train_mount_total_length_meters(collect_track_followers(track_object))
        if self.adjust_train_length:
            track_settings.train_length_meters = proposed_total_length

        station_message = ""
        if self.adjust_station_length:
            station = get_adjustable_seam_station(track_object)
            if station is not None:
                station_length = resize_seam_station(station, proposed_total_length)
                station_message = f"; Station resized backward to {station_length:.2f} m"

        tag_track_placement_update(track_object)
        tag_redraw_view3d()
        train_message = f"; train length set to {proposed_total_length:.2f} m" if self.adjust_train_length else ""
        self.report({"INFO"}, f"Updated {empty_object.name} length to {self.length_meters:.2f} m{train_message}{station_message}")
        return {"FINISHED"}


class COASTERMIXER_OT_create_train_followers(bpy.types.Operator):
    bl_idname = "coaster_mixer.create_train_followers"
    bl_label = "Create Car Mounts"
    bl_description = "Create empty train mounts spaced in meters behind the train front, ready to parent train cars to"
    bl_options = {"REGISTER", "UNDO"}

    car_count: bpy.props.IntProperty(
        name="Car Count",
        description="Number of train mounts to create",
        min=1,
        max=64,
        default=4,
    )
    length_meters: bpy.props.FloatProperty(
        name="Length",
        description="Car length in meters; the same value is used as spacing between train mounts",
        min=0.01,
        subtype="DISTANCE",
        default=2.4,
    )
    adjust_train_length: bpy.props.BoolProperty(
        name="Set Train Length",
        description="Set the physical train length to the resulting full mount layout length",
        default=True,
    )
    adjust_station_length: bpy.props.BoolProperty(
        name="Resize Station to Train",
        description="Match the seam Station to the calculated train length by moving its entry backward while keeping its exit at the coaster start",
        default=True,
    )

    @classmethod
    def poll(cls, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        return track_object is not None

    def invoke(self, context, _event):
        track_object, _track_settings = resolve_active_track_settings(context)
        mounts = collect_track_followers(track_object) if track_object is not None else []
        if mounts:
            self.car_count = 1
            self.length_meters = max(get_train_mount_length_meters(mounts, len(mounts) - 1), 0.01)
        return context.window_manager.invoke_props_dialog(self, width=420)

    def draw(self, context):
        layout = self.layout
        column = layout.column()
        column.use_property_split = True
        column.use_property_decorate = False
        column.prop(self, "car_count")
        column.prop(self, "length_meters")

        track_object, track_settings = resolve_active_track_settings(context)
        mounts = collect_track_followers(track_object) if track_object is not None else []
        current_total_length = get_train_mount_total_length_meters(mounts)
        proposed_length = current_total_length + self.car_count * self.length_meters
        proposal = layout.box()
        proposal.label(text=f"Train length: {current_total_length:.2f} m -> {proposed_length:.2f} m", icon="DRIVER_DISTANCE")
        proposal.prop(self, "adjust_train_length")

        station = get_adjustable_seam_station(track_object) if track_object is not None else None
        station_column = proposal.column()
        station_column.enabled = station is not None
        station_column.prop(self, "adjust_station_length")
        if station is not None:
            station_length = min(proposed_length, station["maximum_length"])
            station_column.label(text=f"Station: {station['current_length']:.2f} m → {station_length:.2f} m")
            station_column.label(text="Exit stays at coaster start; entry moves backward.", icon="INFO")
            if proposed_length > station["maximum_length"] + 1.0e-3:
                station_column.label(text="Limited by the final track piece length.", icon="ERROR")
        else:
            proposal.label(text="No adjustable Station ending at the coaster start.", icon="INFO")

    def execute(self, context):
        track_object, track_settings = resolve_active_track_settings(context)
        if track_object is None:
            self.report({"WARNING"}, "Select a curve object first")
            return {"CANCELLED"}

        generated_collection = get_generated_track_collection(track_object, "Train Rig")
        ensure_train_front_empty(track_object, track_settings, generated_collection)
        assign_rna_property(track_settings, "train_rig_mode", "STANDARD")
        mounts = collect_track_followers(track_object)
        base_offset = get_train_mount_total_length_meters(mounts)
        for car_index in range(self.car_count):
            offset_meters = base_offset + self.length_meters * (car_index + 1)
            empty_object = bpy.data.objects.new(f"{track_object.name} Car {car_index + 1:02d}", None)
            empty_object.empty_display_type = "PLAIN_AXES"
            empty_object.empty_display_size = 0.5
            generated_collection.objects.link(empty_object)
            ensure_follower_drivers(track_object, empty_object, offset_meters=offset_meters)
            mark_train_mount(empty_object)

        proposed_length = base_offset + self.car_count * self.length_meters
        if self.adjust_train_length:
            track_settings.train_length_meters = proposed_length

        station_message = ""
        if self.adjust_station_length:
            station = get_adjustable_seam_station(track_object)
            if station is not None:
                station_length = resize_seam_station(station, proposed_length)
                station_message = f"; Station resized backward to {station_length:.2f} m"

        tag_track_placement_update(track_object)
        tag_redraw_view3d()
        train_message = f"; train length set to {proposed_length:.2f} m" if self.adjust_train_length else ""
        noun = "train mount" if self.car_count == 1 else "train mounts"
        self.report({"INFO"}, f"Created {self.car_count} {noun}{train_message}{station_message}")
        return {"FINISHED"}


class COASTERMIXER_OT_create_train_camera(bpy.types.Operator):
    bl_idname = "coaster_mixer.create_train_camera"
    bl_label = "Create Ride Camera"
    bl_description = "Create a bank-following camera above a main train empty, aimed at a track-driven look-ahead target"
    bl_options = {"REGISTER", "UNDO"}

    height_meters: bpy.props.FloatProperty(
        name="Height",
        description="Camera height above the mounted train empty in meters",
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
        mounts = collect_main_train_mounts(track_object)
        if len(mounts) < 1:
            self.report({"WARNING"}, "Create or attach at least one main train empty before creating the ride camera")
            return {"CANCELLED"}

        mount_object = mounts[1] if len(mounts) > 1 else mounts[0]
        target_object = bpy.data.objects.new(f"{track_object.name} Camera Look Ahead", None)
        target_object.empty_display_type = "PLAIN_AXES"
        target_object.empty_display_size = 0.25
        target_object["coaster_mixer_camera_target"] = True
        camera_collection = get_generated_track_collection(track_object, "Cameras")
        camera_collection.objects.link(target_object)
        ensure_follower_drivers(track_object, target_object, offset_meters=-self.look_ahead_meters)

        camera_data = bpy.data.cameras.new(f"{track_object.name} Ride Camera")
        camera_data.lens = self.lens_millimeters
        camera_object = bpy.data.objects.new(camera_data.name, camera_data)
        camera_collection.objects.link(camera_object)
        camera_object.parent = mount_object
        camera_object.location = (0.0, 0.0, self.height_meters)
        camera_object.rotation_euler = (0.0, 0.0, 0.0)

        aim_constraint = camera_object.constraints.new(type="TRACK_TO")
        aim_constraint.name = "Coaster Mixer Look Ahead"
        aim_constraint.target = target_object
        aim_constraint.track_axis = "TRACK_NEGATIVE_Z"
        aim_constraint.up_axis = "UP_Y"
        aim_constraint.influence = 0.0

        camera_settings = camera_object.coaster_mixer_camera
        camera_settings.track_object = track_object
        camera_settings.mount_object = mount_object
        camera_settings.target_object = target_object
        camera_settings.offset_xyz = (0.0, 0.0, self.height_meters)
        camera_settings.look_ahead_meters = self.look_ahead_meters
        camera_settings.target_offset_xyz = (0.0, 0.0, self.height_meters)
        target_object.coaster_mixer_follower.source_mount_object = mount_object

        if self.make_active:
            context.scene.camera = camera_object

        place_track_followers(track_object)
        refresh_view_layer()
        tag_redraw_view3d()
        self.report({"INFO"}, f"Created ride camera above {mount_object.name}")
        return {"FINISHED"}


class COASTERMIXER_OT_remove_train_camera(bpy.types.Operator):
    bl_idname = "coaster_mixer.remove_train_camera"
    bl_label = "Delete Ride Camera"
    bl_description = "Delete this ride camera and its managed look-ahead helper"
    bl_options = {"REGISTER", "UNDO"}

    camera_name: bpy.props.StringProperty(name="Camera")

    @classmethod
    def poll(cls, context):
        return resolve_active_track_object(context) is not None

    def execute(self, context):
        track_object = resolve_active_track_object(context)
        camera_object = bpy.data.objects.get(self.camera_name)
        if camera_object is None or camera_object.type != "CAMERA":
            self.report({"WARNING"}, "Ride camera is no longer available")
            return {"CANCELLED"}
        camera_settings = getattr(camera_object, "coaster_mixer_camera", None)
        if camera_settings is None or camera_settings.track_object != track_object:
            self.report({"WARNING"}, "Camera does not belong to the active coaster")
            return {"CANCELLED"}

        target_object = camera_settings.target_object
        camera_data = camera_object.data
        bpy.data.objects.remove(camera_object, do_unlink=True)

        if (
            target_object is not None
            and target_object.type == "EMPTY"
            and target_object.get("coaster_mixer_camera_target", False)
            and bpy.data.objects.get(target_object.name) is not None
            and get_camera_target_owner(track_object, target_object) is None
        ):
            remove_follower_drivers(target_object)
            bpy.data.objects.remove(target_object, do_unlink=True)

        if camera_data is not None and camera_data.users == 0:
            bpy.data.cameras.remove(camera_data)
        tag_redraw_view3d()
        self.report({"INFO"}, "Deleted ride camera")
        return {"FINISHED"}


class COASTERMIXER_OT_add_wheel_spin_binding(bpy.types.Operator):
    bl_idname = "coaster_mixer.add_wheel_spin_binding"
    bl_label = "Add Wheel Spin Driver"
    bl_description = "Add a managed wheel-spin driver binding for a wheel bone collection"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        return track_object is not None

    def execute(self, context):
        track_object, track_settings = resolve_active_track_settings(context)
        if track_object is None or track_settings is None:
            return {"CANCELLED"}

        binding = track_settings.wheel_spin_bindings.add()
        binding.binding_key = generate_wheel_spin_binding_key()

        active_object = getattr(context.view_layer.objects, "active", None)
        if active_object is not None and active_object.type == "ARMATURE":
            binding.armature_object = active_object
            collection_items = get_armature_bone_collection_items(binding, context)
            if collection_items and collection_items[0][0]:
                binding.bone_collection_name = collection_items[0][0]

        track_settings.active_wheel_spin_binding_index = len(track_settings.wheel_spin_bindings) - 1
        sync_wheel_spin_bindings()
        tag_redraw_view3d()
        self.report({"INFO"}, "Added wheel spin driver binding")
        return {"FINISHED"}


class COASTERMIXER_OT_duplicate_wheel_spin_binding(bpy.types.Operator):
    bl_idname = "coaster_mixer.duplicate_wheel_spin_binding"
    bl_label = "Duplicate Wheel Spin Driver"
    bl_description = "Duplicate this wheel setup so it can be assigned to another bone collection"
    bl_options = {"REGISTER", "UNDO"}

    binding_index: bpy.props.IntProperty(name="Binding Index", min=0, default=0)

    @classmethod
    def poll(cls, context):
        track_object, track_settings = resolve_active_track_settings(context)
        return track_object is not None and track_settings is not None

    def execute(self, context):
        _track_object, track_settings = resolve_active_track_settings(context)
        if track_settings is None:
            return {"CANCELLED"}
        if self.binding_index < 0 or self.binding_index >= len(track_settings.wheel_spin_bindings):
            self.report({"WARNING"}, "Wheel spin binding is no longer available")
            return {"CANCELLED"}

        source = track_settings.wheel_spin_bindings[self.binding_index]
        source_values = {
            "armature_object": source.armature_object,
            "bone_collection_name": source.bone_collection_name,
            "wheel_diameter_meters": source.wheel_diameter_meters,
            "rotation_axis": source.rotation_axis,
            "invert_rotation": source.invert_rotation,
        }
        duplicate = track_settings.wheel_spin_bindings.add()
        duplicate.binding_key = generate_wheel_spin_binding_key()
        for attribute, value in source_values.items():
            if attribute == "bone_collection_name" and not value:
                continue
            setattr(duplicate, attribute, value)

        track_settings.active_wheel_spin_binding_index = len(track_settings.wheel_spin_bindings) - 1
        sync_wheel_spin_bindings()
        tag_redraw_view3d()
        self.report({"INFO"}, "Duplicated wheel spin driver binding")
        return {"FINISHED"}


class COASTERMIXER_OT_remove_wheel_spin_binding(bpy.types.Operator):
    bl_idname = "coaster_mixer.remove_wheel_spin_binding"
    bl_label = "Remove Wheel Spin Driver"
    bl_description = "Remove this managed wheel-spin driver binding and clean up its bone drivers"
    bl_options = {"REGISTER", "UNDO"}

    binding_index: bpy.props.IntProperty(name="Binding Index", min=0, default=0)

    @classmethod
    def poll(cls, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        return track_object is not None

    def execute(self, context):
        track_object, track_settings = resolve_active_track_settings(context)
        if track_object is None or track_settings is None:
            return {"CANCELLED"}
        if self.binding_index < 0 or self.binding_index >= len(track_settings.wheel_spin_bindings):
            self.report({"WARNING"}, "Wheel spin binding is no longer available")
            return {"CANCELLED"}

        track_settings.wheel_spin_bindings.remove(self.binding_index)
        track_settings.active_wheel_spin_binding_index = clamp(
            track_settings.active_wheel_spin_binding_index,
            0,
            max(len(track_settings.wheel_spin_bindings) - 1, 0),
        )
        sync_wheel_spin_bindings()
        tag_redraw_view3d()
        self.report({"INFO"}, "Removed wheel spin driver binding")
        return {"FINISHED"}


def create_or_refresh_wheelcarrier_helpers(track_object, target_collection):
    mounts = collect_main_train_mounts(track_object)
    if not mounts:
        return 0, 0

    created_count = 0
    refreshed_count = 0
    for mount_object in mounts:
        existing_helpers = {
            helper_object.get("coaster_mixer_wheelcarrier_side", ""): helper_object
            for helper_object in get_bound_wheelcarrier_helpers(mount_object)
        }
        for side_identifier, side_label in (("L", "Left"), ("R", "Right")):
            helper_object = existing_helpers.get(side_identifier)
            if helper_object is None:
                helper_object = bpy.data.objects.new(f"{mount_object.name} Wheelcarrier {side_label}", None)
                helper_object.empty_display_type = "PLAIN_AXES"
                helper_object.empty_display_size = 0.2
                target_collection.objects.link(helper_object)
                created_count += 1
            else:
                refreshed_count += 1
            helper_object["coaster_mixer_wheelcarrier_helper"] = True
            helper_object["coaster_mixer_wheelcarrier_side"] = side_identifier
            helper_object.coaster_mixer_follower.source_mount_object = mount_object
            helper_object.coaster_mixer_follower.reverse_forward_axis = mount_object.coaster_mixer_follower.reverse_forward_axis
            ensure_follower_drivers(track_object, helper_object, offset_meters=0.0)
    return created_count, refreshed_count


def get_wheelcarrier_helper_reference_count(track_object):
    return sum(len(helper_object.children) for helper_object in collect_wheelcarrier_helpers(track_object))


def disable_wheelcarrier_helpers(track_object):
    removed_count = 0
    detached_count = 0
    for helper_object in list(collect_wheelcarrier_helpers(track_object)):
        if helper_object.children:
            remove_follower_drivers(helper_object)
            helper_object.pop("coaster_mixer_wheelcarrier_helper", None)
            helper_object.pop("coaster_mixer_wheelcarrier_side", None)
            helper_object.coaster_mixer_follower.source_mount_object = None
            detached_count += 1
            continue
        remove_object_hierarchy(helper_object)
        removed_count += 1
    return removed_count, detached_count


class COASTERMIXER_OT_create_wheelcarrier_helpers(bpy.types.Operator):
    bl_idname = "coaster_mixer.create_wheelcarrier_helpers"
    bl_label = "Create Wheelcarrier Pairs"
    bl_description = "Create or refresh symmetric left and right wheelcarrier helper empties for every main train empty"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        return track_object is not None and len(collect_main_train_mounts(track_object)) > 0

    def execute(self, context):
        track_object, track_settings = resolve_active_track_settings(context)
        mounts = collect_main_train_mounts(track_object)
        if not mounts:
            self.report({"WARNING"}, "Create or attach at least one main train empty first")
            return {"CANCELLED"}

        target_collection = get_generated_track_collection(track_object, "Wheelcarriers")
        track_settings.wheelcarrier_helpers_enabled = True
        created_count, refreshed_count = create_or_refresh_wheelcarrier_helpers(track_object, target_collection)

        tag_track_placement_update(track_object)
        tag_redraw_view3d()
        self.report({"INFO"}, f"Created {created_count} and refreshed {refreshed_count} wheelcarrier helpers")
        return {"FINISHED"}


class COASTERMIXER_OT_toggle_wheelcarrier_helpers(bpy.types.Operator):
    bl_idname = "coaster_mixer.toggle_wheelcarrier_helpers"
    bl_label = "Toggle Wheelcarrier Helpers"
    bl_description = "Enable or disable wheelcarrier helper empties for the current train"
    bl_options = {"REGISTER", "UNDO"}

    enable: bpy.props.BoolProperty(name="Enable", default=True)
    detach_referenced: bpy.props.BoolProperty(
        name="Detach Referenced Helpers",
        description="Disable wheelcarrier helpers even when other objects are still parented to them",
        default=False,
    )

    @classmethod
    def poll(cls, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        return track_object is not None

    def invoke(self, context, _event):
        track_object, _track_settings = resolve_active_track_settings(context)
        if self.enable or track_object is None:
            return self.execute(context)
        if get_wheelcarrier_helper_reference_count(track_object) > 0:
            return context.window_manager.invoke_props_dialog(self, width=420)
        return self.execute(context)

    def draw(self, context):
        layout = self.layout
        track_object, _track_settings = resolve_active_track_settings(context)
        reference_count = get_wheelcarrier_helper_reference_count(track_object) if track_object is not None else 0
        layout.label(text=f"{reference_count} child object(s) are still parented to wheelcarrier helpers.", icon="ERROR")
        layout.label(text="Disabling will detach those helpers from Coaster Mixer and keep them as regular empties.")
        layout.prop(self, "detach_referenced")

    def execute(self, context):
        track_object, track_settings = resolve_active_track_settings(context)
        if track_object is None or track_settings is None:
            return {"CANCELLED"}

        if self.enable:
            target_collection = get_generated_track_collection(track_object, "Wheelcarriers")
            track_settings.wheelcarrier_helpers_enabled = True
            created_count, refreshed_count = create_or_refresh_wheelcarrier_helpers(track_object, target_collection)
            tag_track_placement_update(track_object)
            tag_redraw_view3d()
            self.report({"INFO"}, f"Enabled wheelcarrier helpers ({created_count} created, {refreshed_count} refreshed)")
            return {"FINISHED"}

        reference_count = get_wheelcarrier_helper_reference_count(track_object)
        if reference_count > 0 and not self.detach_referenced:
            self.report({"WARNING"}, "Wheelcarrier helpers still have child objects; confirm detaching them first")
            return {"CANCELLED"}

        removed_count, detached_count = disable_wheelcarrier_helpers(track_object)
        track_settings.wheelcarrier_helpers_enabled = False
        tag_track_placement_update(track_object)
        tag_redraw_view3d()
        self.report({"INFO"}, f"Disabled wheelcarrier helpers ({removed_count} removed, {detached_count} detached)")
        return {"FINISHED"}


class COASTERMIXER_OT_detach_follower(bpy.types.Operator):
    bl_idname = "coaster_mixer.detach_follower"
    bl_label = "Remove Car Mount"
    bl_description = "Remove this train mount and any train objects parented to it"
    bl_options = {"REGISTER", "UNDO"}

    empty_name: bpy.props.StringProperty(name="Empty Name")

    def execute(self, _context):
        empty_object = bpy.data.objects.get(self.empty_name)
        if empty_object is None:
            self.report({"WARNING"}, f"Object '{self.empty_name}' not found")
            return {"CANCELLED"}

        if bool(empty_object.get("coaster_mixer_train_mount", False)):
            remove_train_mount(empty_object)
            tag_redraw_view3d()
            self.report({"INFO"}, "Removed train mount")
            return {"FINISHED"}

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
    bl_description = "Bake standalone train, wheel, camera, and metric animation that plays without the add-on"
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
        original_frame = scene.frame_current
        try:
            if track_object.get(STANDALONE_BAKE_PROPERTY, False):
                clear_standalone_visual_bake(track_object)
            placement_objects = list(collect_track_placement_objects(track_object))
            placement_objects.extend(collect_ride_cameras(track_object))
            placement_objects = list(dict.fromkeys(placement_objects))
            object_samples = {
                object_ref: {path: [[] for _axis in range(3)] for path in ("location", "rotation_euler")}
                for object_ref in placement_objects
            }
            wheel_samples = []
            for binding in get_track_wheel_spin_bindings(track_object):
                armature_object = binding.armature_object
                axis_index = get_wheel_spin_axis_index(binding.rotation_axis)
                direction_sign = -1.0 if binding.invert_rotation else 1.0
                radians_per_meter = (
                    2.0 / max(binding.wheel_diameter_meters, 1.0e-6) * direction_sign
                )
                for pose_bone in get_wheel_spin_pose_bones(armature_object, binding.bone_collection_name):
                    wheel_samples.append(
                        (armature_object, pose_bone, axis_index, radians_per_meter, [])
                    )
            metric_samples = {
                "cm_baked_speed_mps": [],
                "cm_baked_lateral_g": [],
                "cm_baked_vertical_g": [],
                "cm_baked_longitudinal_g": [],
                "cm_baked_total_g": [],
            }
            travel_values = []
            route = get_resolved_route(track_object)
            window_manager.progress_begin(frame_start, frame_end)
            frame_values = []
            for frame in range(frame_start, frame_end + 1):
                sample = sample_simulation_trajectory(scene, track_object, track_settings, frame)
                if sample is None:
                    self.report({"WARNING"}, "The route is empty; nothing to bake")
                    return {"CANCELLED"}
                front_meters, speed_mps, _stop_remaining = sample
                travel_meters = sample_simulation_travel_distance(
                    scene, track_object, track_settings, frame
                )
                frame_values.append((frame, front_meters))
                travel_values.append((frame, travel_meters))
                assign_rna_property(track_settings, "train_front_route_meters", front_meters)
                assign_rna_property(track_settings, "train_travel_distance_meters", travel_meters)
                assign_rna_property(scene_settings, "simulation_current_speed_mps", speed_mps)
                place_track_followers(track_object, front_meters)
                for object_ref, paths in object_samples.items():
                    for data_path in ("location", "rotation_euler"):
                        value = getattr(object_ref, data_path)
                        for axis_index in range(3):
                            paths[data_path][axis_index].append((frame, value[axis_index]))
                for _armature, _pose_bone, _axis_index, radians_per_meter, values in wheel_samples:
                    values.append((frame, travel_meters * radians_per_meter))
                metrics = get_simulation_overlay_metrics(
                    scene, track_object, track_settings, route, front_meters, speed_mps
                )
                metric_samples["cm_baked_speed_mps"].append((frame, speed_mps))
                if metrics is not None:
                    metric_samples["cm_baked_lateral_g"].append((frame, metrics["lateral_g_signed"]))
                    metric_samples["cm_baked_vertical_g"].append((frame, metrics["vertical_g_signed"]))
                    metric_samples["cm_baked_longitudinal_g"].append((frame, metrics["longitudinal_g_signed"]))
                    metric_samples["cm_baked_total_g"].append((frame, metrics["total_g"]))
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

            clear_action_fcurve(track_object, TRAIN_TRAVEL_DISTANCE_DATA_PATH)
            insert_dense_fcurve_keyframes(track_object, TRAIN_TRAVEL_DISTANCE_DATA_PATH, travel_values)
            track_object[STANDALONE_BAKE_PROPERTY] = True
            for object_ref, paths in object_samples.items():
                object_ref[STANDALONE_BAKE_TRACK_PROPERTY] = track_object.name
                for data_path, axes in paths.items():
                    for axis_index, values in enumerate(axes):
                        insert_dense_indexed_fcurve_keyframes(
                            object_ref, data_path, axis_index, values
                        )
            for armature_object, pose_bone, axis_index, _radians_per_meter, values in wheel_samples:
                remove_wheel_spin_driver_axis(armature_object, pose_bone, axis_index)
                insert_dense_indexed_fcurve_keyframes(
                    armature_object,
                    get_wheel_spin_bone_data_path(pose_bone),
                    axis_index,
                    values,
                )
            for property_name, values in metric_samples.items():
                scene[property_name] = values[0][1] if values else 0.0
                insert_dense_fcurve_keyframes(scene, f'["{property_name}"]', values)

            self.place_trigger_markers(scene, frame_start, frame_end)
            assign_simulation_enabled(scene_settings, False)
            scene.frame_set(original_frame)
        except Exception as exc:
            self.report({"ERROR"}, f"Simulation bake failed: {exc}")
            return {"CANCELLED"}
        finally:
            window_manager.progress_end()

        baked_frame_count = frame_end - frame_start + 1
        bake_seconds = perf_counter() - bake_started
        self.report(
            {"INFO"},
            f"Baked {baked_frame_count} standalone frames in {bake_seconds:.2f}s and disabled runtime simulation",
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
            mapped_index = resolve_trajectory_index(cache, max(frame, 0))
            for channel, value in events_by_index.get(mapped_index, ()):
                scene.timeline_markers.new(f"{marker_prefix}{channel}={value:g}", frame=frame)


class COASTERMIXER_OT_clear_baked_simulation(bpy.types.Operator):
    bl_idname = "coaster_mixer.clear_baked_simulation"
    bl_label = "Clear Baked Keys"
    bl_description = "Remove standalone baked animation and restore live placement and wheel drivers"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        track_object, _track_settings = resolve_active_track_settings(context)
        return track_object is not None and has_baked_path_animation(track_object)

    def execute(self, context):
        track_object, track_settings = resolve_active_track_settings(context)
        if track_object is None:
            self.report({"WARNING"}, "Select a curve object first")
            return {"CANCELLED"}

        cleared_visuals = clear_standalone_visual_bake(track_object)
        cleared_front = clear_action_fcurve(track_object, TRAIN_FRONT_METERS_DATA_PATH)
        clear_action_fcurve(track_object, TRAIN_TRAVEL_DISTANCE_DATA_PATH)
        if not cleared_front and not cleared_visuals:
            self.report({"WARNING"}, "No baked path-factor keys found")
            return {"CANCELLED"}

        scene_settings = getattr(context.scene, "coaster_mixer_scene", None)
        invalidate_simulation_trajectory()
        if scene_settings is not None:
            assign_simulation_enabled(scene_settings, True)
            apply_simulation_frame(context.scene, track_object, track_settings)
        context.scene.frame_set(context.scene.frame_current)
        self.report({"INFO"}, "Cleared standalone bake and resumed live simulation")
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
    block.control_tree = create_default_control_tree("Station", move_offset=station_length)
    block.control_template = "LOAD_STATION"
    settings.active_zone_index = 1
    settings.active_block_group_index = 0
    context.scene.coaster_mixer_scene.simulation_start_route_meters = 0.0
    block_group_update(settings, context)
    context.scene.frame_set(0)
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
