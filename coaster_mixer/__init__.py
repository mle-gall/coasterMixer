# SPDX-FileCopyrightText: 2026 Coaster Mixer contributors
# SPDX-License-Identifier: GPL-3.0-or-later

bl_info = {
    "name": "Coaster Mixer",
    "author": "Coaster Mixer contributors",
    "version": (0, 3, 0),
    "blender": (5, 0, 0),
    "location": "View3D > Sidebar > Coaster",
    "description": "Prototype tools for authoring coaster track zones on a guide curve",
    "category": "Object",
}

"""Coaster Mixer add-on package and Blender registration entry point."""

from . import runtime
from .ui import *

# Blender invokes these runtime callbacks after every module has loaded. Keep
# the import graph one-way, then provide the three deliberate higher-layer
# hooks here instead of creating circular imports.
runtime.compile_control_tree = compile_control_tree
runtime.get_clamped_block_group_index = get_clamped_block_group_index
runtime.seed_default_station = seed_default_station

CLASSES = (
    CoasterMixerControlSocket,
    CoasterMixerControlTree,
    COASTERMIXER_ND_block_entered,
    COASTERMIXER_ND_set_transport,
    COASTERMIXER_ND_set_brake,
    COASTERMIXER_ND_set_brake_hold,
    COASTERMIXER_ND_release_brake,
    COASTERMIXER_ND_wait,
    COASTERMIXER_ND_wait_position,
    COASTERMIXER_ND_wait_speed,
    COASTERMIXER_ND_trigger,
    COASTERMIXER_ND_dispatch,
    CoasterMixerZone,
    CoasterMixerAction,
    CoasterMixerSensor,
    CoasterMixerBlockMember,
    CoasterMixerBlockGroup,
    CoasterMixerConnection,
    CoasterMixerFollowerSettings,
    CoasterMixerTrackSettings,
    CoasterMixerSceneSettings,
    COASTERMIXER_UL_zones,
    COASTERMIXER_UL_actions,
    COASTERMIXER_OT_add_zone,
    COASTERMIXER_OT_duplicate_zone,
    COASTERMIXER_OT_remove_zone,
    COASTERMIXER_OT_apply_control_template,
    COASTERMIXER_OT_add_block_group,
    COASTERMIXER_OT_remove_block_group,
    COASTERMIXER_OT_edit_control_graph,
    COASTERMIXER_OT_add_active_zone_to_block,
    COASTERMIXER_OT_create_block_from_active_zone,
    COASTERMIXER_OT_remove_block_member,
    COASTERMIXER_OT_add_action,
    COASTERMIXER_OT_remove_action,
    COASTERMIXER_OT_move_action,
    COASTERMIXER_OT_add_sensor,
    COASTERMIXER_OT_remove_sensor,
    COASTERMIXER_OT_snap_start_to_station,
    COASTERMIXER_OT_setup_driven_empty,
    COASTERMIXER_OT_attach_selected_followers,
    COASTERMIXER_OT_create_train_followers,
    COASTERMIXER_OT_create_train_camera,
    COASTERMIXER_OT_detach_follower,
    COASTERMIXER_OT_add_connection,
    COASTERMIXER_OT_remove_connection,
    COASTERMIXER_OT_reset_simulation,
    COASTERMIXER_OT_bake_simulation,
    COASTERMIXER_OT_clear_baked_simulation,
    COASTERMIXER_OT_set_root_from_active,
    COASTERMIXER_OT_clear_curve_setup,
    COASTERMIXER_OT_select_piece,
    COASTERMIXER_PT_coaster,
    COASTERMIXER_PT_route,
    COASTERMIXER_PT_piece,
    COASTERMIXER_PT_connections,
    COASTERMIXER_PT_train,
    COASTERMIXER_PT_simulation,
    COASTERMIXER_PT_blocks,
    COASTERMIXER_PT_setup_utilities,
)


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)

    bpy.types.Object.coaster_mixer_track = bpy.props.PointerProperty(type=CoasterMixerTrackSettings)
    bpy.types.Object.coaster_mixer_follower = bpy.props.PointerProperty(type=CoasterMixerFollowerSettings)
    bpy.types.Scene.coaster_mixer_scene = bpy.props.PointerProperty(type=CoasterMixerSceneSettings)
    bpy.types.NODE_MT_add.append(draw_control_node_add_menu)
    ensure_driver_namespace()
    ensure_viewport_draw_handler()
    ensure_control_selection_subscription()
    ensure_frame_change_handler()
    ensure_depsgraph_update_handler()
    ensure_undo_redo_handlers()
    ensure_load_post_handler()


def unregister():
    bpy.types.NODE_MT_add.remove(draw_control_node_add_menu)
    remove_load_post_handler()
    remove_undo_redo_handlers()
    remove_depsgraph_update_handler()
    remove_frame_change_handler()
    remove_viewport_draw_handler()
    remove_control_selection_subscription()
    remove_driver_namespace()
    clear_runtime_caches()
    del bpy.types.Scene.coaster_mixer_scene
    del bpy.types.Object.coaster_mixer_follower
    del bpy.types.Object.coaster_mixer_track

    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
