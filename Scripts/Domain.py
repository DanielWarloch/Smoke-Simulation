import bpy


bpy.data.collections['DOMAIN'].objects['domain'].select_set(True)
bpy.ops.object.modifier_add(type='FLUID')
bpy.context.object.modifiers["Fluid"].fluid_type = 'DOMAIN'
bpy.context.object.modifiers["Fluid"].domain_settings.domain_type = 'GAS'


# Ustawienia domeny
bpy.context.object.modifiers["Fluid"].domain_settings.resolution_max = 24
bpy.context.object.modifiers["Fluid"].domain_settings.use_adaptive_domain = True
bpy.context.object.modifiers["Fluid"].domain_settings.additional_res = 256
bpy.context.object.modifiers["Fluid"].domain_settings.adapt_margin = 4

bpy.context.object.modifiers["Fluid"].domain_settings.cache_frame_start = 1
bpy.context.object.modifiers["Fluid"].domain_settings.cache_frame_end = 90

bpy.context.object.modifiers["Fluid"].domain_settings.display_thickness = 25
bpy.context.object.modifiers["Fluid"].domain_settings.alpha = 5
bpy.context.object.modifiers["Fluid"].domain_settings.beta = 1
bpy.context.object.modifiers["Fluid"].domain_settings.time_scale = 1.4
bpy.context.object.modifiers["Fluid"].domain_settings.cfl_condition = 4.5

bpy.context.object.modifiers["Fluid"].domain_settings.use_adaptive_timesteps = True
bpy.context.object.modifiers["Fluid"].domain_settings.timesteps_max = 4
bpy.context.object.modifiers["Fluid"].domain_settings.timesteps_min = 1

bpy.context.object.modifiers["Fluid"].domain_settings.use_collision_border_front = True
bpy.context.object.modifiers["Fluid"].domain_settings.use_collision_border_left = True
bpy.context.object.modifiers["Fluid"].domain_settings.use_collision_border_top = True
bpy.context.object.modifiers["Fluid"].domain_settings.use_collision_border_back = True
bpy.context.object.modifiers["Fluid"].domain_settings.use_collision_border_right = True
bpy.context.object.modifiers["Fluid"].domain_settings.use_collision_border_bottom = True

bpy.context.object.modifiers["Fluid"].domain_settings.vorticity = 0
bpy.context.object.modifiers["Fluid"].domain_settings.adapt_margin = 4
bpy.context.object.modifiers["Fluid"].domain_settings.adapt_threshold = 0.02


bpy.context.object.modifiers["Fluid"].domain_settings.cache_directory = "//cache_fluid5"

# Wizualizacja gêstoœci
bpy.context.object.modifiers["Fluid"].domain_settings.use_color_ramp = True
bpy.context.object.modifiers["Fluid"].domain_settings.coba_field = 'DENSITY'
