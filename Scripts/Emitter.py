import bpy


bpy.ops.object.select_all(action='DESELECT')

bpy.data.collections['EMITTER'].objects['Emitter'].select_set(True)

# Tworzenie Emittera
bpy.ops.object.modifier_add(type='FLUID')
bpy.context.object.modifiers["Fluid"].fluid_type = 'FLOW'
bpy.context.object.modifiers["Fluid"].flow_settings.flow_type = 'SMOKE'

# Ustawienia Emittera
bpy.context.object.modifiers["Fluid"].flow_settings.surface_distance = 10
bpy.context.object.modifiers["Fluid"].flow_settings.volume_density = 1

bpy.data.collections['EMITTER'].objects['Emitter'].select_set(False)

