import bpy

bpy.ops.object.select_all(action='DESELECT')

#Dodawanie Effectora
bpy.data.collections['Effector'].objects['class'].select_set(True)
bpy.ops.object.modifier_add(type='FLUID')
bpy.context.object.modifiers["Fluid"].fluid_type = 'EFFECTOR'
bpy.context.object.modifiers["Fluid"].effector_settings.surface_distance = 0.001
bpy.data.collections['Effector'].objects['class'].select_set(False)