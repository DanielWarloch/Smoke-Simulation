
######################################################################
## LIBRARIES
######################################################################
from manta import *
import os.path, shutil, math, sys, gc, multiprocessing, platform, time

withMPBake = False # Bake files asynchronously
withMPSave = True # Save files asynchronously
isWindows = platform.system() != 'Darwin' and platform.system() != 'Linux'
# TODO (sebbas): Use this to simulate Windows multiprocessing (has default mode spawn)
#try:
#    multiprocessing.set_start_method('spawn')
#except:
#    pass

bpy = sys.modules.get('bpy')
if bpy is not None:
    sys.executable = bpy.app.binary_path_python

######################################################################
## VARIABLES
######################################################################

mantaMsg('Fluid variables')
dim_s124     = 3
res_s124     = 24
gravity_s124 = vec3(0, 0, -1)
gs_s124      = vec3(19, 24, 7)
maxVel_s124  = 0

doOpen_s124          = True
boundConditions_s124 = 'xXyYzZ'
boundaryWidth_s124   = 1

using_smoke_s124        = True
using_liquid_s124       = False
using_noise_s124        = False
using_adaptTime_s124    = True
using_obstacle_s124     = True
using_guiding_s124      = False
using_fractions_s124    = False
using_invel_s124        = False
using_outflow_s124      = False
using_sndparts_s124     = False
using_speedvectors_s124 = False

# Fluid time params
timeTotal_s124    = 0
timePerFrame_s124 = 0
frameLength_s124  = 0.104167
dt0_s124          = 0.104167
cflCond_s124      = 4
timestepsMin_s124 = 1
timestepsMax_s124 = 4

# Fluid diffusion / viscosity
domainSize_s124 = 13.8598 # longest domain side in meters
viscosity_s124 = 1e-06 / (domainSize_s124*domainSize_s124) # kinematic viscosity in m^2/s

# Factor to convert blender velocities to manta velocities
toMantaUnitsFac_s124 = (1.0 / (1.0 / res_s124))
 # = dt/dx * 1/dt 
mantaMsg('Smoke variables low')
preconditioner_s124    = PcMGDynamic
using_colors_s124      = False
using_heat_s124        = True
using_fire_s124        = False
using_noise_s124       = False
vorticity_s124         = 0
buoyancy_dens_s124     = float(5) / float(13.8598)
buoyancy_heat_s124     = float(1) / float(13.8598)
dissolveSpeed_s124     = 5
using_logdissolve_s124 = True
using_dissolve_s124    = False
flameVorticity_s124    = 0.1875
burningRate_s124       = 0.75
flameSmoke_s124        = 1
ignitionTemp_s124      = 1.5
maxTemp_s124           = 3
flameSmokeColor_s124   = vec3(0.7,0.7,0.7)

######################################################################
## SOLVERS
######################################################################

mantaMsg('Solver base')
s124 = Solver(name='solver_base124', gridSize=gs_s124, dim=dim_s124)

######################################################################
## GRIDS
######################################################################

mantaMsg('Fluid alloc data')
flags_s124       = s124.create(FlagGrid)
vel_s124         = s124.create(MACGrid)
velC_s124        = s124.create(MACGrid)
x_vel_s124       = s124.create(RealGrid)
y_vel_s124       = s124.create(RealGrid)
z_vel_s124       = s124.create(RealGrid)
pressure_s124    = s124.create(RealGrid)
phiObs_s124      = s124.create(LevelsetGrid)
phiIn_s124       = s124.create(LevelsetGrid)
phiOut_s124      = s124.create(LevelsetGrid)
forces_s124      = s124.create(Vec3Grid)
x_force_s124     = s124.create(RealGrid)
y_force_s124     = s124.create(RealGrid)
z_force_s124     = s124.create(RealGrid)
obvel_s124       = None

# Keep track of important objects in dict to load them later on
fluid_data_dict_final_s124  = dict(vel=vel_s124)
fluid_data_dict_resume_s124 = dict(phiObs=phiObs_s124, phiIn=phiIn_s124, phiOut=phiOut_s124, flags=flags_s124)

mantaMsg('Smoke alloc')
shadow_s124     = s124.create(RealGrid)
emissionIn_s124 = s124.create(RealGrid)
density_s124    = s124.create(RealGrid)
densityIn_s124  = s124.create(RealGrid)
heat_s124       = None # allocated dynamically
heatIn_s124     = None
flame_s124      = None
fuel_s124       = None
react_s124      = None
fuelIn_s124     = None
reactIn_s124    = None
color_r_s124    = None
color_g_s124    = None
color_b_s124    = None
color_r_in_s124 = None
color_g_in_s124 = None
color_b_in_s124 = None

# Keep track of important objects in dict to load them later on
smoke_data_dict_final_s124 = dict(density=density_s124, shadow=shadow_s124)
smoke_data_dict_resume_s124 = dict(densityIn=densityIn_s124, emissionIn=emissionIn_s124)

# Sanity check, clear grids first
if 'heat_s124' in globals(): del heat_s124
if 'heatIn_s124' in globals(): del heatIn_s124

mantaMsg('Allocating heat')
heat_s124   = s124.create(RealGrid)
heatIn_s124 = s124.create(RealGrid)

# Add objects to dict to load them later on
if 'smoke_data_dict_final_s124' in globals():
    smoke_data_dict_final_s124.update(heat=heat_s124)
if 'smoke_data_dict_resume_s124' in globals():
    smoke_data_dict_resume_s124.update(heatIn=heatIn_s124)

mantaMsg('Allocating obstacle data')
numObs_s124     = s124.create(RealGrid)
phiObsIn_s124   = s124.create(LevelsetGrid)
obvel_s124      = s124.create(MACGrid)
obvelC_s124     = s124.create(Vec3Grid)
x_obvel_s124    = s124.create(RealGrid)
y_obvel_s124    = s124.create(RealGrid)
z_obvel_s124    = s124.create(RealGrid)

if 'fluid_data_dict_resume_s124' in globals():
    fluid_data_dict_resume_s124.update(phiObsIn=phiObsIn_s124)

######################################################################
## ADAPTIVE TIME
######################################################################

mantaMsg('Fluid adaptive time stepping')
s124.frameLength  = frameLength_s124
s124.timestepMin  = s124.frameLength / max(1, timestepsMax_s124)
s124.timestepMax  = s124.frameLength / max(1, timestepsMin_s124)
s124.cfl          = cflCond_s124
s124.timePerFrame = timePerFrame_s124
s124.timestep     = dt0_s124
s124.timeTotal    = timeTotal_s124
#mantaMsg('timestep: ' + str(s124.timestep) + ' // timPerFrame: ' + str(s124.timePerFrame) + ' // frameLength: ' + str(s124.frameLength) + ' // timeTotal: ' + str(s124.timeTotal) )

def fluid_adapt_time_step_124():
    mantaMsg('Fluid adapt time step')
    
    # time params are animatable
    s124.frameLength = frameLength_s124
    s124.cfl         = cflCond_s124
    
    # ensure that vel grid is full (remember: adaptive domain can reallocate solver)
    copyRealToVec3(sourceX=x_vel_s124, sourceY=y_vel_s124, sourceZ=z_vel_s124, target=vel_s124)
    maxVel_s124 = vel_s124.getMax() if vel_s124 else 0
    if using_adaptTime_s124:
        mantaMsg('Adapt timestep, maxvel: ' + str(maxVel_s124))
        s124.adaptTimestep(maxVel_s124)

######################################################################
## IMPORT
######################################################################

def fluid_file_import_s124(dict, path, framenr, file_format):
    try:
        framenr = fluid_cache_get_framenr_formatted_124(framenr)
        for name, object in dict.items():
            file = os.path.join(path, name + '_' + framenr + file_format)
            if os.path.isfile(file):
                object.load(file)
            else:
                mantaMsg('Could not load file ' + str(file))
    except:
        mantaMsg('exception found')
        #mantaMsg(str(e))
        pass # Just skip file load errors for now

def fluid_cache_get_framenr_formatted_124(framenr):
    return str(framenr).zfill(4) # framenr with leading zeroes

def fluid_load_data_124(path, framenr, file_format, resumable):
    mantaMsg('Fluid load data, frame ' + str(framenr))
    fluid_file_import_s124(dict=fluid_data_dict_final_s124, path=path, framenr=framenr, file_format=file_format)
    
    if resumable:
        fluid_file_import_s124(dict=fluid_data_dict_resume_s124, path=path, framenr=framenr, file_format=file_format)
        
        # When adaptive domain bake is resumed we need correct values in xyz vel grids
        copyVec3ToReal(source=vel_s124, targetX=x_vel_s124, targetY=y_vel_s124, targetZ=z_vel_s124)

def smoke_load_data_124(path, framenr, file_format, resumable):
    mantaMsg('Smoke load data')
    fluid_file_import_s124(dict=smoke_data_dict_final_s124, path=path, framenr=framenr, file_format=file_format)
    if resumable:
        fluid_file_import_s124(dict=smoke_data_dict_resume_s124, path=path, framenr=framenr, file_format=file_format)

######################################################################
## PRE/POST STEPS
######################################################################

def fluid_pre_step_124():
    mantaMsg('Fluid pre step')
    
    phiObs_s124.setConst(9999)
    phiOut_s124.setConst(9999)
    
    # Main vel grid is copied in adapt time step function
    
    # translate obvels (world space) to grid space
    if using_obstacle_s124:
        # Average out velocities from multiple obstacle objects at one cell
        x_obvel_s124.safeDivide(numObs_s124)
        y_obvel_s124.safeDivide(numObs_s124)
        z_obvel_s124.safeDivide(numObs_s124)
        
        x_obvel_s124.multConst(toMantaUnitsFac_s124)
        y_obvel_s124.multConst(toMantaUnitsFac_s124)
        z_obvel_s124.multConst(toMantaUnitsFac_s124)
        
        copyRealToVec3(sourceX=x_obvel_s124, sourceY=y_obvel_s124, sourceZ=z_obvel_s124, target=obvelC_s124)
    
    # translate invels (world space) to grid space
    if using_invel_s124:
        x_invel_s124.multConst(toMantaUnitsFac_s124)
        y_invel_s124.multConst(toMantaUnitsFac_s124)
        z_invel_s124.multConst(toMantaUnitsFac_s124)
        copyRealToVec3(sourceX=x_invel_s124, sourceY=y_invel_s124, sourceZ=z_invel_s124, target=invelC_s124)
    
    if using_guiding_s124:
        weightGuide_s124.multConst(0)
        weightGuide_s124.addConst(alpha_sg124)
        interpolateMACGrid(source=guidevel_sg124, target=velT_s124)
        velT_s124.multConst(vec3(gamma_sg124))
    
    # translate external forces (world space) to grid space
    x_force_s124.multConst(toMantaUnitsFac_s124)
    y_force_s124.multConst(toMantaUnitsFac_s124)
    z_force_s124.multConst(toMantaUnitsFac_s124)
    copyRealToVec3(sourceX=x_force_s124, sourceY=y_force_s124, sourceZ=z_force_s124, target=forces_s124)
    
    # If obstacle has velocity, i.e. is a moving obstacle, switch to dynamic preconditioner
    if using_smoke_s124 and using_obstacle_s124 and obvelC_s124.getMax() > 0:
        mantaMsg('Using dynamic preconditioner')
        preconditioner_s124 = PcMGDynamic
    else:
        mantaMsg('Using static preconditioner')
        preconditioner_s124 = PcMGStatic

def fluid_post_step_124():
    mantaMsg('Fluid post step')
    forces_s124.clear()
    x_force_s124.clear()
    y_force_s124.clear()
    z_force_s124.clear()
    
    if using_guiding_s124:
        weightGuide_s124.clear()
    if using_invel_s124:
        x_invel_s124.clear()
        y_invel_s124.clear()
        z_invel_s124.clear()
        invel_s124.clear()
        invelC_s124.clear()
    
    # Copy vel grid to reals grids (which Blender internal will in turn use for vel access)
    copyVec3ToReal(source=vel_s124, targetX=x_vel_s124, targetY=y_vel_s124, targetZ=z_vel_s124)

######################################################################
## STEPS
######################################################################

def smoke_adaptive_step_124(framenr):
    mantaMsg('Manta step, frame ' + str(framenr))
    s124.frame = framenr
    
    fluid_pre_step_124()
    
    flags_s124.initDomain(boundaryWidth=0, phiWalls=phiObs_s124, outflow=boundConditions_s124)
    
    if using_obstacle_s124:
        mantaMsg('Initializing obstacle levelset')
        phiObsIn_s124.fillHoles(maxDepth=int(res_s124), boundaryWidth=2)
        extrapolateLsSimple(phi=phiObsIn_s124, distance=int(res_s124/2), inside=True)
        extrapolateLsSimple(phi=phiObsIn_s124, distance=3, inside=False)
        phiObs_s124.join(phiObsIn_s124)
        
        # Using boundaryWidth=2 to not search beginning from walls (just a performance optimization)
        # Additional sanity check: fill holes in phiObs which can result after joining with phiObsIn
        phiObs_s124.fillHoles(maxDepth=int(res_s124), boundaryWidth=2)
        extrapolateLsSimple(phi=phiObs_s124, distance=int(res_s124/2), inside=True)
        extrapolateLsSimple(phi=phiObs_s124, distance=3, inside=False)
    
    mantaMsg('Initializing fluid levelset')
    extrapolateLsSimple(phi=phiIn_s124, distance=int(res_s124/2), inside=True)
    extrapolateLsSimple(phi=phiIn_s124, distance=3, inside=False)
    
    if using_outflow_s124:
        phiOut_s124.join(phiOutIn_s124)
    
    setObstacleFlags(flags=flags_s124, phiObs=phiObs_s124, phiOut=phiOut_s124, phiIn=phiIn_s124)
    flags_s124.fillGrid()
    
    if timePerFrame_s124 == 0: # Only apply inflow once per frame
        mantaMsg('Smoke inflow at frame: ' + str(framenr))
        applyEmission(flags=flags_s124, target=density_s124, source=densityIn_s124, emissionTexture=emissionIn_s124, type=FlagInflow|FlagOutflow)
        if using_heat_s124:
            applyEmission(flags=flags_s124, target=heat_s124, source=heatIn_s124, emissionTexture=emissionIn_s124, type=FlagInflow|FlagOutflow)
        
        if using_colors_s124:
            applyEmission(flags=flags_s124, target=color_r_s124, source=color_r_in_s124, emissionTexture=emissionIn_s124, type=FlagInflow|FlagOutflow)
            applyEmission(flags=flags_s124, target=color_g_s124, source=color_g_in_s124, emissionTexture=emissionIn_s124, type=FlagInflow|FlagOutflow)
            applyEmission(flags=flags_s124, target=color_b_s124, source=color_b_in_s124, emissionTexture=emissionIn_s124, type=FlagInflow|FlagOutflow)
        
        if using_fire_s124:
            applyEmission(flags=flags_s124, target=fuel_s124, source=fuelIn_s124, emissionTexture=emissionIn_s124, type=FlagInflow|FlagOutflow)
            applyEmission(flags=flags_s124, target=react_s124, source=reactIn_s124, emissionTexture=emissionIn_s124, type=FlagInflow|FlagOutflow)
        
    mantaMsg('Smoke step / s124.frame: ' + str(s124.frame))
    if using_fire_s124:
        process_burn_124()
    smoke_step_124()
    if using_fire_s124:
        update_flame_124()
    
    s124.step()
    
    fluid_post_step_124()

def smoke_step_124():
    mantaMsg('Smoke step low')
    
    if using_dissolve_s124:
        mantaMsg('Dissolving smoke')
        dissolveSmoke(flags=flags_s124, density=density_s124, heat=heat_s124, red=color_r_s124, green=color_g_s124, blue=color_b_s124, speed=dissolveSpeed_s124, logFalloff=using_logdissolve_s124)
    
    mantaMsg('Advecting density')
    advectSemiLagrange(flags=flags_s124, vel=vel_s124, grid=density_s124, order=2)
    
    if using_heat_s124:
        mantaMsg('Advecting heat')
        advectSemiLagrange(flags=flags_s124, vel=vel_s124, grid=heat_s124, order=2)
    
    if using_fire_s124:
        mantaMsg('Advecting fire')
        advectSemiLagrange(flags=flags_s124, vel=vel_s124, grid=fuel_s124, order=2)
        advectSemiLagrange(flags=flags_s124, vel=vel_s124, grid=react_s124, order=2)
    
    if using_colors_s124:
        mantaMsg('Advecting colors')
        advectSemiLagrange(flags=flags_s124, vel=vel_s124, grid=color_r_s124, order=2)
        advectSemiLagrange(flags=flags_s124, vel=vel_s124, grid=color_g_s124, order=2)
        advectSemiLagrange(flags=flags_s124, vel=vel_s124, grid=color_b_s124, order=2)
    
    mantaMsg('Advecting velocity')
    advectSemiLagrange(flags=flags_s124, vel=vel_s124, grid=vel_s124, order=2)
    
    if doOpen_s124 or using_outflow_s124:
        resetOutflow(flags=flags_s124, real=density_s124)
    
    mantaMsg('Vorticity')
    if using_fire_s124:
        flame_s124.copyFrom(fuel_s124) # temporarily misuse flame grid as vorticity storage
        flame_s124.multConst(flameVorticity_s124)
    vorticityConfinement(vel=vel_s124, flags=flags_s124, strengthGlobal=vorticity_s124, strengthCell=flame_s124 if using_fire_s124 else None)
    
    if using_heat_s124:
        mantaMsg('Adding heat buoyancy')
        addBuoyancy(flags=flags_s124, density=heat_s124, vel=vel_s124, gravity=gravity_s124, coefficient=buoyancy_heat_s124)
    mantaMsg('Adding buoyancy')
    addBuoyancy(flags=flags_s124, density=density_s124, vel=vel_s124, gravity=gravity_s124, coefficient=buoyancy_dens_s124)
    
    mantaMsg('Adding forces')
    addForceField(flags=flags_s124, vel=vel_s124, force=forces_s124)
    
    if using_obstacle_s124:
        mantaMsg('Extrapolating object velocity')
        # ensure velocities inside of obs object, slightly add obvels outside of obs object
        extrapolateVec3Simple(vel=obvelC_s124, phi=phiObsIn_s124, distance=int(res_s124/2), inside=True)
        extrapolateVec3Simple(vel=obvelC_s124, phi=phiObsIn_s124, distance=3, inside=False)
        resampleVec3ToMac(source=obvelC_s124, target=obvel_s124)
    
    # Cells inside obstacle should not contain any density, fire, etc.
    resetInObstacle(flags=flags_s124, density=density_s124, vel=vel_s124, heat=heat_s124, fuel=fuel_s124, flame=flame_s124, red=color_r_s124, green=color_g_s124, blue=color_b_s124)
    
    # add initial velocity
    if using_invel_s124:
        resampleVec3ToMac(source=invelC_s124, target=invel_s124)
        setInitialVelocity(flags=flags_s124, vel=vel_s124, invel=invel_s124)
    
    mantaMsg('Walls')
    setWallBcs(flags=flags_s124, vel=vel_s124, obvel=obvel_s124 if using_obstacle_s124 else None)
    
    if using_guiding_s124:
        mantaMsg('Guiding and pressure')
        PD_fluid_guiding(vel=vel_s124, velT=velT_s124, flags=flags_s124, weight=weightGuide_s124, blurRadius=beta_sg124, pressure=pressure_s124, tau=tau_sg124, sigma=sigma_sg124, theta=theta_sg124, preconditioner=preconditioner_s124, zeroPressureFixing=not doOpen_s124)
    else:
        mantaMsg('Pressure')
        solvePressure(flags=flags_s124, vel=vel_s124, pressure=pressure_s124, preconditioner=preconditioner_s124, zeroPressureFixing=not doOpen_s124) # closed domains require pressure fixing

def process_burn_124():
    mantaMsg('Process burn')
    processBurn(fuel=fuel_s124, density=density_s124, react=react_s124, red=color_r_s124, green=color_g_s124, blue=color_b_s124, heat=heat_s124, burningRate=burningRate_s124, flameSmoke=flameSmoke_s124, ignitionTemp=ignitionTemp_s124, maxTemp=maxTemp_s124, flameSmokeColor=flameSmokeColor_s124)

def update_flame_124():
    mantaMsg('Update flame')
    updateFlame(react=react_s124, flame=flame_s124)

######################################################################
## MAIN
######################################################################

# Helper function to call cache load functions
def load(frame, cache_resumable):
    fluid_load_data_124(os.path.join(cache_dir, 'data'), frame, file_format_data, cache_resumable)
    smoke_load_data_124(os.path.join(cache_dir, 'data'), frame, file_format_data, cache_resumable)
    if using_noise_s124:
        smoke_load_noise_124(os.path.join(cache_dir, 'noise'), frame, file_format_noise, cache_resumable)
    if using_guiding_s124:
        fluid_load_guiding_124(os.path.join(cache_dir, 'guiding'), frame, file_format_data)

# Helper function to call step functions
def step(frame):
    smoke_adaptive_step_124(frame)
    if using_noise_s124:
        smoke_step_noise_124(frame)

gui = None
if (GUI):
    gui=Gui()
    gui.show()
    gui.pause()

cache_resumable       = True
cache_dir             = 'D:\home\daniel\Desktop\Studia nauka\Semestr IV\SDSZ\V4\cache_fluid\'
file_format_data      = '.uni'
file_format_noise     = '.vdb'
file_format_particles = '.vdb'
file_format_mesh      = '.bobj.gz'

# Start and stop for simulation
current_frame  = 1
end_frame      = 2500

# How many frame to load from cache
from_cache_cnt = 100

loop_cnt = 0
while current_frame <= end_frame:
    
    # Load already simulated data from cache:
    if loop_cnt < from_cache_cnt:
        load(current_frame, cache_resumable)
    
    # Otherwise simulate new data
    else:
        while(s124.frame <= current_frame):
            if using_adaptTime_s124:
                fluid_adapt_time_step_124()
            step(current_frame)
    
    current_frame += 1
    loop_cnt += 1
    
    if gui:
        gui.pause()
