
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
dim_s3     = 3
res_s3     = 24
gravity_s3 = vec3(0, 0, -1)
gs_s3      = vec3(19, 24, 7)
maxVel_s3  = 0

doOpen_s3          = True
boundConditions_s3 = 'xXyYzZ'
boundaryWidth_s3   = 1

using_smoke_s3        = True
using_liquid_s3       = False
using_noise_s3        = False
using_adaptTime_s3    = True
using_obstacle_s3     = True
using_guiding_s3      = False
using_fractions_s3    = False
using_invel_s3        = False
using_outflow_s3      = False
using_sndparts_s3     = False
using_speedvectors_s3 = False

# Fluid time params
timeTotal_s3    = 0
timePerFrame_s3 = 0
frameLength_s3  = 0.104167
dt0_s3          = 0.104167
cflCond_s3      = 4
timestepsMin_s3 = 1
timestepsMax_s3 = 4

# Fluid diffusion / viscosity
domainSize_s3 = 13.8598 # longest domain side in meters
viscosity_s3 = 1e-06 / (domainSize_s3*domainSize_s3) # kinematic viscosity in m^2/s

# Factor to convert blender velocities to manta velocities
toMantaUnitsFac_s3 = (1.0 / (1.0 / res_s3))
 # = dt/dx * 1/dt 
mantaMsg('Smoke variables low')
preconditioner_s3    = PcMGDynamic
using_colors_s3      = False
using_heat_s3        = True
using_fire_s3        = False
using_noise_s3       = False
vorticity_s3         = 0
buoyancy_dens_s3     = float(5) / float(13.8598)
buoyancy_heat_s3     = float(1) / float(13.8598)
dissolveSpeed_s3     = 5
using_logdissolve_s3 = True
using_dissolve_s3    = False
flameVorticity_s3    = 0.1875
burningRate_s3       = 0.75
flameSmoke_s3        = 1
ignitionTemp_s3      = 1.5
maxTemp_s3           = 3
flameSmokeColor_s3   = vec3(0.7,0.7,0.7)

######################################################################
## SOLVERS
######################################################################

mantaMsg('Solver base')
s3 = Solver(name='solver_base3', gridSize=gs_s3, dim=dim_s3)

######################################################################
## GRIDS
######################################################################

mantaMsg('Fluid alloc data')
flags_s3       = s3.create(FlagGrid)
vel_s3         = s3.create(MACGrid)
velC_s3        = s3.create(MACGrid)
x_vel_s3       = s3.create(RealGrid)
y_vel_s3       = s3.create(RealGrid)
z_vel_s3       = s3.create(RealGrid)
pressure_s3    = s3.create(RealGrid)
phiObs_s3      = s3.create(LevelsetGrid)
phiIn_s3       = s3.create(LevelsetGrid)
phiOut_s3      = s3.create(LevelsetGrid)
forces_s3      = s3.create(Vec3Grid)
x_force_s3     = s3.create(RealGrid)
y_force_s3     = s3.create(RealGrid)
z_force_s3     = s3.create(RealGrid)
obvel_s3       = None

# Keep track of important objects in dict to load them later on
fluid_data_dict_final_s3  = dict(vel=vel_s3)
fluid_data_dict_resume_s3 = dict(phiObs=phiObs_s3, phiIn=phiIn_s3, phiOut=phiOut_s3, flags=flags_s3)

mantaMsg('Smoke alloc')
shadow_s3     = s3.create(RealGrid)
emissionIn_s3 = s3.create(RealGrid)
density_s3    = s3.create(RealGrid)
densityIn_s3  = s3.create(RealGrid)
heat_s3       = None # allocated dynamically
heatIn_s3     = None
flame_s3      = None
fuel_s3       = None
react_s3      = None
fuelIn_s3     = None
reactIn_s3    = None
color_r_s3    = None
color_g_s3    = None
color_b_s3    = None
color_r_in_s3 = None
color_g_in_s3 = None
color_b_in_s3 = None

# Keep track of important objects in dict to load them later on
smoke_data_dict_final_s3 = dict(density=density_s3, shadow=shadow_s3)
smoke_data_dict_resume_s3 = dict(densityIn=densityIn_s3, emissionIn=emissionIn_s3)

# Sanity check, clear grids first
if 'heat_s3' in globals(): del heat_s3
if 'heatIn_s3' in globals(): del heatIn_s3

mantaMsg('Allocating heat')
heat_s3   = s3.create(RealGrid)
heatIn_s3 = s3.create(RealGrid)

# Add objects to dict to load them later on
if 'smoke_data_dict_final_s3' in globals():
    smoke_data_dict_final_s3.update(heat=heat_s3)
if 'smoke_data_dict_resume_s3' in globals():
    smoke_data_dict_resume_s3.update(heatIn=heatIn_s3)

mantaMsg('Allocating obstacle data')
numObs_s3     = s3.create(RealGrid)
phiObsIn_s3   = s3.create(LevelsetGrid)
obvel_s3      = s3.create(MACGrid)
obvelC_s3     = s3.create(Vec3Grid)
x_obvel_s3    = s3.create(RealGrid)
y_obvel_s3    = s3.create(RealGrid)
z_obvel_s3    = s3.create(RealGrid)

if 'fluid_data_dict_resume_s3' in globals():
    fluid_data_dict_resume_s3.update(phiObsIn=phiObsIn_s3)

######################################################################
## ADAPTIVE TIME
######################################################################

mantaMsg('Fluid adaptive time stepping')
s3.frameLength  = frameLength_s3
s3.timestepMin  = s3.frameLength / max(1, timestepsMax_s3)
s3.timestepMax  = s3.frameLength / max(1, timestepsMin_s3)
s3.cfl          = cflCond_s3
s3.timePerFrame = timePerFrame_s3
s3.timestep     = dt0_s3
s3.timeTotal    = timeTotal_s3
#mantaMsg('timestep: ' + str(s3.timestep) + ' // timPerFrame: ' + str(s3.timePerFrame) + ' // frameLength: ' + str(s3.frameLength) + ' // timeTotal: ' + str(s3.timeTotal) )

def fluid_adapt_time_step_3():
    mantaMsg('Fluid adapt time step')
    
    # time params are animatable
    s3.frameLength = frameLength_s3
    s3.cfl         = cflCond_s3
    
    # ensure that vel grid is full (remember: adaptive domain can reallocate solver)
    copyRealToVec3(sourceX=x_vel_s3, sourceY=y_vel_s3, sourceZ=z_vel_s3, target=vel_s3)
    maxVel_s3 = vel_s3.getMax() if vel_s3 else 0
    if using_adaptTime_s3:
        mantaMsg('Adapt timestep, maxvel: ' + str(maxVel_s3))
        s3.adaptTimestep(maxVel_s3)

######################################################################
## IMPORT
######################################################################

def fluid_file_import_s3(dict, path, framenr, file_format):
    try:
        framenr = fluid_cache_get_framenr_formatted_3(framenr)
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

def fluid_cache_get_framenr_formatted_3(framenr):
    return str(framenr).zfill(4) # framenr with leading zeroes

def fluid_load_data_3(path, framenr, file_format, resumable):
    mantaMsg('Fluid load data, frame ' + str(framenr))
    fluid_file_import_s3(dict=fluid_data_dict_final_s3, path=path, framenr=framenr, file_format=file_format)
    
    if resumable:
        fluid_file_import_s3(dict=fluid_data_dict_resume_s3, path=path, framenr=framenr, file_format=file_format)
        
        # When adaptive domain bake is resumed we need correct values in xyz vel grids
        copyVec3ToReal(source=vel_s3, targetX=x_vel_s3, targetY=y_vel_s3, targetZ=z_vel_s3)

def smoke_load_data_3(path, framenr, file_format, resumable):
    mantaMsg('Smoke load data')
    fluid_file_import_s3(dict=smoke_data_dict_final_s3, path=path, framenr=framenr, file_format=file_format)
    if resumable:
        fluid_file_import_s3(dict=smoke_data_dict_resume_s3, path=path, framenr=framenr, file_format=file_format)

######################################################################
## PRE/POST STEPS
######################################################################

def fluid_pre_step_3():
    mantaMsg('Fluid pre step')
    
    phiObs_s3.setConst(9999)
    phiOut_s3.setConst(9999)
    
    # Main vel grid is copied in adapt time step function
    
    # translate obvels (world space) to grid space
    if using_obstacle_s3:
        # Average out velocities from multiple obstacle objects at one cell
        x_obvel_s3.safeDivide(numObs_s3)
        y_obvel_s3.safeDivide(numObs_s3)
        z_obvel_s3.safeDivide(numObs_s3)
        
        x_obvel_s3.multConst(toMantaUnitsFac_s3)
        y_obvel_s3.multConst(toMantaUnitsFac_s3)
        z_obvel_s3.multConst(toMantaUnitsFac_s3)
        
        copyRealToVec3(sourceX=x_obvel_s3, sourceY=y_obvel_s3, sourceZ=z_obvel_s3, target=obvelC_s3)
    
    # translate invels (world space) to grid space
    if using_invel_s3:
        x_invel_s3.multConst(toMantaUnitsFac_s3)
        y_invel_s3.multConst(toMantaUnitsFac_s3)
        z_invel_s3.multConst(toMantaUnitsFac_s3)
        copyRealToVec3(sourceX=x_invel_s3, sourceY=y_invel_s3, sourceZ=z_invel_s3, target=invelC_s3)
    
    if using_guiding_s3:
        weightGuide_s3.multConst(0)
        weightGuide_s3.addConst(alpha_sg3)
        interpolateMACGrid(source=guidevel_sg3, target=velT_s3)
        velT_s3.multConst(vec3(gamma_sg3))
    
    # translate external forces (world space) to grid space
    x_force_s3.multConst(toMantaUnitsFac_s3)
    y_force_s3.multConst(toMantaUnitsFac_s3)
    z_force_s3.multConst(toMantaUnitsFac_s3)
    copyRealToVec3(sourceX=x_force_s3, sourceY=y_force_s3, sourceZ=z_force_s3, target=forces_s3)
    
    # If obstacle has velocity, i.e. is a moving obstacle, switch to dynamic preconditioner
    if using_smoke_s3 and using_obstacle_s3 and obvelC_s3.getMax() > 0:
        mantaMsg('Using dynamic preconditioner')
        preconditioner_s3 = PcMGDynamic
    else:
        mantaMsg('Using static preconditioner')
        preconditioner_s3 = PcMGStatic

def fluid_post_step_3():
    mantaMsg('Fluid post step')
    forces_s3.clear()
    x_force_s3.clear()
    y_force_s3.clear()
    z_force_s3.clear()
    
    if using_guiding_s3:
        weightGuide_s3.clear()
    if using_invel_s3:
        x_invel_s3.clear()
        y_invel_s3.clear()
        z_invel_s3.clear()
        invel_s3.clear()
        invelC_s3.clear()
    
    # Copy vel grid to reals grids (which Blender internal will in turn use for vel access)
    copyVec3ToReal(source=vel_s3, targetX=x_vel_s3, targetY=y_vel_s3, targetZ=z_vel_s3)

######################################################################
## STEPS
######################################################################

def smoke_adaptive_step_3(framenr):
    mantaMsg('Manta step, frame ' + str(framenr))
    s3.frame = framenr
    
    fluid_pre_step_3()
    
    flags_s3.initDomain(boundaryWidth=0, phiWalls=phiObs_s3, outflow=boundConditions_s3)
    
    if using_obstacle_s3:
        mantaMsg('Initializing obstacle levelset')
        phiObsIn_s3.fillHoles(maxDepth=int(res_s3), boundaryWidth=2)
        extrapolateLsSimple(phi=phiObsIn_s3, distance=int(res_s3/2), inside=True)
        extrapolateLsSimple(phi=phiObsIn_s3, distance=3, inside=False)
        phiObs_s3.join(phiObsIn_s3)
        
        # Using boundaryWidth=2 to not search beginning from walls (just a performance optimization)
        # Additional sanity check: fill holes in phiObs which can result after joining with phiObsIn
        phiObs_s3.fillHoles(maxDepth=int(res_s3), boundaryWidth=2)
        extrapolateLsSimple(phi=phiObs_s3, distance=int(res_s3/2), inside=True)
        extrapolateLsSimple(phi=phiObs_s3, distance=3, inside=False)
    
    mantaMsg('Initializing fluid levelset')
    extrapolateLsSimple(phi=phiIn_s3, distance=int(res_s3/2), inside=True)
    extrapolateLsSimple(phi=phiIn_s3, distance=3, inside=False)
    
    if using_outflow_s3:
        phiOut_s3.join(phiOutIn_s3)
    
    setObstacleFlags(flags=flags_s3, phiObs=phiObs_s3, phiOut=phiOut_s3, phiIn=phiIn_s3)
    flags_s3.fillGrid()
    
    if timePerFrame_s3 == 0: # Only apply inflow once per frame
        mantaMsg('Smoke inflow at frame: ' + str(framenr))
        applyEmission(flags=flags_s3, target=density_s3, source=densityIn_s3, emissionTexture=emissionIn_s3, type=FlagInflow|FlagOutflow)
        if using_heat_s3:
            applyEmission(flags=flags_s3, target=heat_s3, source=heatIn_s3, emissionTexture=emissionIn_s3, type=FlagInflow|FlagOutflow)
        
        if using_colors_s3:
            applyEmission(flags=flags_s3, target=color_r_s3, source=color_r_in_s3, emissionTexture=emissionIn_s3, type=FlagInflow|FlagOutflow)
            applyEmission(flags=flags_s3, target=color_g_s3, source=color_g_in_s3, emissionTexture=emissionIn_s3, type=FlagInflow|FlagOutflow)
            applyEmission(flags=flags_s3, target=color_b_s3, source=color_b_in_s3, emissionTexture=emissionIn_s3, type=FlagInflow|FlagOutflow)
        
        if using_fire_s3:
            applyEmission(flags=flags_s3, target=fuel_s3, source=fuelIn_s3, emissionTexture=emissionIn_s3, type=FlagInflow|FlagOutflow)
            applyEmission(flags=flags_s3, target=react_s3, source=reactIn_s3, emissionTexture=emissionIn_s3, type=FlagInflow|FlagOutflow)
        
    mantaMsg('Smoke step / s3.frame: ' + str(s3.frame))
    if using_fire_s3:
        process_burn_3()
    smoke_step_3()
    if using_fire_s3:
        update_flame_3()
    
    s3.step()
    
    fluid_post_step_3()

def smoke_step_3():
    mantaMsg('Smoke step low')
    
    if using_dissolve_s3:
        mantaMsg('Dissolving smoke')
        dissolveSmoke(flags=flags_s3, density=density_s3, heat=heat_s3, red=color_r_s3, green=color_g_s3, blue=color_b_s3, speed=dissolveSpeed_s3, logFalloff=using_logdissolve_s3)
    
    mantaMsg('Advecting density')
    advectSemiLagrange(flags=flags_s3, vel=vel_s3, grid=density_s3, order=2)
    
    if using_heat_s3:
        mantaMsg('Advecting heat')
        advectSemiLagrange(flags=flags_s3, vel=vel_s3, grid=heat_s3, order=2)
    
    if using_fire_s3:
        mantaMsg('Advecting fire')
        advectSemiLagrange(flags=flags_s3, vel=vel_s3, grid=fuel_s3, order=2)
        advectSemiLagrange(flags=flags_s3, vel=vel_s3, grid=react_s3, order=2)
    
    if using_colors_s3:
        mantaMsg('Advecting colors')
        advectSemiLagrange(flags=flags_s3, vel=vel_s3, grid=color_r_s3, order=2)
        advectSemiLagrange(flags=flags_s3, vel=vel_s3, grid=color_g_s3, order=2)
        advectSemiLagrange(flags=flags_s3, vel=vel_s3, grid=color_b_s3, order=2)
    
    mantaMsg('Advecting velocity')
    advectSemiLagrange(flags=flags_s3, vel=vel_s3, grid=vel_s3, order=2)
    
    if doOpen_s3 or using_outflow_s3:
        resetOutflow(flags=flags_s3, real=density_s3)
    
    mantaMsg('Vorticity')
    if using_fire_s3:
        flame_s3.copyFrom(fuel_s3) # temporarily misuse flame grid as vorticity storage
        flame_s3.multConst(flameVorticity_s3)
    vorticityConfinement(vel=vel_s3, flags=flags_s3, strengthGlobal=vorticity_s3, strengthCell=flame_s3 if using_fire_s3 else None)
    
    if using_heat_s3:
        mantaMsg('Adding heat buoyancy')
        addBuoyancy(flags=flags_s3, density=heat_s3, vel=vel_s3, gravity=gravity_s3, coefficient=buoyancy_heat_s3)
    mantaMsg('Adding buoyancy')
    addBuoyancy(flags=flags_s3, density=density_s3, vel=vel_s3, gravity=gravity_s3, coefficient=buoyancy_dens_s3)
    
    mantaMsg('Adding forces')
    addForceField(flags=flags_s3, vel=vel_s3, force=forces_s3)
    
    if using_obstacle_s3:
        mantaMsg('Extrapolating object velocity')
        # ensure velocities inside of obs object, slightly add obvels outside of obs object
        extrapolateVec3Simple(vel=obvelC_s3, phi=phiObsIn_s3, distance=int(res_s3/2), inside=True)
        extrapolateVec3Simple(vel=obvelC_s3, phi=phiObsIn_s3, distance=3, inside=False)
        resampleVec3ToMac(source=obvelC_s3, target=obvel_s3)
    
    # Cells inside obstacle should not contain any density, fire, etc.
    resetInObstacle(flags=flags_s3, density=density_s3, vel=vel_s3, heat=heat_s3, fuel=fuel_s3, flame=flame_s3, red=color_r_s3, green=color_g_s3, blue=color_b_s3)
    
    # add initial velocity
    if using_invel_s3:
        resampleVec3ToMac(source=invelC_s3, target=invel_s3)
        setInitialVelocity(flags=flags_s3, vel=vel_s3, invel=invel_s3)
    
    mantaMsg('Walls')
    setWallBcs(flags=flags_s3, vel=vel_s3, obvel=obvel_s3 if using_obstacle_s3 else None)
    
    if using_guiding_s3:
        mantaMsg('Guiding and pressure')
        PD_fluid_guiding(vel=vel_s3, velT=velT_s3, flags=flags_s3, weight=weightGuide_s3, blurRadius=beta_sg3, pressure=pressure_s3, tau=tau_sg3, sigma=sigma_sg3, theta=theta_sg3, preconditioner=preconditioner_s3, zeroPressureFixing=not doOpen_s3)
    else:
        mantaMsg('Pressure')
        solvePressure(flags=flags_s3, vel=vel_s3, pressure=pressure_s3, preconditioner=preconditioner_s3, zeroPressureFixing=not doOpen_s3) # closed domains require pressure fixing

def process_burn_3():
    mantaMsg('Process burn')
    processBurn(fuel=fuel_s3, density=density_s3, react=react_s3, red=color_r_s3, green=color_g_s3, blue=color_b_s3, heat=heat_s3, burningRate=burningRate_s3, flameSmoke=flameSmoke_s3, ignitionTemp=ignitionTemp_s3, maxTemp=maxTemp_s3, flameSmokeColor=flameSmokeColor_s3)

def update_flame_3():
    mantaMsg('Update flame')
    updateFlame(react=react_s3, flame=flame_s3)

######################################################################
## MAIN
######################################################################

# Helper function to call cache load functions
def load(frame, cache_resumable):
    fluid_load_data_3(os.path.join(cache_dir, 'data'), frame, file_format_data, cache_resumable)
    smoke_load_data_3(os.path.join(cache_dir, 'data'), frame, file_format_data, cache_resumable)
    if using_noise_s3:
        smoke_load_noise_3(os.path.join(cache_dir, 'noise'), frame, file_format_noise, cache_resumable)
    if using_guiding_s3:
        fluid_load_guiding_3(os.path.join(cache_dir, 'guiding'), frame, file_format_data)

# Helper function to call step functions
def step(frame):
    smoke_adaptive_step_3(frame)
    if using_noise_s3:
        smoke_step_noise_3(frame)

gui = None
if (GUI):
    gui=Gui()
    gui.show()
    gui.pause()

cache_resumable       = True
cache_dir             = 'D:\home\daniel\Desktop\Studia nauka\Semestr IV\SDSZ\V5\cache_fluid\'
file_format_data      = '.uni'
file_format_noise     = '.vdb'
file_format_particles = '.vdb'
file_format_mesh      = '.bobj.gz'

# Start and stop for simulation
current_frame  = 1
end_frame      = 2000

# How many frame to load from cache
from_cache_cnt = 100

loop_cnt = 0
while current_frame <= end_frame:
    
    # Load already simulated data from cache:
    if loop_cnt < from_cache_cnt:
        load(current_frame, cache_resumable)
    
    # Otherwise simulate new data
    else:
        while(s3.frame <= current_frame):
            if using_adaptTime_s3:
                fluid_adapt_time_step_3()
            step(current_frame)
    
    current_frame += 1
    loop_cnt += 1
    
    if gui:
        gui.pause()
