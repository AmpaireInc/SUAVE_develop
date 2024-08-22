## @ingroup Methods-Missions-Segments-Climb
# Constant_Throttle_Constant_Speed.py
# 
# Created:  Jul 2024, J. Jacobs
# Modified: 
# Derived from Constant_Throttle_Constant Speed

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np
from SUAVE.Methods.Missions.Segments.Common.Aerodynamics import update_atmosphere
from SUAVE.Core import Units

# ----------------------------------------------------------------------
#  Unpack Unknowns
# ----------------------------------------------------------------------

## @ingroup Methods-Missions-Segments-Climb
def unpack_body_angle(segment):
    """Unpacks and sets the proper value for body angle

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
    state.unknowns.body_angle                      [Radians]

    Outputs:
    state.conditions.frames.body.inertial_rotation [Radians]

    Properties Used:
    N/A
    """          

    # unpack unknowns
    theta      = segment.state.unknowns.body_angle

    # apply unknowns
    segment.state.conditions.frames.body.inertial_rotations[:,1] = theta[:,0]      


# ----------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------

## @ingroup Methods-Missions-Segments-Climb
def initialize_conditions(segment):
    """Sets the specified conditions which are given for the segment type.
    
    Assumptions:
    Constant throttle estting, with a constant true airspeed

    Source:
    N/A

    Inputs:
    segment.calibrated_air_speed                        [meters/second]
    segment.throttle                                    [Unitless]
    segment.altitude_start                              [meters]
    segment.altitude_end                                [meters]
    segment.state.numerics.dimensionless.control_points [Unitless]
    conditions.freestream.density                       [kilograms/meter^3]

    Outputs:
    conditions.frames.inertial.velocity_vector  [meters/second]
    conditions.propulsion.throttle              [Unitless]

    Properties Used:
    N/A
    """         
    
    # unpack
    throttle   = segment.throttle
    cas        = segment.calibrated_air_speed   
    alt0       = segment.altitude_start 
    altf       = segment.altitude_end
    conditions = segment.state.conditions  
    t_nondim   = segment.state.numerics.dimensionless.control_points

    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]


    """
    #Update atmosphere for starting altitude and calculate initial airspeed
    conditions.freestream.altitude[0,0] = alt0
    """

    # discretize on altitude
    alt = t_nondim * (altf-alt0) + alt0
    
    # pack conditions
    conditions.freestream.altitude[:,0] = alt[:,0] # positive altitude in this context
    update_atmosphere(segment) # set initial altitude parameters
    air_speed = air_speed_from_cas(segment, cas)

    # pack conditions  
    conditions.propulsion.throttle[:,0] = throttle
    conditions.frames.inertial.velocity_vector[:,0] = air_speed[:,0] # start up value
    

    return

## @ingroup Methods-Missions-Segments-Climb
def update_differentials_altitude(segment):
    """On each iteration creates the differentials and integration funcitons from knowns about the problem. Sets the time at each point. Must return in dimensional time, with t[0] = 0
    
    Assumptions:
    Constant throttle setting, with a constant true airspeed.

    Source:
    N/A

    Inputs:
    segment.climb_angle                         [radians]
    state.conditions.frames.inertial.velocity_vector [meter/second]
    segment.altitude_start                      [meters]
    segment.altitude_end                        [meters]

    Outputs:
    state.conditions.frames.inertial.time       [seconds]
    conditions.frames.inertial.position_vector  [meters]
    conditions.freestream.altitude              [meters]

    Properties Used:
    N/A
    """   

    # unpack
    t = segment.state.numerics.dimensionless.control_points
    D = segment.state.numerics.dimensionless.differentiate
    I = segment.state.numerics.dimensionless.integrate

    
    # Unpack segment initials
    alt0       = segment.altitude_start 
    altf       = segment.altitude_end    
    conditions = segment.state.conditions  
    v          = segment.state.conditions.frames.inertial.velocity_vector
    
    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]    
    
    # get overall time step
    vz = -v[:,2,None] # Inertial velocity is z down
    dz = altf- alt0    
    dt = dz / np.dot(I[-1,:],vz)[-1] # maintain column array
    
    # Integrate vz to get altitudes
    alt = alt0 + np.dot(I*dt,vz)

    # rescale operators
    t = t * dt

    # pack
    t_initial = segment.state.conditions.frames.inertial.time[0,0]
    segment.state.conditions.frames.inertial.time[:,0] = t_initial + t[:,0]
    conditions.frames.inertial.position_vector[:,2] = -alt[:,0] # z points down
    conditions.freestream.altitude[:,0]             =  alt[:,0] # positive altitude in this context    

    return

# ----------------------------------------------------------------------
#  Update Velocity Vector from Wind Angle
# ----------------------------------------------------------------------

## @ingroup Methods-Missions-Segments-Climb
def update_velocity_vector_from_wind_angle(segment):
    
    # unpack
    conditions = segment.state.conditions 
    cas        = segment.calibrated_air_speed
    alpha      = segment.state.unknowns.wind_angle[:,0][:,None]
    theta      = segment.state.unknowns.body_angle[:,0][:,None]
    
    
    # Flight path angle
    gamma = theta-alpha

    # process
    v_mag = air_speed_from_cas(segment, cas)

    v_x =  v_mag * np.cos(gamma)
    v_z = -v_mag * np.sin(gamma) # z points down

    # pack
    conditions.frames.inertial.velocity_vector[:,0] = v_x[:,0]
    conditions.frames.inertial.velocity_vector[:,2] = v_z[:,0]
    #conditions.freestream.velocity[:,0]             = v_mag

    #print("Vcas: ", cas) #debug
    #print("Vtrue: ", v_mag[0,0]) #debug

    return #conditions




def air_speed_from_cas(segment, cas):

    #Unpack
    conditions = segment.state.conditions

    # determine airspeed from calibrated airspeed
    #update_atmosphere(segment) # get density for airspeed
    density  = conditions.freestream.density[:,0]  
    pressure = conditions.freestream.pressure[:,0] 

    MSL_data  = segment.analyses.atmosphere.compute_values(0.0,0.0)
    pressure0 = MSL_data.pressure[0]

    kcas  = cas / Units.knots
    delta = pressure / pressure0 

    mach = 2.236*((((1+4.575e-7*kcas**2)**3.5-1)/delta + 1)**0.2857 - 1)**0.5

    qc  = pressure * ((1+0.2*mach**2)**3.5 - 1)
    eas = cas * (pressure/pressure0)**0.5*(((qc/pressure+1)**0.286-1)/((qc/pressure0+1)**0.286-1))**0.5
    
    air_speed = eas/np.sqrt(density/MSL_data.density[0])

    air_speed = air_speed.reshape(-1,1) #convert to proper 2D array format

    return air_speed