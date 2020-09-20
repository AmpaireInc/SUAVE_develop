## @ingroup Methods-Missions-Segments-Climb
# Constant_Throttle_Constant_EAS.py
# 
# Created:  Jul 2014, SUAVE Team
# Modified: Jan 2016, E. Botero

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np
import SUAVE
from SUAVE.Methods.Utilities.Chebyshev  import chebyshev_data

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
    segment.air_speed                                   [meters/second]
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
    throttle    = segment.throttle
    eas         = segment.equivalent_air_speed
    alt0        = segment.altitude_start 
    altf        = segment.altitude_end
    t_nondim    = segment.state.numerics.dimensionless.control_points
    conditions  = segment.state.conditions  

    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]

    # pack conditions  
    # Update freestream to get speed of sound
    SUAVE.Methods.Missions.Segments.Common.Aerodynamics.update_atmosphere(segment)
    density   = conditions.freestream.density[:,0]   
    MSL_data  = segment.analyses.atmosphere.compute_values(0.0,segment.temperature_deviation)
    air_speed = eas/np.sqrt(density/MSL_data.density[0])[:,None]  
    
    conditions.propulsion.throttle[:,0] = throttle
    conditions.frames.inertial.velocity_vector[:,0] = air_speed[:,0] # start up value

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
    x = segment.state.numerics.dimensionless.control_points
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
    #dt = segment.state.unknowns.time
    #dt = dz / np.dot(I[-1,:],vz)[-1] # maintain column array
    
    alt = x * dz + alt0
    
    ## Integrate vz to get altitudes
    #alt = alt0 + np.dot(I*dt,vz)

    # rescale operators
    t = (I @ (1/vz)) * dz

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
    eas        = segment.equivalent_air_speed
    SUAVE.Methods.Missions.Segments.Common.Aerodynamics.update_atmosphere(segment)
    density   = conditions.freestream.density[:,0]   
    MSL_data  = segment.analyses.atmosphere.compute_values(0.0,segment.temperature_deviation)
    air_speed = eas/np.sqrt(density/MSL_data.density[0])[:,None]
    
    # process velocity vector
    v_mag = air_speed
    #v_mag      = np.linalg.norm(segment.state.conditions.frames.inertial.velocity_vector,axis=1) 
    alpha      = segment.state.unknowns.wind_angle[:,0][:,None]
    theta      = segment.state.unknowns.body_angle[:,0][:,None]
    
    # Flight path angle
    gamma = theta-alpha

    # process
    v_x =  v_mag * np.cos(gamma)
    v_z = -v_mag * np.sin(gamma) # z points down

    # pack
    conditions.frames.inertial.velocity_vector[:,0] = v_x[:,0]
    conditions.frames.inertial.velocity_vector[:,2] = v_z[:,0]

    return conditions

def residual_total_forces(segment):
    """Takes the summation of forces and makes a residual from the accelerations.

    Assumptions:
    No higher order terms.

    Source:
    N/A

    Inputs:
    segment.state.conditions.frames.inertial.total_force_vector   [Newtons]
    segment.state.conditions.frames.inertial.acceleration_vector  [meter/second^2]
    segment.state.conditions.weights.total_mass                   [kilogram]

    Outputs:
    segment.state.residuals.forces                                [Unitless]

    Properties Used:
    N/A
    """        
    
    FT = segment.state.conditions.frames.inertial.total_force_vector
    a  = segment.state.conditions.frames.inertial.acceleration_vector
    m  = segment.state.conditions.weights.total_mass    
    final_alt = -segment.state.conditions.frames.inertial.position_vector[-1,2]
    time = segment.state.conditions.frames.inertial.time
    
    segment.state.residuals.forces[:,0] = FT[:,0]/m[:,0] - a[:,0]
    segment.state.residuals.forces[:,1] = FT[:,2]/m[:,0] - a[:,2]       
    #segment.state.residuals.final_altitude = (final_alt - segment.altitude_end)

    return

## @ingroup Methods-Missions-Segments-Climb
def update_differentials_time(segment):
    """ Scales the differential operators (integrate and differentiate) based on mission time
    
        Assumptions:
        N/A
        
        Inputs:
            numerics.dimensionless:           
                control_points                    [array]
                differentiate                     [array]
                integrate                         [array]
            state.conditions.frames.inertial.time [seconds]
            
        Outputs:
            numerics.time:           
                control_points        [array]
                differentiate         [array]
                integrate             [array]

        Properties Used:
        N/A
                                
    """     
    
    # unpack
    numerics = segment.state.numerics
    x = numerics.dimensionless.control_points
    D = numerics.dimensionless.differentiate
    I = numerics.dimensionless.integrate
    
    # rescale time
    time = segment.state.conditions.frames.inertial.time[:,0]
    t_initial = time[0]
    t    = time - t_initial
    T    = time[-1] - time[0]
    
    t, D, I = chebyshev_data(N=len(time), x=t/t[-1])
    
    # rescale operators
    D = D / T
    I = I * T
    
    # pack
    numerics.time.control_points = t
    numerics.time.differentiate  = D
    numerics.time.integrate      = I

    return