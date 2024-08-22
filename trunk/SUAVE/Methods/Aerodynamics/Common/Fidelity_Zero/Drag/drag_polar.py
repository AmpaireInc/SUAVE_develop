## @ingroup Methods-Aerodynamics-Fidelity_Zero-Drag
# total_aircraft.py
# 
# Created:  Jul 2024, J. Jacobs
# Modified: 

# ----------------------------------------------------------------------
#  Total Aircraft Drag from Given Drag Polar
# ----------------------------------------------------------------------


#temp import
import sys
import numpy as np
from SUAVE.Core import Data

## @ingroup Methods-Aerodynamics-Fidelity_Zero-Drag
def drag_polar(state,settings,geometry):
    """ This computes the total drag of an aircraft and stores
    that data in the conditions structure.

    Assumptions:
    None

    Source:
    N/A

    Inputs:
    settings.
      drag_coefficient_increment                   [Unitless]
      lift_to_drag_adjustment                      [Unitless] (.1 is 10% increase in L/D)
    state.conditions.aerodynamics.drag_breakdown.
      trim_corrected_drag                          [Unitless]
      spoiler_drag                                 [Unitless]

    Outputs:
    aircraft_total_drag (drag coefficient)         [Unitless]

    Properties Used:
    N/A
    """    

    # Unpack inputs
    conditions    = state.conditions
    configuration = settings
    polar_main = geometry.drag_polar_main 
    polar_sub = geometry.drag_polar_sub
    CL = conditions.aerodynamics.lift_coefficient
    rho = conditions.freestream.density#[:,0,None]
    thrust_vector = conditions.frames.body.thrust_force_vector#[:,0]
    velocity = conditions.freestream.velocity
    wing_area = geometry.reference_area

    thrust = np.transpose(np.atleast_2d(np.linalg.norm(thrust_vector, axis=1)))
    #print(thrust, rho, velocity, wing_area)
    
    
    
    #Calculate Thrust Drag Coefficient
    TC_drag = thrust / (.5 * rho * velocity**2 * wing_area)
    #print(TC_drag[0,0])

    drag_coefficient_increment = configuration.drag_coefficient_increment
    drag_polar_drag        = calculate_from_polar(polar_main, polar_sub, CL, TC_drag)
    spoiler_drag               = conditions.aerodynamics.drag_breakdown.spoiler_drag 

    aircraft_total_drag = 0.0
    # Add drag_coefficient_increment
    aircraft_total_drag += drag_polar_drag + drag_coefficient_increment + spoiler_drag
    conditions.aerodynamics.drag_breakdown.drag_coefficient_increment = drag_coefficient_increment
    
    # Add L/D correction
    aircraft_total_drag = aircraft_total_drag/(1.+configuration.lift_to_drag_adjustment) 

    # Store to results
    conditions.aerodynamics.drag_breakdown.total = aircraft_total_drag
    conditions.aerodynamics.drag_coefficient     = aircraft_total_drag

    #print(drag_polar_drag)

    #sys.exit( 0 )

    return aircraft_total_drag


###New Code

    
def calculate_from_polar(polar_main, polar_sub, lift_coefficient, thrust_coefficient):
    
    ###Unpack Inputs
    CL = lift_coefficient
    CD0 = polar_main.CD0
    TC = thrust_coefficient
    drag_polar_slope = polar_main.slope # CD/CL^2
    A = polar_sub.A
    B = polar_sub.B
    C = polar_sub.C

    CD = CD0 + (drag_polar_slope * CL**2) + (A * TC**2) + (B * TC) + C

    return CD


if __name__ == '__main__':
    
    #Build the data structure.  There's probably a better way to do this
    drag_polar               = Data()
    drag_polar.cruise        = Data()
    drag_polar.cruise.two    = Data()
    drag_polar.cruise.one    = Data()
    drag_polar.cruise.zero   = Data()
    drag_polar.climb         = Data()
    drag_polar.climb.two     = Data()
    drag_polar.climb.one     = Data()
    drag_polar.climb.zero    = Data()
    drag_polar.takeoff       = Data()
    drag_polar.takeoff.two   = Data()
    drag_polar.takeoff.one   = Data()
    drag_polar.takeoff.zero  = Data()
    
    
    
    #Cruise Config
    drag_polar.cruise.CD0    = .0325
    drag_polar.cruise.slope  = .04684
    drag_polar.cruise.two.A  = .0
    drag_polar.cruise.two.B  = .140
    drag_polar.cruise.two.C  = -.0055
    drag_polar.cruise.one.A  = .857
    drag_polar.cruise.one.B  = .07
    drag_polar.cruise.one.C  = -.004
    drag_polar.cruise.zero.A = .0
    drag_polar.cruise.zero.B = .0
    drag_polar.cruise.zero.C = .0
    
    #Climb Config
    drag_polar.climb.CD0    = .0325
    drag_polar.climb.slope  = .04684
    drag_polar.climb.two.A  = .0
    drag_polar.climb.two.B  = .0893
    drag_polar.climb.two.C  = -.0018
    drag_polar.climb.one.A  = .22917
    drag_polar.climb.one.B  = .0893
    drag_polar.climb.one.C  = -.0008
    drag_polar.climb.zero.A = .0
    drag_polar.climb.zero.B = .0
    drag_polar.climb.zero.C = .0
    
    #Takeoff Config
    drag_polar.takeoff.CD0    = .06518
    drag_polar.takeoff.slope  = .0539
    drag_polar.takeoff.two.A  = .0
    drag_polar.takeoff.two.B  = .0558
    drag_polar.takeoff.two.C  = -.01398
    drag_polar.takeoff.one.A  = .750
    drag_polar.takeoff.one.B  = .0558
    drag_polar.takeoff.one.C  = -.01398
    drag_polar.takeoff.zero.A = .0
    drag_polar.takeoff.zero.B = .0
    drag_polar.takeoff.zero.C = .0


    #size_array = [[1][1][1][1][1][1][1][1][1][1][1][1][1][1][1][1]]
    CL = .5 #* size_array
    polar_main = drag_polar.climb
    polar_sub = drag_polar.climb.two
    TC = .09 #* size_array

    CD = calculate_from_polar(polar_main, polar_sub, CL, TC)
    print(CL**2, CD)
    
