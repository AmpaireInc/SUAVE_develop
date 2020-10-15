## @ingroupMethods-Noise-Certification
# approach_noise.py
# 
# Created:  Oct 2020, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np 
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools import compute_noise

# ----------------------------------------------------------------------        
#   Approach noise
# ----------------------------------------------------------------------     

## @ingroupMethods-Noise-Certification 
def approach_noise(mission,noise_config):  
    """This method calculates approach noise of a turbofan aircraft
            
    Assumptions:
        N/A

    Source:
        N/A 

    Inputs:
        mission
        aircraft configuration 

    Outputs: 
        SPL    -  [dB]

    Properties Used:
        N/A 
        
    """ 
    # Update number of control points for noise      
    approach_initialization                                     = mission.evaluate()   
    n_points                                                    = np.ceil(approach_initialization.segments.climb.conditions.frames.inertial.time[-1] /0.5 +1)
    mission.npoints_takeoff_sign                                = np.sign(n_points) 
    mission.segments.climb.state.numerics.number_control_points = np.minimum(200, np.abs(n_points))

    # Set up analysis 
    noise_segment            = mission.segments.descent 
    noise_analyses           = noise_segment.analyses   
    noise_config.engine_flag = 1  

    noise_result_approach = compute_noise(noise_config,noise_analyses,noise_segment)
        
    return noise_result_approach
