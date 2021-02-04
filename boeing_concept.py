#boeing_concept.py

# Created:  Feb 2021, E. Botero
# Modified: 


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np
import matplotlib.pyplot as plt  

from SUAVE.Core import Data, Units
from SUAVE.Input_Output.OpenVSP.vsp_read import vsp_read
from SUAVE.Methods.Aerodynamics.Common.Fidelity_Zero.Lift import VLM
from SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_segmented_planform import wing_segmented_planform

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():

    # First import the geometry
    vehicle = vsp_read('boeing_n_d_t.vsp3',units_type='inches')
    
    vehicle.fuselages.pop('fueslage')
    vehicle.wings.pop('tail')
        
    vehicle.reference_area = vehicle.wings.gross_wing_b__t___d_.areas.reference
        
    
    # Setup conditions
    conditions = setup_conditions()
    
    # Run
    results = analyze(vehicle, conditions)
    
    print(results)
    
    
    return


# ----------------------------------------------------------------------
#  setup_conditions
# ----------------------------------------------------------------------


def setup_conditions():
        
    #aoas  = np.array([-2,0,2,4,6,-2,0,2,4,6,-2,0,2,4,6,-2,0,2,4,6,-2,0,2,4,6,-2,0,2,4,6]) * Units.degrees
    #machs = np.array([0.4,0.4,0.4,0.4,0.4,0.8,0.8,0.8,0.8,0.8,1.4,1.4,1.4,1.4,1.4,1.6,1.6,1.6,1.6,1.6,1.8,1.8,1.8,1.8,1.8,2,2,2,2,2])
    
    
    #aoas  = np.array([6.,2.,2.,6.]) * Units.degrees
    #machs = np.array([0.4,1.,2.0,2.0])    
    
    #aoas  = np.array([6.,6]) * Units.degrees
    #machs = np.array([0.4,1.4]) 
    
    aoas  = np.array([0.,2.,4.,6.,8.,10,0.,2.,4.,6.,8.,10,0.,2.,4.,6.,8.,10,0.,2.,4.,6.,8.,10]) * Units.degrees
    machs = np.array([1.4,1.4,1.4,1.4,1.4,1.4,1.6,1.6,1.6,1.6,1.6,1.6,1.8,1.8,1.8,1.8,1.8,1.8,2.0,2.0,2.0,2.0,2.0,2.0])        
    
    #aoas  = xv.flatten()
    #machs = yv.flatten()
    
    conditions              = Data()
    conditions.aerodynamics = Data()
    conditions.freestream   = Data()
    conditions.freestream.velocity          = np.atleast_2d(100.*np.ones_like(aoas))
    conditions.aerodynamics.angle_of_attack = np.atleast_2d(aoas).T
    conditions.freestream.mach_number       = np.atleast_2d(machs).T

    return conditions

# ----------------------------------------------------------------------
#  analyze
# ----------------------------------------------------------------------



def analyze(config,conditions):
    
    
    results = Data()
    
    S                                  = config.reference_area
    settings                           = Data()
    settings.number_spanwise_vortices  = 25
    settings.number_chordwise_vortices = 10
    settings.propeller_wake_model      = None
    settings.spanwise_cosine_spacing   = True

    CL, CDi, CM, CL_wing, CDi_wing, cl_y , cdi_y , CP ,Velocity_Profile = VLM(conditions, settings, config)

    results.CDi  = CDi
    results.CL   = CL
    results.mach = conditions.freestream.mach_number
    results.aoa  = conditions.aerodynamics.angle_of_attack
    
    print('CL')
    print(CL)
    
    print('CDi')
    print(CDi)

    return results




if __name__ == '__main__': 
    main()    
    plt.show()
