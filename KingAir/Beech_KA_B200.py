# Beech_KA_B200.py
#
# Created:  Oct 2023, J Jacobs (modified from Beech99.py)
# Modified: 

""" setup file for the Beech King Air B200 aircraft
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import numpy as np
import SUAVE
from SUAVE.Core import Units
from SUAVE.Core import (
    Data, Container,
)
from SUAVE.Methods.Geometry.Three_Dimensional.compute_span_location_from_chord_length import compute_span_location_from_chord_length
from SUAVE.Methods.Flight_Dynamics.Static_Stability.Approximations.datcom import datcom
from SUAVE.Methods.Flight_Dynamics.Static_Stability.Approximations.Supporting_Functions.trapezoid_ac_x import trapezoid_ac_x
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import wing_planform


#hacky for prop section
from pathlib import Path 
from scipy import interpolate as interp
import pandas as pd

#Need to scrub all the numbers below.  Some updated, some not.

def vehicle_setup():
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------
    
    vehicle = SUAVE.Vehicle()
    vehicle.tag                                 = 'Beech_KA_B200'
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------
    
    vehicle.mass_properties.max_takeoff               = 12500.0 * Units.lbs
    vehicle.mass_properties.operating_empty           = 8283.0 * Units.lbs
    vehicle.mass_properties.empty                     = 8283.0 * Units.lbs #Basic Operating with one pilot, unusable fuel, etc
    vehicle.mass_properties.max_zero_fuel             = 11000 * Units.lbs
    vehicle.mass_properties.max_payload               = 4962. * Units.lbs
    vehicle.mass_properties.max_fuel                  = 4000. * Units.lbs
    
    vehicle.mass_properties.takeoff                   = 12500.0 * Units.lbs 

    vehicle.mass_properties.cargo                     =     0.0  # kg

    #vehicle.mass_properties.center_of_gravity         = [[16.8, 0, 1.6]] #no units?
    vehicle.mass_properties.center_of_gravity = np.array([[17.2,0,0]]) * Units.feet 
    vehicle.mass_properties.moments_of_inertia.tensor = [[10 ** 5, 0, 0],[0, 10 ** 6, 0,],[0,0, 10 ** 7]] 

    # Systems
    vehicle.systems.accessories    = "business"
    vehicle.systems.control        = "non-powered"
    
    # Passengers
    vehicle.passengers             = 0
    
    # envelope properties
    vehicle.envelope.ultimate_load = 5.5
    vehicle.envelope.limit_load    = 2.5

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------

    
    wing = SUAVE.Components.Wings.Wing()
    wing.tag                                 = 'main_wing'
    wing.areas.reference                     = 303.0 * Units.feet**2
    wing.spans.projected                     = 54.5  * Units.feet  
    wing.chords.mean_aerodynamic             = 5.625 * Units.feet
    wing.chords.root                         = 9.0 * Units.feet
    wing.sweeps.leading_edge                 = 4.0   * Units.deg
    wing.sweeps.quarter_chord                = 0.0   * Units.deg 
    wing.taper                               = 0.47
    wing.dihedral                            = 6.0 * Units.deg
    wing.aspect_ratio                        = wing.spans.projected**2/wing.areas.reference
    wing.symmetric                           = True
    wing.vertical                            = False
    wing.origin                              = [[15.* Units.feet  ,0,0]]
    wing.aerodynamic_center                  = np.array([trapezoid_ac_x(wing), 0. , 0. ])
    wing.dynamic_pressure_ratio              = 1.0
    wing.ep_alpha                            = 0.0
    span_location_mac                        = compute_span_location_from_chord_length(wing, wing.chords.mean_aerodynamic)
    mac_le_offset                            =.8*np.sin(wing.sweeps.leading_edge)*span_location_mac  #assume that 80% of the chord difference is from leading edge sweep
    wing.mass_properties.center_of_gravity[0][0]=.3*wing.chords.mean_aerodynamic+mac_le_offset
    wing.areas.exposed           = 0.80 * wing.areas.wetted
    
    wing = wing_planform(wing)
    
    #Mach                                  = np.array([0.152])
    #reference                             = SUAVE.Core.Container()
    #conditions                            = Data()
    #conditions.lift_curve_slope           = datcom(wing,Mach)    
    #conditions.weights                    =Data()
    #conditions.weights.total_mass         =np.array([[vehicle.mass_properties.max_takeoff]])    
    #wing.CL_alpha                         = conditions.lift_curve_slope
    vehicle.reference_area                = wing.areas.reference
                                
    # control surfaces -------------------------------------------
    flap                       = SUAVE.Components.Wings.Control_Surfaces.Flap() 
    flap.tag                   = 'flap' 
    flap.span_fraction_start   = 0.15    # not correct, only placeholder
    flap.span_fraction_end     = 0.324   # not correct, only placeholder 
    flap.deflection            = 1.0 * Units.deg
    flap.chord_fraction        = 0.19    # not correct, only placeholder
    wing.append_control_surface(flap)    
       
    vehicle.append_component(wing)
    
    #main_wing_CLa = wing.CL_alpha
    #main_wing_ar  = wing.aspect_ratio
    
    
    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------
    
    wing = SUAVE.Components.Wings.Wing()
    wing.tag                      = 'horizontal_stabilizer'
    wing.areas.reference          = 68.0 * Units.feet**2
    wing.spans.projected          = 18.417  * Units.feet
    wing.sweeps.leading_edge      = 21.0   * Units.deg # <<GUESS - UPDATE!>
    wing.sweeps.quarter_chord     = 17.0   * Units.deg 
    wing.taper                    = 3.1/6.17 # <<GUESS - UPDATE!>
    wing.aspect_ratio             = wing.spans.projected**2/wing.areas.reference
    wing.origin                   = [[38.3* Units.feet,0,0]] # <<GUESS - UPDATE!>
    wing.symmetric                = True
    wing.vertical                 = False
    wing.dynamic_pressure_ratio   = 0.95
    #wing.ep_alpha                 = 2.0*main_wing_CLa/np.pi/main_wing_ar
    wing.aerodynamic_center       = np.array([trapezoid_ac_x(wing), 0.0, 0.0])
    #wing.CL_alpha                 = datcom(wing,Mach)
    wing = wing_planform(wing)
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------
    
    wing = SUAVE.Components.Wings.Wing()
    wing.tag                      = 'vertical_stabilizer'
    wing.areas.reference          = 68.0 * Units.feet**2
    wing.spans.projected          = 18.417  * Units.feet
    wing.sweeps.leading_edge      = 21.0   * Units.deg # 
    wing.sweeps.quarter_chord     = 17.0   * Units.deg 
    wing.taper                    = 3.1/6.17 
    wing.aspect_ratio             = wing.spans.projected**2/wing.areas.reference
    wing.origin                   = [[38.3* Units.feet,0,0]] # <<GUESS - UPDATE!>
    wing.symmetric                = False
    wing.vertical                 = True
    wing                         = wing_planform(wing)
    wing.dynamic_pressure_ratio   = 0.95
    #wing.ep_alpha                 = 2.0*main_wing_CLa/np.pi/main_wing_ar
    wing.aerodynamic_center       = np.array([trapezoid_ac_x(wing), 0.0, 0.0])
    #wing.CL_alpha                 = datcom(wing,Mach)
    wing = wing_planform(wing)
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------
    
    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag                  = 'fuselage'
    
    fuselage.number_coach_seats    = vehicle.passengers
    fuselage.seats_abreast         = 2
    fuselage.seat_pitch            = 30. * Units.inches

    fuselage.fineness.nose         = 1.28
    fuselage.fineness.tail         = 3.48

    fuselage.lengths.nose          = 6.0 * Units.ft
    fuselage.lengths.tail          = 9.0* Units.ft
    fuselage.lengths.cabin         = 15* Units.ft  
    fuselage.lengths.fore_space    = 0.
    fuselage.lengths.aft_space     = 0.
    
    fuselage.x_root_quarter_chord = 5.4 * Units.feet
    fuselage.lengths.total        = 44.0  * Units.feet
    fuselage.width                = 5.4   * Units.feet 
    fuselage.heights.maximum       = 5.4 * Units.feet    
    fuselage.heights.at_quarter_length          = 5.4 * Units.feet    
    fuselage.heights.at_three_quarters_length   = 5.4 * Units.feet    
    fuselage.heights.at_wing_root_quarter_chord = 5.4 * Units.feet  
    fuselage.areas.side_projected  = 250.20 *Units.ft**2
    fuselage.areas.wetted          = 327.01 * Units.ft**2
    fuselage.areas.front_projected = 12.0* Units.ft**2
    fuselage.effective_diameter    = 5.4 * Units.feet 

    fuselage.differential_pressure = 10**5 * Units.pascal    # Maximum differential pressure
    vehicle.append_component(fuselage)
    
    
    # ------------------------------------------------------------------
    #   Known Drag Polars - Source: Army CEFLY LANCER Study
    # ------------------------------------------------------------------
    """drag_polar is a data structure containing the drag polar constants in
       the format drag_polar.[configuration].{number of engines operating}.[attribute]
    """
    
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
    
    vehicle.drag_polar = drag_polar
    
    
    # ------------------------------------------------------------------
    #   Fuel
    # ------------------------------------------------------------------
 
    fuel                                      = SUAVE.Components.Physical_Component()
    fuel.origin                               = wing.origin
    fuel.mass_properties.center_of_gravity    = wing.mass_properties.center_of_gravity
    fuel.mass_properties.mass                 = vehicle.mass_properties.max_takeoff-vehicle.mass_properties.max_zero_fuel
    
    """#find zero_fuel_center_of_gravity
    cg                     = vehicle.mass_properties.center_of_gravity
    MTOW                   = vehicle.mass_properties.max_takeoff
    fuel_cg                = fuel.origin+fuel.mass_properties.center_of_gravity
    fuel_mass              = fuel.mass_properties.mass 
    sum_moments_less_fuel  = (cg*MTOW-fuel_cg*fuel_mass)
    
    vehicle.mass_properties.zero_fuel_center_of_gravity = sum_moments_less_fuel/vehicle.mass_properties.max_zero_fuel
    """
    vehicle.fuel           =  fuel

    
    # ------------------------------------------------------------------
    #   Piston Propeller Network
    # ------------------------------------------------------------------  
    
    # build network
    net                                         = SUAVE.Components.Energy.Networks.Internal_Combustion_Propeller_Table2()
    net.tag                                     = 'internal_combustion'
    net.number_of_engines                       = 2.
    net.origin            = [[120.0 * Units.inches,104.0 * Units.inches,5 * Units.inches] ,[120.0 * Units.inches,-104.00 * Units.inches,5 * Units.inches]]
    net.rated_speed                             = 2100. * Units.rpm
    #net.rated_power                             = 1100.  * Units.hp  #Not currently used in IC model
    net.engine_length                           = 5. * Units.ft
    net.identical_propellers                    = True
    
    # Component 1 the engine                    
    engine                                  = SUAVE.Components.Energy.Converters.Internal_Combustion_Engine()
    engine.sea_level_power                  = 850. * Units.horsepower
    engine.flat_rate_altitude               = 30000. * Units.ft
    engine.rated_speed                      = 2100. * Units.rpm
    engine.power_specific_fuel_consumption  = 0.601   #.601 for PT6
    
    net.engines.append(engine)
    
    
    # Propeller
    prop = SUAVE.Components.Energy.Converters.Propeller_Known_etap()
    prop.number_of_blades       = 3.0
    prop.freestream_velocity    = 155.   * Units.knots
    prop.angular_velocity       = 2100.  * Units.rpm
    prop.tip_radius             = 98./2. * Units.inches
    prop.hub_radius             = 12.     * Units.inches
    prop.design_Cl              = 0.8
    prop.design_altitude        = 10000. * Units.feet
    prop.design_power           = .99 * 850. * Units.horsepower 
    prop.variable_pitch          = True
    
    # Custom hacky prop stuff.  Exists as a function, but I couldn't get it to load the normal way
    prop_table_path = "C:\Git-Local\SUAVE\SUAVE_develop\KingAir\KA200_prop_table.csv"
    #Convert the path string to an OS path
    
    os_path = Path(prop_table_path)
    
    # Load the CSV
    raw_table = pd.read_csv(os_path,index_col=0,header=0,dtype=np.float64) 
    
   
    prop.eta_vals = raw_table.to_numpy()

    prop.Cp_vals = raw_table.index.to_numpy()
    
    J_vals = raw_table.columns.to_numpy()

    prop.J_vals = [eval(i) for i in J_vals]
          
    
    
    """
    prop.airfoil_geometry    = ['C:\\Aero\\SUAVE-2.5.2\\SUAVE-2.5.2\\regression\\scripts\\Vehicles\\Airfoils\\NACA_4412.txt']
    prop.airfoil_polars         = [['C:\\Aero\\SUAVE-2.5.2\\SUAVE-2.5.2\\regression\\scripts\\Vehicles\\Airfoils\\Polars\\NACA_4412_polar_Re_50000.txt' ,
                                'C:\\Aero\\SUAVE-2.5.2\\SUAVE-2.5.2\\regression\\scripts\\Vehicles\\Airfoils\\Polars\\NACA_4412_polar_Re_100000.txt' ,
                                'C:\\Aero\\SUAVE-2.5.2\\SUAVE-2.5.2\\regression\\scripts\\Vehicles\\Airfoils\\Polars\\NACA_4412_polar_Re_200000.txt' ,
                                'C:\\Aero\\SUAVE-2.5.2\\SUAVE-2.5.2\\regression\\scripts\\Vehicles\\Airfoils\\Polars\\NACA_4412_polar_Re_500000.txt' ,
                                'C:\\Aero\\SUAVE-2.5.2\\SUAVE-2.5.2\\regression\\scripts\\Vehicles\\Airfoils\\Polars\\NACA_4412_polar_Re_1000000.txt']]  
    
    prop.airfoil_polar_stations = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    
    prop                        = propeller_design(prop)    
    """
    
    #prop                        = propeller_design(prop) 
    net.propellers.append(prop)

    
    
    # Replace the network
    vehicle.append_component(net)
    
    return vehicle
  
def configs_setup(vehicle):
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------
    configs = SUAVE.Components.Configs.Config.Container()
    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)
    
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------ 
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    config.drag_polar_main = vehicle.drag_polar.cruise
    config.drag_polar_sub = vehicle.drag_polar.cruise.two
    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Climb Configuration
    # ------------------------------------------------------------------ 
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'climb'
    config.drag_polar_main = vehicle.drag_polar.climb
    config.drag_polar_sub = vehicle.drag_polar.climb.two
    configs.append(config)
                   
    #note: takeoff and landing configurations taken from 737 - update
    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------ 
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'    
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    config.V2_VS_ratio = 1.21
    config.maximum_lift_coefficient = 2. 
    config.drag_polar_main = vehicle.drag_polar.takeoff
    config.drag_polar_sub = vehicle.drag_polar.takeoff.two
    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------  
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'    
    config.wings['main_wing'].flaps_angle = 30. * Units.deg
    config.wings['main_wing'].slats_angle = 25. * Units.deg
    config.Vref_VS_ratio = 1.23
    config.maximum_lift_coefficient = 2.
    config.drag_polar_main = vehicle.drag_polar.takeoff
    config.drag_polar_sub = vehicle.drag_polar.takeoff.two
    configs.append(config)
    
    
    # done!
    return configs