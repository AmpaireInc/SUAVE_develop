# tut_mission_B737.py
# 
# Created:  Aug 2014, SUAVE Team
# Modified: Aug 2017, SUAVE Team
#           Mar 2020, E. Botero

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# General Python Imports
import numpy as np
# Numpy is a commonly used mathematically computing package. It contains many frequently used
# mathematical functions and is faster than native Python, especially when using vectorized
# quantities.
import matplotlib.pyplot as plt
# Matplotlib's pyplot can be used to generate a large variety of plots. Here it is used to create
# visualizations of the aircraft's performance throughout the mission.

# SUAVE Imports
import SUAVE
assert SUAVE.__version__=='2.5.2', 'These tutorials only work with the SUAVE 2.5.2 release'
from SUAVE.Core import Data, Units 
# The Data import here is a native SUAVE data structure that functions similarly to a dictionary.
#   However, iteration directly returns values, and values can be retrieved either with the 
#   typical dictionary syntax of "entry['key']" or the more class-like "entry.key". For this to work
#   properly, all keys must be strings.
# The Units import is used to allow units to be specified in the vehicle setup (or elsewhere).
#   This is because SUAVE functions generally operate using metric units, so inputs must be 
#   converted. To use a length of 20 feet, set l = 20 * Units.ft . Additionally, to convert to SUAVE
#   output back to a desired units, use l_ft = l_m / Units.ft
from SUAVE.Plots.Performance.Mission_Plots import *
# These are a variety of plotting routines that simplify the plotting process for commonly 
# requested metrics. Plots of specifically desired metrics can also be manually created.


from copy import deepcopy
import scipy.optimize

#Import aircraft
import sys
sys.path.insert(0,'C:/Aero/SUAVE/')
import Beech_KA_B200

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    """This function gets the vehicle configuration, analysis settings, and then runs the mission.
    Once the mission is complete, the results are plotted."""
    
    # Extract vehicle configurations and the analysis settings that go with them
    configs, analyses = full_setup()

    # Size each of the configurations according to a given set of geometry relations
    simple_sizing(configs)

    # Perform operations needed to make the configurations and analyses usable in the mission
    configs.finalize()
    analyses.finalize()

    # Determine the vehicle weight breakdown (independent of mission fuel usage)
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()      

    # Perform a mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()

    # Plot all mission results, including items such as altitude profile and L/D
    plot_mission(results)

    return

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():
    """This function gets the baseline vehicle and creates modifications for different 
    configurations, as well as the mission and analyses to go with those configurations."""

    # Collect baseline vehicle data and changes when using different configuration settings
#    vehicle  = vehicle_setup()
#    configs  = configs_setup(vehicle)

    # vehicle data
    vehicle  = Beech_KA_B200.vehicle_setup()
    configs  = Beech_KA_B200.configs_setup(vehicle)

    # Get the analyses to be used when different configurations are evaluated
    configs_analyses = analyses_setup(configs)

    # Create the mission that will be flown
    mission  = mission_setup(configs_analyses,vehicle)
    missions_analyses = missions_setup(mission)

    # Add the analyses to the proper containers
    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = SUAVE.Analyses.Analysis.Container()

    # Build a base analysis for each configuration. Here the base analysis is always used, but
    # this can be modified if desired for other cases.
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):
    """This is the baseline set of analyses to be used with this vehicle. Of these, the most
    commonly changed are the weights and aerodynamics methods."""

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_Transport()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Known_Drag_Polar()
    aerodynamics.geometry = vehicle
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()
    stability.geometry = vehicle
    analyses.append(stability)

    # ------------------------------------------------------------------
    #  Energy
    energy = SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.networks
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    return analyses    

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------


def simple_sizing(configs):
    """This function applies a few basic geometric sizing relations and modifies the landing
    configuration."""

    base = configs.base
    # Update the baseline data structure to prepare for changes
    base.pull_base()

    # Revise the zero fuel weight. This will only affect the base configuration. To do all
    # configurations, this should be specified in the top level vehicle definition.
    base.mass_properties.max_zero_fuel = 0.9 * base.mass_properties.max_takeoff 

    # Estimate wing areas
    for wing in base.wings:
        wing.areas.wetted   = 2.0 * wing.areas.reference
        wing.areas.exposed  = 0.8 * wing.areas.wetted
        wing.areas.affected = 0.6 * wing.areas.wetted

    # Store how the changes compare to the baseline configuration
    base.store_diff()

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------
    landing = configs.landing

    # Make sure base data is current
    landing.pull_base()

    # Add a landing weight parameter. This is used in field length estimation and in
    # initially the landing mission segment type.
    landing.mass_properties.landing = 0.85 * base.mass_properties.takeoff

    # Store how the changes compare to the baseline configuration
    landing.store_diff()

    return

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses,vehicle):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # Airport
    # The airport parameters are used in calculating field length and noise. They are not
    # directly used in mission performance estimation
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # Unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # Base segment 
    base_segment = Segments.Segment()
    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Throttle, Constant Speed
    # ------------------------------------------------------------------


    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    # It is important that all segment tags must be unique for proper evaluation. At the moment 
    # this is not automatically enforced. 
    segment.tag = "climb_1"

    # The analysis settings for mission segment are chosen here. These analyses include information
    # on the vehicle configuration.
    segment.analyses.extend( analyses.takeoff )

    segment.altitude_start = 0.0   * Units.ft
    segment.altitude_end   = 2000.0   * Units.ft
    segment.air_speed      = 125.0 * Units['knots'] # POH Vy
    segment.throttle       = 1.
    
    ones_row                                          = segment.state.ones_row
    segment.state.numerics.number_control_points      = 16
    
    segment = vehicle.networks.internal_combustion.add_unknowns_and_residuals_to_segment(segment,rpm=2100)
   


    # Add to misison
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "climb_2"

    segment.analyses.extend( analyses.climb )

    segment.altitude_end              = 11000.   * Units.ft
    segment.calibrated_air_speed      = 138.0 * Units['knots'] #POH best cruise climb to 10k
    segment.throttle                  = 1.0

    
    ones_row                                          = segment.state.ones_row
    segment.state.numerics.number_control_points      = 16
    
    ba0 = 15.25
    ba1 = 14.5
    
    ba_guess_array = np.linspace(ba0,ba1,segment.state.numerics.number_control_points).reshape(-1,1)
    
    segment.state.unknowns.body_angle = ba_guess_array * Units.deg #12. * ones_row(1) * Units.deg 
    segment.state.unknowns.wind_angle = 6.5 * ones_row(1) * Units.deg
    #segment.settings.root_finder = scipy.optimize.broyden1
    #segment.state.numerics.max_evaluations = 500
    segment = vehicle.networks.internal_combustion.add_unknowns_and_residuals_to_segment(segment,rpm=2000)
    
    # Add to mission
    mission.append_segment(segment)
    
    """
    # ------------------------------------------------------------------
    #   Third Climb Segment: constant Mach, Constant Rate
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_Mach_Constant_Rate(base_segment)
    segment.tag = "climb_3"

    segment.analyses.extend( analyses.cruise )

    segment.altitude_end   = 30000.   * Units.ft
    segment.mach_number    = .45
    segment.climb_rate     = 200.   * Units['ft/min']
    
    ones_row                                          = segment.state.ones_row
    segment.state.numerics.number_control_points      = 16
    segment.state.unknowns.throttle                 = 0.70 * ones_row(1)
    #segment.state.conditions.propulsion.rpm         = 2000. * Units.rpm * ones_row(1)
    segment = vehicle.networks.internal_combustion.add_unknowns_and_residuals_to_segment(segment,rpm=2000)
    
    # Add to mission
    mission.append_segment(segment)

    """
    
    """
    # ------------------------------------------------------------------
    #   Third Climb Segment: constant Speed, Constant Rate
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Constant_CAS_Constant_Rate(base_segment)
    segment.tag = "climb_3"

    segment.analyses.extend( analyses.cruise )

    segment.altitude_end   = 20000.   * Units.ft
    segment.calibrated_air_speed      = 140.0 * Units['knots'] #POH best cruise climb to 20k
    segment.climb_rate     = 800.   * Units['ft/min']
    
    ones_row                                          = segment.state.ones_row
    segment.state.numerics.number_control_points      = 16
    segment.state.unknowns.throttle                 = 0.70 * ones_row(1)
    #segment.state.conditions.propulsion.rpm         = 2000. * Units.rpm * ones_row(1)
    segment = vehicle.networks.internal_combustion.add_unknowns_and_residuals_to_segment(segment,rpm=2000)
    
    # Add to mission
    mission.append_segment(segment)
    """
    
    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.cruise )
    #analyses.cruise.aerodynamics.settings.drag_coefficient_increment = .02
    segment.air_speed  = 175. * Units['knots']
    segment.distance   = 100. * Units.nautical_miles
    
    ones_row                                          = segment.state.ones_row
    segment.state.numerics.number_control_points      = 16
    segment.state.unknowns.throttle                 = 0.70 * ones_row(1)
    #segment.state.conditions.propulsion.rpm         = 2000. * Units.rpm * ones_row(1)
    segment = vehicle.networks.internal_combustion.add_unknowns_and_residuals_to_segment(segment,rpm=2000)

    # Add to mission
    mission.append_segment(segment)
    
    """
    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1"

    segment.analyses.extend( analyses.cruise )

    segment.altitude_end   = 5000.   * Units.ft
    segment.air_speed      = 250. * Units['knots']
    segment.descent_rate     = 200.   * Units['ft/min']
    
    ones_row                                          = segment.state.ones_row
    segment.state.numerics.number_control_points      = 16
    segment.state.unknowns.throttle                 = 0.50 * ones_row(1)
    #segment.state.conditions.propulsion.rpm         = 2000. * Units.rpm * ones_row(1)
    segment = vehicle.networks.internal_combustion.add_unknowns_and_residuals_to_segment(segment,rpm=2000)
    
    # Add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_2"

    segment.analyses.extend( analyses.cruise )

    segment.altitude_end   = 2000.   * Units.ft
    segment.air_speed      = 220. * Units['knots']
    segment.descent_rate     = 200   * Units['ft/min']
    
    ones_row                                          = segment.state.ones_row
    segment.state.numerics.number_control_points      = 16
    segment.state.unknowns.throttle                 = 0.40 * ones_row(1)
    #segment.state.conditions.propulsion.rpm         = 2000. * Units.rpm * ones_row(1)
    segment = vehicle.networks.internal_combustion.add_unknowns_and_residuals_to_segment(segment,rpm=2000)
    
    # Add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_3"

    segment.analyses.extend( analyses.landing )
    # While it is set to zero here and therefore unchanged, a drag increment can be used if
    # desired. This can avoid negative throttle values if drag generated by the base airframe
    # is insufficient for the desired descent speed and rate.
    analyses.landing.aerodynamics.settings.spoiler_drag_increment = 0.00

    segment.altitude_end   = 1000.   * Units.ft
    segment.air_speed      = 150.0 * Units['knots']
    segment.descent_rate     = 500.   * Units['ft/min']
    
    ones_row                                          = segment.state.ones_row
    segment.state.numerics.number_control_points      = 16
    segment.state.unknowns.throttle                 = 0.50 * ones_row(1)
    #segment.state.conditions.propulsion.rpm         = 2000. * Units.rpm * ones_row(1)
    segment = vehicle.networks.internal_combustion.add_unknowns_and_residuals_to_segment(segment,rpm=2000)
    
    # Add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Fourth Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_4"

    segment.analyses.extend( analyses.landing )
    analyses.landing.aerodynamics.settings.spoiler_drag_increment = 0.20

    segment.altitude_end   = 0.   * Units.ft
    segment.air_speed      = 100.0 * Units['knots']
    segment.descent_rate     = 500.   * Units['ft/min']
    
    ones_row                                          = segment.state.ones_row
    segment.state.numerics.number_control_points      = 16
    segment.state.unknowns.throttle                 = 0.50 * ones_row(1)
    #segment.state.conditions.propulsion.rpm         = 2000. * Units.rpm * ones_row(1)
    segment = vehicle.networks.internal_combustion.add_unknowns_and_residuals_to_segment(segment,rpm=2000)
    
    # Add to mission
    mission.append_segment(segment)
    """

    
    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission

def missions_setup(base_mission):
    """This allows multiple missions to be incorporated if desired, but only one is used here."""

    # Setup the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    # Only one mission (the base mission) is defined in this case
    missions.base = base_mission

    return missions  

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,line_style='bo-'):
    """This function plots the results of the mission analysis and saves those results to 
    png files."""

    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style)
    
    # Plot Propulsion Conditions
    plot_propulsion_conditions(results, line_style)
    
    # Plot Fuel Use 
    plot_fuel_use(results, line_style)
    
    # Plot Aerodynamic Forces 
    plot_aerodynamic_forces(results, line_style)
    
    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results, line_style)
    
    # Drag Components
    plot_drag_components(results, line_style)
    
    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results, line_style)
    
    # Plot Velocities 
    plot_aircraft_velocities(results, line_style)      
        
    return

# This section is needed to actually run the various functions in the file
if __name__ == '__main__': 
    main()    
    # The show commands makes the plots actually appear
    plt.show()