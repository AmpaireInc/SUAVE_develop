# -*- coding: utf-8 -*-
"""
Created on Thu Nov 30 12:05:57 2023

@author: JoshuaJacobs
"""

# tut_payload_range.py
#
# Created:  Aug 2014, SUAVE Team
# Modified: Apr 2016, T. Orra
#           Aug 2017, E. Botero

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import SUAVE
assert SUAVE.__version__=='2.5.2', 'These tutorials only work with the SUAVE 2.5.2 release'
from SUAVE.Core import Units
from SUAVE.Methods.Performance  import payload_range
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import wing_planform
from SUAVE.Plots.Performance.Mission_Plots import *

#Import aircraft
import sys
sys.path.insert(0,'C:/Aero/SUAVE-2.5.2/SUAVE-2.5.2/regression/scripts/Vehicles/')
#import Beech_KA_B200
import Cessna_172


import pylab as plt


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():

    # define the problem
    configs, analyses = full_setup()

    configs.finalize()
    analyses.finalize()

    # weight analysis
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()

    # mission analysis
    mission = analyses.missions
    results = mission.evaluate()

    # run payload diagram
    config = configs.base
    cruise_segment_tag = "cruise"
    reserves = 100.
    payload_range_results = payload_range(config,mission,cruise_segment_tag,reserves)

    # plot the results
    plot_mission(results)

    return

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():

    # vehicle data
    #vehicle  = Beech_KA_B200.vehicle_setup()
    #configs  = Beech_KA_B200.configs_setup(vehicle)
    vehicle  = Cessna_172.vehicle_setup()
    configs  = Cessna_172.configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(configs_analyses,vehicle)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = mission

    return configs, analyses

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    # adjust analyses for configs

    # takeoff_analysis
    analyses.takeoff.aerodynamics.drag_coefficient_increment = 0.1000

    return analyses

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    weights = SUAVE.Analyses.Weights.Weights_Transport()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.000
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()
    stability.geometry = vehicle
    analyses.append(stability)

    # ------------------------------------------------------------------
    #  Propulsion Analysis
    energy= SUAVE.Analyses.Energy.Energy()
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
#   Define the Mission
# ----------------------------------------------------------------------
def mission_setup(analyses,vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'KA200 H570 test mission'

    # atmospheric model
    atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()
    planet = SUAVE.Attributes.Planets.Earth()

    #airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    
    """
    # ------------------------------------------------------------------
    #   First Climb Segment
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1"

    # connect vehicle configuration
    segment.analyses.extend( analyses.takeoff )

    # define segment attributes
    segment.altitude_start = 0.0   * Units.ft
    segment.altitude_end   = 1000.   * Units.ft
    segment.air_speed      = 125. * Units.knots
    segment.climb_rate     = 800.0   * Units['ft/s']
    
    ones_row                                          = segment.state.ones_row
    segment.state.numerics.number_control_points      = 16
    segment.state.unknowns.throttle                 = .85 * ones_row(1)
    segment.state.conditions.propulsion.rpm         = 2000.  * ones_row(1)
    #segment.state.residuals.network              = 0.0 * ones_row(1)
    
    #segment.process.iterate.unknowns.network = analyses['cruise'].aerodynamics.geometry.networks.internal_combustion.unpack_unknowns
    #segment.process.iterate.residuals.network = analyses['cruise'].aerodynamics.geometry.networks.internal_combustion.residuals

    #segment = vehicle.networks.internal_combustion.add_unknowns_and_residuals_to_segment(segment,rpm=2600)

    segment.process.iterate.conditions.stability = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip
    
    # add to misison
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Climb Segment
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    segment.altitude_end   = 8000.   * Units.ft
    segment.air_speed      = 145. * Units.knots
    segment.climb_rate     = 400   * Units['ft/s']
    #segment = vehicle.networks.internal_combustion.add_unknowns_and_residuals_to_segment(segment,rpm=2600)
    
    ones_row                                          = segment.state.ones_row
    segment.state.numerics.number_control_points      = 16
    segment.state.unknowns.throttle                 = .85 * ones_row(1)
    segment.state.conditions.propulsion.rpm         = 2000.  * ones_row(1)


    segment.process.iterate.conditions.stability = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip
    
    # add to mission
    mission.append_segment(segment)
    
    """
    """    # ------------------------------------------------------------------
    #   Third Climb Segment
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_3"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    segment.altitude_end = 10.668 * Units.km
    segment.air_speed    = 226.0  * Units['m/s']
    segment.climb_rate   = 3.0    * Units['m/s']

    # add to mission
    mission.append_segment(segment)

    """
    # ------------------------------------------------------------------
    #   Cruise Segment
    # ------------------------------------------------------------------
    """ 
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    segment.atmosphere = atmosphere
    segment.planet     = planet

    segment.air_speed  = 150. * Units.knots 
    segment.distance   = 1000. * Units.nmi
    segment.altitude   = 8000. * Units.ft

    ones_row                                          = segment.state.ones_row
    segment.state.numerics.number_control_points      = 16
    segment.state.unknowns.throttle                 = 0.85 * ones_row(1)
    segment.state.conditions.propulsion.rpm         = 2000. * Units.rpm * ones_row(1)

    segment.process.iterate.conditions.stability = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip
    """
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.cruise )

    segment.altitude  = 8500. * Units.feet
    segment.air_speed = 116.   * Units.knots
    segment.distance  = 100. * Units.nautical_mile
    
    ones_row                                        = segment.state.ones_row   
    segment.state.numerics.number_control_points    = 16
    segment.state.unknowns.throttle                 = .5 * ones_row(1)
    segment = vehicle.networks.internal_combustion.add_unknowns_and_residuals_to_segment(segment,rpm=2600)
    
    
    segment.process.iterate.conditions.stability    = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip    

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Descent Segment
    # ------------------------------------------------------------------
    """
    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1"

    # connect vehicle configuration
    segment.analyses.extend( analyses.cruise )

    # segment attributes
    segment.altitude_end = 10000.0   * Units.ft
    segment.air_speed    = 160.0 * Units.knots
    segment.descent_rate = 4.5   * Units['m/s']
    #segment = vehicle.networks.internal_combustion.add_unknowns_and_residuals_to_segment(segment,rpm=2600)


    segment.process.iterate.conditions.stability = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip
    
    # add to mission
    mission.append_segment(segment)
    """
    """    # ------------------------------------------------------------------
    #   Second Descent Segment
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_2"

    # connect vehicle configuration
    segment.analyses.extend( analyses.landing )

    # segment attributes
    segment.altitude_end = 6.0   * Units.km
    segment.air_speed    = 195.0 * Units['m/s']
    segment.descent_rate = 5.0   * Units['m/s']

    # add to mission
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Descent Segment
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_3"

    # connect vehicle configuration
    segment.analyses.extend( analyses.landing )

    # segment attributes
    segment.altitude_end = 4.0   * Units.km
    segment.air_speed    = 170.0 * Units['m/s']
    segment.descent_rate = 5.0   * Units['m/s']

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Fourth Descent Segment
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_4"

    segment.analyses.extend( analyses.landing )

    segment.altitude_end = 2.0   * Units.km
    segment.air_speed    = 150.0 * Units['m/s']
    segment.descent_rate = 5.0   * Units['m/s']


    # add to mission
    mission.append_segment(segment)
    """
    # ------------------------------------------------------------------
    #   Fifth Descent Segment
    # ------------------------------------------------------------------
    """
    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_5"

    segment.analyses.extend( analyses.landing )

    segment.altitude_end = 0.0   * Units.km
    segment.air_speed    = 125. * Units.knots
    segment.descent_rate = 3.0   * Units['m/s']
    #segment = vehicle.networks.internal_combustion.add_unknowns_and_residuals_to_segment(segment,rpm=2600)


    segment.process.iterate.conditions.stability = SUAVE.Methods.skip
    segment.process.finalize.post_process.stability = SUAVE.Methods.skip
    
    # append to mission
    mission.append_segment(segment)
    """
    # ------------------------------------------------------------------
    #   Mission definition complete
    # ------------------------------------------------------------------

    return mission

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,line_style='bo-'):

    # Plot Flight Conditions 
    plot_flight_conditions(results, line_style)
    
    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results, line_style)    
    
    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results, line_style)  
    
    plot_disc_power_loading(results, line_style)
    
    plot_propeller_conditions(results, line_style)


    return


if __name__ == '__main__':
    main()
    plt.show()