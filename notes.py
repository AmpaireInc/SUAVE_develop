# -*- coding: utf-8 -*-
"""
Created on Thu Aug 22, 2024

@author: JoshuaJacobs
"""
"""

Handoff notes:
    
KA main working mission file is tut_mission_KA200.py
Have tried to get the Payload-Range mission working, but haven't spent a lot of time on it.
Intent is to use OEM KA B200 configuration for baseline to verify tool accuracy

Differences from standard SUAVE:
    -internal combustion network duplicated and modified to use a prop table
    -aerodynamics modified to use known drag polars
    -added a mission climb segment based on constant CAS and constant throttle to try to emulate what pilots actually fly
    -commit notes are pretty meh... sorry
    
To do's:
    -Update KA200 physical parameters and document sources.  Most are from random internet sources or the POH and should be verified
    -Figure out why constant CAS climb segment oscillates
    -Validate results are accurate / appropriate
    -Create / integrate model for H570 powertrain
    -Create missions per CDR for comparison of baseline vs H570 KA