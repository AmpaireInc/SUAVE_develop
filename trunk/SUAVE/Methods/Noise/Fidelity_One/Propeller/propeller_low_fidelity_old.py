# noise_propeller_low_fidelty.py
#
# Created:  Feb 2018, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Units , Data
import numpy as np
from scipy.special import jv 

from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools.decibel_arithmetic import pressure_ratio_to_SPL_arithmetic
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools.dbA_noise          import A_weighting

from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools import pnl_noise
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools import noise_tone_correction
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools import epnl_noise
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools import atmospheric_attenuation
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools import noise_geometric
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools import SPL_arithmetic
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools import senel_noise
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools import dbA_noise
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools import SPL_harmonic_to_third_octave

## @ingroupMethods-Noise-Fidelity_One-Propeller
def propeller_low_old_fidelity(propeller,segment,settings, mic_loc, harmonic_test ):
    ''' This computes the SPL of rotors and propellers using Frequency based methods'
    
    Source:
        1. Herniczek, M., Feszty, D., Meslioui, S., Park, Jong Applicability of Early Acoustic Theory for Modern Propeller Design
        2. Schlegel, R., King, R., and Muli, H., Helicopter Rotor Noise Generation and Propagation, Technical Report, 
        US Army Aviation Material Laboratories, Fort Eustis, VA, 1966
  
    Assumptions:
        - Empirical based procedure.           
        - Hanson method used to compute rotational noise  
        - Vortex noise is computed using the method outlined by Schlegel et. al 
        
    Inputs:
        - noise_data	 - SUAVE type vehicle
        - Note that the notation is different from the reference, the speed of sound is denotes as a not c
        and airfoil thickness is denoted with "t" and not "h"
    
    Outputs:
        SPL     (using 3 methods*)   - Overall Sound Pressure Level, [dB] 
        SPL_dBA (using 3 methods*)   - Overall Sound Pressure Level, [dBA] 
    
    Properties Used:
        N/A   
    '''
    
    # unpack 
    conditions           = segment.state.conditions
    microphone_location  = conditions.noise.microphone_locations
    angle_of_attack      = conditions.aerodynamics.angle_of_attack 
    velocity_vector      = conditions.frames.inertial.velocity_vector
    freestream           = conditions.freestream
    ctrl_pts             = len(angle_of_attack) 
    
    # create empty arrays for results  
    SPL         = np.zeros(ctrl_pts)
    SPL_v       = np.zeros_like(SPL)  
    SPL_dBA     = np.zeros_like(SPL)    
    SPL_v_dBA   = np.zeros_like(SPL)
    
    if harmonic_test.any():  
        SPL  = np.zeros((ctrl_pts,len(harmonic_test)))
        SPL_v  = np.zeros_like(SPL) 

    # loop for control points  
    for i in range(ctrl_pts):    
        total_p_pref_r_H          = []   
        total_p_pref_Hv_dBA       = []
         
        auc_opts = conditions.noise.sources[propeller].acoustic_outputs 
        
        if harmonic_test.any():
            harmonics    = harmonic_test
        else:
            harmonics    = np.arange(1,21) 
            
        num_h = len(harmonics)
          
        SPL_r_H         = np.zeros(num_h) 
        p_pref_r_H      = np.zeros_like(SPL_r_H)  
        SPL_r_H_dBA     = np.zeros_like(SPL_r_H)   
        p_pref_r_H_dBA  = np.zeros_like(SPL_r_H)
        f               = np.zeros_like(SPL_r_H)

        # -----------------------------------------------------------------------
        # Rotational Noise 
        # -----------------------------------------------------------------------        
        for h in range(num_h):
            m              = harmonics[h]                                     # harmonic number 
            p_ref          = 2e-5                                             # referece atmospheric pressure
            a              = freestream.speed_of_sound[i][0]                  # speed of sound
            rho            = freestream.density[i][0]                         # air density 
            x              = microphone_location[i,mic_loc,0]                 # x relative position from observer
            y              = microphone_location[i,mic_loc,1]                 # y relative position from observer 
            z              = microphone_location[i,mic_loc,2]                 # z relative position from observer
            Vx             = velocity_vector[i][0]                            # x velocity of propeller  
            Vy             = velocity_vector[i][1]                            # y velocity of propeller 
            Vz             = velocity_vector[i][2]                            # z velocity of propeller 
            thrust_angle   = auc_opts.thrust_angle                            # propeller thrust angle
            AoA            = angle_of_attack[i][0]                            # vehicle angle of attack                                            
            N              = auc_opts.number_of_engines                       # numner of Rotors
            B              = auc_opts.number_of_blades                        # number of rotor blades
            omega          = auc_opts.omega[i]                                # angular velocity          
            T              = auc_opts.blade_thrust[i]                         # propeller/rotor blade thrust     
            T_distribution = auc_opts.blade_thrust_distribution[i]            # propeller/rotor blade thrust distribution  
            dT_dR          = auc_opts.blade_dT_dR[i]                          # differential thrust distribution
            dT_dr          = auc_opts.blade_dT_dr[i]                          # nondimensionalized differential thrust distribution 
            Q              = auc_opts.blade_torque[i]                         # propeller/rotor blade torque    
            Q_distribution = auc_opts.blade_torque_distribution[i]            # propeller/rotor blade torque distribution  
            dQ_dR          = auc_opts.blade_dT_dR[i]                          # differential torque distribution
            dQ_dr          = auc_opts.blade_dT_dr[i]                          # nondimensionalized differential torque distribution
            R              = auc_opts.radius_distribution                     # radial location     
            c              = auc_opts.chord_distribution                      # blade chord    
            beta           = auc_opts.twist_distribution                      # twist distribution  
            t              = auc_opts.max_thickness_distribution              # twist distribution
            MCA            = auc_opts.mid_chord_aligment                      # Mid Chord Alighment 
                                                                              
            f[h]          = B*omega*m/(2*np.pi)   
            n             = len(R)
            R_tip         = R[-1]                                              # Rotor Tip Radius  
            D             = 2*R_tip                                            # propeller diameter    
            r             = R/R_tip                                            # non dimensional radius distribution 
            dR            = R[1] - R[0]                                        
            dr            = r[1] - r[0]                                        
            S             = np.sqrt(x**2 + y**2 + z**2)                        # distance between rotor and the observer    
            theta         = np.arccos(x/S)                                     
            alpha         = AoA + thrust_angle                                 
            Y             = np.sqrt(y**2 + z**2)                               # observer distance from propeller axis          
            V             = np.sqrt(Vx**2 + Vy**2 + Vz**2)                     # velocity magnitude
            M             = V/a                                                # Mach number  
            V_tip         = R_tip*omega                                        # blade_tip_speed 
            M_t           = V_tip/a                                            # tip Mach number 
            M_s           = np.sqrt(M**2 + (r**2)*(M_t**2))                    # section relative Mach number 
            r_t           = R_tip                                              # propeller tip radius
            phi           = np.arctan(z/y)                                     # tangential angle  
            theta_r       = np.arccos(np.cos(theta)*np.sqrt(1 - (M**2)*\
                            (np.sin(theta))**2) + M*(np.sin(theta))**2 )       # theta angle in the retarded reference frame
            theta_r_prime = np.arccos(np.cos(theta_r)*np.cos(alpha) + \
                            np.sin(theta_r)*np.sin(phi)*np.sin(alpha) )    
            phi_prime     = np.arccos((np.sin(theta_r)/np.sin(theta_r_prime))\
                                      *np.cos(phi))                            # phi angle relative to propeller shaft axis                                                   
            phi_s         = ((2*m*B*M_t)/(M_s*(1 - M*np.cos(theta_r))))\
                            *(MCA/D)                                           # phase lag due to sweep
            S_r           = Y/(np.sin(theta_r))                                # distance in retarded reference frame 
            k_x           = ((2*m*B*c*M_t)/(M_s*(1 - M*np.cos(theta_r))))      # wave number 
            psi_L         = np.zeros(n)
            psi_V         = np.zeros(n)
            
            for idx in range(n):
                if k_x[idx] == 0:                        
                    psi_V[idx] = 2/3                                           # normalized thickness souce transforms
                    psi_L[idx] = 1                                             # normalized loading souce transforms
                else:  
                    psi_V[idx] = (8/(k_x[idx]**2))*((2/k_x[idx])*np.sin(0.5*k_x[idx])\
                                    - np.cos(0.5*k_x[idx]))                    # normalized thickness souce transforms           
                    psi_L[idx] = (2/k_x[idx])*np.sin(0.5*k_x[idx])             # normalized loading souce transforms
                    
            # sound pressure for loading noise 
            p_mL_H = ((  1j*m*B*M_t*np.sin(theta_r)*np.exp(1j*m*B*((omega*S_r/a)+(phi_prime - np.pi/2))) )\
                      / (2*np.sqrt(2)*np.pi*Y*r_t*(1 - M*np.cos(theta_r)))  ) \
                      *np.trapz(( (   (np.cos(theta_r_prime)/(1 - M*np.cos(theta_r)))*(dT_dr) \
                                      - (1/((r**2)*M_t*r_t))*(dQ_dr)  ) * np.exp(1j*phi_s) *\
                           (jv(m*B,((m*B*r*M_t*np.sin(theta_r_prime))/(1 - M*np.cos(theta_r))))) \
                           * psi_L  ),x = r,dx = dr)
            
            p_mL_H[np.isinf(p_mL_H)] = 0
            p_mL_H = abs(p_mL_H)
            
            # sound pressure for thickness noise 
            p_mT_H = (-(rho*(a**2)*B*np.sin(theta_r)*np.exp(1j*m*B*((omega*S_r/a)+(phi_prime - np.pi/2))))\
                      /(4*np.sqrt(2)*np.pi*(Y/D)*(1 - M*np.cos(theta_r)))) \
                    *np.trapz(((M_s**2)*(t/c)*np.exp(1j*phi_s)*(jv(m*B,((m*B*r*M_t*np.sin(theta_r_prime))\
                    /(1 - M*np.cos(theta_r)))))*(k_x**2)*psi_V ),x = r,dx = dr)
            
            p_mT_H[np.isinf(p_mT_H)] = 0  
            p_mT_H  = abs(p_mT_H)
            
            # unweighted rotational sound pressure level
            SPL_r_H[h]        = 10*np.log10(N*((p_mL_H**2 + p_mT_H**2 )/(p_ref**2)))  
            p_pref_r_H[h]     = 10**(SPL_r_H[h]/10)  
            SPL_r_H_dBA[h]    = A_weighting(SPL_r_H[h],f[h])
            p_pref_r_H_dBA[h] = 10**(SPL_r_H_dBA[h]/10)   
            
        # ------------------------------------------------------------------------------------
        # Broadband Noise (Vortex Noise)   
        # ------------------------------------------------------------------------------------ 
        V_07      = V_tip*0.70/(Units.feet)                                      # blade velocity at r/R_tip = 0.7 
        St        = 0.28                                                         # Strouhal number             
        t_avg     = np.mean(t)/(Units.feet)                                      # thickness
        c_avg     = np.mean(c)/(Units.feet)                                      # average chord  
        beta_07   = beta[round(n*0.70)]                                          # blade angle of attack at r/R = 0.7
        h_val     = t_avg*np.cos(beta_07) + c_avg*np.sin(beta_07)                # projected blade thickness                   
        f_peak    = (V_07*St)/h_val                                              # V - blade velocity at a radial location of 0.7              
        A_blade   = (np.trapz(c, dx = dR))/(Units.feet**2) # area of blade      
        CL_07     = 2*np.pi*beta_07
        S_feet    = S/(Units.feet)
        SPL_300ft = 10*np.log10(((6.1E-27)*A_blade*V_07**6)/(10**-16)) + 20*np.log(CL_07/0.4)  
        SPL_v     = SPL_300ft - 20*np.log10(S_feet/300)                          # CORRECT 
        
        # estimation of A-Weighting for Vortex Noise  
        f_spectrum  = [0.5*f_peak,1*f_peak,2*f_peak,4*f_peak,8*f_peak,16*f_peak] # spectrum
        fr          = f_spectrum/f_peak                                          # frequency ratio  
        SPL_weight  = [7.92 , 4.17 , 8.33 , 8.75 ,12.92 , 13.33]                 # SPL weight
        SPL_v       = np.ones_like(SPL_weight)*SPL_v - SPL_weight                # SPL correction
        
        dim         = len(f_spectrum)
        C           = np.zeros(dim) 
        p_pref_v_dBA= np.zeros(dim-1)
        SPL_v_dbAi  = np.zeros(dim)
        
        for j in range(dim):
            SPL_v_dbAi[j] = A_weighting(SPL_v[j],f_spectrum[j])
        
        for j in range(dim-1):
            C[j]        = (SPL_v_dbAi[j+1] - SPL_v_dbAi[j])/(np.log10(fr[j+1]) - np.log10(fr[j])) 
            C[j+1]      = SPL_v_dbAi[j+1] - C[j]*np.log10(fr[j+1])   
            p_pref_v_dBA[j] = (10**(0.1*C[j+1]))* (  ((fr[j+1]**(0.1*C[j] + 1 ))/(0.1*C[j] + 1 )) - ((fr[j]**(0.1*C[j] + 1 ))/(0.1*C[j] + 1 )) )
            
            
        # collecting unweighted pressure ratios   
        total_p_pref_r_H.append(p_pref_r_H)    
        
        # collecting weighted pressure ratios with vortex noise included  
        total_p_pref_Hv_dBA.append(np.concatenate([p_pref_r_H_dBA,p_pref_v_dBA])) 
         
        SPL_v_dBA[i] = pressure_ratio_to_SPL_arithmetic(p_pref_v_dBA)
        
        if harmonic_test.any():  
            SPL[i]  = SPL_r_H 
            
        else:
            # Rotational SPL (Unweighted)     
            SPL[i]       = pressure_ratio_to_SPL_arithmetic(total_p_pref_r_H)  
            SPL_v[i]     = SPL_arithmetic(np.atleast_2d(SPL_v)) 
        
        # A- Weighted Rotational and Vortex SPL  
        SPL_dBA[i]       = pressure_ratio_to_SPL_arithmetic(total_p_pref_Hv_dBA)
    
    # total noise , adding vortex and rotational noise
    SPL_tot     =  SPL_arithmetic(np.atleast_2d(np.concatenate((SPL,SPL_v))))
    
    # convert to 1/3 octave spectrum 
    SPL_spectrum = SPL_harmonic_to_third_octave(SPL,f,settings)
    
    # Pack Results
    propeller_noise = Data() 
    propeller_noise.SPL          = SPL_tot
    propeller_noise.SPL_spectrum = SPL_spectrum
    propeller_noise.SPL_dBA      = SPL_dBA   
    
    return propeller_noise
