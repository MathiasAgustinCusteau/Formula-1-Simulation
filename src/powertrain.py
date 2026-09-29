#%%
#===================================================#
#   Model of a 1.6L V6 internal combustion engine   #
#===================================================#

import numpy as np
from scipy.interpolate import PchipInterpolator
import plotly.graph_objects as go

#Approximated empirical RPM-Torque data
base_rpm = [4000,6000,8000,10500,12000,14000,15000]
base_torque = [380,395,410,430,362,294,266]

t_max= PchipInterpolator(base_rpm, base_torque)

def ice_torque(actual_rpm, alfa_pedal):
    # 1. base variables
    actual_rpm = np.clip(actual_rpm, 4000, 15000) #mas adelante esta linea deberia irse ya que actual_rpm deberia ya estar clipeado
    v_piston = (2 * 0.053 * actual_rpm) / 60.0
    
    # Termodynamic constants
    K = (1e5 * 1.6e-3) / (4 * np.pi)
    p_0 = 50.0
    p_max_bar = 250.0
    
    # ==========================================
    # 2. Idle alfa value calculation
    # ==========================================
    t_friccion_base = K * (0.25 + 0.004 * p_0 + 0.01 * v_piston + 0.0012 * (v_piston**2))
    delta_t_friccion = K * (0.004 * (p_max_bar - p_0))
    
    a = delta_t_friccion
    b = t_max(actual_rpm) - delta_t_friccion + t_friccion_base
    c = -t_friccion_base
    
    # Positive root
    alfa_idle = (-b + np.sqrt(b**2 - 4*a*c)) / (2*a)
    
    # ==========================================
    # 3. Real alfa
    # ==========================================
    if actual_rpm <= 4000*1.01: 
        alfa_real = max(alfa_pedal, alfa_idle)
    else:
        alfa_real = alfa_pedal
        
    # ==========================================
    # 4. Final calculation
    # ==========================================
    p_real = p_0 + alfa_real * (p_max_bar - p_0)
    fmep_real = 0.25 + 0.004 * p_real + 0.01 * v_piston + 0.0012 * (v_piston**2)
    t_friction_real = (fmep_real * 10**5 * 1.6e-3) / (4 * np.pi)
    
    # Torque neto final
    t_ice = alfa_real * t_max(actual_rpm) - (1 - alfa_real) * t_friction_real
    
    return float(t_ice)

import numpy as np

def mguk_torque(soc, ot_state, alpha, beta, actual_rpm, deploy_map, dt, lap_harvested_energy):
    """
    Calculates the MGU-K (Kinetic Motor Generator Unit) torque and energy delta.
    
    Parameters:
    soc (float): State of Charge in Joules.
    ot_state (int): Overtake button state (1 for active, 0 for inactive).
    alpha (float): Throttle pedal input [0.0 to 1.0].
    beta (float): Brake pedal input [0.0 to 1.0].
    actual_rpm (float): Current Engine RPM.
    deploy_map (float): ECU deployment strategy aggressiveness [0.0 to 1.0].
    dt (float): Integration time step in seconds.
    
    Returns:
    tuple: (net_torque_nm, energy_delta_joules)
    """
    p_max = 120000.0  # Maximum power in Watts (120 kW regulation limit)
    max_energy = 4000000.0  # 4 MJ capacity per lap
    w = actual_rpm * (np.pi / 30.0)  # Convert RPM to rad/s
    eff = 0.95  # Electrical/Mechanical efficiency
    max_harvest_lap = 2000000
    
    # ==========================================
    # 1. HARVEST STATE (Absolute Priority)
    # Triggers on braking or Lift & Coast
    # ==========================================
    if beta > 0 or (alpha == 0 and beta == 0): 
        if soc >= max_energy or lap_harvested_energy >= max_harvest_lap: 
            return 0.0, 0.0
            
        required_t = 30.0 + (beta * 170.0)
        expected_p = required_t * w
        recovered_p = min(expected_p, p_max)
        

        incoming_energy = recovered_p * eff * dt
        if soc + incoming_energy > max_energy:
            incoming_energy = max_energy - soc

            recovered_p = incoming_energy / (eff * dt)

        if lap_harvested_energy + incoming_energy > max_harvest_lap:
            incoming_energy = max_harvest_lap - lap_harvested_energy

            recovered_p = incoming_energy / (eff * dt)
            
        t_mguk = -(recovered_p / w)
        return float(t_mguk), float(incoming_energy)

    # ==========================================
    # 2. DEPLOY STATE
    # Triggers only with throttle input and available energy
    # ==========================================
    elif alpha > 0 and soc > 0:
        if ot_state == 1:
            p_deploy = p_max
        else:
            p_deploy = deploy_map * p_max

        required_energy = p_deploy * dt
        

        if soc < required_energy:
            deployed_energy = soc
            p_deploy = deployed_energy / dt 
        else:
            deployed_energy = required_energy


        theoretical_t_mguk = (p_deploy * eff) / w
        t_mguk = min(theoretical_t_mguk, 200.0) # Cap the maximum output due to rules and safety
        
        return float(t_mguk), float(-deployed_energy)

    # ==========================================
    # 3. NEUTRAL STATE
    # Throttle applied but battery is completely empty
    # ==========================================
    return 0.0, 0.0


