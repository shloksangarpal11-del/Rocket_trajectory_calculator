import numpy as np
import pandas as pd
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# --- 1. Environmental & Physics Constants ---
G = 9.81              # Gravity (m/s^2)
RHO_0 = 1.225         # Sea level air density (kg/m^3)
H_SCALE = 8500        # Atmospheric scale height (m)

# --- 2. The ODE System (Physics Engine) ---
def rocket_dynamics(t, state, thrust, m_dot, m_dry, theta, cd, area):
    x, z, vx, vz, m = state
    
    # Engine logic
    if m > m_dry:
        thrust_active = thrust
        dm_dt = -m_dot
    else:
        thrust_active = 0
        dm_dt = 0
        
    # Kinematics
    v_mag = np.sqrt(vx**2 + vz**2)
    
    # Aerodynamic Drag Calculation
    # Using an exponential atmosphere model: rho = rho_0 * e^(-h/H)
    if z < 0: 
        rho = RHO_0 # Cap density if it momentarily dips below 0 at impact
    else:
        rho = RHO_0 * np.exp(-z / H_SCALE)
        
    # F_drag = 0.5 * rho * v^2 * Cd * A
    f_drag = 0.5 * rho * (v_mag**2) * cd * area
    
    # Split drag into x and z components (opposing velocity vector)
    if v_mag != 0:
        drag_x = f_drag * (vx / v_mag)
        drag_z = f_drag * (vz / v_mag)
    else:
        drag_x = 0
        drag_z = 0
        
    # Sum of Forces = Mass * Acceleration
    ax = (thrust_active * np.cos(theta) - drag_x) / m
    az = (thrust_active * np.sin(theta) - drag_z) / m - G
    
    return [vx, vz, ax, az, dm_dt]

# --- 3. Terminal Event (Ground Impact) ---
def ground_impact(t, state, *args):
    return state[1] 

ground_impact.terminal = True  
ground_impact.direction = -1   

# --- 4. The Calculator Function ---
def calculate_trajectory(row):
    # Extract and convert parameters
    theta_rad = np.radians(row['Theta'])
    m_dot = row['Thrust'] / (row['ISP'] * G)
    
    initial_state = [0, 0, 0, 0, row['M_Initial']]
    time_span = (0, 3000) # Max 3000 seconds of flight time
    
    # Run the solver
    solution = solve_ivp(
        rocket_dynamics, 
        time_span, 
        initial_state, 
        args=(row['Thrust'], m_dot, row['M_Dry'], theta_rad, row['Cd'], row['Area']),
        events=ground_impact,
        max_step=0.1 
    )
    return solution

# --- 5. Main Execution Block ---
if __name__ == "__main__":
    
    print("\n" + "="*50)
    print(" 🚀 ATMOSPHERIC TRAJECTORY SIMULATOR (3-DOF) ")
    print("="*50)

    try:
        # Read the external CSV file
        df = pd.read_csv('trajectory_parameters.csv')
        print(f"Successfully loaded {len(df)} launch configurations from trajectory_parameters.csv")
    except FileNotFoundError:
        print("Error: 'trajectory_parameters.csv' not found.")
        print("Ensure the file is saved in the exact same folder as this Python script.")
        exit()

    plt.figure(figsize=(12, 6))
    
    # Process each configuration
    for index, row in df.iterrows():
        name = row['Launch_Name']
        sol = calculate_trajectory(row)
        
        # Extract terminal conditions
        x_final = sol.y[0][-1]
        z_max = np.max(sol.y[1])
        t_final = sol.t[-1]
        vx_final = sol.y[2][-1]
        vz_final = sol.y[3][-1]
        
        # Calculate impact metrics
        impact_velocity = np.sqrt(vx_final**2 + vz_final**2)
        impact_angle = np.degrees(np.arctan2(vz_final, vx_final))
        
        # Print structured output to terminal
        print(f"\n[{name}] Launch Profile: {row['Theta']}°")
        print(f"  ├─ Max Altitude:   {z_max/1000:.2f} km")
        print(f"  ├─ Total Range:    {x_final/1000:.2f} km")
        print(f"  ├─ Flight Time:    {t_final:.1f} s")
        print(f"  ├─ Impact Vel:     {impact_velocity/343:.2f} Mach ({impact_velocity:.1f} m/s)")
        print(f"  └─ Impact Angle:   {abs(impact_angle):.1f}° (relative to ground)")
        
        # Plotting
        plt.plot(sol.y[0]/1000, sol.y[1]/1000, label=f"{name} ({row['Theta']}°)", linewidth=2)

    # Render Visuals
    plt.title('Rocket Trajectory Comparison (Atmospheric Drag Included)', fontsize=14, fontweight='bold')
    plt.xlabel('Downrange Distance (km)', fontsize=12)
    plt.ylabel('Altitude (km)', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.axhline(0, color='black', linewidth=1.5)
    
    plt.ylim(bottom=0)
    plt.xlim(left=0)
    plt.legend(loc='upper right', framealpha=1)
    
    print("\nRendering visualization...")
    plt.tight_layout()
    plt.show()
