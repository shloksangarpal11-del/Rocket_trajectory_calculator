# Rocket_trajectory_calculator
3-DOF Atmospheric Rocket & Missile Trajectory Simulator
A Python-based 3-Degree-of-Freedom (3-DOF) dynamic flight mechanics simulator designed to model rocket trajectories under the influence of variable thrust, dynamic mass decrement, gravity, and altitude-dependent aerodynamic drag.
The project processes launch parameters in batch mode via a CSV file using Pandas, solves the governing system of coupled Ordinary Differential Equations (ODEs) with SciPy, and visualizes comparative flight trajectories with Matplotlib.

Features
Dynamic Physics Engine: Solves 2D/3-DOF equations of motion using `scipy.integrate.solve_ivp`.
Mass Decrement Modeling: Tracks propellant burn rates ($\dot{m}$) dynamically using Specific Impulse ($I_{sp}$) and thrust profiles until dry mass is reached.
Exponential Atmosphere Model: Incorporates air density decay $\rho(z)$ as a function of altitude to simulate real aerodynamic drag.
Batch Processing via CSV: Load and evaluate multiple launch profiles simultaneously using Pandas.
Impact & Flight Vector Analysis: Automatically calculates maximum altitude, downrange impact distance, total flight time, impact velocity (in Mach numbers), and terminal impact angle.
Ground Impact Event Detection: Utilizes SciPy event handling to terminate integration upon ground contact ($z \le 0$).

Installation & Setup
Prerequisites
Ensure you have Python 3.8+ installed on your system.
Install Required Libraries
Run the following command in your terminal or Command Prompt to install the dependencies:
```bash
pip install numpy pandas scipy matplotlib
```

📁 Repository Structure
```text
├── trajectory_simulator.py     # Main Python simulation script
├── trajectory_parameters.csv   # Input dataset containing launch configurations
└── README.md                   # Project documentation
```

CSV Input Format (`trajectory_parameters.csv`)
Place `trajectory_parameters.csv` in the same folder as `trajectory_simulator.py`.
Required Column Headers
Parameter	Unit	Description
`Launch_Name`	String	Identifier for the vehicle/mission
`M_Initial`	$\text{kg}$	Total vehicle mass at ignition (propellant + dry mass)
`M_Dry`	$\text{kg}$	Vehicle mass after complete propellant depletion
`Thrust`	$\text{N}$	Constant vacuum/atmospheric engine thrust
`ISP`	$\text{s}$	Specific Impulse of the engine
`Theta`	$\text{deg}$	Launch angle above horizontal ($0^\circ$ to $90^\circ$)
`Cd`	Dimensionless	Aerodynamic drag coefficient
`Area`	$\text{m}^2$	Cross-sectional reference area of the vehicle
Example Dataset
```csv
Launch_Name,M_Initial,M_Dry,Thrust,ISP,Theta,Cd,Area
Heavy_Lifter,5000,1500,150000,250,75,0.40,3.14
Supersonic_Test,2000,500,100000,280,45,0.25,0.78
RCS_Proto,10,3,150,220,85,0.40,0.05
```


🧮 Mathematical & Physical Model
1. State Vector
The trajectory solver tracks five state variables over time:

S(t) = [ x  z  v_x  v_z  m ]^T

**$x, z$: Downrange distance ($\text{m}$) and Altitude ($\text{m}$)
$v_x, v_z$: Horizontal and Vertical velocity components ($\text{m/s}$)
$m$: Instantaneous vehicle mass ($\text{kg}$)**
---
2. Mass Flow Rate
Propellant depletion rate $\dot{m}$ is calculated as:
$$\dot{m} = \frac{T}{I_{sp} \cdot g_0}$$
where $g_0 = 9.81 \text{ m/s}^2$.
Mass flow rate is active while $m > m_{\text{dry}}$; once fuel is exhausted ($m \le m_{\text{dry}}$), thrust drops to zero ($T = 0$) and $\dot{m} = 0$.
---
3. Atmospheric Density Model
Air density $\rho(z)$ decreases exponentially with altitude according to standard atmospheric scale height:
$$\rho(z) = \rho_0 \cdot e^{-z / H}$$
$\rho_0 = 1.225 \text{ kg/m}^3$ (Sea-level air density)
$H = 8500 \text{ m}$ (Atmospheric scale height)
---
4. Aerodynamic Drag Forces
Aerodynamic drag magnitude is given by:
$$F_D = \frac{1}{2} \rho(z) \cdot v^2 \cdot C_D \cdot A$$
where $v = \sqrt{v_x^2 + v_z^2}$. Drag components opposing motion are:
$$F_{D,x} = F_D \cdot \frac{v_x}{v}, \quad F_{D,z} = F_D \cdot \frac{v_z}{v}$$
---
5. System of Governing Differential Equations
The derivatives fed into `solve_ivp` are:
$$\frac{dx}{dt} = v_x$$
$$\frac{dz}{dt} = v_z$$
$$\frac{dv_x}{dt} = \frac{T \cos(\theta) - F_{D,x}}{m}$$
$$\frac{dv_z}{dt} = \frac{T \sin(\theta) - F_{D,z}}{m} - g_0$$
$$\frac{dm}{dt} = -\dot{m}$$
---
📈 Terminal Output & Metrics
Upon executing the script (`python trajectory_simulator.py`), the console prints detailed performance outputs for each vehicle configuration:
```text
==================================================
 🚀 ATMOSPHERIC TRAJECTORY SIMULATOR (3-DOF) 
==================================================
Successfully loaded 3 launch configurations from trajectory_parameters.csv

[Heavy_Lifter] Launch Profile: 75.0°
  ├─ Max Altitude:   83.91 km
  ├─ Total Range:    85.16 km
  ├─ Flight Time:    341.3 s
  ├─ Impact Vel:     0.44 Mach (149.3 m/s)
  └─ Impact Angle:   88.5° (relative to ground)

[Supersonic_Test] Launch Profile: 45.0°
  ├─ Max Altitude:   94.34 km
  ├─ Total Range:    366.17 km
  ├─ Flight Time:    325.0 s
  ├─ Impact Vel:     0.76 Mach (259.2 m/s)
  └─ Impact Angle:   67.5° (relative to ground)
```
Additionally, a high-resolution Matplotlib plot renders downrange distance against altitude for visual comparison across all test cases.

How to Run
Clone or download this repository.
Ensure your `trajectory_parameters.csv` file is populated with valid rocket parameters.
Run the Python script:
```bash
   python trajectory_simulator.py
   ```
