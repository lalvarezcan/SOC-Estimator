# README.md

**Title:** State-of-Charge Estimation for 4-Cell Li-Ion Battery Using Simscape Lookup Tables and EKF  
**Authors:** [Your Name]  
**Date:** [Date]  

---

## Abstract  
This repository implements a first-order equivalent circuit model of a 4-cell Li-ion battery pack using lookup tables for all electrochemical parameters (OCV, $R_0$, $R_1$, $C_1$) as demonstrated in the MathWorks example. An Extended Kalman Filter (EKF) is designed for real-time state-of-charge (SOC) estimation. The project includes:

- Table-based parameterization following the *Estimate Battery SOC Using Kalman Filter* example (MathWorks).  
- Simulink and MATLAB scripts for lookup-table generation and EKF implementation.  
- Simulation results and guidance for embedded deployment.

## Keywords  
Li-ion, Battery, Equivalent Circuit, Lookup Table, Kalman Filter, SOC Estimation, Simulink, Embedded

---

## 1. Introduction  
Reliable SOC estimation ensures safe and efficient battery operation. MathWorks provides a standard workflow using Simscape’s table-based battery model with lookup tables for temperature- and SOC-dependent parameters, paired with an EKF for SOC estimation. This repository adapts that workflow to a 4-cell pack.

---

## 2. Lookup-Table-Based Equivalent Circuit Model  
### 2.1 Model Overview  
This project uses the **Simscape Battery (Table-Based)** block configured with the following tables at multiple temperatures (e.g., 5 °C, 20 °C, 40 °C):  
- **Open-Circuit Voltage (OCV)** vs. SOC  
- **Ohmic Resistance** $R_0$ vs. SOC  
- **Polarization Resistance** $R_1$ vs. SOC  
- **Polarization Capacitance** $C_1$ vs. SOC  

By using lookup tables rather than closed-form fits, the model automatically interpolates values for intermediate SOC or temperature conditions, matching the approach in the MathWorks example.

The equivalent circuit is a 1-RC Thevenin model per cell, replicated for a 4-cell series pack. Each cell’s terminal voltage is:  
```math
U_{t,i} = U_{OCV,i}(SOC_i, T) - I \\cdot R_0(SOC_i, T) - U_{1,i},
``` 
with dynamics:  
```math
\dot{SOC}_i = -\frac{I}{Q_{nom}}, \quad \dot{U}_{1,i} = -\frac{1}{R_1(SOC_i,T) C_1(SOC_i,T)} U_{1,i} + \frac{1}{C_1(SOC_i,T)} I
```
### 2.2 Parameter Table Generation  
- **Data Sources:** Use HPPC test data or manufacturer curves to populate tables.  
- **Script:** `generate_lookup_tables.m` reads raw test data and outputs 2-D tables (SOC × Temperature) for OCV, $R_0$, $R_1$, $C_1$.  
- **Validation:** Plot and compare interpolated values to measurements as shown in `model_parameters.m`.

---

## 3. EKF Observer Design  
### 3.1 Nonlinear State-Space Formulation  
Define per-cell state vector $x_i = [SOC_i, U_{1,i}]^T$, input $u=I$, and output $y_i=U_{t,i}$. Dynamics and measurement follow from Section 2.1 with lookup-table calls in place of analytic parameter functions.

### 3.2 Discretization and Linearization  
- Use a fixed sample time $T_s$ (e.g., 1 s).  
- Discretize dynamics via Euler or exact methods in MATLAB Function blocks.  
- Compute Jacobians $F_k = \\partial f/\\partial x$, $H_k = \\partial h/\\partial x$ numerically.

### 3.3 Noise Covariances  
- Process noise $Q$ for model mismatch and Coulomb counting errors.  
- Measurement noise $R$ from voltage sensor resolution.  
- Tuning guidance follows the MathWorks example.

### 3.4 EKF Implementation  
Use the **Extended Kalman Filter** block from the Control System Toolbox. Specify:  
- **State transition** (`StateFcn`): MATLAB Function block implementing the discrete lookup-table model.  
- **Measurement** (`OutputFcn`): returns cell voltages from the model.  
- **Jacobian functions**: numerical approximations or inline functions.  
- **Noise matrices** $Q$, $R$.

---

## 4. Simulation Setup  
- **Environment:** MATLAB R2024a, Simulink, Simscape Battery library.  
- **Files:**  
  - `generate_lookup_tables.m`  
  - `model_parameters.m`  
  - `battery_pack_model.slx`  
  - `ekf_observer.slx`  
- **Profiles:** Standard HPPC and dynamic drive cycles at various temperatures.

---

## 5. Results  
- SOC estimation error plots vs. true SOC.  
- RMSE across temperature conditions.  
- Convergence speed from different initial errors.

---

## 6. Embedded Deployment  
- **Code Generation:** Configure Embedded Coder for fixed-step target.  
- **Fixed-Point Support:** Convert lookup-table access and EKF to fixed-point if necessary.  
- **Testing:** Conduct SIL and HIL using Simulink Test and hardware interface.

---

## 7. Conclusion  
This repository demonstrates the MathWorks recommended approach—using table-based parameterization and an EKF block—for robust SOC estimation in a multi-cell Li-ion pack.

---

## References  
- MathWorks: *Estimate Battery SOC Using Kalman Filter* example.  
- Simscape Battery User’s Guide.

---

## Repository Structure 
```
├── model_parameters.m       # load and plot R0,R1,C1 maps
├── battery_model.slx        # Simulink battery pack model
├── ekf_observer.slx         # EKF implementation
├── data/                    # HPPC test data and lookup tables
└── README.md                # this document
```
