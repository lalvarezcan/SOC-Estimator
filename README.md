# README.md

**Title:** State-of-Charge Estimation for 4-Cell NCA Li-Ion Battery  
**Authors:** [Your Name]  
**Date:** [Date]  

---

## Abstract  
This repository provides a complete implementation of a state-based equivalent circuit model for a 4-cell NCA Li-ion battery, along with an Extended Kalman Filter (EKF) observer for state-of-charge (SOC) estimation. The model incorporates SOC- and temperature-dependent parameters (R0, R1, C1) and is implemented in MATLAB/Simulink. The EKF observer design, parameter identification, simulation results, and guidelines for embedded deployment are documented.

## Keywords  
Li-ion, NCA, Equivalent Circuit Model, State-of-Charge, Kalman Filter, Observer, Simulink, Embedded

---

## 1. Introduction  
Accurate SOC estimation is critical for battery safety and performance. This work presents:

1. A first-order Thevenin equivalent circuit model for a 4-cell NCA battery with full SOC & temperature dependencies.  
2. Parameterization procedures from HPPC data.  
3. Design and implementation of an EKF-based SOC observer.  
4. Simulation case studies in Simulink.  
5. Considerations for real-time embedded deployment.

---

## 2. Equivalent Circuit Modeling  
### 2.1 Model Structure  
- **Open-Circuit Voltage (OCV):** $U_{OCV}(SOC)$ via polynomial or lookup table.  
- **Ohmic Resistance $R_0(SOC,T)$:** Arrhenius-scaled polynomial.  
- **Polarization RC Network:** $R_1(SOC,T)$ and $C_1(SOC,T)$ for transient dynamics.  
- **State Equations:**  
  $ \dot{SOC} = -\frac{I}{Q_{nom}}, \quad \dot{U}_1 = -\frac{1}{R_1 C_1} U_1 + \frac{1}{C_1} I $ 
- **Output Equation:**  
  $U_t = U_{OCV}(SOC) - I R_0 - U_1$

### 2.2 Parameter Identification  
- HPPC tests at multiple SOC points and temperatures.  
- Fit coefficients $a_i,b_i,c_i$ and activation energies $E_a$.  
- Generate 2D lookup tables or closed-form expressions.

---

## 3. Observer Design: Extended Kalman Filter  
### 3.1 Nonlinear State-Space Formulation  
- **States:** $[SOC, U_1]^T$  
- **Input:** Current $I$  
- **Output:** Terminal voltage $U_t$  
- **Dynamics:** Defined in Section 2.1.

### 3.2 Discretization  
- Sample time $T_s$ using forward Euler or exact discretization.  
- Discrete state-transition $x_{k+1} = f_d(x_k,u_k)$ and output $y_k = h(x_k,u_k)$.

### 3.3 Jacobians  
- **State Jacobian** $F_k = \partial f_d/\partial x$  
- **Output Jacobian** $H_k = \partial h/\partial x$  
- Compute analytically or numerically.

### 3.4 Noise Covariances  
- Process noise $Q$ for model uncertainty and Coulomb-counting error.  
- Measurement noise $R$ from voltage sensor.  
- Guidelines for tuning.

### 3.5 EKF Algorithm  
Predict and update equations:  
```matlab
% Predict
x̂_{k+1|k} = f_d(x̂_{k|k}, u_k);
P_{k+1|k} = F_k * P_{k|k} * F_k' + Q;

% Update
K_{k+1} = P_{k+1|k} * H_{k+1}' * (H_{k+1} * P_{k+1|k} * H_{k+1}' + R)^-1;
x̂_{k+1|k+1} = x̂_{k+1|k} + K_{k+1} * (y_{k+1} - h(x̂_{k+1|k}, u_k));
P_{k+1|k+1} = (eye(size(K_{k+1} * H_{k+1})) - K_{k+1} * H_{k+1}) * P_{k+1|k};
```

---

## 4. Simulation Setup  
- **Environment:** MATLAB R2024a, Simulink, Simscape Battery blocks.  
- **Scripts:**  
  - `model_parameters.m`: loads/plots R0,R1,C1 maps.  
  - `battery_model.slx`: cell and pack model.  
  - `ekf_observer.slx`: EKF block implementation.  
- **Tests:**  
  - HPPC profile at various temperatures.  
  - Dynamic load cycles.

---

## 5. Results  
- Convergence of SOC estimate from 20% error.  
- Performance metrics: RMSE vs true SOC, response to transients.  
- Sensitivity to $Q,R$ tuning.

---

## 6. Embedded Deployment  
- **Code Generation:** Simulink Coder settings for fixed-step.  
- **Fixed-Point:** guidelines for quantization (if no FPU).  
- **Real-Time Testing:** SIL and HIL workflows.  
- **Initialization:** OCV-based or memory recall.

---

## 7. Conclusion  
A robust EKF observer based on a first-order RC model has been developed and tested. The repository facilitates replication, extension, and deployment in BMS applications.

---

## References  
1. Tran *et al.* (2021), Parameterization of NCA EIS models.  
2. Zhang *et al.* (2018), Temperature effects on EC parameters.  
3. MathWorks, Simscape Battery Documentation.  
4. Hu *et al.* (2010), Adaptive Luenberger Observer.  
5. Lagraoui *et al.* (2015), Luenberger vs EKF comparison.

---

## Repository Structure  
```
├── model_parameters.m       # load and plot R0,R1,C1 maps
├── battery_model.slx        # Simulink battery pack model
├── ekf_observer.slx         # EKF implementation
├── data/                    # HPPC test data and lookup tables
└── README.md                # this document
```
