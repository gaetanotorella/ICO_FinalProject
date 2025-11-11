# Model Predictive Control for Powered Descent Guidance and Control

This project develops a **Model Predictive Control (MPC)** framework for the **powered descent phase** of a spacecraft or reusable launch vehicle. The study focuses on the terminal phase of planetary landing, where thrust must be modulated to achieve a safe and precise touchdown while respecting physical and operational constraints. The implementation is carried out in **MATLAB** using the **YALMIP** optimization toolbox, based on a simplified **6-DoF linearized model** of a thrust-vectored rocket.

---

## Objective

The objective is to design and simulate an **autonomous guidance and control law** capable of performing precise powered descent under explicit state and input constraints.  
The controller continuously optimizes thrust and attitude commands to minimize the landing error, reduce fuel consumption, and ensure smooth, stable motion within feasible actuator limits.

The main goals include:

1. **Modeling the spacecraft dynamics** through a rigid-body formulation including translational and rotational motion under thrust vectoring.  
2. **Designing an MPC scheme** that predicts future states and selects optimal control inputs over a receding horizon.  
3. **Demonstrating closed-loop stability** and constraint satisfaction during a simulated descent toward a planetary surface.

---

## Spacecraft Model

The system represents a **6-degree-of-freedom rocket lander** actuated by thrust vectoring.  
Key characteristics:

- Translational dynamics governed by thrust and gravity.  
- Rotational dynamics defined by roll, pitch, and yaw moments.  
- Four control inputs: roll, pitch, yaw torques, and thrust force.  
- Simplified assumptions:
  - Flat surface and uniform gravity.  
  - Negligible aerodynamic effects.  
  - Diagonal inertia matrix and decoupled axes.  

Linearization around a **hover equilibrium** (vertical attitude, thrust balancing weight) yields a **linear time-invariant (LTI)** model suitable for MPC prediction.  
The resulting discrete-time model captures small perturbations in position, velocity, and attitude during descent.

---

## Model Predictive Control Strategy

MPC solves a **finite-horizon quadratic optimization problem** at each time step. The predictive model forecasts state evolution, and an optimal control sequence is chosen to minimize a cost function while enforcing constraints.

### Cost Function

The cost integrates two competing objectives:

- **State regulation:** minimize deviation from the desired landing state (zero position, velocity, and attitude errors).  
- **Control effort:** penalize large torque or thrust variations to achieve fuel-efficient maneuvers.

The trade-off between tracking accuracy and control smoothness is tuned by weighting matrices \( Q \) and \( R \).  
High weights on vertical position and velocity improve touchdown precision, while higher attitude weights limit excessive tilt.

### Constraints

Explicit constraints are embedded in the MPC problem:

- **Input bounds:** limits on thrust magnitude and attitude torques.  
- **State bounds:** maximum roll, pitch, and yaw angles.  
- **Altitude constraint:** altitude must remain nonnegative to avoid surface penetration.

Soft constraints are applied to attitude limits using slack variables to preserve feasibility under near-saturation conditions.

### Advantages

- Handles multivariable interactions and constraints directly.  
- Predicts future motion to anticipate braking and tilt maneuvers.  
- Provides smoother and more accurate trajectories than PID or LQR approaches.  
- Naturally integrates actuator and safety limits into control synthesis.

### Limitations

- Computational load due to repeated optimization at each time step.  
- Sensitivity to model mismatch and parameter uncertainty.  
- Weight tuning can affect performance and landing aggressiveness.

---

## Implementation Environment

The controller is implemented in **MATLAB** using **YALMIP**, a modeling toolbox for convex optimization.  
The workflow involves:

1. Definition and discretization of the linearized dynamics.  
2. Formulation of the MPC cost and constraint set.  
3. Use of quadratic programming solvers (e.g., SeDuMi, SDPT3) for online optimization.  
4. Closed-loop simulation with stepwise control updates.  

The simulation reproduces a representative descent trajectory from an initial altitude and velocity toward a soft landing.  
Position, velocity, and attitude histories are recorded, together with control and acceleration profiles.

---

## Evaluation and Results

Simulation results demonstrate:

- Stable convergence of the spacecraft to the landing point.  
- Smooth thrust modulation without oscillations.  
- Attitude angles remain within operational limits.  
- Velocity components decay monotonically to zero at touchdown.  
- Control profiles reflect realistic thrust and torque levels suitable for implementation.

The results confirm that MPC ensures both **trajectory precision** and **constraint satisfaction**, validating its suitability for autonomous landing guidance and reusable vehicle recovery.

---

## Dependencies

- **MATLAB / Simulink**  
- **YALMIP** (for MPC optimization modeling)  
- **QP/SOCP solver** (e.g., SeDuMi, SDPT3, Gurobi)  
- **Custom visualization scripts** for trajectory and state evolution plotting  

---

## Demo Video

### Landing
https://github.com/user-attachments/assets/03c1a3af-bbb0-41de-977c-2db78f0a6703

---

## References

1. C. A. Pascucci, S. Bennani, A. Bemporad, *“Model Predictive Control for Powered Descent Guidance and Control”*, **European Control Conference**, 2015.  
2. G. Zaragoza Prous et al., *“A Decreasing Horizon MPC for Landing Reusable Launch Vehicles”*, **Aerospace**, 2025.  
3. E. N. Hartley, *“A Tutorial on Model Predictive Control for Spacecraft Rendezvous”*, **ECC**, 2015.  
4. J. Löfberg, *“YALMIP: A Toolbox for Modeling and Optimization in MATLAB”*, 2004.

---
