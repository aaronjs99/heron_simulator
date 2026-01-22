# Heron Simulation — Hydrodynamics and Vessel Dynamics

Technical documentation for the physical vessel model and environmental interactions of the Heron USV simulation within the SLAM GRANDE framework.

---

## 1. Physical Representation and Hydrostatics

The Heron Unmanned Surface Vessel (USV) simulation utilizes a rigid-body representation with integrated hydrostatic and hydrodynamic force calculation modules. This approach ensures high-fidelity behavior for surface operations, autonomous navigation testing, and SLAM validation.

### 1.1 Buoyancy and Stability
Hydrostatic buoyancy is modeled according to 3D displaced volume integration. The vessel's transverse and longitudinal stability is governed by calculated metacentric heights ($GM$), ensuring appropriate restorative moments during pitching and rolling operations.

| Parameter | Metric Value | Description |
|-----------|--------------|-------------|
| Normal Draft | 0.02 m | Submerged height at equilibrium |
| Hull Height | 0.32 m | Total vertical extent of the hull structure |
| Surface Area | Optimized | Area utilized for hydrostatic pressure integration |

### 1.2 Coordinate Frames
The simulation adheres to standard ENU (East-North-Up) conventions for world coordinates, with body-fixed coordinates following the ISO 10303 convention (X-forward, Y-left, Z-up).

---

## 2. Hydrodynamic Force Modeling

Hydrodynamic effects are calculated using a quasi-static approach that accounts for viscous drag and added mass effects.

### 2.1 Damping Coefficients
Vessel damping is modeled as a summation of linear and quadratic components for each degree of freedom (DOF):

$$F_{drag} = -(L \cdot v + Q \cdot v|v|)$$

Where:
- $v$ is the instantaneous velocity in the respective DOF.
- $L$ is the linear damping coefficient (dominates at low $Re$).
- $Q$ is the quadratic damping coefficient (dominates at high $Re$).

These coefficients have been empirically tuned to match the operational performance envelope of the physical Heron platform.

---

## 3. Propulsion and Thruster Modeling

The simulation implements a decentralized thruster model that translates normalized control efforts into physical Newtonian forces.

### 3.1 Empirical Thrust Curve
Thruster output is governed by a non-linear mapping derived from stationary bollard pull tests and acceleration trials. The simulation uses linear interpolation across measured data points to ensure realistic propulsion characteristics.

| Input (Normalized) | Surge Force ($F_x$) [N] |
|-------------------|-------------------------|
| -1.0 (Full Astern) | -19.88 |
| 0.0 (Neutral) | 0.00 |
| 1.0 (Full Ahead) | 33.60 |

### 3.2 Actuator Dynamics
The `cmd_drive_translate` module serves as the primary bridge between the high-level autonomy stack (heron_controller) and the Gazebo physics engine. It performs real-time interpolation of the 11-point empirical thrust model and applies resultant Wrench vectors to the thruster links.

---

## 4. Environmental Forcing

### 4.1 Surface Currents
Oceanic and lacustrine currents can be simulated by applying global force vectors within the world configuration. The simulation supports both constant and spatially-varying current maps to test navigation robustness.

### 4.2 Sea State and Waves
The simulation supports environmental wave modeling to evaluate sensor stability and motion compensation algorithms. Wave parameters (amplitude, period, and direction) are configurable to simulate conditions ranging from calm inland waters to moderate sea states.

---

## 5. Technical Validation

The fidelity of this hydrodynamic model has been verified against experimental data from field trials conducted at Columbia Lake and other maritime test sites. The SLAM GRANDE simulation environment provides a critical bridge between theoretical algorithm design and physical deployment.

## References
1. **Fossen, T. I. (2011).** *Handbook of Marine Craft Hydrodynamics and Motion Control*. John Wiley & Sons.
2. **Clearpath Robotics.** *Heron USV Technical Specifications and Operational Guidelines*.
3. **SLAM GRANDE Research Group.** *Empirical Validation of USV Simulation Parameters via Bollard Pull Testing*.

