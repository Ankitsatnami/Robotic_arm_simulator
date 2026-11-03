# Fanuc Cobot Kinematics Playground
### A Real-Time 6-DOF Robotic Arm Simulation & Kinematics Research Platform

[![Live Demo](https://img.shields.io/badge/Live%20Demo-GitHub%20Pages-brightgreen?style=for-the-badge&logo=github)](https://ankitsatnami.github.io/Robotic_arm_simulator/)
[![Platform](https://img.shields.io/badge/Platform-WebGL%20%7C%20Three.js-blue?style=for-the-badge)](https://ankitsatnami.github.io/Robotic_arm_simulator/)
[![Author](https://img.shields.io/badge/Author-Ankit%20Satnami-orange?style=for-the-badge&logo=github)](https://github.com/Ankitsatnami)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](LICENSE)

---

## 🔬 Project Overview & Abstract

**Fanuc Cobot Kinematics Playground** is an interactive, browser-based 3D robotics simulation and kinematics verification platform developed by **Ankit Satnami**. The platform implements rigorous forward kinematics (FK), closed-form analytical inverse kinematics (IK) yielding all 8 distinct geometric configurations, differential kinematics via a real-time $6 \times 6$ geometric Jacobian, singularity diagnostics, trajectory interpolation (MOVEJ and MOVEL), and multi-platform controller code synthesis.

Designed as an open-access research and educational testbed, the platform provides real-time visualization of Denavit-Hartenberg (DH) coordinate transformations, Yoshikawa manipulability indices, and physical robot workspace envelopes.

- **Primary Live Deployment**: [https://ankitsatnami.github.io/Robotic_arm_simulator/](https://ankitsatnami.github.io/Robotic_arm_simulator/)
- **Mirror Deployment**: [https://eight-kitchen-mngsz.ship.place/](https://eight-kitchen-mngsz.ship.place/)
- **GitHub Repository**: [https://github.com/Ankitsatnami/Robotic_arm_simulator](https://github.com/Ankitsatnami/Robotic_arm_simulator)
- **Principal Author**: [Ankit Satnami](https://github.com/Ankitsatnami)

---

## 📐 Mathematical & Kinematic Formulations

### 1. Forward Kinematics (Denavit-Hartenberg Formulation)
For each joint link $i \in \{1, \dots, 6\}$, the relative spatial transformation between coordinate frame $\{i-1\}$ and frame $\{i\}$ is described by the standard Denavit-Hartenberg (DH) homogeneous transformation matrix $A_i \in \mathrm{SE}(3)$:

$$
A_i = \mathrm{Rot}_{z}(\theta_i) \cdot \mathrm{Trans}_{z}(d_i) \cdot \mathrm{Trans}_{x}(a_i) \cdot \mathrm{Rot}_{x}(\alpha_i)
$$

$$
A_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

The cumulative end-effector pose $T_0^6$ relative to the base frame is computed through chained post-multiplication:

$$
T_0^6 = \prod_{i=1}^{6} A_i(\theta_i) = \begin{bmatrix}
R_{3 \times 3} & \mathbf{p}_{3 \times 1} \\
\mathbf{0}_{1 \times 3} & 1
\end{bmatrix}
$$

where $R \in \mathrm{SO}(3)$ represents the orientation matrix (parameterized into Fanuc standard Yaw-Pitch-Roll $W$-$P$-$R$ angles) and $\mathbf{p} = [X, Y, Z]^T$ represents tool center position in millimeters.

---

### 2. Analytical Inverse Kinematics (Pieper's Decoupling Criterion)
For 6-DOF industrial manipulators featuring a spherical wrist where axes 4, 5, and 6 intersect at a single point (the wrist center $P_w$), the inverse kinematics problem decouples into two independent sub-problems:

1. **Position Inverse Kinematics**: The wrist center coordinates $\mathbf{p}_w = [x_w, y_w, z_w]^T$ are extracted by offsetting the target position by tool length $d_6$ along the approach axis $\mathbf{a}$:
   $$\mathbf{p}_w = \mathbf{p} - d_6 R \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$
   Joints $\theta_1, \theta_2, \theta_3$ are then analytically solved via geometric triangulation.
2. **Orientation Inverse Kinematics**: With $R_0^3(\theta_1, \theta_2, \theta_3)$ computed, the wrist orientation matrix is isolated:
   $$R_3^6 = (R_0^3)^T R$$
   Spherical wrist angles $\theta_4, \theta_5, \theta_6$ are derived using an algebraic Euler Z-Y-Z decomposition.

#### The 8 Closed-Form Kinematic Configurations:
The solver analytically produces and displays all 8 possible branch solutions:

| Configuration Code | Shoulder ($J_1 / J_2$) | Elbow ($J_3$) | Wrist ($J_5$) | Description |
|---|---|---|---|---|
| **F D T, 0, 0, 0** | Front / Top | Down | Flip ($\theta_5 > 0$) | Standard forward reach, flipped wrist |
| **N D T, 0, 0, 0** | Front / Top | Down | Non-flip ($\theta_5 \le 0$) | **Nominal FANUC CRX-10iA pose** |
| **F U T, 0, 0, 0** | Front / Top | Up | Flip | Overhead reach, elbow elevated |
| **N U T, 0, 0, 0** | Front / Top | Up | Non-flip | Elbow up, inverted wrist |
| **F U B, 0, 0, 0** | Back / Bottom | Up | Flip | Backward workspace sector reach |
| **N U B, 0, 0, 0** | Back / Bottom | Up | Non-flip | Backward elbow up |
| **F D B, 0, 0, 0** | Back / Bottom | Down | Flip | Backward reach, elbow down |
| **N D B, 0, 0, 0** | Back / Bottom | Down | Non-flip | Backward low profile |

Users can click any configuration pill to smoothly interpolate the physical arm between solutions using dynamic shortest-path joint interpolation.

---

### 3. Geometric Jacobian & Singularity Analysis
The velocity kinematics relation maps joint rates $\dot{\mathbf{q}} \in \mathbb{R}^6$ to task-space linear velocity $\mathbf{v}$ and angular velocity $\boldsymbol{\omega}$:

$$
\begin{bmatrix} \mathbf{v} \\ \boldsymbol{\omega} \end{bmatrix} = J(\mathbf{q}) \dot{\mathbf{q}} = \begin{bmatrix} J_v(\mathbf{q}) \\ J_\omega(\mathbf{q}) \end{bmatrix} \dot{\mathbf{q}}
$$

For revolute joints, each column $i$ of the geometric Jacobian $J_i$ is computed from link transformation history:

$$
J_i = \begin{bmatrix} \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}) \\ \mathbf{z}_{i-1} \end{bmatrix}
$$

#### Singularity Diagnostics & Manipulability:
Kinematic singularities occur when $\det(J) \to 0$, causing loss of instantaneous degrees of freedom and infinite joint velocities for finite Cartesian velocities. The platform continuously monitors:
- **Wrist Singularity**: Alignment of axes 4 and 6 ($\theta_5 \to 0$).
- **Elbow Singularity**: Full extension or folding of Link 2 and Link 3 ($\theta_3 \to 0 \text{ or } \pm 180^\circ$).
- **Shoulder Singularity**: Wrist center crossing the $J_1$ azimuth rotation axis.
- **Yoshikawa's Manipulability Index**:
  $$\mu(\mathbf{q}) = \sqrt{\det(J(\mathbf{q}) J^T(\mathbf{q}))}$$
  Displayed live in the HUD with automated color-coded alerts (`Nominal`, `Warning`, `Singular`).

---

## 🤖 Supported Manipulator Profiles

The simulator provides parameterized kinematic definitions, standard DH parameters, payload capacities, and maximum joint velocity specs:

| Robot Manipulator | Kinematic Type | Reach (mm) | Payload | Repeatability | Weight |
|---|---|---|---|---|---|
| **Fanuc CRX-10iA** | 6-DOF Collaborative | 1249 mm | 10 kg | $\pm 0.04$ mm | 40 kg |
| **Universal Robots UR5e** | 6-DOF Collaborative | 850 mm | 5 kg | $\pm 0.03$ mm | 20.6 kg |
| **KUKA KR6 R900 (Agilus)** | 6-DOF High-Speed Industrial | 901 mm | 6 kg | $\pm 0.02$ mm | 52 kg |
| **Unimation PUMA 560** | 6-DOF Classic Academic Standard | 878 mm | 4 kg | $\pm 0.10$ mm | 53 kg |

---

## 🛠️ System Architecture

```
Robotic_arm_simulator/
├── index.html            # Core WebGL 3D Simulator application
├── README.md             # Research documentation & technical reference
├── LICENSE               # MIT Open-Source License
├── css/
│   └── style.css         # Responsive dark industrial UI theme
├── js/
│   ├── kinematics.js     # DH transformation engine, 8-solution IK & Jacobian
│   ├── robot-model.js    # Procedural PBR robot geometries, joints & gripper
│   ├── trajectory.js     # MOVEJ / MOVEL motion interpolator & sequencer
│   ├── learning.js       # Interactive curricula & code generator
│   ├── main.js           # Three.js orchestrator & DOM event dispatcher
│   ├── three.min.js      # Three.js 3D library (bundled)
│   ├── OrbitControls.js  # Camera navigation controls
│   └── gsap.min.js       # Smooth tweening and interpolation engine
├── server.ps1            # Local HTTP server script (PowerShell)
└── start_simulator.bat   # Local execution launcher
```

---

## 💻 Multi-Platform Code Synthesis

The platform features an automated code generation engine that converts the active simulation state into deployable controller scripts:

* **Python (NumPy / SciPy)**: Analytic forward kinematics and transformation matrix validation scripts.
* **FANUC Karel / Teach Pendant (TP)**: Native industrial controller position registers (`PR[1]`) with config flags.
* **C++ (ROS 2 MoveIt 2)**: Standard ROS 2 geometry messages (`geometry_msgs::msg::PoseStamped`) and trajectory waypoints.
* **MATLAB Robotics System Toolbox**: RigidBodyTree definitions and kinematics validation functions.

---

## 🌐 Deployment & Access

### Online Web Access
The project is continuously deployed to GitHub Pages and mirrored for global access:
* **GitHub Pages**: [https://ankitsatnami.github.io/Robotic_arm_simulator/](https://ankitsatnami.github.io/Robotic_arm_simulator/)
* **Cloud Mirror**: [https://eight-kitchen-mngsz.ship.place/](https://eight-kitchen-mngsz.ship.place/)

### Local Execution
To run locally, clone the repository and launch any local web server or open `index.html` directly:
```bash
git clone https://github.com/Ankitsatnami/Robotic_arm_simulator.git
cd Robotic_arm_simulator
```
Using Python:
```bash
python -m http.server 8080
```
Or using Node.js:
```bash
npx serve .
```

---

## 👤 Author & Credit

- **Creator & Lead Developer**: **Ankit Satnami**
- **GitHub Profile**: [@Ankitsatnami](https://github.com/Ankitsatnami)
- **Repository**: [https://github.com/Ankitsatnami/Robotic_arm_simulator](https://github.com/Ankitsatnami/Robotic_arm_simulator)

---

## 📖 Citation

If you use this simulator or its kinematic algorithms for academic coursework, research, or development, please cite it as:

```bibtex
@misc{satnami2026fanucplayground,
  author = {Satnami, Ankit},
  title = {Fanuc Cobot Kinematics Playground: A Real-Time 6-DOF Robotic Arm Simulation & Kinematics Research Platform},
  year = {2026},
  publisher = {GitHub},
  howpublished = {\url{https://github.com/Ankitsatnami/Robotic_arm_simulator}}
}
```

---

## 📄 License

This project is licensed under the [MIT License](LICENSE) &copy; 2026 **Ankit Satnami**.
