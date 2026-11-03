/**
 * Interactive Learning Center, Tutorials, Guided Experiments & Multi-Language Code Generator
 */

class LearningCenter {
    constructor(kinematics, robotModel, onPoseApply) {
        this.kinematics = kinematics;
        this.robotModel = robotModel;
        this.onPoseApply = onPoseApply;

        this.lessons = [
            {
                id: 'dof',
                title: '1. Degrees of Freedom (DOF)',
                subtitle: 'Spatial Positioning in 3D Space (6-DOF Manipulator)',
                content: `
                    <h4>What is Degree of Freedom (DOF)?</h4>
                    <p>In robotics, <strong>Degrees of Freedom (DOF)</strong> defines the number of independent variables or joint motions required to completely position and orient an object in 3D space.</p>
                    
                    <div class="learning-callout">
                        <strong>Formula for Spatial Movement:</strong><br>
                        A rigid object in 3-dimensional space has <strong>6 degrees of freedom</strong>:
                        <ul>
                            <li><strong>3 Translation DOFs (Position):</strong> Move along $X$, $Y$, and $Z$ axes.</li>
                            <li><strong>3 Rotation DOFs (Orientation):</strong> Rotate about $X$ (Roll $R$), $Y$ (Pitch $P$), and $Z$ (Yaw $W$).</li>
                        </ul>
                    </div>

                    <h4>Why 6 Joints for an Industrial Cobot?</h4>
                    <p>Robots with fewer than 6 DOFs (like a 4-DOF SCARA or 3-DOF Delta robot) cannot reach every arbitrary angle or tilt in space. A 6-DOF articulated arm (like the <strong>Fanuc CRX-10iA</strong>) provides full dexterity: the first 3 joints ($J_1, J_2, J_3$) position the wrist center in 3D space, and the last 3 spherical wrist joints ($J_4, J_5, J_6$) rotate the tool to any desired orientation!</p>
                `,
                demoPose: [0, 20, 60, 0, -80, 0],
                actionLabel: 'Load Standard 6-DOF Pose'
            },
            {
                id: 'fk',
                title: '2. Forward Kinematics (FK)',
                subtitle: 'Mapping Joint Angles to Cartesian Tool Coordinates',
                content: `
                    <h4>From Joint Angles to 3D Space</h4>
                    <p><strong>Forward Kinematics (FK)</strong> calculates the exact 3D Cartesian position $(X, Y, Z)$ and orientation $(W, P, R)$ of the end-effector from given joint angles $[\theta_1, \theta_2, \theta_3, \theta_4, \theta_5, \theta_6]$.</p>

                    <div class="learning-callout">
                        <strong>Mathematical Formulation:</strong><br>
                        Each joint link has a 4x4 Homogeneous Transformation Matrix $A_i$. The total transformation from Base Frame (0) to Tool Flange (6) is obtained by sequential matrix multiplication:
                        $$T_0^6 = A_1(\theta_1) \cdot A_2(\theta_2) \cdot A_3(\theta_3) \cdot A_4(\theta_4) \cdot A_5(\theta_5) \cdot A_6(\theta_6)$$
                    </div>

                    <p>FK always has a unique, unambiguous single solution for any set of joint angles. You can verify this live in the <strong>Live Matrix</strong> tab as you drag joint sliders!</p>
                `,
                demoPose: [-45, 30, 45, 20, -50, 15],
                actionLabel: 'Test Forward Kinematics Pose'
            },
            {
                id: 'dh',
                title: '3. Denavit-Hartenberg (DH) Convention',
                subtitle: 'Standardized 4-Parameter Kinematic Modeling',
                content: `
                    <h4>The Denavit-Hartenberg (DH) Standard</h4>
                    <p>In 1955, Jacques Denavit and Richard Hartenberg introduced a compact 4-parameter convention to describe the spatial relationship between consecutive joint axes:</p>
                    
                    <table class="dh-mini-table">
                        <thead>
                            <tr><th>Parameter</th><th>Symbol</th><th>Description</th></tr>
                        </thead>
                        <tbody>
                            <tr><td><strong>Joint Angle</strong></td><td>$\theta_i$</td><td>Rotation angle about $Z_{i-1}$ axis (Variable for revolute joints)</td></tr>
                            <tr><td><strong>Link Offset</strong></td><td>$d_i$</td><td>Distance along $Z_{i-1}$ axis from $X_{i-1}$ to $X_i$</td></tr>
                            <tr><td><strong>Link Length</strong></td><td>$a_i$</td><td>Distance along $X_i$ axis (length of the link)</td></tr>
                            <tr><td><strong>Link Twist</strong></td><td>$\alpha_i$</td><td>Angle from $Z_{i-1}$ to $Z_i$ measured about $X_i$</td></tr>
                        </tbody>
                    </table>

                    <p>The standard DH 4x4 matrix for link $i$ is:</p>
                    <div class="matrix-code-block">
                        $$A_i = \\begin{bmatrix} 
                        \\cos\\theta_i & -\\sin\\theta_i\\cos\\alpha_i & \\sin\\theta_i\\sin\\alpha_i & a_i\\cos\\theta_i \\\\ 
                        \\sin\\theta_i & \\cos\\theta_i\\cos\\alpha_i & -\\cos\\theta_i\\sin\\alpha_i & a_i\\sin\\theta_i \\\\ 
                        0 & \\sin\\alpha_i & \\cos\\alpha_i & d_i \\\\ 
                        0 & 0 & 0 & 1 
                        \\end{bmatrix}$$
                    </div>
                `,
                demoPose: [0, 0, 0, 0, 0, 0],
                actionLabel: 'Reset to Zero DH Baseline'
            },
            {
                id: 'ik',
                title: '4. Inverse Kinematics & 8 Configurations',
                subtitle: 'Why up to 8 Different Arm Poses Reach the Same Target',
                content: `
                    <h4>Inverse Kinematics (IK) Multiplicity</h4>
                    <p>Given a target position $(X,Y,Z)$ and orientation $(W,P,R)$, <strong>Inverse Kinematics (IK)</strong> solves for the required joint angles $[\theta_1, \dots, \theta_6]$.</p>

                    <p>For an articulated 6-axis arm with a spherical wrist, there are <strong>up to 8 distinct geometric configurations</strong> that put the tool in the exact same Cartesian spot:</p>

                    <div class="ik-solutions-explainer">
                        <div class="explainer-card">
                            <strong>1. Shoulder (Top vs Bottom / Front vs Back):</strong>
                            <p>Base joint can point toward the target ($T$) or rotate 180° backwards ($B$) reaching over the top.</p>
                        </div>
                        <div class="explainer-card">
                            <strong>2. Elbow (Up vs Down):</strong>
                            <p>For the same shoulder position, the elbow can bend downward ($D$) or arch upward ($U$).</p>
                        </div>
                        <div class="explainer-card">
                            <strong>3. Wrist (Non-Flip vs Flip):</strong>
                            <p>The spherical wrist can align with positive pitch ($N$) or flip 180° ($F$) with inverted roll angles ($J_4, J_6$).</p>
                        </div>
                    </div>

                    <p>This produces $2 \\times 2 \\times 2 = 8$ configurations: <code>F D T</code>, <code>N D T</code>, <code>F U T</code>, <code>N U T</code>, <code>F U B</code>, <code>N U B</code>, <code>F D B</code>, <code>N D B</code>.</p>
                `,
                demoPose: [-62, 26, 78, 69, -68, 75],
                actionLabel: 'Load Screenshot Configuration (N D T)'
            },
            {
                id: 'singularities',
                title: '5. Kinematic Singularities & Jacobian',
                subtitle: 'Loss of Degrees of Freedom & Infinite Speeds',
                content: `
                    <h4>What is a Kinematic Singularity?</h4>
                    <p>A singularity occurs when the manipulator's <strong>Jacobian Matrix $J$</strong> drops in rank (meaning its determinant $\\det(J) \\to 0$). At a singularity, the robot loses the ability to move in one or more Cartesian directions, and commanding linear motion through it would require <strong>infinite joint velocity</strong>!</p>

                    <h4>The Three Classic Singularities:</h4>
                    <ul>
                        <li><strong>Wrist Singularity ($\theta_5 = 0°$):</strong> Axes of Joint 4 and Joint 6 become collinear. The robot cannot rotate its tool pitch without moving other joints.</li>
                        <li><strong>Shoulder Singularity:</strong> The wrist center intersects the $Z_0$ base rotation axis. The arm must rotate infinitely fast around $J_1$ to move across the center.</li>
                        <li><strong>Elbow / Boundary Singularity:</strong> Arm is fully stretched out to its maximum reach envelope ($D = L_2 + L_3$). The arm cannot extend any further.</li>
                    </ul>
                `,
                demoPose: [0, 45, 45, 0, 0, 0], // Exact Wrist Singularity (J5 = 0)
                actionLabel: 'Demonstrate Wrist Singularity (J5 = 0°)'
            },
            {
                id: 'motion',
                title: '6. Motion Planning (MOVEJ vs MOVEL)',
                subtitle: 'Joint-Space PTP vs Cartesian Linear Interpolation',
                content: `
                    <h4>PTP (MOVEJ) vs Linear (MOVEL)</h4>
                    <div class="comparison-grid">
                        <div class="comp-col">
                            <h5>MOVEJ (Joint-Space Motion)</h5>
                            <ul>
                                <li>All 6 joints start and stop at the same time.</li>
                                <li>Linear interpolation in joint angles: $q(t) = q_0 + t(q_1 - q_0)$.</li>
                                <li>The tool path curves through 3D space.</li>
                                <li><strong>Immune to singularities!</strong> Safest motion for moving between distant stations.</li>
                            </ul>
                        </div>
                        <div class="comp-col">
                            <h5>MOVEL (Cartesian Linear Motion)</h5>
                            <ul>
                                <li>The tool moves along a strict straight line in 3D space: $P(t) = P_0 + t(P_1 - P_0)$.</li>
                                <li>Inverse kinematics must be calculated continuously along the path.</li>
                                <li><strong>Can fail or trigger fault alarms</strong> if the straight line passes through a singularity or exceeds joint speed limits!</li>
                            </ul>
                        </div>
                    </div>
                `,
                demoPose: [0, 20, 60, 0, -80, 0],
                actionLabel: 'Ready For Trajectory Sequencing'
            }
        ];
    }

    /**
     * Multi-Language Code Generator
     * Formats current robot joint angles and pose into executable code
     */
    generateCode(language, jointsDeg, cartesian, config) {
        const [j1, j2, j3, j4, j5, j6] = jointsDeg.map(v => Number(v.toFixed(2)));
        const { x, y, z, w, p, r } = cartesian;

        switch (language.toLowerCase()) {
            case 'python':
                return `import numpy as np

# Robot Joint Coordinates (Degrees)
q_deg = np.array([${j1}, ${j2}, ${j3}, ${j4}, ${j5}, ${j6}])
q_rad = np.deg2rad(q_deg)

# Cartesian Target Pose (FANUC Standard)
# Position (mm): X=${x}, Y=${y}, Z=${z}
# Orientation (deg): W=${w}, P=${p}, R=${r}
# Configuration: ${config.fullString}

def forward_kinematics_dh(q):
    """Computes end-effector position for Fanuc CRX-10iA"""
    # Link lengths in mm
    d1 = 245.0; a2 = 710.0; d4 = 540.0; d6 = 160.0
    
    # DH Parameter Matrix: [theta, d, a, alpha]
    dh = [
        [q[0], d1, 0.0, np.pi/2],
        [q[1], 0.0, a2, 0.0],
        [q[2], 0.0, 0.0, np.pi/2],
        [q[3], d4, 0.0, -np.pi/2],
        [q[4], 0.0, 0.0, np.pi/2],
        [q[5], d6, 0.0, 0.0]
    ]
    
    T = np.eye(4)
    for theta, d, a, alpha in dh:
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        A = np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d],
            [0,   0,      0,     1]
        ])
        T = T @ A
    return T

T06 = forward_kinematics_dh(q_rad)
print("Transformation Matrix T_0^6:\\n", np.round(T06, 3))
print("Computed Tool Center Point (mm):", T06[:3, 3])
`;

            case 'fanuc':
                return `/PROG ROBOT_SIM_MOTION
/ATTR
COMMENT = "Generated Kinematics Path";
DEFAULT_GROUP = 1,*,*,*,*;
/MN
! Current Fanuc CRX-10iA Cobot Position
! Configuration: ${config.fullString}

  1:  ! Joint Space Move to target waypoint
  2:  J P[1] 100% FINE ;
  3:  ! Linear Cartesian Move with tool orientation
  4:  L P[2] 250mm/sec FINE ;
  5:  WAIT 0.50(sec) ;
  6:  ! Gripper Actuation: RO[1] = ON (Clamp)
  7:  RO[1:Gripper]=ON ;
/POS
P[1]{
   GP1:
    UF : 0, UT : 1,
    J1 = ${j1} deg,
    J2 = ${j2} deg,
    J3 = ${j3} deg,
    J4 = ${j4} deg,
    J5 = ${j5} deg,
    J6 = ${j6} deg
};
P[2]{
   GP1:
    UF : 0, UT : 1,
    CONFIG : '${config.flip} ${config.elbow} ${config.shoulder}, 0, 0, 0',
    X = ${x} mm,  Y = ${y} mm,  Z = ${z} mm,
    W = ${w} deg, P = ${p} deg, R = ${r} deg
};
/END
`;

            case 'cpp':
                return `// ROS 2 MoveIt 2 C++ Motion Controller
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

void executePoseGoal(moveit::planning_interface::MoveGroupInterface& move_group) {
    // Joint Target Configuration
    std::vector<double> joint_positions = {
        ${(j1 * Math.PI / 180).toFixed(4)},  // Joint 1 (rad)
        ${(j2 * Math.PI / 180).toFixed(4)},  // Joint 2 (rad)
        ${(j3 * Math.PI / 180).toFixed(4)},  // Joint 3 (rad)
        ${(j4 * Math.PI / 180).toFixed(4)},  // Joint 4 (rad)
        ${(j5 * Math.PI / 180).toFixed(4)},  // Joint 5 (rad)
        ${(j6 * Math.PI / 180).toFixed(4)}   // Joint 6 (rad)
    };

    move_group.setJointValueTarget(joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        RCLCPP_INFO(rclcpp::get_logger("Robot"), "Joint target reached!");
    }
}
`;

            case 'matlab':
                return `% MATLAB Robotics System Toolbox
% Fanuc CRX-10iA Cobot Model Setup

robot = rigidBodyTree('DataFormat', 'row');

% Joint Values in Radians
q = deg2rad([${j1}, ${j2}, ${j3}, ${j4}, ${j5}, ${j6}]);

% Target Cartesian Pose (X, Y, Z in meters)
targetPos = [${(x / 1000).toFixed(4)}, ${(y / 1000).toFixed(4)}, ${(z / 1000).toFixed(4)}];
eul = deg2rad([${w}, ${p}, ${r}]); % ZYX Euler angles
targetRot = eul2rotm(eul, 'ZYX');

targetTransform = trvec2tform(targetPos) * rotm2tform(targetRot);
disp('Desired Target Transform:');
disp(targetTransform);
`;

            case 'json':
            default:
                return JSON.stringify({
                    robot: "Fanuc CRX-10iA",
                    jointDegrees: [j1, j2, j3, j4, j5, j6],
                    cartesian: { x, y, z, w, p, r },
                    configuration: config.fullString,
                    units: {
                        position: "millimeters",
                        angles: "degrees"
                    }
                }, null, 2);
        }
    }
}

window.LearningCenter = LearningCenter;
