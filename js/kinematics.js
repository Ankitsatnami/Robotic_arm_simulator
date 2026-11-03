/**
 * Kinematics Engine for 6-DOF Robotic Manipulators
 * Supports Forward Kinematics (FK), Closed-Form Analytical 8-Solution Inverse Kinematics (IK),
 * Denavit-Hartenberg (DH) Matrix Pipeline, Geometric Jacobian (6x6), Singularity Diagnostics,
 * and Manipulability Analysis.
 */

class KinematicsEngine {
    constructor() {
        // Robot models specifications with standard DH parameters: [theta, d (mm), a (mm), alpha (rad)]
        this.robotProfiles = {
            'CRX-10iA': {
                name: 'Fanuc CRX-10iA (Standard)',
                type: 'Cobot 6-DOF',
                reach: 1249, // mm
                payload: '10 kg',
                repeatability: '±0.04 mm',
                weight: '40 kg',
                // Physical link lengths in millimeters
                dh: [
                    { name: 'J1', d: 245.0, a: 0.0,   alpha: Math.PI / 2,  min: -180, max: 180, speed: 120 },
                    { name: 'J2', d: 0.0,   a: 710.0, alpha: 0.0,          min: -180, max: 180, speed: 120 },
                    { name: 'J3', d: 0.0,   a: 0.0,   alpha: Math.PI / 2,  min: -180, max: 180, speed: 180 },
                    { name: 'J4', d: 540.0, a: 0.0,   alpha: -Math.PI / 2, min: -190, max: 190, speed: 180 },
                    { name: 'J5', d: 0.0,   a: 0.0,   alpha: Math.PI / 2,  min: -180, max: 180, speed: 180 },
                    { name: 'J6', d: 160.0, a: 0.0,   alpha: 0.0,          min: -225, max: 225, speed: 220 }
                ],
                home: [0, 0, 0, 0, 0, 0],
                ready: [0, 20, 60, 0, -80, 0],
                colorTheme: {
                    body: 0xf3f4f6,      // Clean glossy white cobot
                    accent: 0x10b981,    // Fanuc CRX green rings
                    joints: 0x1f2937,    // Dark metallic charcoal
                    effector: 0x111827   // Tool flange
                }
            },
            'UR5e': {
                name: 'Universal Robots UR5e',
                type: 'Collaborative 6-DOF',
                reach: 850,
                payload: '5 kg',
                repeatability: '±0.03 mm',
                weight: '20.6 kg',
                dh: [
                    { name: 'J1', d: 162.5, a: 0.0,    alpha: Math.PI / 2,  min: -360, max: 360, speed: 180 },
                    { name: 'J2', d: 0.0,   a: -425.0, alpha: 0.0,          min: -360, max: 360, speed: 180 },
                    { name: 'J3', d: 0.0,   a: -392.2, alpha: 0.0,          min: -360, max: 360, speed: 180 },
                    { name: 'J4', d: 133.3, a: 0.0,    alpha: Math.PI / 2,  min: -360, max: 360, speed: 180 },
                    { name: 'J5', d: 99.7,  a: 0.0,    alpha: -Math.PI / 2, min: -360, max: 360, speed: 180 },
                    { name: 'J6', d: 99.6,  a: 0.0,    alpha: 0.0,          min: -360, max: 360, speed: 180 }
                ],
                home: [0, 0, 0, 0, 0, 0],
                ready: [0, -45, 90, -45, 90, 0],
                colorTheme: {
                    body: 0xc8cdd0,
                    accent: 0x0284c7,
                    joints: 0x334155,
                    effector: 0x0f172a
                }
            },
            'KUKA-KR6': {
                name: 'KUKA KR6 R900 (Agilus)',
                type: 'Industrial 6-DOF',
                reach: 901,
                payload: '6 kg',
                repeatability: '±0.02 mm',
                weight: '52 kg',
                dh: [
                    { name: 'J1', d: 400.0, a: 25.0,  alpha: -Math.PI / 2, min: -170, max: 170, speed: 360 },
                    { name: 'J2', d: 0.0,   a: 455.0, alpha: 0.0,          min: -190, max: 45,  speed: 300 },
                    { name: 'J3', d: 0.0,   a: 35.0,  alpha: Math.PI / 2,  min: -120, max: 156, speed: 360 },
                    { name: 'J4', d: 420.0, a: 0.0,   alpha: -Math.PI / 2, min: -185, max: 185, speed: 381 },
                    { name: 'J5', d: 0.0,   a: 0.0,   alpha: Math.PI / 2,  min: -120, max: 120, speed: 388 },
                    { name: 'J6', d: 80.0,  a: 0.0,   alpha: 0.0,          min: -350, max: 350, speed: 615 }
                ],
                home: [0, 0, 0, 0, 0, 0],
                ready: [0, -30, 80, 0, 40, 0],
                colorTheme: {
                    body: 0xea580c,      // KUKA iconic industrial orange
                    accent: 0x1e293b,
                    joints: 0x0f172a,
                    effector: 0x334155
                }
            },
            'PUMA-560': {
                name: 'Unimation PUMA 560 (Classic)',
                type: 'Academic Standard 6-DOF',
                reach: 878,
                payload: '4 kg',
                repeatability: '±0.1 mm',
                weight: '53 kg',
                dh: [
                    { name: 'J1', d: 671.8, a: 0.0,   alpha: -Math.PI / 2, min: -160, max: 160, speed: 100 },
                    { name: 'J2', d: 0.0,   a: 431.8, alpha: 0.0,          min: -225, max: 45,  speed: 100 },
                    { name: 'J3', d: 150.05,a: -20.3, alpha: Math.PI / 2,  min: -45,  max: 225, speed: 100 },
                    { name: 'J4', d: 433.1, a: 0.0,   alpha: -Math.PI / 2, min: -110, max: 170, speed: 150 },
                    { name: 'J5', d: 0.0,   a: 0.0,   alpha: Math.PI / 2,  min: -100, max: 100, speed: 150 },
                    { name: 'J6', d: 56.5,  a: 0.0,   alpha: 0.0,          min: -266, max: 266, speed: 200 }
                ],
                home: [0, 0, 0, 0, 0, 0],
                ready: [0, -45, 90, 0, 45, 0],
                colorTheme: {
                    body: 0xfacc15,      // Classic yellow
                    accent: 0x1f2937,
                    joints: 0x374151,
                    effector: 0x111827
                }
            }
        };

        this.currentModelKey = 'CRX-10iA';
    }

    getModel() {
        return this.robotProfiles[this.currentModelKey];
    }

    setModel(key) {
        if (this.robotProfiles[key]) {
            this.currentModelKey = key;
            return true;
        }
        return false;
    }

    degToRad(deg) {
        return (deg * Math.PI) / 180;
    }

    radToDeg(rad) {
        return (rad * 180) / Math.PI;
    }

    /**
     * Compute individual DH Transformation Matrix A_i for Link i
     * Standard Denavit-Hartenberg matrix:
     * RotZ(theta) * TransZ(d) * TransX(a) * RotX(alpha)
     */
    getDHMatrix(thetaRad, d, a, alphaRad) {
        const ct = Math.cos(thetaRad);
        const st = Math.sin(thetaRad);
        const ca = Math.cos(alphaRad);
        const sa = Math.sin(alphaRad);

        return [
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d],
            [0,   0,        0,       1]
        ];
    }

    /**
     * Multiply two 4x4 matrices M1 * M2
     */
    multiplyMatrices(M1, M2) {
        const result = [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ];
        for (let i = 0; i < 4; i++) {
            for (let j = 0; j < 4; j++) {
                let sum = 0;
                for (let k = 0; k < 4; k++) {
                    sum += M1[i][k] * M2[k][j];
                }
                result[i][j] = sum;
            }
        }
        return result;
    }

    /**
     * Forward Kinematics (FK)
     * Joint angles in degrees: [j1, j2, j3, j4, j5, j6]
     * Returns:
     * - cumulativeMatrices: array of T_0^i for each link i (0 to 6)
     * - linkMatrices: individual A_i matrices
     * - cartesian: { x, y, z, w, p, r }
     * - T06: 4x4 matrix
     */
    forwardKinematics(jointAnglesDeg) {
        const model = this.getModel();
        const dh = model.dh;

        const linkMatrices = [];
        const cumulativeMatrices = [];

        // Base frame T_0^0 = Identity
        let currentT = [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ];
        cumulativeMatrices.push(JSON.parse(JSON.stringify(currentT)));

        for (let i = 0; i < 6; i++) {
            const thetaRad = this.degToRad(jointAnglesDeg[i]);
            const Ai = this.getDHMatrix(thetaRad, dh[i].d, dh[i].a, dh[i].alpha);
            linkMatrices.push(Ai);
            currentT = this.multiplyMatrices(currentT, Ai);
            cumulativeMatrices.push(JSON.parse(JSON.stringify(currentT)));
        }

        const T06 = currentT;
        const x = T06[0][3];
        const y = T06[1][3];
        const z = T06[2][3];

        // Extract Fanuc W-P-R (Euler angles: Yaw about Z, Pitch about Y, Roll about X, or standard XYZ)
        // Fanuc WPR is:
        // R = Rz(w) * Ry(p) * Rx(r) (W=Yaw/Z, P=Pitch/Y, R=Roll/X)
        const R = [
            [T06[0][0], T06[0][1], T06[0][2]],
            [T06[1][0], T06[1][1], T06[1][2]],
            [T06[2][0], T06[2][1], T06[2][2]]
        ];

        let p_pitch = Math.atan2(-R[2][0], Math.hypot(R[0][0], R[1][0]));
        let w_yaw = 0;
        let r_roll = 0;

        if (Math.abs(Math.cos(p_pitch)) > 1e-5) {
            w_yaw = Math.atan2(R[1][0], R[0][0]);
            r_roll = Math.atan2(R[2][1], R[2][2]);
        } else {
            // Gimbal lock
            w_yaw = Math.atan2(-R[0][1], R[1][1]);
            r_roll = 0;
        }

        return {
            cumulativeMatrices,
            linkMatrices,
            T06,
            cartesian: {
                x: Number(x.toFixed(2)),
                y: Number(y.toFixed(2)),
                z: Number(z.toFixed(2)),
                w: Number(this.radToDeg(w_yaw).toFixed(2)),
                p: Number(this.radToDeg(p_pitch).toFixed(2)),
                r: Number(this.radToDeg(r_roll).toFixed(2))
            },
            config: this.determineConfiguration(jointAnglesDeg)
        };
    }

    /**
     * Determine Fanuc-style configuration flags:
     * - Flip (F) vs Non-flip (N): Joint 5 sign (N: J5 >= 0, F: J5 < 0)
     * - Up (U) vs Down (D): Elbow configuration (D: J3 downward, U: J3 upward)
     * - Top (T) vs Bottom (B): Shoulder front/back
     * - Turn numbers: full revolutions for J1, J4, J6 (usually 0, 0, 0)
     */
    determineConfiguration(joints) {
        const [j1, j2, j3, j4, j5, j6] = joints;

        const flip = j5 <= 0 ? 'N' : 'F';
        const elbow = j3 >= 0 ? 'D' : 'U';
        const shoulder = j2 >= -45 ? 'T' : 'B';

        const turn1 = Math.floor((j1 + 180) / 360);
        const turn4 = Math.floor((j4 + 180) / 360);
        const turn6 = Math.floor((j6 + 180) / 360);

        const configKey = `${flip} ${elbow} ${shoulder}`;
        const fullString = `${configKey}, ${turn1}, ${turn4}, ${turn6}`;

        return {
            flip,
            elbow,
            shoulder,
            configKey,
            fullString
        };
    }

    /**
     * Create Rotation matrix from Fanuc W-P-R Euler angles (in degrees)
     * R = Rz(W) * Ry(P) * Rx(R)
     */
    wprToRotationMatrix(wDeg, pDeg, rDeg) {
        const w = this.degToRad(wDeg);
        const p = this.degToRad(pDeg);
        const r = this.degToRad(rDeg);

        const cw = Math.cos(w), sw = Math.sin(w);
        const cp = Math.cos(p), sp = Math.sin(p);
        const cr = Math.cos(r), sr = Math.sin(r);

        return [
            [
                cw * cp,
                cw * sp * sr - sw * cr,
                cw * sp * cr + sw * sr
            ],
            [
                sw * cp,
                sw * sp * sr + cw * cr,
                sw * sp * cr - cw * sr
            ],
            [
                -sp,
                cp * sr,
                cp * cr
            ]
        ];
    }

    /**
     * Analytical Inverse Kinematics (IK) for 6-DOF Spherical Wrist Robot (CRX-10iA style)
     * Computes all 8 configuration solutions:
     * Configurations:
     * - F D T, 0, 0, 0
     * - N D T, 0, 0, 0
     * - F U T, 0, 0, 0
     * - N U T, 0, 0, 0
     * - F U B, 0, 0, 0
     * - N U B, 0, 0, 0
     * - F D B, 0, 0, 0
     * - N D B, 0, 0, 0
     */
    inverseKinematicsAllSolutions(target) {
        const model = this.getModel();
        const dh = model.dh;
        const d1 = dh[0].d;
        const a2 = dh[1].a !== 0 ? Math.abs(dh[1].a) : 710.0;
        const a3 = dh[2].a !== 0 ? Math.abs(dh[2].a) : 0.0;
        const d4 = dh[3].d !== 0 ? Math.abs(dh[3].d) : 540.0;
        const d6 = dh[5].d !== 0 ? Math.abs(dh[5].d) : 160.0;

        const { x, y, z, w, p, r } = target;
        const R06 = this.wprToRotationMatrix(w, p, r);

        // Tool vector (Z-axis of end-effector frame, column 2)
        const ax = R06[0][2];
        const ay = R06[1][2];
        const az = R06[2][2];

        // Step 1: Wrist Center Position P_w = P_ee - d6 * a
        const Pwx = x - d6 * ax;
        const Pwy = y - d6 * ay;
        const Pwz = z - d6 * az;

        const solutions = [];

        // Shoulder configurations: Top/Front vs Bottom/Back (or Forward vs Backward)
        const shoulderCases = [
            { flag: 'T', offset: 0 },
            { flag: 'B', offset: Math.PI }
        ];

        // Elbow configurations: Down vs Up
        const elbowCases = [
            { flag: 'D', sign: 1 },
            { flag: 'U', sign: -1 }
        ];

        // Wrist configurations: Non-flip vs Flip
        const wristCases = [
            { flag: 'N', flip: false },
            { flag: 'F', flip: true }
        ];

        for (const sCase of shoulderCases) {
            // Joint 1: Base angle
            let theta1Base = Math.atan2(Pwy, Pwx);
            let theta1 = theta1Base + sCase.offset;

            // Normalize theta1 into [-PI, PI]
            theta1 = Math.atan2(Math.sin(theta1), Math.cos(theta1));

            // Planar reach for J2-J3
            const r_xy = Math.hypot(Pwx, Pwy) * (sCase.flag === 'T' ? 1 : -1);
            const r_z = Pwz - d1;
            const D_sq = r_xy * r_xy + r_z * r_z;
            const D = Math.sqrt(D_sq);

            // Effective link 2 and link 3
            const L2 = a2;
            const L3 = Math.hypot(a3, d4);

            // Reachability check
            if (D > L2 + L3 || D < Math.abs(L2 - L3)) {
                // Point unreachable, skip or clamp
                continue;
            }

            // Law of cosines for elbow angle
            const cosBeta = (L2 * L2 + L3 * L3 - D_sq) / (2 * L2 * L3);
            const clampedCosBeta = Math.max(-1, Math.min(1, cosBeta));
            const beta = Math.acos(clampedCosBeta);

            for (const eCase of elbowCases) {
                // Joint 3
                let theta3 = eCase.sign * (Math.PI - beta);

                // Joint 2
                const alphaTarget = Math.atan2(r_z, r_xy);
                const cosGamma = (L2 * L2 + D_sq - L3 * L3) / (2 * L2 * D);
                const clampedCosGamma = Math.max(-1, Math.min(1, cosGamma));
                const gamma = Math.acos(clampedCosGamma);

                let theta2 = alphaTarget - eCase.sign * gamma;

                // Spherical wrist orientation
                for (const wCase of wristCases) {
                    const anglesDeg = [
                        this.radToDeg(theta1),
                        this.radToDeg(theta2),
                        this.radToDeg(theta3),
                        0, 0, 0
                    ];

                    // Compute R_0^3
                    const fk03 = this.forwardKinematics([anglesDeg[0], anglesDeg[1], anglesDeg[2], 0, 0, 0]);
                    const T03 = fk03.cumulativeMatrices[3];

                    // R_0^3
                    const R03 = [
                        [T03[0][0], T03[0][1], T03[0][2]],
                        [T03[1][0], T03[1][1], T03[1][2]],
                        [T03[2][0], T03[2][1], T03[2][2]]
                    ];

                    // R_3^6 = (R_0^3)^T * R_0^6
                    const R36 = [
                        [0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0]
                    ];

                    for (let row = 0; row < 3; row++) {
                        for (let col = 0; col < 3; col++) {
                            let sum = 0;
                            for (let k = 0; k < 3; k++) {
                                sum += R03[k][row] * R06[k][col];
                            }
                            R36[row][col] = sum;
                        }
                    }

                    // Extract Euler Z-Y-Z for spherical wrist J4, J5, J6 from R36
                    let theta5, theta4, theta6;

                    // cos(theta5) = R36[2][2]
                    const cos5 = Math.max(-1, Math.min(1, R36[2][2]));
                    if (wCase.flip) {
                        theta5 = -Math.acos(cos5);
                    } else {
                        theta5 = Math.acos(cos5);
                    }

                    if (Math.abs(Math.sin(theta5)) > 1e-4) {
                        theta4 = Math.atan2(R36[1][2] / Math.sin(theta5), R36[0][2] / Math.sin(theta5));
                        theta6 = Math.atan2(R36[2][1] / Math.sin(theta5), -R36[2][0] / Math.sin(theta5));
                    } else {
                        // Singularity (theta5 close to 0)
                        theta4 = 0;
                        theta6 = Math.atan2(-R36[0][1], R36[0][0]);
                    }

                    // Normalize angles to degrees
                    const sol = [
                        this.normalizeDeg(this.radToDeg(theta1)),
                        this.normalizeDeg(this.radToDeg(theta2)),
                        this.normalizeDeg(this.radToDeg(theta3)),
                        this.normalizeDeg(this.radToDeg(theta4)),
                        this.normalizeDeg(this.radToDeg(theta5)),
                        this.normalizeDeg(this.radToDeg(theta6))
                    ];

                    const configName = `${wCase.flag} ${eCase.flag} ${sCase.flag}, 0, 0, 0`;

                    solutions.push({
                        configName,
                        configCode: `${wCase.flag} ${eCase.flag} ${sCase.flag}`,
                        flip: wCase.flag,
                        elbow: eCase.flag,
                        shoulder: sCase.flag,
                        joints: sol.map(v => Number(v.toFixed(1))),
                        valid: this.checkJointLimits(sol)
                    });
                }
            }
        }

        return solutions;
    }

    normalizeDeg(deg) {
        let d = deg % 360;
        if (d > 180) d -= 360;
        if (d < -180) d += 360;
        return d;
    }

    checkJointLimits(joints) {
        const model = this.getModel();
        for (let i = 0; i < 6; i++) {
            if (joints[i] < model.dh[i].min || joints[i] > model.dh[i].max) {
                return false;
            }
        }
        return true;
    }

    /**
     * Geometric Jacobian Matrix (6x6)
     * J = [ J_v ] (3x6 linear velocity)
     *     [ J_w ] (3x6 angular velocity)
     * For revolute joint i:
     * J_v,i = z_{i-1} x (p_e - p_{i-1})
     * J_w,i = z_{i-1}
     */
    computeJacobian(jointAnglesDeg) {
        const fk = this.forwardKinematics(jointAnglesDeg);
        const T = fk.cumulativeMatrices;

        const Pe = [T[6][0][3], T[6][1][3], T[6][2][3]];

        const J = [
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0]
        ];

        for (let i = 0; i < 6; i++) {
            // z_{i-1} is column 2 of rotation matrix T[i]
            const z = [T[i][0][2], T[i][1][2], T[i][2][2]];
            // p_{i-1} is column 3 of T[i]
            const p = [T[i][0][3], T[i][1][3], T[i][2][3]];

            // r = p_e - p_{i-1}
            const rx = Pe[0] - p[0];
            const ry = Pe[1] - p[1];
            const rz = Pe[2] - p[2];

            // J_v = z x r
            const Jvx = z[1] * rz - z[2] * ry;
            const Jvy = z[2] * rx - z[0] * rz;
            const Jvz = z[0] * ry - z[1] * rx;

            // Fill columns
            J[0][i] = Jvx / 1000.0; // convert mm to meters for normalized scaling
            J[1][i] = Jvy / 1000.0;
            J[2][i] = Jvz / 1000.0;
            J[3][i] = z[0];
            J[4][i] = z[1];
            J[5][i] = z[2];
        }

        // Calculate Manipulability index (Yoshikawa measure) = sqrt(det(J * J^T))
        const JJT = this.multiplyMatrixWithTranspose(J);
        const detJJT = this.determinant6x6(JJT);
        const manipulability = Math.sqrt(Math.max(0, detJJT));

        // Singularity condition detection
        const j5 = Math.abs(jointAnglesDeg[4]);
        const isWristSingularity = j5 < 5.0 || Math.abs(j5 - 180) < 5.0;

        const shoulderReach = Math.hypot(Pe[0], Pe[1]);
        const isShoulderSingularity = shoulderReach < 80.0;

        const maxReach = this.getModel().reach;
        const totalDist = Math.hypot(Pe[0], Pe[1], Pe[2] - this.getModel().dh[0].d);
        const isBoundarySingularity = Math.abs(totalDist - maxReach) < 40.0;

        return {
            matrix: J,
            manipulability: Number(manipulability.toFixed(4)),
            isSingular: manipulability < 0.005 || isWristSingularity || isShoulderSingularity || isBoundarySingularity,
            singularities: {
                wrist: isWristSingularity,
                shoulder: isShoulderSingularity,
                boundary: isBoundarySingularity
            }
        };
    }

    multiplyMatrixWithTranspose(A) {
        const rows = A.length;
        const cols = A[0].length;
        const result = Array(rows).fill(0).map(() => Array(rows).fill(0));

        for (let i = 0; i < rows; i++) {
            for (let j = 0; j < rows; j++) {
                let sum = 0;
                for (let k = 0; k < cols; k++) {
                    sum += A[i][k] * A[j][k];
                }
                result[i][j] = sum;
            }
        }
        return result;
    }

    determinant6x6(M) {
        // LU decomposition determinant calculation
        const n = 6;
        const A = M.map(row => [...row]);
        let det = 1;

        for (let i = 0; i < n; i++) {
            let pivot = i;
            for (let j = i + 1; j < n; j++) {
                if (Math.abs(A[j][i]) > Math.abs(A[pivot][i])) {
                    pivot = j;
                }
            }

            if (Math.abs(A[pivot][i]) < 1e-12) {
                return 0;
            }

            if (pivot !== i) {
                [A[i], A[pivot]] = [A[pivot], A[i]];
                det = -det;
            }

            det *= A[i][i];

            for (let j = i + 1; j < n; j++) {
                const factor = A[j][i] / A[i][i];
                for (let k = i + 1; k < n; k++) {
                    A[j][k] -= factor * A[i][k];
                }
            }
        }

        return det;
    }
}

// Global instance for browser consumption
window.KinematicsEngine = KinematicsEngine;
