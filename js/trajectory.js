/**
 * Trajectory Planner & Motion Sequencer
 * Supports Joint Space PTP (MOVEJ), Cartesian Linear Interpolation (MOVEL),
 * 3D visual toolpath tracing, Waypoint Recording, and Automated Pick & Place demo.
 */

class TrajectoryPlanner {
    constructor(kinematics, robotModel, onJointUpdate) {
        this.kinematics = kinematics;
        this.robotModel = robotModel;
        this.onJointUpdate = onJointUpdate;

        this.waypoints = [];
        this.isPlaying = false;
        this.isPaused = false;
        this.isLooping = false;
        this.motionType = 'MOVEJ'; // 'MOVEJ' or 'MOVEL'
        this.speedPercent = 50;

        this.currentWaypointIndex = 0;
        this.progress = 0; // 0 to 1 between waypoints
        this.activeTrajectoryPath = null;

        // Visual Toolpath Line
        this.pathPoints = [];
        this.pathLine = null;
        this.initVisualPath();

        // Default Pick & Place Program
        this.loadDefaultProgram();
    }

    initVisualPath() {
        const material = new THREE.LineBasicMaterial({
            color: 0x06b6d4,
            linewidth: 2,
            transparent: true,
            opacity: 0.8
        });
        const geometry = new THREE.BufferGeometry();
        this.pathLine = new THREE.Line(geometry, material);
        this.robotModel.scene.add(this.pathLine);
    }

    clearPathTrace() {
        this.pathPoints = [];
        this.pathLine.geometry.setFromPoints([]);
    }

    addPathPoint(vector3) {
        if (this.pathPoints.length > 500) {
            this.pathPoints.shift();
        }
        this.pathPoints.push(vector3.clone());
        this.pathLine.geometry.setFromPoints(this.pathPoints);
    }

    loadDefaultProgram() {
        // Preset Pick & Place sequence for Fanuc CRX
        this.waypoints = [
            {
                id: 1,
                name: 'Ready Pose',
                joints: [0, 20, 60, 0, -80, 0],
                gripper: 1.0,
                speed: 60,
                delay: 200
            },
            {
                id: 2,
                name: 'Pick Approach',
                joints: [-52, 38, 55, 0, -85, -52],
                gripper: 1.0,
                speed: 50,
                delay: 100
            },
            {
                id: 3,
                name: 'Pick Grasp',
                joints: [-52, 48, 62, 0, -85, -52],
                gripper: 0.0, // Grab
                speed: 25,
                delay: 400
            },
            {
                id: 4,
                name: 'Pick Retract',
                joints: [-52, 28, 50, 0, -80, -52],
                gripper: 0.0,
                speed: 40,
                delay: 100
            },
            {
                id: 5,
                name: 'Drop Approach',
                joints: [52, 28, 50, 0, -80, 52],
                gripper: 0.0,
                speed: 50,
                delay: 100
            },
            {
                id: 6,
                name: 'Drop Place',
                joints: [52, 48, 62, 0, -85, 52],
                gripper: 1.0, // Release
                speed: 25,
                delay: 400
            },
            {
                id: 7,
                name: 'Return Ready',
                joints: [0, 20, 60, 0, -80, 0],
                gripper: 1.0,
                speed: 60,
                delay: 200
            }
        ];
    }

    recordCurrentAsWaypoint(currentJoints, currentGripper, name) {
        const id = this.waypoints.length + 1;
        this.waypoints.push({
            id,
            name: name || `P[${id}] Waypoint`,
            joints: [...currentJoints],
            gripper: currentGripper,
            speed: this.speedPercent,
            delay: 100
        });
        return this.waypoints;
    }

    removeWaypoint(index) {
        if (index >= 0 && index < this.waypoints.length) {
            this.waypoints.splice(index, 1);
        }
    }

    play() {
        if (this.waypoints.length < 2) return;
        this.isPlaying = true;
        this.isPaused = false;
        this.currentWaypointIndex = 0;
        this.progress = 0;
        this.clearPathTrace();
    }

    pause() {
        this.isPaused = true;
    }

    resume() {
        this.isPaused = false;
    }

    stop() {
        this.isPlaying = false;
        this.isPaused = false;
        this.progress = 0;
    }

    /**
     * Smooth Hermite / S-Curve interpolation factor
     */
    smoothStep(t) {
        return t * t * (3 - 2 * t);
    }

    /**
     * Update trajectory per animation frame (delta in seconds)
     */
    update(deltaSec) {
        if (!this.isPlaying || this.isPaused || this.waypoints.length < 2) return;

        const pA = this.waypoints[this.currentWaypointIndex];
        const nextIdx = (this.currentWaypointIndex + 1) % this.waypoints.length;
        const pB = this.waypoints[nextIdx];

        // Speed calculation based on speed percent
        const effectiveSpeed = Math.max(0.05, (this.speedPercent / 100) * 1.5);
        this.progress += deltaSec * effectiveSpeed;

        if (this.progress >= 1.0) {
            this.progress = 0;
            this.currentWaypointIndex++;

            if (this.currentWaypointIndex >= this.waypoints.length - 1) {
                if (!this.isLooping) {
                    this.isPlaying = false;
                    // Apply exact final pose
                    this.applyInterpolatedPose(pB.joints, pB.gripper);
                    return;
                }
                this.currentWaypointIndex = 0;
            }
        }

        const t = this.smoothStep(this.progress);

        if (this.motionType === 'MOVEJ') {
            // Joint Space Interpolation
            const interpolatedJoints = [];
            for (let i = 0; i < 6; i++) {
                const j = pA.joints[i] + (pB.joints[i] - pA.joints[i]) * t;
                interpolatedJoints.push(j);
            }
            const interpGripper = pA.gripper + (pB.gripper - pA.gripper) * t;
            this.applyInterpolatedPose(interpolatedJoints, interpGripper);
        } else {
            // Cartesian Linear Interpolation (MOVEL)
            const fkA = this.kinematics.forwardKinematics(pA.joints);
            const fkB = this.kinematics.forwardKinematics(pB.joints);

            const cX = fkA.cartesian.x + (fkB.cartesian.x - fkA.cartesian.x) * t;
            const cY = fkA.cartesian.y + (fkB.cartesian.y - fkA.cartesian.y) * t;
            const cZ = fkA.cartesian.z + (fkB.cartesian.z - fkA.cartesian.z) * t;
            const cW = fkA.cartesian.w + (fkB.cartesian.w - fkA.cartesian.w) * t;
            const cP = fkA.cartesian.p + (fkB.cartesian.p - fkA.cartesian.p) * t;
            const cR = fkA.cartesian.r + (fkB.cartesian.r - fkA.cartesian.r) * t;

            const ikSols = this.kinematics.inverseKinematicsAllSolutions({
                x: cX, y: cY, z: cZ, w: cW, p: cP, r: cR
            });

            if (ikSols.length > 0) {
                // Find solution closest to pA.joints
                let bestSol = ikSols[0];
                let minDiff = Infinity;
                for (const sol of ikSols) {
                    let diff = 0;
                    for (let k = 0; k < 6; k++) {
                        diff += Math.abs(sol.joints[k] - pA.joints[k]);
                    }
                    if (diff < minDiff) {
                        minDiff = diff;
                        bestSol = sol;
                    }
                }
                const interpGripper = pA.gripper + (pB.gripper - pA.gripper) * t;
                this.applyInterpolatedPose(bestSol.joints, interpGripper);
            }
        }

        // Trace visual TCP path
        if (this.robotModel.tcpNode) {
            const worldPos = new THREE.Vector3();
            this.robotModel.tcpNode.getWorldPosition(worldPos);
            this.addPathPoint(worldPos);
        }
    }

    applyInterpolatedPose(joints, gripper) {
        this.robotModel.updateJointAngles(joints);
        this.robotModel.setGripperState(gripper);
        if (this.onJointUpdate) {
            this.onJointUpdate(joints, gripper);
        }
    }
}

window.TrajectoryPlanner = TrajectoryPlanner;
