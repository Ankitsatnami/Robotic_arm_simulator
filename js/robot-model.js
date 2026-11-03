/**
 * Procedural 3D Robotic Arm Builder & Scene Entities
 * Generates ultra-high-fidelity PBR models of Fanuc CRX-10iA Cobot, UR5e, KUKA, etc.
 * Features realistic articulated joint hierarchies, pneumatic gripper fingers,
 * coordinate frame gizmos, joint rotation rings, and interactive graspable objects.
 */

class Robot3DModel {
    constructor(scene, kinematics) {
        this.scene = scene;
        this.kinematics = kinematics;
        this.rootGroup = new THREE.Group();
        this.scene.add(this.rootGroup);

        // Joint object nodes
        this.jointNodes = [];
        this.linkMeshes = [];
        this.axisHelpers = [];
        this.frameHelpers = [];
        this.gripperFingers = [];
        this.gripperOpenDistance = 45; // mm
        this.gripperState = 0.8; // 0 = closed, 1 = fully open

        // Interactive Pick & Place items
        this.graspableObjects = [];
        this.graspedObject = null;

        // Visual flags
        this.showJointAxes = true;
        this.showFrames = true;
        this.showTCPFrame = true;
        this.showWorkspaceCloud = false;
        this.workspaceCloud = null;

        this.initMaterials();
        this.buildRobot();
        this.buildTableAndEnvironment();
        this.buildGraspableObjects();
    }

    initMaterials() {
        const model = this.kinematics.getModel();
        const colors = model.colorTheme;

        // High-Vibrancy PBR Materials so robot shines in all viewports
        this.materials = {
            body: new THREE.MeshStandardMaterial({
                color: 0xf8fafc,       // Crisp Fanuc Cobot White
                roughness: 0.18,
                metalness: 0.08
            }),
            accent: new THREE.MeshStandardMaterial({
                color: 0x10b981,       // Vibrant Fanuc CRX Signature Green
                roughness: 0.2,
                metalness: 0.15,
                emissive: 0x10b981,
                emissiveIntensity: 0.3
            }),
            joints: new THREE.MeshStandardMaterial({
                color: 0x334155,       // Elegant Slate Charcoal
                roughness: 0.35,
                metalness: 0.5
            }),
            darkMetal: new THREE.MeshStandardMaterial({
                color: 0x475569,       // Brushed Steel / Aluminum
                roughness: 0.3,
                metalness: 0.65
            }),
            chrome: new THREE.MeshStandardMaterial({
                color: 0xf1f5f9,
                roughness: 0.1,
                metalness: 0.95
            }),
            gripperBody: new THREE.MeshStandardMaterial({
                color: 0x334155,
                roughness: 0.35,
                metalness: 0.6
            }),
            gripperFinger: new THREE.MeshStandardMaterial({
                color: 0x06b6d4,       // Cyan Grip Pads
                roughness: 0.25,
                metalness: 0.2,
                emissive: 0x06b6d4,
                emissiveIntensity: 0.25
            }),
            table: new THREE.MeshStandardMaterial({
                color: 0x1e222d,       // Matte Dark Worktable
                roughness: 0.7,
                metalness: 0.2
            }),
            targetPad: new THREE.MeshStandardMaterial({
                color: 0x10b981,
                transparent: true,
                opacity: 0.35,
                roughness: 0.5
            })
        };
    }

    /**
     * Build the 6-DOF Articulated Arm matching Fanuc CRX-10iA aesthetic
     */
    buildRobot() {
        // Clear previous meshes if any
        while (this.rootGroup.children.length > 0) {
            this.rootGroup.remove(this.rootGroup.children[0]);
        }

        this.jointNodes = [];
        this.linkMeshes = [];
        this.axisHelpers = [];
        this.frameHelpers = [];

        const model = this.kinematics.getModel();
        const dh = model.dh;

        // Scale factor: visual models scaled to meters (1 Three.js unit = 1000 mm)
        const scale = 0.001;

        // 1. BASE PEDESTAL (Fixed to ground)
        const baseGroup = new THREE.Group();
        this.rootGroup.add(baseGroup);

        // Mounting flange (Charcoal dark base matching screenshot)
        const baseFlangeGeo = new THREE.CylinderGeometry(0.13, 0.14, 0.04, 48);
        const baseFlange = new THREE.Mesh(baseFlangeGeo, this.materials.joints);
        baseFlange.position.y = 0.04 / 2;
        baseFlange.castShadow = true;
        baseFlange.receiveShadow = true;
        baseGroup.add(baseFlange);

        // Green base accent ring
        const baseRingGeo = new THREE.CylinderGeometry(0.128, 0.128, 0.015, 48);
        const baseRing = new THREE.Mesh(baseRingGeo, this.materials.accent);
        baseRing.position.y = 0.04 + 0.015 / 2;
        baseGroup.add(baseRing);

        // Base connector side box with dual silver pins (matching screenshot)
        const connBoxGeo = new THREE.BoxGeometry(0.04, 0.03, 0.05);
        const connBox = new THREE.Mesh(connBoxGeo, this.materials.joints);
        connBox.position.set(0.14, 0.02, 0);
        baseGroup.add(connBox);

        const pinGeo = new THREE.CylinderGeometry(0.005, 0.005, 0.035, 16);
        pinGeo.rotateZ(Math.PI / 2);
        const pin1 = new THREE.Mesh(pinGeo, this.materials.chrome);
        pin1.position.set(0.165, 0.022, 0.012);
        baseGroup.add(pin1);
        const pin2 = new THREE.Mesh(pinGeo, this.materials.chrome);
        pin2.position.set(0.165, 0.022, -0.012);
        baseGroup.add(pin2);

        // Base riser cylinder
        const baseRiserHeight = (dh[0].d * scale) - 0.05;
        const baseRiserGeo = new THREE.CylinderGeometry(0.115, 0.12, baseRiserHeight, 48);
        const baseRiser = new THREE.Mesh(baseRiserGeo, this.materials.body);
        baseRiser.position.y = 0.05 + baseRiserHeight / 2;
        baseRiser.castShadow = true;
        baseGroup.add(baseRiser);

        // JOINT 1 (Yaw: Z-axis rotation)
        const j1Node = new THREE.Group();
        j1Node.position.y = dh[0].d * scale;
        baseGroup.add(j1Node);
        this.jointNodes.push(j1Node);

        // J1 Shoulder Turret Housing
        const shoulderTurretGeo = new THREE.CylinderGeometry(0.11, 0.115, 0.14, 48);
        const shoulderTurret = new THREE.Mesh(shoulderTurretGeo, this.materials.body);
        shoulderTurret.castShadow = true;
        j1Node.add(shoulderTurret);

        // Shoulder cross-axis connector (horizontal hub for J2)
        const shoulderHubGeo = new THREE.CylinderGeometry(0.095, 0.095, 0.18, 36);
        shoulderHubGeo.rotateZ(Math.PI / 2);
        const shoulderHub = new THREE.Mesh(shoulderHubGeo, this.materials.joints);
        shoulderHub.position.set(0.05, 0.06, 0);
        shoulderHub.castShadow = true;
        j1Node.add(shoulderHub);

        // JOINT 2 (Shoulder Pitch: Y/Z-axis in Three.js)
        const j2Node = new THREE.Group();
        j2Node.position.set(0, 0.06, 0);
        j1Node.add(j2Node);
        this.jointNodes.push(j2Node);

        // Link 2: Upper Arm (Long slender aerodynamic white limb)
        const a2Len = (dh[1].a !== 0 ? Math.abs(dh[1].a) : 710) * scale;
        const upperArmGroup = new THREE.Group();
        j2Node.add(upperArmGroup);

        const upperArmCylGeo = new THREE.CylinderGeometry(0.08, 0.085, a2Len - 0.1, 40);
        const upperArmCyl = new THREE.Mesh(upperArmCylGeo, this.materials.body);
        upperArmCyl.position.y = a2Len / 2;
        upperArmCyl.castShadow = true;
        upperArmGroup.add(upperArmCyl);

        // Green styling stripe down upper arm
        const stripeGeo = new THREE.BoxGeometry(0.015, a2Len * 0.7, 0.17);
        const stripe = new THREE.Mesh(stripeGeo, this.materials.accent);
        stripe.position.y = a2Len / 2;
        upperArmGroup.add(stripe);

        // Shoulder joint cap
        const shoulderCapGeo = new THREE.CylinderGeometry(0.09, 0.09, 0.02, 36);
        shoulderCapGeo.rotateZ(Math.PI / 2);
        const shoulderCap = new THREE.Mesh(shoulderCapGeo, this.materials.accent);
        shoulderCap.position.set(0.095, 0, 0);
        upperArmGroup.add(shoulderCap);

        // JOINT 3 (Elbow Pitch)
        const j3Node = new THREE.Group();
        j3Node.position.y = a2Len;
        upperArmGroup.add(j3Node);
        this.jointNodes.push(j3Node);

        // Elbow housing hub
        const elbowHubGeo = new THREE.CylinderGeometry(0.08, 0.08, 0.16, 36);
        elbowHubGeo.rotateZ(Math.PI / 2);
        const elbowHub = new THREE.Mesh(elbowHubGeo, this.materials.joints);
        elbowHub.castShadow = true;
        j3Node.add(elbowHub);

        // Elbow green accent ring
        const elbowRingGeo = new THREE.CylinderGeometry(0.082, 0.082, 0.02, 36);
        elbowRingGeo.rotateZ(Math.PI / 2);
        const elbowRing = new THREE.Mesh(elbowRingGeo, this.materials.accent);
        j3Node.add(elbowRing);

        // Link 3 & 4: Forearm
        const d4Len = (dh[3].d !== 0 ? Math.abs(dh[3].d) : 540) * scale;
        const forearmGroup = new THREE.Group();
        j3Node.add(forearmGroup);

        const forearmCylGeo = new THREE.CylinderGeometry(0.065, 0.075, d4Len - 0.08, 36);
        const forearmCyl = new THREE.Mesh(forearmCylGeo, this.materials.body);
        forearmCyl.position.y = d4Len / 2;
        forearmCyl.castShadow = true;
        forearmGroup.add(forearmCyl);

        // JOINT 4 (Forearm Roll: axial rotation)
        const j4Node = new THREE.Group();
        j4Node.position.y = d4Len;
        forearmGroup.add(j4Node);
        this.jointNodes.push(j4Node);

        // Wrist 1 housing
        const w1Geo = new THREE.CylinderGeometry(0.062, 0.065, 0.08, 32);
        const w1Mesh = new THREE.Mesh(w1Geo, this.materials.darkMetal);
        w1Mesh.castShadow = true;
        j4Node.add(w1Mesh);

        // JOINT 5 (Wrist Pitch)
        const j5Node = new THREE.Group();
        j5Node.position.y = 0.04;
        j4Node.add(j5Node);
        this.jointNodes.push(j5Node);

        // Wrist 2 compact pitch joint
        const w2Geo = new THREE.CylinderGeometry(0.055, 0.055, 0.11, 32);
        w2Geo.rotateZ(Math.PI / 2);
        const w2Mesh = new THREE.Mesh(w2Geo, this.materials.joints);
        w2Mesh.castShadow = true;
        j5Node.add(w2Mesh);

        const w2RingGeo = new THREE.CylinderGeometry(0.057, 0.057, 0.015, 32);
        w2RingGeo.rotateZ(Math.PI / 2);
        const w2Ring = new THREE.Mesh(w2RingGeo, this.materials.accent);
        j5Node.add(w2Ring);

        // JOINT 6 (Tool Flange Twist / Roll)
        const d6Len = (dh[5].d !== 0 ? Math.abs(dh[5].d) : 160) * scale;
        const j6Node = new THREE.Group();
        j6Node.position.y = 0.05;
        j5Node.add(j6Node);
        this.jointNodes.push(j6Node);

        // Tool Flange (ISO standard circular mount)
        const flangeGeo = new THREE.CylinderGeometry(0.05, 0.05, 0.025, 32);
        const flangeMesh = new THREE.Mesh(flangeGeo, this.materials.darkMetal);
        flangeMesh.position.y = 0.025 / 2;
        flangeMesh.castShadow = true;
        j6Node.add(flangeMesh);

        // Tool Flange green ring
        const flangeRingGeo = new THREE.CylinderGeometry(0.052, 0.052, 0.008, 32);
        const flangeRing = new THREE.Mesh(flangeRingGeo, this.materials.accent);
        flangeRing.position.y = 0.015;
        j6Node.add(flangeRing);

        // BUILD PNEUMATIC GRIPPER
        this.buildGripper(j6Node, 0.025);

        // Visual Axis & Frame Helpers
        this.buildHelpers();
    }

    /**
     * Build two-finger parallel gripper
     */
    buildGripper(parent, offsetZ) {
        this.gripperGroup = new THREE.Group();
        this.gripperGroup.position.y = offsetZ;
        parent.add(this.gripperGroup);

        // Gripper actuator body
        const bodyGeo = new THREE.BoxGeometry(0.09, 0.05, 0.055);
        const body = new THREE.Mesh(bodyGeo, this.materials.gripperBody);
        body.position.y = 0.025;
        body.castShadow = true;
        this.gripperGroup.add(body);

        // Guide rails
        const railGeo = new THREE.CylinderGeometry(0.004, 0.004, 0.08, 16);
        railGeo.rotateZ(Math.PI / 2);
        const rail = new THREE.Mesh(railGeo, this.materials.chrome);
        rail.position.y = 0.035;
        this.gripperGroup.add(rail);

        // Two Finger Mounts
        this.gripperFingers = [];

        // Left Finger
        const leftFingerGroup = new THREE.Group();
        leftFingerGroup.position.set(-0.025, 0.05, 0);
        this.gripperGroup.add(leftFingerGroup);

        const fingerGeo = new THREE.BoxGeometry(0.012, 0.05, 0.02);
        const fingerLeft = new THREE.Mesh(fingerGeo, this.materials.darkMetal);
        fingerLeft.position.y = 0.025;
        fingerLeft.castShadow = true;
        leftFingerGroup.add(fingerLeft);

        const padGeo = new THREE.BoxGeometry(0.004, 0.035, 0.018);
        const padLeft = new THREE.Mesh(padGeo, this.materials.gripperFinger);
        padLeft.position.set(0.006, 0.028, 0);
        leftFingerGroup.add(padLeft);

        // Right Finger
        const rightFingerGroup = new THREE.Group();
        rightFingerGroup.position.set(0.025, 0.05, 0);
        this.gripperGroup.add(rightFingerGroup);

        const fingerRight = new THREE.Mesh(fingerGeo, this.materials.darkMetal);
        fingerRight.position.y = 0.025;
        fingerRight.castShadow = true;
        rightFingerGroup.add(fingerRight);

        const padRight = new THREE.Mesh(padGeo, this.materials.gripperFinger);
        padRight.position.set(-0.006, 0.028, 0);
        rightFingerGroup.add(padRight);

        this.gripperFingers = [leftFingerGroup, rightFingerGroup];

        // TCP Point (Tool Center Point)
        this.tcpNode = new THREE.Group();
        this.tcpNode.position.set(0, 0.085, 0); // TCP at fingertips center
        this.gripperGroup.add(this.tcpNode);

        // TCP Coordinate Frame
        const tcpAxes = new THREE.AxesHelper(0.12);
        this.tcpNode.add(tcpAxes);
        this.frameHelpers.push(tcpAxes);
    }

    /**
     * Animate gripper opening / closing
     * state: 0 (closed) to 1 (fully open)
     */
    setGripperState(state) {
        this.gripperState = Math.max(0, Math.min(1, state));
        const halfSpread = (0.008 + this.gripperState * 0.032); // 8mm to 40mm

        if (this.gripperFingers.length === 2) {
            this.gripperFingers[0].position.x = -halfSpread;
            this.gripperFingers[1].position.x = halfSpread;
        }

        // Check object grasping logic
        this.evaluateGrasping();
    }

    /**
     * Coordinate frames & Joint rotation axis indicators
     */
    buildHelpers() {
        this.axisHelpers = [];

        // Add visual rotation rings at each joint
        const jointAxesConfigs = [
            { radius: 0.14, color: 0x3b82f6, axis: 'Y' }, // J1
            { radius: 0.12, color: 0x10b981, axis: 'X' }, // J2
            { radius: 0.10, color: 0xef4444, axis: 'X' }, // J3
            { radius: 0.08, color: 0xf59e0b, axis: 'Y' }, // J4
            { radius: 0.07, color: 0x8b5cf6, axis: 'X' }, // J5
            { radius: 0.06, color: 0x06b6d4, axis: 'Y' }  // J6
        ];

        this.jointNodes.forEach((node, idx) => {
            const conf = jointAxesConfigs[idx];
            const ringGeo = new THREE.TorusGeometry(conf.radius, 0.003, 16, 48, Math.PI * 1.5);
            if (conf.axis === 'X') {
                ringGeo.rotateZ(Math.PI / 2);
            } else if (conf.axis === 'Z') {
                ringGeo.rotateX(Math.PI / 2);
            }
            const ringMat = new THREE.MeshBasicMaterial({
                color: conf.color,
                transparent: true,
                opacity: 0.8
            });
            const ringMesh = new THREE.Mesh(ringGeo, ringMat);
            node.add(ringMesh);
            this.axisHelpers.push(ringMesh);

            // Local Coordinate Frame
            const axes = new THREE.AxesHelper(0.08);
            node.add(axes);
            this.frameHelpers.push(axes);
        });

        this.updateHelperVisibility();
    }

    updateHelperVisibility() {
        this.axisHelpers.forEach(h => h.visible = this.showJointAxes);
        this.frameHelpers.forEach(f => f.visible = this.showFrames);
    }

    /**
     * Update 3D joint rotations from degree angles array [j1, j2, j3, j4, j5, j6]
     */
    updateJointAngles(jointsDeg) {
        if (!this.jointNodes || this.jointNodes.length < 6) return;

        const rads = jointsDeg.map(d => (d * Math.PI) / 180);

        // Fanuc CRX kinematics mapping to Three.js Euler axes
        // Joint 1: Base rotation around Y axis (Three.js vertical)
        this.jointNodes[0].rotation.y = -rads[0];

        // Joint 2: Shoulder pitch around X axis (pointing upwards)
        this.jointNodes[1].rotation.x = -rads[1];

        // Joint 3: Elbow pitch around X axis
        this.jointNodes[2].rotation.x = rads[2];

        // Joint 4: Forearm roll around Y axis
        this.jointNodes[3].rotation.y = rads[3];

        // Joint 5: Wrist pitch around X axis
        this.jointNodes[4].rotation.x = -rads[4];

        // Joint 6: Flange roll around Y axis
        this.jointNodes[5].rotation.y = rads[5];

        // Update grasped item if carrying one
        if (this.graspedObject) {
            const tcpWorld = new THREE.Vector3();
            this.tcpNode.getWorldPosition(tcpWorld);
            this.graspedObject.position.copy(tcpWorld);
        }
    }

    /**
     * Build Industrial Workspace Environment (Studio Dark Grid matching screenshot)
     */
    buildTableAndEnvironment() {
        const envGroup = new THREE.Group();
        this.scene.add(envGroup);

        // Ground shadow plane (receives soft robot shadows)
        const floorGeo = new THREE.PlaneGeometry(10.0, 10.0);
        floorGeo.rotateX(-Math.PI / 2);
        const floorMat = new THREE.ShadowMaterial({ opacity: 0.5 });
        const floor = new THREE.Mesh(floorGeo, floorMat);
        floor.position.y = -0.001;
        floor.receiveShadow = true;
        envGroup.add(floor);

        // Floor Grid with 100mm markings matching screenshot
        const gridHelper = new THREE.GridHelper(5.0, 50, 0x1e293b, 0x0f172a);
        gridHelper.position.y = 0;
        envGroup.add(gridHelper);

        // Ground Target Pick Zone
        const pickZoneGeo = new THREE.PlaneGeometry(0.2, 0.2);
        pickZoneGeo.rotateX(-Math.PI / 2);
        const pickZone = new THREE.Mesh(pickZoneGeo, new THREE.MeshStandardMaterial({
            color: 0x0284c7,
            transparent: true,
            opacity: 0.25
        }));
        pickZone.position.set(-0.35, 0.001, 0.35);
        envGroup.add(pickZone);

        // Ground Target Drop Zone
        const dropZone = new THREE.Mesh(pickZoneGeo, this.materials.targetPad);
        dropZone.position.set(0.35, 0.001, 0.35);
        envGroup.add(dropZone);
    }

    /**
     * Graspable sample blocks for Pick & Place interactive simulation
     */
    buildGraspableObjects() {
        this.graspableObjects = [];

        // Red Cube
        const cubeGeo = new THREE.BoxGeometry(0.045, 0.045, 0.045);
        const cubeMat = new THREE.MeshStandardMaterial({
            color: 0xef4444,
            roughness: 0.3,
            metalness: 0.2
        });
        const cube = new THREE.Mesh(cubeGeo, cubeMat);
        cube.position.set(-0.35, 0.0225, 0.35);
        cube.castShadow = true;
        cube.name = 'Red Block';
        this.scene.add(cube);
        this.graspableObjects.push(cube);

        // Cyan Cylinder
        const cylGeo = new THREE.CylinderGeometry(0.022, 0.022, 0.05, 24);
        const cylMat = new THREE.MeshStandardMaterial({
            color: 0x06b6d4,
            roughness: 0.3,
            metalness: 0.2
        });
        const cyl = new THREE.Mesh(cylGeo, cylMat);
        cyl.position.set(-0.25, 0.025, 0.35);
        cyl.castShadow = true;
        cyl.name = 'Cyan Pin';
        this.scene.add(cyl);
        this.graspableObjects.push(cyl);
    }

    /**
     * Evaluate magnetic/pneumatic grasp of objects near TCP
     */
    evaluateGrasping() {
        if (!this.tcpNode) return;

        const tcpPos = new THREE.Vector3();
        this.tcpNode.getWorldPosition(tcpPos);

        if (this.gripperState < 0.35) {
            // Gripper closed: Try to latch nearby object
            if (!this.graspedObject) {
                for (const obj of this.graspableObjects) {
                    const dist = tcpPos.distanceTo(obj.position);
                    if (dist < 0.065) {
                        this.graspedObject = obj;
                        break;
                    }
                }
            }
        } else {
            // Gripper opened: Release object
            if (this.graspedObject) {
                // Drop to table surface (y = 0.025)
                this.graspedObject.position.y = 0.025;
                this.graspedObject = null;
            }
        }
    }

    /**
     * Generate 3D Reachable Workspace Envelope Cloud
     */
    toggleWorkspaceCloud(visible) {
        this.showWorkspaceCloud = visible;
        if (this.workspaceCloud) {
            this.workspaceCloud.visible = visible;
            return;
        }

        if (!visible) return;

        // Generate Monte Carlo point cloud of forward kinematics
        const count = 4000;
        const positions = new Float32Array(count * 3);
        const colors = new Float32Array(count * 3);

        const model = this.kinematics.getModel();
        const dh = model.dh;

        for (let i = 0; i < count; i++) {
            // Random joint sample within limits
            const randomJoints = dh.map(joint => {
                return joint.min + Math.random() * (joint.max - joint.min);
            });

            const fk = this.kinematics.forwardKinematics(randomJoints);
            const x = fk.cartesian.x * 0.001;
            const y = fk.cartesian.z * 0.001; // Z in robotics is up in Three.js Y
            const z = -fk.cartesian.y * 0.001;

            positions[i * 3] = x;
            positions[i * 3 + 1] = y;
            positions[i * 3 + 2] = z;

            // Gradient color based on reach distance
            const r = Math.hypot(x, y, z);
            colors[i * 3] = 0.0;
            colors[i * 3 + 1] = 0.7 + (r * 0.2);
            colors[i * 3 + 2] = 0.9;
        }

        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));

        const mat = new THREE.PointsMaterial({
            size: 0.015,
            vertexColors: true,
            transparent: true,
            opacity: 0.4
        });

        this.workspaceCloud = new THREE.Points(geo, mat);
        this.scene.add(this.workspaceCloud);
    }

    resetObjectPositions() {
        if (this.graspableObjects[0]) {
            this.graspableObjects[0].position.set(-0.35, 0.0225, 0.45);
        }
        if (this.graspableObjects[1]) {
            this.graspableObjects[1].position.set(-0.25, 0.025, 0.55);
        }
        this.graspedObject = null;
    }
}

window.Robot3DModel = Robot3DModel;
