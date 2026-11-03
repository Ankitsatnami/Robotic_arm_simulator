/**
 * Main Application Orchestrator
 * Integrates Three.js 3D Viewport, OrbitControls, Kinematics Engine,
 * Live Matrix Renderers, Degree of Freedom (DOF) tables, Trajectory Sequencer,
 * and UI Event Handlers.
 */

class RoboticArmSimulator {
    constructor() {
        this.currentJoints = [-62, 26, 78, 69, -68, 75]; // Initial state matching user screenshot!
        this.targetJoints = [...this.currentJoints];
        this.animatingJoints = false;

        this.initEngines();
        this.init3DScene();
        this.initUI();
        this.bindEvents();
        this.updateAll(true);
        this.animate();

        // Ensure canvas has correct dimensions and renders first frame
        setTimeout(() => {
            this.onWindowResize();
            this.renderer.render(this.scene, this.camera);
        }, 120);
    }

    initEngines() {
        this.kinematics = new KinematicsEngine();
        this.learning = new LearningCenter(this.kinematics, null, (pose) => this.smoothMoveTo(pose));
    }

    init3DScene() {
        const wrapper = document.getElementById('canvas-wrapper') || document.getElementById('viewport-container');
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0c0e14);

        // Responsive dimensions
        const width = wrapper.clientWidth || (window.innerWidth - 360);
        const height = wrapper.clientHeight || (window.innerHeight - 340);
        const aspect = width / height;

        // Camera setup - Centered on robot arm
        this.camera = new THREE.PerspectiveCamera(45, aspect, 0.05, 50);
        this.camera.position.set(1.5, 0.9, 1.8);

        // WebGL Renderer with High-Quality PBR settings
        this.renderer = new THREE.WebGLRenderer({
            canvas: document.getElementById('canvas-3d'),
            antialias: true,
            alpha: false,
            powerPreference: 'high-performance'
        });
        this.renderer.setSize(width, height);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.2;

        // Orbit Controls (with fallback protection)
        if (typeof THREE.OrbitControls === 'function') {
            this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
            this.controls.enableDamping = true;
            this.controls.dampingFactor = 0.05;
            this.controls.target.set(0, 0.55, 0.1);
            this.controls.maxPolarAngle = Math.PI / 2 + 0.05;
            this.controls.minDistance = 0.3;
            this.controls.maxDistance = 6.0;
        } else {
            console.warn('OrbitControls fallback');
            this.controls = { update: () => {}, target: { set: () => {} } };
        }

        // Lighting Architecture (Studio Industrial Rig)
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.85);
        this.scene.add(ambientLight);

        const hemiLight = new THREE.HemisphereLight(0xffffff, 0x1e293b, 0.7);
        this.scene.add(hemiLight);

        // Key light with sharp soft shadows
        const keyLight = new THREE.DirectionalLight(0xffffff, 1.2);
        keyLight.position.set(2.5, 4.0, 3.0);
        keyLight.castShadow = true;
        keyLight.shadow.mapSize.width = 2048;
        keyLight.shadow.mapSize.height = 2048;
        keyLight.shadow.camera.near = 0.5;
        keyLight.shadow.camera.far = 10;
        keyLight.shadow.camera.left = -1.5;
        keyLight.shadow.camera.right = 1.5;
        keyLight.shadow.camera.top = 1.5;
        keyLight.shadow.camera.bottom = -1.5;
        keyLight.shadow.bias = -0.0005;
        this.scene.add(keyLight);

        // Cyan Rim / Back light for crisp cobot silhouette
        const rimLight = new THREE.DirectionalLight(0x06b6d4, 1.0);
        rimLight.position.set(-3.0, 2.5, -2.5);
        this.scene.add(rimLight);

        // Warm Fill Light
        const fillLight = new THREE.DirectionalLight(0x94a3b8, 0.6);
        fillLight.position.set(1.0, 0.5, -2.0);
        this.scene.add(fillLight);

        // Build 3D Robot & Environment
        this.robotModel = new Robot3DModel(this.scene, this.kinematics);
        this.learning.robotModel = this.robotModel;

        // Trajectory Planner
        this.trajectory = new TrajectoryPlanner(
            this.kinematics,
            this.robotModel,
            (joints, gripper) => {
                this.currentJoints = [...joints];
                this.updateUIJointValues();
                this.updateAll(false);
            }
        );

        // Resize Listener
        window.addEventListener('resize', () => this.onWindowResize());
        setTimeout(() => this.onWindowResize(), 100);
    }

    onWindowResize() {
        const wrapper = document.getElementById('canvas-wrapper') || document.getElementById('viewport-container');
        if (!wrapper) return;
        const width = wrapper.clientWidth;
        const height = wrapper.clientHeight;
        if (width <= 0 || height <= 0) return;
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    initUI() {
        // Populate Robot Model Select
        const modelSelect = document.getElementById('model-select');
        modelSelect.innerHTML = '';
        for (const [key, profile] of Object.entries(this.kinematics.robotProfiles)) {
            const opt = document.createElement('option');
            opt.value = key;
            opt.textContent = profile.name;
            if (key === this.kinematics.currentModelKey) opt.selected = true;
            modelSelect.appendChild(opt);
        }

        // Initialize Learning Lessons
        this.renderLessonsNav();
        this.showLesson(0);

        // Initialize Waypoints Table
        this.renderWaypointsTable();

        // Update DOF Detail Table
        this.renderDOFTable();

        // Initialize Generated Code
        this.updateGeneratedCode();
    }

    bindEvents() {
        const bind = (id, event, fn) => {
            const el = document.getElementById(id);
            if (el) el.addEventListener(event, fn);
        };

        // 1. Robot Model Change
        bind('model-select', 'change', (e) => {
            this.kinematics.setModel(e.target.value);
            this.robotModel.initMaterials();
            this.robotModel.buildRobot();
            this.renderDOFTable();
            this.currentJoints = [...this.kinematics.getModel().home];
            this.updateAll(true);
        });

        // 2. Joint Sliders & Steppers
        for (let i = 1; i <= 6; i++) {
            const slider = document.getElementById(`slider-j${i}`);
            const decBtn = document.getElementById(`btn-dec-j${i}`);
            const incBtn = document.getElementById(`btn-inc-j${i}`);

            if (slider) {
                slider.addEventListener('input', (e) => {
                    this.currentJoints[i - 1] = parseFloat(e.target.value);
                    this.updateAll(true);
                });
            }

            if (decBtn && slider) {
                decBtn.addEventListener('click', () => {
                    this.currentJoints[i - 1] = Math.max(slider.min, this.currentJoints[i - 1] - 1);
                    slider.value = this.currentJoints[i - 1];
                    this.updateAll(true);
                });
            }

            if (incBtn && slider) {
                incBtn.addEventListener('click', () => {
                    this.currentJoints[i - 1] = Math.min(slider.max, this.currentJoints[i - 1] + 1);
                    slider.value = this.currentJoints[i - 1];
                    this.updateAll(true);
                });
            }
        }

        // 3. Cartesian Target Inputs & Solve IK
        const cartesianInputs = ['x', 'y', 'z', 'w', 'p', 'r'];
        cartesianInputs.forEach(axis => {
            bind(`input-${axis}`, 'change', () => this.solveAndApplyIK());
        });

        bind('btn-solve-ik', 'click', () => this.solveAndApplyIK());

        // 4. Quick Pose Buttons
        bind('btn-zero', 'click', () => this.smoothMoveTo([0, 0, 0, 0, 0, 0]));
        bind('btn-home', 'click', () => this.smoothMoveTo(this.kinematics.getModel().home));
        bind('btn-ready', 'click', () => this.smoothMoveTo(this.kinematics.getModel().ready));
        bind('btn-screenshot-pose', 'click', () => this.smoothMoveTo([-62, 26, 78, 69, -68, 75]));

        // 5. Available Solution Pills
        const solButtons = document.querySelectorAll('.sol-btn');
        solButtons.forEach(btn => {
            btn.addEventListener('click', () => {
                const targetConfigCode = btn.getAttribute('data-config');
                this.applySpecificSolution(targetConfigCode);
            });
        });

        // 6. Gripper Controls
        const gripperSlider = document.getElementById('gripper-slider');
        if (gripperSlider) {
            gripperSlider.addEventListener('input', (e) => {
                const val = parseFloat(e.target.value) / 100;
                this.robotModel.setGripperState(val);
                this.updateGripperButtons(val);
            });
        }

        bind('btn-grip-open', 'click', () => {
            if (gripperSlider) gripperSlider.value = 100;
            this.robotModel.setGripperState(1.0);
            this.updateGripperButtons(1.0);
        });

        bind('btn-grip-close', 'click', () => {
            if (gripperSlider) gripperSlider.value = 0;
            this.robotModel.setGripperState(0.0);
            this.updateGripperButtons(0.0);
        });

        bind('btn-reset-objects', 'click', () => {
            this.robotModel.resetObjectPositions();
        });

        // 7. Viewport Camera Preset Buttons
        bind('btn-view-persp', 'click', () => this.setCameraView(1.5, 0.9, 1.8, 0, 0.55, 0));
        bind('btn-view-top', 'click', () => this.setCameraView(0.01, 2.5, 0.1, 0, 0, 0.1));
        bind('btn-view-front', 'click', () => this.setCameraView(0, 0.7, 2.0, 0, 0.6, 0));
        bind('btn-view-side', 'click', () => this.setCameraView(2.0, 0.7, 0.1, 0, 0.6, 0.1));

        // 8. Visual Overlay Toggles
        const btnToggleAxes = document.getElementById('btn-toggle-axes');
        if (btnToggleAxes) {
            btnToggleAxes.addEventListener('click', (e) => {
                this.robotModel.showJointAxes = !this.robotModel.showJointAxes;
                e.currentTarget.classList.toggle('active', this.robotModel.showJointAxes);
                this.robotModel.updateHelperVisibility();
            });
        }

        const btnToggleFrames = document.getElementById('btn-toggle-frames');
        if (btnToggleFrames) {
            btnToggleFrames.addEventListener('click', (e) => {
                this.robotModel.showFrames = !this.robotModel.showFrames;
                e.currentTarget.classList.toggle('active', this.robotModel.showFrames);
                this.robotModel.updateHelperVisibility();
            });
        }

        const btnToggleCloud = document.getElementById('btn-toggle-cloud');
        if (btnToggleCloud) {
            btnToggleCloud.addEventListener('click', (e) => {
                const active = !this.robotModel.showWorkspaceCloud;
                e.currentTarget.classList.toggle('active', active);
                this.robotModel.toggleWorkspaceCloud(active);
            });
        }

        // 9. Bottom Drawer Tabs
        const drawerTabs = document.querySelectorAll('.drawer-tab');
        drawerTabs.forEach(tab => {
            tab.addEventListener('click', () => {
                drawerTabs.forEach(t => t.classList.remove('active'));
                tab.classList.add('active');

                const drawer = document.getElementById('bottom-drawer');
                if (drawer && drawer.classList.contains('collapsed')) {
                    drawer.classList.remove('collapsed');
                    const toggleBtn = document.getElementById('btn-toggle-drawer');
                    if (toggleBtn) toggleBtn.textContent = '▼ Collapse Details Panel';
                    setTimeout(() => this.onWindowResize(), 220);
                }

                const targetPaneId = tab.getAttribute('data-target');
                document.querySelectorAll('.drawer-content-pane').forEach(p => p.classList.remove('active'));
                const pane = document.getElementById(targetPaneId);
                if (pane) pane.classList.add('active');

                if (targetPaneId === 'pane-code') {
                    this.updateGeneratedCode();
                }
            });
        });

        // Drawer collapse / expand
        bind('btn-toggle-drawer', 'click', () => {
            const drawer = document.getElementById('bottom-drawer');
            if (!drawer) return;
            drawer.classList.toggle('collapsed');
            const isCollapsed = drawer.classList.contains('collapsed');
            const toggleBtn = document.getElementById('btn-toggle-drawer');
            if (toggleBtn) {
                toggleBtn.textContent = isCollapsed ? '▲ Expand Details Panel' : '▼ Collapse Details Panel';
            }
            setTimeout(() => this.onWindowResize(), 220);
        });

        // 10. Trajectory Controls
        bind('btn-traj-play', 'click', () => this.trajectory.play());
        bind('btn-traj-pause', 'click', () => this.trajectory.pause());
        bind('btn-traj-stop', 'click', () => this.trajectory.stop());

        const btnLoop = document.getElementById('btn-traj-loop');
        if (btnLoop) {
            btnLoop.addEventListener('click', (e) => {
                this.trajectory.isLooping = !this.trajectory.isLooping;
                e.currentTarget.classList.toggle('active', this.trajectory.isLooping);
            });
        }

        bind('motion-type-select', 'change', (e) => {
            this.trajectory.motionType = e.target.value;
        });

        const speedSlider = document.getElementById('traj-speed-slider');
        if (speedSlider) {
            speedSlider.addEventListener('input', (e) => {
                this.trajectory.speedPercent = parseInt(e.target.value);
                const display = document.getElementById('traj-speed-val');
                if (display) display.textContent = `${e.target.value}%`;
            });
        }

        bind('btn-add-waypoint', 'click', () => {
            this.trajectory.recordCurrentAsWaypoint(this.currentJoints, this.robotModel.gripperState);
            this.renderWaypointsTable();
        });

        bind('btn-clear-trace', 'click', () => {
            this.trajectory.clearPathTrace();
        });

        // 11. Code Generator Language Select & Copy
        bind('code-lang-select', 'change', () => this.updateGeneratedCode());

        bind('btn-copy-code', 'click', () => {
            const block = document.getElementById('code-output-block');
            if (!block) return;
            navigator.clipboard.writeText(block.textContent).then(() => {
                const btn = document.getElementById('btn-copy-code');
                if (btn) {
                    const orig = btn.textContent;
                    btn.textContent = 'Copied!';
                    setTimeout(() => btn.textContent = orig, 1500);
                }
            });
        });

        // 12. Info Modal
        const infoModal = document.getElementById('info-modal');
        bind('btn-info', 'click', () => {
            if (infoModal) infoModal.style.display = 'flex';
        });
        bind('btn-close-modal', 'click', () => {
            if (infoModal) infoModal.style.display = 'none';
        });
        if (infoModal) {
            infoModal.addEventListener('click', (e) => {
                if (e.target === infoModal) infoModal.style.display = 'none';
            });
        }
    }

    setCameraView(px, py, pz, tx, ty, tz) {
        gsap.to(this.camera.position, {
            x: px, y: py, z: pz,
            duration: 0.8,
            ease: 'power2.inOut'
        });
        gsap.to(this.controls.target, {
            x: tx, y: ty, z: tz,
            duration: 0.8,
            ease: 'power2.inOut',
            onUpdate: () => this.controls.update()
        });
    }

    updateGripperButtons(val) {
        document.getElementById('btn-grip-open').classList.toggle('active', val > 0.5);
        document.getElementById('btn-grip-close').classList.toggle('active', val <= 0.5);
    }

    updateUIJointValues() {
        for (let i = 1; i <= 6; i++) {
            const val = this.currentJoints[i - 1];
            const slider = document.getElementById(`slider-j${i}`);
            const display = document.getElementById(`val-j${i}`);
            if (slider && display) {
                slider.value = val;
                display.textContent = `${Math.round(val)}°`;
            }
        }
    }

    /**
     * Master update pipeline:
     * 1. 3D visual robot arm articulation
     * 2. Forward Kinematics calculation
     * 3. Cartesian target inputs (X, Y, Z, W, P, R)
     * 4. Current Configuration string
     * 5. Available 8 Solutions highlighting
     * 6. 4x4 Transformation Matrix & DH table
     * 7. 6x6 Jacobian Matrix & Singularity diagnostics
     */
    updateAll(updateInputs = true) {
        // 1. 3D mesh joint updates
        this.robotModel.updateJointAngles(this.currentJoints);

        // 2. Forward Kinematics
        const fk = this.kinematics.forwardKinematics(this.currentJoints);

        // 3. Cartesian UI Display
        if (updateInputs) {
            document.getElementById('input-x').value = fk.cartesian.x;
            document.getElementById('input-y').value = fk.cartesian.y;
            document.getElementById('input-z').value = fk.cartesian.z;
            document.getElementById('input-w').value = fk.cartesian.w;
            document.getElementById('input-p').value = fk.cartesian.p;
            document.getElementById('input-r').value = fk.cartesian.r;
        }

        // 4. Current Configuration
        const configBanner = document.getElementById('current-config-display');
        configBanner.textContent = `Current Config: ${fk.config.fullString}`;

        // 5. Update Solution Pills Highlight
        const activeConfigCode = fk.config.configKey; // e.g. "N D T"
        document.querySelectorAll('.sol-btn').forEach(btn => {
            const btnCode = btn.getAttribute('data-config');
            btn.classList.toggle('active', btnCode === activeConfigCode);
        });

        // 6. Update Live Transformation Matrix Tab
        this.renderLiveMatrix(fk.T06, fk.cumulativeMatrices);

        // 7. Update Live Jacobian & Singularities
        const jac = this.kinematics.computeJacobian(this.currentJoints);
        this.renderJacobian(jac);

        // 8. Update DH Parameters Live Values
        this.updateLiveDHTable();

        // 9. Update Joint Readout Labels
        this.updateUIJointValues();
    }

    /**
     * Render 4x4 Homogeneous Transformation Matrix
     */
    renderLiveMatrix(T06, cumulative) {
        const matrixBody = document.getElementById('live-matrix-body');
        if (!matrixBody) return;

        let html = '';
        for (let row = 0; row < 4; row++) {
            html += '<tr>';
            for (let col = 0; col < 4; col++) {
                const val = T06[row][col];
                const formatted = val.toFixed(3);
                const isPos = col === 3 && row < 3;
                const isRot = col < 3 && row < 3;
                const cls = isPos ? 'pos-col' : (isRot ? 'rot-col' : '');
                html += `<td class="${cls}">${formatted}</td>`;
            }
            html += '</tr>';
        }
        matrixBody.innerHTML = html;

        // Tool coordinates summary
        document.getElementById('mat-summary-pos').textContent =
            `P = [ ${T06[0][3].toFixed(1)}, ${T06[1][3].toFixed(1)}, ${T06[2][3].toFixed(1)} ] mm`;
    }

    /**
     * Render 6x6 Jacobian & Singularity Warnings
     */
    renderJacobian(jac) {
        const jacContainer = document.getElementById('jacobian-cells');
        if (!jacContainer) return;

        let html = '';
        for (let r = 0; r < 6; r++) {
            for (let c = 0; c < 6; c++) {
                const val = jac.matrix[r][c].toFixed(2);
                html += `<div class="jac-cell">${val}</div>`;
            }
        }
        jacContainer.innerHTML = html;

        // Manipulability badge
        document.getElementById('jac-manipulability').textContent =
            `Manipulability Index (μ): ${jac.manipulability}`;

        // Singularity Status Indicators
        const dot = document.getElementById('singularity-status-dot');
        const text = document.getElementById('singularity-status-text');

        if (jac.singularities.wrist) {
            dot.className = 'status-dot danger';
            text.innerHTML = '<span style="color:var(--accent-red)">WRIST SINGULARITY (J5 ≈ 0°)</span>';
        } else if (jac.singularities.shoulder) {
            dot.className = 'status-dot warning';
            text.innerHTML = '<span style="color:var(--accent-orange)">SHOULDER SINGULARITY (Near Center)</span>';
        } else if (jac.singularities.boundary) {
            dot.className = 'status-dot warning';
            text.innerHTML = '<span style="color:var(--accent-orange)">BOUNDARY SINGULARITY (Max Reach)</span>';
        } else {
            dot.className = 'status-dot';
            text.innerHTML = '<span style="color:var(--accent-green)">KINEMATICS NOMINAL (Dexterous)</span>';
        }
    }

    /**
     * Update Degree of Freedom (DOF) Specifications Table
     */
    renderDOFTable() {
        const tableBody = document.getElementById('dof-table-body');
        if (!tableBody) return;

        const model = this.kinematics.getModel();
        const dh = model.dh;

        let html = '';
        dh.forEach((joint, idx) => {
            html += `
                <tr>
                    <td><span class="dof-badge">J${idx + 1}</span></td>
                    <td><strong>${joint.name}</strong></td>
                    <td>Revolute</td>
                    <td>${idx === 0 || idx === 3 || idx === 5 ? 'Yaw / Roll' : 'Pitch'}</td>
                    <td>${joint.min}° to +${joint.max}°</td>
                    <td>${joint.speed} °/s</td>
                    <td id="dof-live-val-${idx}">${this.currentJoints[idx]}°</td>
                </tr>
            `;
        });
        tableBody.innerHTML = html;
    }

    updateLiveDHTable() {
        for (let i = 0; i < 6; i++) {
            const cell = document.getElementById(`dof-live-val-${i}`);
            if (cell) {
                cell.textContent = `${Math.round(this.currentJoints[i])}°`;
            }
        }
    }

    /**
     * Solve Inverse Kinematics from current Cartesian target inputs
     */
    solveAndApplyIK() {
        const target = {
            x: parseFloat(document.getElementById('input-x').value) || 0,
            y: parseFloat(document.getElementById('input-y').value) || 0,
            z: parseFloat(document.getElementById('input-z').value) || 0,
            w: parseFloat(document.getElementById('input-w').value) || 0,
            p: parseFloat(document.getElementById('input-p').value) || 0,
            r: parseFloat(document.getElementById('input-r').value) || 0
        };

        const solutions = this.kinematics.inverseKinematicsAllSolutions(target);

        if (solutions.length === 0) {
            alert('Cartesian target is outside reachable workspace envelope!');
            return;
        }

        // Find solution closest to current joint angles to minimize movement
        let bestSol = solutions[0];
        let minDiff = Infinity;

        for (const sol of solutions) {
            let diff = 0;
            for (let i = 0; i < 6; i++) {
                diff += Math.abs(sol.joints[i] - this.currentJoints[i]);
            }
            if (diff < minDiff) {
                minDiff = diff;
                bestSol = sol;
            }
        }

        this.smoothMoveTo(bestSol.joints);
    }

    /**
     * Apply specific analytical solution from the 8 configuration pills
     */
    applySpecificSolution(configCode) {
        const fk = this.kinematics.forwardKinematics(this.currentJoints);
        const solutions = this.kinematics.inverseKinematicsAllSolutions(fk.cartesian);

        const match = solutions.find(s => s.configCode === configCode);
        if (match) {
            this.smoothMoveTo(match.joints);
        } else {
            console.warn(`Configuration ${configCode} unreachable for current target`);
        }
    }

    /**
     * Smoothly animate robot joints from current state to target joints
     */
    smoothMoveTo(targetJointsDeg) {
        if (this.animatingJoints) return;
        this.animatingJoints = true;

        const startJoints = [...this.currentJoints];
        const tweenObj = { t: 0 };

        gsap.to(tweenObj, {
            t: 1,
            duration: 0.7,
            ease: 'power2.out',
            onUpdate: () => {
                for (let i = 0; i < 6; i++) {
                    this.currentJoints[i] = startJoints[i] + (targetJointsDeg[i] - startJoints[i]) * tweenObj.t;
                }
                this.updateAll(true);
            },
            onComplete: () => {
                this.currentJoints = [...targetJointsDeg];
                this.updateAll(true);
                this.animatingJoints = false;
            }
        });
    }

    /**
     * Learning Curriculum Rendering
     */
    renderLessonsNav() {
        const navContainer = document.getElementById('lesson-nav-list');
        if (!navContainer) return;

        navContainer.innerHTML = '';
        this.learning.lessons.forEach((lesson, idx) => {
            const item = document.createElement('div');
            item.className = `lesson-nav-item ${idx === 0 ? 'active' : ''}`;
            item.innerHTML = `
                <div class="lesson-nav-title">${lesson.title}</div>
                <div class="lesson-nav-desc">${lesson.subtitle}</div>
            `;
            item.addEventListener('click', () => {
                document.querySelectorAll('.lesson-nav-item').forEach(el => el.classList.remove('active'));
                item.classList.add('active');
                this.showLesson(idx);
            });
            navContainer.appendChild(item);
        });
    }

    showLesson(idx) {
        const lesson = this.learning.lessons[idx];
        const bodyContainer = document.getElementById('lesson-body-pane');
        if (!bodyContainer || !lesson) return;

        bodyContainer.innerHTML = `
            ${lesson.content}
            <div style="margin-top:16px;">
                <button class="primary-btn" id="btn-lesson-action" style="padding:0 16px;">
                    ▶ ${lesson.actionLabel}
                </button>
            </div>
        `;

        const actionBtn = document.getElementById('btn-lesson-action');
        if (actionBtn) {
            actionBtn.addEventListener('click', () => {
                this.smoothMoveTo(lesson.demoPose);
            });
        }

        // Re-render KaTeX if loaded
        if (window.renderMathInElement) {
            try {
                renderMathInElement(bodyContainer, {
                    delimiters: [
                        {left: '$$', right: '$$', display: true},
                        {left: '$', right: '$', display: false}
                    ]
                });
            } catch (err) {
                console.warn('Math typesetting note:', err);
            }
        }
    }

    /**
     * Trajectory Waypoints Table
     */
    renderWaypointsTable() {
        const tbody = document.getElementById('waypoint-table-body');
        if (!tbody) return;

        let html = '';
        this.trajectory.waypoints.forEach((wp, idx) => {
            const jStr = wp.joints.map(j => Math.round(j)).join(', ');
            html += `
                <tr class="waypoint-row ${this.trajectory.currentWaypointIndex === idx ? 'active' : ''}">
                    <td><strong>${wp.name}</strong></td>
                    <td>[${jStr}]</td>
                    <td>${wp.gripper > 0.5 ? 'Open' : 'Clamped'}</td>
                    <td>${wp.speed}%</td>
                    <td>
                        <button class="joint-step-btn" onclick="simulator.removeWaypoint(${idx})" style="display:inline-flex;">✕</button>
                        <button class="joint-step-btn" onclick="simulator.smoothMoveTo([${wp.joints.join(',')}])" style="display:inline-flex;">▶</button>
                    </td>
                </tr>
            `;
        });
        tbody.innerHTML = html;
    }

    removeWaypoint(idx) {
        this.trajectory.removeWaypoint(idx);
        this.renderWaypointsTable();
    }

    /**
     * Multi-Language Code Generator
     */
    updateGeneratedCode() {
        const lang = document.getElementById('code-lang-select').value;
        const fk = this.kinematics.forwardKinematics(this.currentJoints);
        const code = this.learning.generateCode(lang, this.currentJoints, fk.cartesian, fk.config);
        document.getElementById('code-output-block').textContent = code;
    }

    /**
     * Main Animation & Render Loop
     */
    animate() {
        requestAnimationFrame(() => this.animate());

        // Update OrbitControls
        this.controls.update();

        // Update Trajectory Sequencer (delta approx 0.016s)
        this.trajectory.update(0.016);

        // Render 3D Scene
        this.renderer.render(this.scene, this.camera);
    }
}

// Global robust initialization with fallback
function startSimulator() {
    if (window.simulator) return;
    try {
        window.simulator = new RoboticArmSimulator();
        console.log('Fanuc Cobot Kinematics Simulator initialized successfully!');
    } catch (err) {
        console.error('Fatal initialization error:', err);
        const banner = document.createElement('div');
        banner.style = 'position:fixed;top:0;left:0;right:0;background:#dc2626;color:#fff;padding:12px;z-index:999999;font-family:sans-serif;font-size:13px;text-align:center;box-shadow:0 4px 12px rgba(0,0,0,0.5);';
        banner.innerHTML = '<strong>Simulator Init Error:</strong> ' + err.message;
        document.body.prepend(banner);
    }
}

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', startSimulator);
} else {
    startSimulator();
}
window.addEventListener('load', startSimulator);
