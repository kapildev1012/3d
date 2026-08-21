/**
 * VISUALIZER.JS - High-Fidelity 3D Visualizer for Spherical Tensegrity Rover
 *
 * Renders:
 * - 3D Spherical Tensegrity Cage (1.0D Outer Envelope)
 * - Suspended Central Payload Core (0.1D Inner Diameter) & 12 Suspension Cables
 * - Color-coded Dynamic Force Visualization (Cable Tension & Strut Compression)
 * - Mars-like Terrain, Rocks, Craters, Incline Ramps & Ledges
 * - Trajectory Trail, COM Marker, Velocity Vector & Geometry Checkpoint Overlays
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

export class Visualizer {
  constructor(containerElement, roverModel, terrainModel, options = {}) {
    this.container = containerElement;
    this.roverModel = roverModel;
    this.terrainModel = terrainModel;
    this.laneOffset = terrainModel.cfg.modelLaneOffset || 1.5;
    this.options = Object.assign({
      title: 'Simulation View',
      cameraMode: 'chase', // 'chase', 'orbit', 'top', 'side'
      showForceOverlay: true,
      showGeometryCheckpoint: false,
      showDebugLabels: false,
      selectedCable: 0
    }, options);

    this.initThree();
    this.createTerrainMesh();
    this.createRoverObjects();
    this.createTrajectoryTrail();
    this.createGeometryCheckpointOverlay();
  }

  initThree() {
    if (this.resizeObserver) this.resizeObserver.disconnect();
    if (this.renderer) this.renderer.dispose();
    const width = this.container.clientWidth || 600;
    const height = this.container.clientHeight || 450;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x24130e);
    this.scene.fog = new THREE.FogExp2(0x3a1f16, 0.012);

    // Camera (Z is UP in physics simulation)
    this.camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 200);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(-4, -6, 4);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    this.container.innerHTML = '';
    this.container.appendChild(this.renderer.domElement);

    // Orbit Controls with Z-up constraint
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.maxPolarAngle = Math.PI / 2 + 0.05;
    this.controls.target.set(0, 3, 1.2);
    this.controls.update();

    // Warm, dusty Mars lighting based on the supplied rocky-landscape photo.
    const ambLight = new THREE.AmbientLight(0xffd5b8, 0.62);
    this.scene.add(ambLight);

    const sunLight = new THREE.DirectionalLight(0xffe8d6, 1.5);
    sunLight.position.set(12, -15, 25);
    sunLight.castShadow = true;
    sunLight.shadow.mapSize.width = 2048;
    sunLight.shadow.mapSize.height = 2048;
    sunLight.shadow.camera.near = 0.5;
    sunLight.shadow.camera.far = 120;
    sunLight.shadow.camera.left = -12;
    sunLight.shadow.camera.right = 12;
    sunLight.shadow.camera.top = 50;
    sunLight.shadow.camera.bottom = -10;
    this.scene.add(sunLight);

    const hemiLight = new THREE.HemisphereLight(0xd99a75, 0x3b1d15, 0.58);
    hemiLight.position.set(0, 0, 15);
    this.scene.add(hemiLight);

    // Handle Resize
    this.resizeObserver = new ResizeObserver(() => this.onResize());
    this.resizeObserver.observe(this.container);
  }

  onResize() {
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;
    if (width === 0 || height === 0) return;

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  createTerrainMesh() {
    const Nx = 80;
    const Ny = this.terrainModel.course ? 320 : 240;
    const xMin = this.terrainModel.course ? -5 : -8;
    const xMax = this.terrainModel.course ? 5 : 8;
    const yMin = this.terrainModel.course ? 0 : -4;
    const yMax = this.terrainModel.course ? 70 : 45;

    const geometry = new THREE.PlaneGeometry(xMax - xMin, yMax - yMin, Nx, Ny);
    const posAttr = geometry.attributes.position;
    const colors = [];

    // Photo-matched granular Mars palette: iron-rich sand, sunlit ridges,
    // and darker exposed stone on steeper faces.
    const sandShadow = new THREE.Color(0x5a2d20);
    const sandMid = new THREE.Color(0x99563c);
    const sandSun = new THREE.Color(0xc47b55);
    const exposedRock = new THREE.Color(0x3e2924);

    for (let i = 0; i < posAttr.count; i++) {
      const localX = posAttr.getX(i);
      const localY = posAttr.getY(i);
      const worldX = localX;
      const worldY = localY + (yMin + yMax) / 2;

      // Every level is rendered as two independent copies of the same
      // lane-local physical terrain: A on the left, B on the right. Levels
      // 1–9 previously sampled unshifted world x here, so visible rocks and
      // the collision surface disagreed with both rover simulations.
      const laneOffset = worldX < 0 ? -this.laneOffset : this.laneOffset;
      const physicsX = worldX-laneOffset;
      const laneModel = worldX >= 0 ? 'adaptive' : 'fixed';
      const surf = this.terrainModel.eval(physicsX, worldY, laneModel);
      posAttr.setZ(i, surf.h);

      const heightRatio = Math.max(0, Math.min(1, (surf.h+0.28)/0.85));
      const slope = Math.hypot(surf.dhdx, surf.dhdy);
      const grainHash = Math.sin(127.1*physicsX+311.7*worldY)*43758.5453;
      const granularNoise = grainHash-Math.floor(grainHash);
      const grain = 0.88+0.16*granularNoise;
      const c = sandShadow.clone().lerp(sandMid, Math.min(1, 0.30+1.1*heightRatio));
      c.lerp(sandSun, Math.max(0, Math.min(0.55, 0.35*heightRatio)));
      c.lerp(exposedRock, Math.max(0, Math.min(0.58, (slope-0.20)*0.85)));
      c.multiplyScalar(Math.max(0.76, grain));
      colors.push(c.r, c.g, c.b);
    }

    geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
    geometry.computeVertexNormals();

    const material = new THREE.MeshStandardMaterial({
      vertexColors: true,
      roughness: 1.0,
      metalness: 0.0,
      flatShading: true
    });

    this.terrainMesh = new THREE.Mesh(geometry, material);
    this.terrainMesh.position.set(0, (yMin + yMax) / 2, 0);
    this.terrainMesh.receiveShadow = true;
    this.scene.add(this.terrainMesh);

    // Glowing Finish Line Gate Arch at the configured endpoint.
    const goalY = this.terrainModel.course?.goalY || this.terrainModel.cfg.targetGoalY || 25.0;
    const gateGroup = new THREE.Group();
    const pillarGeom = new THREE.CylinderGeometry(0.12, 0.15, 3.5, 16);
    const pillarMat = new THREE.MeshStandardMaterial({
      color: 0x10b981,
      metalness: 0.8,
      roughness: 0.2,
      emissive: 0x059669,
      emissiveIntensity: 0.8
    });

    const pillarLeft = new THREE.Mesh(pillarGeom, pillarMat);
    pillarLeft.position.set(-3.0, goalY, 1.75 + (this.terrainModel.eval(-3.0, goalY).h || 0));
    gateGroup.add(pillarLeft);

    const pillarRight = new THREE.Mesh(pillarGeom, pillarMat);
    pillarRight.position.set(3.0, goalY, 1.75 + (this.terrainModel.eval(3.0, goalY).h || 0));
    gateGroup.add(pillarRight);

    const archGeom = new THREE.CylinderGeometry(0.08, 0.08, 6.0, 16);
    const archBar = new THREE.Mesh(archGeom, pillarMat);
    archBar.rotation.z = Math.PI / 2;
    archBar.position.set(0, goalY, 3.5 + (this.terrainModel.eval(0, goalY).h || 0));
    gateGroup.add(archBar);

    // Glowing Finish Grid Banner Line
    const finishLineGeom = new THREE.PlaneGeometry(6.0, 0.4);
    const finishLineMat = new THREE.MeshBasicMaterial({
      color: 0x10b981,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.7
    });
    const finishLine = new THREE.Mesh(finishLineGeom, finishLineMat);
    finishLine.position.set(0, goalY, 0.02 + (this.terrainModel.eval(0, goalY).h || 0));
    gateGroup.add(finishLine);

    this.scene.add(gateGroup);

    // Explicit task markers for the monitored adaptive lane.
    const startYMarker = this.terrainModel.course
      ? this.terrainModel.course.startY-1.0 : 0;
    const goalTarget = this.terrainModel.cfg.targetDestination || [0, goalY];
    const taskMarkerGroup = new THREE.Group();
    const startMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.16, 20, 20),
      new THREE.MeshStandardMaterial({ color: 0x38bdf8, emissive: 0x0284c7, emissiveIntensity: 1.0 })
    );
    startMarker.position.set(this.laneOffset, startYMarker, this.terrainModel.eval(0, startYMarker, 'adaptive').h+0.18);
    taskMarkerGroup.add(startMarker);
    const goalMarker = new THREE.Mesh(
      new THREE.TorusGeometry(0.30, 0.055, 12, 32),
      new THREE.MeshStandardMaterial({ color: 0x22c55e, emissive: 0x16a34a, emissiveIntensity: 1.2 })
    );
    goalMarker.position.set(goalTarget[0]+this.laneOffset, goalTarget[1], this.terrainModel.eval(goalTarget[0], goalTarget[1], 'adaptive').h+0.08);
    taskMarkerGroup.add(goalMarker);
    const startLabel = this.createTextSprite('START A', 'rgba(56, 189, 248, 1)');
    startLabel.scale.set(1.4, 0.35, 1);
    startLabel.position.set(this.laneOffset, startYMarker, this.terrainModel.eval(0, startYMarker, 'adaptive').h+0.62);
    taskMarkerGroup.add(startLabel);
    const goalLabel = this.createTextSprite('GOAL B', 'rgba(34, 197, 94, 1)');
    goalLabel.scale.set(1.4, 0.35, 1);
    goalLabel.position.set(goalTarget[0]+this.laneOffset, goalTarget[1], this.terrainModel.eval(goalTarget[0], goalTarget[1], 'adaptive').h+0.62);
    taskMarkerGroup.add(goalLabel);
    this.taskMarkerGroup = taskMarkerGroup;
    this.scene.add(taskMarkerGroup);

    if (this.terrainModel.course) {
      const startY = this.terrainModel.course.startY;
      const startMaterial = new THREE.MeshBasicMaterial({
        color: 0x38bdf8, side: THREE.DoubleSide, transparent: true, opacity: 0.72
      });
      const startLine = new THREE.Mesh(new THREE.PlaneGeometry(6, 0.28), startMaterial);
      startLine.position.set(0, startY, this.terrainModel.eval(0, startY).h+0.025);
      this.scene.add(startLine);

      // Build every benchmark obstacle from a small cluster of embedded
      // low-poly shards. The physical terrain underneath uses the same
      // asymmetric obstacle profile, while these varied facets remove the
      // former round, manufactured-looking silhouette.
      for (let obstacleIndex = 0; obstacleIndex < this.terrainModel.course.obstacles.length; obstacleIndex++) {
        const obstacle = this.terrainModel.course.obstacles[obstacleIndex];
        for (const laneOffset of [-this.laneOffset, this.laneOffset]) {
          for (let shard = 0; shard < 5; shard++) {
            const angle = obstacle.yaw+shard*2.39996+obstacleIndex*0.47;
            const radial = shard === 0 ? 0 : 0.18+0.10*((shard+obstacleIndex)%3);
            const localX = obstacle.x+radial*obstacle.radiusX*Math.cos(angle);
            const localY = obstacle.y+radial*obstacle.radiusY*Math.sin(angle);
            const shardHeight = obstacle.height*(shard === 0 ? 0.34 : 0.18+0.035*((shard+2*obstacleIndex)%3));
            const surface = this.terrainModel.eval(localX, localY);
            const geometry = shard%2 === 0
              ? new THREE.DodecahedronGeometry(1, 0)
              : new THREE.IcosahedronGeometry(1, 0);
            const material = new THREE.MeshStandardMaterial({
              color: new THREE.Color(0x382923).lerp(
                new THREE.Color(0x76503d), 0.18+0.10*((shard+obstacleIndex)%4)),
              roughness: 1.0,
              metalness: 0.0,
              flatShading: true
            });
            const mesh = new THREE.Mesh(geometry, material);
            const widthScale = obstacle.radiusX*(shard === 0 ? 0.38 : 0.23+0.035*(shard%3));
            const depthScale = obstacle.radiusY*(shard === 0 ? 0.31 : 0.20+0.03*((shard+1)%3));
            mesh.scale.set(widthScale, depthScale, shardHeight);
            // Z-only rotation keeps every facet below the analytic solid
            // surface; x/y tilt used to lift corners above collision height.
            mesh.rotation.set(0, 0, angle);
            mesh.position.set(localX+laneOffset, localY, surface.h-shardHeight);
            mesh.castShadow = true;
            mesh.receiveShadow = true;
            this.scene.add(mesh);
          }
        }

        // Model B's mandatory sequential crest target. The controller unlocks
        // B1…B10 in order, then proceeds to the final GOAL B marker.
        const checkpointRing = new THREE.Mesh(
          new THREE.TorusGeometry(0.24, 0.035, 10, 26),
          new THREE.MeshStandardMaterial({
            color: 0x22d3ee,
            emissive: 0x0891b2,
            emissiveIntensity: 1.35,
            roughness: 0.35
          })
        );
        checkpointRing.position.set(
          obstacle.x+this.laneOffset,
          obstacle.y,
          this.terrainModel.eval(obstacle.x, obstacle.y, 'adaptive').h+0.06
        );
        this.scene.add(checkpointRing);
        const checkpointLabel = this.createTextSprite(`B${obstacleIndex+1}`, 'rgba(34, 211, 238, 1)');
        checkpointLabel.scale.set(0.72, 0.22, 1);
        checkpointLabel.position.set(
          obstacle.x+this.laneOffset,
          obstacle.y,
          this.terrainModel.eval(obstacle.x, obstacle.y, 'adaptive').h+0.38
        );
        this.scene.add(checkpointLabel);
      }
    }

    // Render embedded low-poly Martian stones and broken mountain ridges.
    if (this.terrainModel.rocks) {
      for (let rock of this.terrainModel.rocks) {
        const rockGeom = new THREE.DodecahedronGeometry(1, 0);
        const shade = new THREE.Color(0x35241f).lerp(
          new THREE.Color(0x704331), 0.25+0.45*(rock.colorSeed ?? 0.5));
        const rockMat = new THREE.MeshStandardMaterial({
          color: shade,
          roughness: 1.0,
          metalness: 0.0,
          flatShading: true
        });
        const offsets = [-this.laneOffset, this.laneOffset];
        for (const laneOffset of offsets) {
          const worldX = rock.x+laneOffset;
          // The two terrain copies meet at x=0. Do not draw a side-scene
          // rock after its lane offset carries it across that boundary,
          // where the mesh would have no matching physical height field.
          if ((laneOffset < 0 && worldX >= 0) || (laneOffset > 0 && worldX < 0)) continue;
          const modelType = laneOffset > 0 ? 'adaptive' : 'fixed';
          const rockMesh = new THREE.Mesh(rockGeom, rockMat);
          const rx = rock.rx || rock.r || 0.2;
          const ry = rock.ry || rock.r || 0.2;
          const renderedHeight = Math.max(0.06, 0.72*rock.h);
          rockMesh.scale.set(rx, ry, renderedHeight);
          rockMesh.rotation.z = rock.yaw || 0;
          rockMesh.rotation.x = 0;
          const crest = this.terrainModel.eval(rock.x, rock.y, modelType).h || 0;
          // Embed the complete mesh below the analytic collision crest.
          rockMesh.position.set(worldX, rock.y, crest-renderedHeight);
          rockMesh.castShadow = true;
          rockMesh.receiveShadow = true;
          this.scene.add(rockMesh);
        }
      }
    }

    // Extra embedded rocks appear only on Model B's right-hand path. Their
    // mesh positions use the same adaptive terrain query as B's contact
    // solver, so the rendered stone and the physical surface stay aligned.
    if (this.terrainModel.course && this.terrainModel.bPathRocks) {
      for (const rock of this.terrainModel.bPathRocks) {
        const rockGeom = new THREE.DodecahedronGeometry(1, 0);
        const rockMat = new THREE.MeshStandardMaterial({
          color: new THREE.Color(0x2f2522).lerp(
            new THREE.Color(0x684536), 0.22+0.34*(rock.colorSeed ?? 0.5)),
          roughness: 1.0,
          metalness: 0.0,
          flatShading: true
        });
        const rockMesh = new THREE.Mesh(rockGeom, rockMat);
        const renderedHeight = Math.max(0.035, 0.74*rock.h);
        rockMesh.scale.set(rock.rx, rock.ry, renderedHeight);
        rockMesh.rotation.z = rock.yaw || 0;
        rockMesh.rotation.x = 0;
        const crest = this.terrainModel.eval(rock.x, rock.y, 'adaptive').h || 0;
        rockMesh.position.set(rock.x+this.laneOffset, rock.y, crest-renderedHeight);
        rockMesh.castShadow = true;
        rockMesh.receiveShadow = true;
        this.scene.add(rockMesh);
      }
    }

    // Physical coarse-sand grains embedded in every obstacle. These meshes
    // correspond to the same Gaussian micro-outcrops used by the solver and
    // make the high-friction summit surface visibly granular.
    if (this.terrainModel.course && this.terrainModel.courseGritRocks) {
      for (const grain of this.terrainModel.courseGritRocks) {
        const geometry = new THREE.IcosahedronGeometry(1, 0);
        const material = new THREE.MeshStandardMaterial({
          color: new THREE.Color(0x4a3026).lerp(
            new THREE.Color(0x8a5a40), 0.18+0.22*(grain.colorSeed ?? 0.5)),
          roughness: 1.0,
          metalness: 0.0,
          flatShading: true
        });
        for (const laneOffset of [-this.laneOffset, this.laneOffset]) {
          const modelType = laneOffset > 0 ? 'adaptive' : 'fixed';
          const mesh = new THREE.Mesh(geometry, material);
          const renderedHeight = Math.max(0.012, 0.72*grain.h);
          mesh.scale.set(grain.rx, grain.ry, renderedHeight);
          mesh.rotation.set(0, 0, grain.yaw);
          const surface = this.terrainModel.eval(grain.x, grain.y, modelType).h;
          mesh.position.set(grain.x+laneOffset, grain.y, surface-renderedHeight);
          mesh.castShadow = true;
          mesh.receiveShadow = true;
          this.scene.add(mesh);
        }
      }
    }
  }

// SCRATCHPAD VISUALIZER DUAL

  createRoverObjects() {
    this.models = {
      A: { nodeMeshes: [], strutMeshes: [], outerCableMeshes: [], coreCableMeshes: [], coreMesh: null },
      B: { nodeMeshes: [], strutMeshes: [], outerCableMeshes: [], coreCableMeshes: [], coreMesh: null }
    };
    const unitCylGeom = new THREE.CylinderGeometry(1, 1, 1, 12);
    // Render the same physical radii used by the contact solver. Visual-only
    // thickness made Model A appear buried even when its mathematical cable
    // centreline was clear of the terrain.
    const nodeGeom = new THREE.SphereGeometry(this.roverModel.cfg.nodeRadius, 18, 18);

    const createModelMeshes = (modelKey, colorTint) => {
      const m = this.models[modelKey];
      for (let i = 0; i < this.roverModel.nOuter; i++) {
        const nodeMat = new THREE.MeshStandardMaterial({ color: colorTint.node, metalness: 0.8, roughness: 0.2, emissive: colorTint.nodeE, emissiveIntensity: 0.5 });
        const nodeMesh = new THREE.Mesh(nodeGeom, nodeMat);
        nodeMesh.castShadow = true;
        this.scene.add(nodeMesh);
        m.nodeMeshes.push(nodeMesh);
      }

      for (let b = 0; b < this.roverModel.bars.length; b++) {
        const strutMat = new THREE.MeshStandardMaterial({ color: colorTint.bar, metalness: 0.85, roughness: 0.15, emissive: colorTint.barE, emissiveIntensity: 0.3 });
        const mesh = new THREE.Mesh(unitCylGeom, strutMat);
        mesh.castShadow = true;
        this.scene.add(mesh);
        m.strutMeshes.push({ mesh, radius: 0.035, material: strutMat });
      }

      for (let s = 0; s < this.roverModel.outerStrings.length; s++) {
        const cableMat = new THREE.MeshStandardMaterial({ color: colorTint.cable, metalness: 0.4, roughness: 0.3, emissive: colorTint.cableE, emissiveIntensity: 0.4 });
        const mesh = new THREE.Mesh(unitCylGeom, cableMat);
        this.scene.add(mesh);
        m.outerCableMeshes.push({ mesh, radius: 0.012, material: cableMat });
      }

      const coreMaterial = new THREE.MeshStandardMaterial({
        color: modelKey === 'B' ? 0x10b981 : 0x94a3b8,
        emissive: modelKey === 'B' ? 0x047857 : 0x334155,
        emissiveIntensity: 0.9, metalness: 0.55, roughness: 0.22
      });
      m.coreMesh = new THREE.Mesh(new THREE.SphereGeometry(this.roverModel.R_core, 20, 20), coreMaterial);
      m.coreMesh.castShadow = true;
      this.scene.add(m.coreMesh);
      for (let i = 0; i < this.roverModel.nOuter; i++) {
        const material = new THREE.MeshStandardMaterial({
          color: modelKey === 'B' ? 0x34d399 : 0x64748b,
          emissive: modelKey === 'B' ? 0x059669 : 0x1e293b,
          emissiveIntensity: 0.45, transparent: true, opacity: 0.82
        });
        const mesh = new THREE.Mesh(unitCylGeom, material);
        this.scene.add(mesh);
        m.coreCableMeshes.push({ mesh, radius: 0.006, material });
      }
    };

    createModelMeshes('A', { node: 0x64748b, nodeE: 0x334155, bar: 0x475569, barE: 0x1e293b, cable: 0x94a3b8, cableE: 0x64748b }); // Stiff baseline: Gray/Slate
    createModelMeshes('B', { node: 0x38bdf8, nodeE: 0x0284c7, bar: 0x3b82f6, barE: 0x1d4ed8, cable: 0xef4444, cableE: 0xdc2626 }); // Adaptive: Bright colors

    this.velArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 0.8, 0xf59e0b);
    this.scene.add(this.velArrow);

    this.controlArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), new THREE.Vector3(0, 0, 0), 1.0, 0x8b5cf6, 0.18, 0.09);
    this.scene.add(this.controlArrow);

    this.comMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.075, 18, 18),
      new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.92 })
    );
    this.scene.add(this.comMarker);

    this.contactMarkers = [];
    this.contactForceArrows = [];
    const contactGeometry = new THREE.SphereGeometry(0.085, 14, 14);
    const maximumContacts = this.roverModel.nOuter+this.roverModel.bars.length;
    for (let i = 0; i < maximumContacts; i++) {
      const marker = new THREE.Mesh(contactGeometry, new THREE.MeshBasicMaterial({ color: 0xf97316, transparent: true, opacity: 0.88 }));
      marker.visible = false;
      this.scene.add(marker);
      this.contactMarkers.push(marker);
      const arrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 0.3, 0xfb7185, 0.09, 0.045);
      arrow.visible = false;
      this.scene.add(arrow);
      this.contactForceArrows.push(arrow);
    }

    this.debugGroup = new THREE.Group();
    this.nodeLabels = [];
    this.cableLabels = [];
    for (let node = 0; node < this.roverModel.nOuter; node++) {
      const label = this.createDebugLabel(`N${node+1}`, '#67e8f9');
      this.nodeLabels.push(label);
      this.debugGroup.add(label);
    }
    for (let cable = 0; cable < this.roverModel.outerStrings.length; cable++) {
      const label = this.createDebugLabel(`C${String(cable+1).padStart(2, '0')}`, '#fef08a');
      this.cableLabels.push(label);
      this.debugGroup.add(label);
    }
    this.debugGroup.visible = this.options.showDebugLabels;
    this.scene.add(this.debugGroup);

    this.supportFaceMesh = new THREE.Mesh(
      new THREE.BufferGeometry(),
      new THREE.MeshBasicMaterial({ color: 0x22c55e, transparent: true, opacity: 0.24, side: THREE.DoubleSide, depthWrite: false })
    );
    this.scene.add(this.supportFaceMesh);

    this.tippingEdgeLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineBasicMaterial({ color: 0xfacc15, transparent: true, opacity: 0.98 })
    );
    this.scene.add(this.tippingEdgeLine);

    this.predictionLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, 0.1, 0)]),
      new THREE.LineDashedMaterial({ color: 0xc084fc, dashSize: 0.12, gapSize: 0.08, transparent: true, opacity: 0.85 })
    );
    this.predictionLine.computeLineDistances();
    this.scene.add(this.predictionLine);

    // Add 3D text sprite for Model B State
    this.stateSprite = this.createTextSprite("ROLLING");
    this.stateSpriteMessage = "ROLLING";
    this.scene.add(this.stateSprite);
  }

  createTextSprite(message, color = 'rgba(16, 185, 129, 1.0)') {
    const canvas = document.createElement('canvas');
    canvas.width = 512; canvas.height = 128;
    const context = canvas.getContext('2d');
    context.font = "Bold 36px monospace";
    context.fillStyle = "rgba(0,0,0,0.32)";
    context.fillRect(0,0,canvas.width,canvas.height);
    context.fillStyle = color;
    context.textAlign = "center";
    context.fillText(message, 256, 75);
    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(2.2, 0.55, 1.0);
    return sprite;
  }

  createDebugLabel(message, color) {
    const canvas = document.createElement('canvas');
    canvas.width = 192; canvas.height = 64;
    const context = canvas.getContext('2d');
    context.fillStyle = 'rgba(2, 6, 23, 0.82)';
    context.fillRect(0, 0, canvas.width, canvas.height);
    context.strokeStyle = color;
    context.strokeRect(1, 1, canvas.width-2, canvas.height-2);
    context.font = 'Bold 30px monospace';
    context.fillStyle = color;
    context.textAlign = 'center';
    context.fillText(message, canvas.width/2, 42);
    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), depthTest: false }));
    sprite.scale.set(0.42, 0.14, 1);
    return sprite;
  }

  createTrajectoryTrail() {
    this.trailFrame = 0;
    this.trails = {};
    for (const [modelKey, color] of [['A', 0x94a3b8], ['B', 0x06b6d4]]) {
      const line = new THREE.Line(
        new THREE.BufferGeometry(),
        new THREE.LineBasicMaterial({ color, linewidth: 3 })
      );
      this.trails[modelKey] = { points: [], line };
      this.scene.add(line);
    }
  }

  createGeometryCheckpointOverlay() {
    this.checkpointGroup = new THREE.Group();

    // Outer Bounding Sphere Wireframe
    const outerGeom = new THREE.SphereGeometry(this.roverModel.R_outer, 24, 24);
    const outerMat = new THREE.MeshBasicMaterial({
      color: 0x06b6d4,
      wireframe: true,
      transparent: true,
      opacity: 0.25
    });
    this.outerWireMesh = new THREE.Mesh(outerGeom, outerMat);
    this.checkpointGroup.add(this.outerWireMesh);

    this.checkpointGroup.visible = this.options.showGeometryCheckpoint;
    this.scene.add(this.checkpointGroup);
  }

  updateDual(dualSimState) {
    const yAxis = new THREE.Vector3(0, 1, 0);

    const updateModel = (modelKey, state, offsetX) => {
      const { q, diag } = state;
      if (!q) return;
      const m = this.models[modelKey];
      const contracting = new Set(diag.contractingCableIndices || []);
      const relaxing = new Set(diag.relaxingCableIndices || []);
      const contactNodes = new Set((diag.contacts || [])
        .filter(contact => contact.kind === 'node')
        .map(contact => contact.nodeIndex));

      for (let i = 0; i < this.roverModel.nOuter; i++) {
        m.nodeMeshes[i].position.set(q[i][0] + offsetX, q[i][1], q[i][2]);
        if (modelKey === 'B') {
          const contacting = contactNodes.has(i);
          m.nodeMeshes[i].material.color.setHex(contacting ? 0xff4d00 : 0x38bdf8);
          m.nodeMeshes[i].material.emissive.setHex(contacting ? 0xdc2626 : 0x0284c7);
          m.nodeMeshes[i].material.emissiveIntensity = contacting ? 1.4 : 0.5;
        }
      }

      for (let b = 0; b < this.roverModel.bars.length; b++) {
        const idxA = this.roverModel.bars[b][0];
        const idxB = this.roverModel.bars[b][1];
        const pA = new THREE.Vector3(q[idxA][0] + offsetX, q[idxA][1], q[idxA][2]);
        const pB = new THREE.Vector3(q[idxB][0] + offsetX, q[idxB][1], q[idxB][2]);
        const item = m.strutMeshes[b];

        const dir = new THREE.Vector3().subVectors(pB, pA);
        const len = dir.length();
        item.mesh.scale.set(item.radius, len, item.radius);
        item.mesh.position.copy(pA.clone().add(pB).multiplyScalar(0.5));
        if (len > 1e-6) item.mesh.quaternion.setFromUnitVectors(yAxis, dir.normalize());

        if (this.options.showForceOverlay && modelKey === 'B' && diag.strutActuated) {
          if (diag.strutActuated[b]) {
            item.material.color.setHex(0xf59e0b);
            item.material.emissive.setHex(0xd97706);
            item.material.emissiveIntensity = 0.85;
          } else {
            item.material.color.setHex(0x3b82f6);
            item.material.emissive.setHex(0x1d4ed8);
            item.material.emissiveIntensity = 0.3;
          }
        }
      }

      for (let s = 0; s < this.roverModel.outerStrings.length; s++) {
        const idxA = this.roverModel.outerStrings[s][0];
        const idxB = this.roverModel.outerStrings[s][1];
        const pA = new THREE.Vector3(q[idxA][0] + offsetX, q[idxA][1], q[idxA][2]);
        const pB = new THREE.Vector3(q[idxB][0] + offsetX, q[idxB][1], q[idxB][2]);
        const item = m.outerCableMeshes[s];

        const dir = new THREE.Vector3().subVectors(pB, pA);
        const len = dir.length();
        const cableTelemetry = diag.cableTelemetry?.[s];
        const forceRatio = cableTelemetry
          ? clamp01(cableTelemetry.force/Math.max(1, diag.monitoring?.maximumCableForce || 1)) : 0;
        item.mesh.scale.set(item.radius, len, item.radius);
        item.mesh.position.copy(pA.clone().add(pB).multiplyScalar(0.5));
        if (len > 1e-6) item.mesh.quaternion.setFromUnitVectors(yAxis, dir.normalize());

        if (this.options.showForceOverlay && modelKey === 'B' && cableTelemetry) {
          const selected = s === this.options.selectedCable;
          const colors = {
            slack: [0xa855f7, 0x7e22ce],
            overload: [0xef4444, 0xdc2626],
            high: [0xf97316, 0xea580c],
            moderate: [0xfacc15, 0xca8a04],
            nominal: [0x22c55e, 0x059669]
          };
          const [color, emissive] = colors[cableTelemetry.state] || colors.nominal;
          item.material.color.setHex(selected ? 0xffffff : color);
          item.material.emissive.setHex(selected ? 0x67e8f9 : emissive);
          item.material.emissiveIntensity = selected ? 1.8 : 0.55+0.8*forceRatio;
        } else if (this.options.showForceOverlay && diag.outerCableActuated && modelKey === 'B') {
          const isAct = diag.outerCableActuated[s];
          const tension = diag.outerCableForces ? diag.outerCableForces[s] : 0;
          const isRelaxed = diag.outerCableRelaxed?.[s];
          if (relaxing.has(s) || isRelaxed) {
            item.material.color.setHex(0xe879f9); item.material.emissive.setHex(0xc026d3); item.material.emissiveIntensity = 1.0;
          } else if (contracting.has(s) || isAct) {
            item.material.color.setHex(0xf59e0b); item.material.emissive.setHex(0xd97706); item.material.emissiveIntensity = 0.9;
          } else if (tension > 100.0) {
            item.material.color.setHex(0x06b6d4); item.material.emissive.setHex(0x0891b2); item.material.emissiveIntensity = 0.7;
          } else {
            item.material.color.setHex(0xef4444); item.material.emissive.setHex(0xb91c1c); item.material.emissiveIntensity = 0.4;
          }
        }
      }

      const core = diag.corePosition || diag.centroid;
      if (core && m.coreMesh) {
        const corePoint = new THREE.Vector3(core[0]+offsetX, core[1], core[2]);
        m.coreMesh.position.copy(corePoint);
        for (let i = 0; i < this.roverModel.nOuter; i++) {
          const nodePoint = new THREE.Vector3(q[i][0]+offsetX, q[i][1], q[i][2]);
          const item = m.coreCableMeshes[i];
          const direction = new THREE.Vector3().subVectors(nodePoint, corePoint);
          const length = direction.length();
          item.mesh.scale.set(item.radius, length, item.radius);
          item.mesh.position.copy(corePoint.clone().add(nodePoint).multiplyScalar(0.5));
          if (length > 1e-6) item.mesh.quaternion.setFromUnitVectors(yAxis, direction.normalize());
        }
      }
    };

    updateModel('A', dualSimState.simA, -this.laneOffset);
    updateModel('B', dualSimState.simB, this.laneOffset);

    this.trailFrame++;
    if (this.trailFrame % 8 === 0) {
      for (const [modelKey, state, offsetX] of [
        ['A', dualSimState.simA, -this.laneOffset],
        ['B', dualSimState.simB, this.laneOffset]
      ]) {
        const centroid = state.diag.centroid;
        const trail = this.trails[modelKey];
        trail.points.push(new THREE.Vector3(centroid[0]+offsetX, centroid[1], centroid[2]+0.04));
        if (trail.points.length > 1800) trail.points.shift();
        const oldGeometry = trail.line.geometry;
        trail.line.geometry = new THREE.BufferGeometry().setFromPoints(trail.points);
        oldGeometry.dispose();
      }
    }

    const refPosB = dualSimState.simB.diag.centroid || [0, 0, 0];
    const refPosB_World = [refPosB[0]+this.laneOffset, refPosB[1], refPosB[2]];
    const payloadCore = dualSimState.simB.diag.corePosition || refPosB;
    this.comMarker.position.set(payloadCore[0]+this.laneOffset, payloadCore[1], payloadCore[2]);

    const activeContacts = dualSimState.simB.diag.contacts || [];
    for (let i = 0; i < this.contactMarkers.length; i++) {
      const marker = this.contactMarkers[i];
      const arrow = this.contactForceArrows[i];
      const contact = activeContacts[i];
      marker.visible = Boolean(contact);
      arrow.visible = Boolean(contact);
      if (contact) {
        marker.position.set(contact.position[0]+this.laneOffset, contact.position[1], contact.position[2]+0.03);
        const normalForce = contact.normalForce || 0;
        const resultant = contact.normal.map((value, axis) =>
          value*normalForce+(contact.frictionForce?.[axis] || 0));
        const magnitude = Math.hypot(...resultant);
        const direction = magnitude > 1e-9 ? resultant.map(value => value/magnitude) : contact.normal;
        arrow.position.copy(marker.position);
        arrow.setDirection(new THREE.Vector3(...direction));
        arrow.setLength(Math.min(1.2, 0.12+0.12*Math.sqrt(Math.max(0, magnitude))), 0.10, 0.05);
        const overload = normalForce > 0.25*(dualSimState.simB.diag.monitoring?.maximumCableForce || 900);
        marker.material.color.setHex(overload ? 0xef4444 : 0xf97316);
      }
    }

    if (this.debugGroup) {
      this.debugGroup.visible = this.options.showDebugLabels;
      if (this.debugGroup.visible) {
        for (let node = 0; node < this.nodeLabels.length; node++) {
          const position = dualSimState.simB.q[node];
          this.nodeLabels[node].position.set(position[0]+this.laneOffset, position[1], position[2]+0.16);
        }
        for (let cable = 0; cable < this.cableLabels.length; cable++) {
          const [first, second] = this.roverModel.outerStrings[cable];
          const a = dualSimState.simB.q[first];
          const b = dualSimState.simB.q[second];
          this.cableLabels[cable].position.set(0.5*(a[0]+b[0])+this.laneOffset, 0.5*(a[1]+b[1]), 0.5*(a[2]+b[2])+0.05);
        }
      }
    }

    const supportFace = dualSimState.simB.diag.supportFace || [];
    if (supportFace.length === 3) {
      const points = supportFace.map(node => {
        const position = dualSimState.simB.q[node];
        return new THREE.Vector3(position[0]+this.laneOffset, position[1], position[2]+0.006);
      });
      const oldGeometry = this.supportFaceMesh.geometry;
      this.supportFaceMesh.geometry = new THREE.BufferGeometry().setFromPoints(points);
      this.supportFaceMesh.geometry.setIndex([0, 1, 2]);
      this.supportFaceMesh.geometry.computeVertexNormals();
      oldGeometry.dispose();
      this.supportFaceMesh.visible = true;
    } else {
      this.supportFaceMesh.visible = false;
    }

    const targetEdge = dualSimState.simB.diag.targetEdge || [];
    if (targetEdge.length === 2) {
      const points = targetEdge.map(node => {
        const position = dualSimState.simB.q[node];
        return new THREE.Vector3(position[0]+this.laneOffset, position[1], position[2]+0.018);
      });
      const oldGeometry = this.tippingEdgeLine.geometry;
      this.tippingEdgeLine.geometry = new THREE.BufferGeometry().setFromPoints(points);
      oldGeometry.dispose();
      this.tippingEdgeLine.visible = true;
    } else {
      this.tippingEdgeLine.visible = false;
    }

    const stateStr = this.terrainModel.course
      ? `${dualSimState.simB.diag.obstaclePhase || dualSimState.simB.diag.state || 'ROLLING'} · ${dualSimState.simB.diag.activeObstacleId || 'course'}`
      : `${dualSimState.simB.diag.state || 'ROLLING'}`;
    if (this.stateSprite && stateStr !== this.stateSpriteMessage) {
      this.scene.remove(this.stateSprite);
      this.stateSprite.material.map.dispose();
      this.stateSprite.material.dispose();
      this.stateSprite = this.createTextSprite(stateStr);
      this.stateSpriteMessage = stateStr;
      this.scene.add(this.stateSprite);
    }
    this.stateSprite.position.set(refPosB_World[0], refPosB_World[1], refPosB_World[2] + 1.2);

    if (this.checkpointGroup) {
      this.checkpointGroup.position.set(refPosB_World[0], refPosB_World[1], refPosB_World[2]);
    }

    if (dualSimState.simB.diag.velocityVector) {
      const [vx, vy, vz] = dualSimState.simB.diag.velocityVector;
      const speed = Math.sqrt(vx*vx + vy*vy + vz*vz);
      this.velArrow.position.set(refPosB_World[0], refPosB_World[1], refPosB_World[2]);
      if (speed > 0.05) {
        this.velArrow.setDirection(new THREE.Vector3(vx/speed, vy/speed, vz/speed));
        this.velArrow.setLength(Math.min(1.5, speed * 0.8), 0.15, 0.08);
        this.velArrow.visible = true;
      } else {
        this.velArrow.visible = false;
      }
    }

    if (dualSimState.simB.diag.desiredDirection) {
      const [dx, dy, dz] = dualSimState.simB.diag.desiredDirection;
      const magnitude = Math.sqrt(dx*dx + dy*dy + dz*dz);
      this.controlArrow.position.set(refPosB_World[0], refPosB_World[1], refPosB_World[2] + 0.12);
      if (magnitude > 1e-6) {
        this.controlArrow.setDirection(new THREE.Vector3(dx/magnitude, dy/magnitude, dz/magnitude));
        this.controlArrow.setLength(1.0, 0.18, 0.09);
        this.controlArrow.visible = true;
      } else {
        this.controlArrow.visible = false;
      }
    }

    const predictedPath = dualSimState.simB.diag.predictedPath || [];
    if (predictedPath.length >= 2) {
      const points = predictedPath.map(point => new THREE.Vector3(point[0]+this.laneOffset, point[1], point[2]+0.08));
      const oldGeometry = this.predictionLine.geometry;
      this.predictionLine.geometry = new THREE.BufferGeometry().setFromPoints(points);
      oldGeometry.dispose();
      this.predictionLine.computeLineDistances();
      this.predictionLine.visible = true;
    } else {
      this.predictionLine.visible = false;
    }

    if (this.options.cameraMode === 'chase') {
      const cx = (dualSimState.simA.diag.centroid[0]-this.laneOffset+refPosB_World[0])/2.0;
      const cy = (dualSimState.simA.diag.centroid[1] + refPosB_World[1]) / 2.0;
      const cz = (dualSimState.simA.diag.centroid[2] + refPosB_World[2]) / 2.0;
      this.controls.target.set(cx, cy, cz);
      this.camera.position.set(cx - 3.5, cy - 6.5, cz + 3.0);
    }

    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  setCameraMode(mode) {
    this.options.cameraMode = mode;
  }

  toggleForceOverlay(visible) {
    this.options.showForceOverlay = visible;
  }

  toggleGeometryCheckpoint(visible) {
    this.options.showGeometryCheckpoint = visible;
    if (this.checkpointGroup) this.checkpointGroup.visible = visible;
  }

  toggleDebugLabels(visible) {
    this.options.showDebugLabels = visible;
    if (this.debugGroup) this.debugGroup.visible = visible;
  }

  setSelectedCable(index) {
    this.options.selectedCable = Number.isFinite(index) ? index : 0;
  }
}

const clamp01 = value => Math.max(0, Math.min(1, value));
