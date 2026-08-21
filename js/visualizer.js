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
    this.options = Object.assign({
      title: 'Simulation View',
      cameraMode: 'chase', // 'chase', 'orbit', 'top', 'side'
      showForceOverlay: true,
      showGeometryCheckpoint: false
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
    this.scene.background = new THREE.Color(0x070a12);
    this.scene.fog = new THREE.FogExp2(0x070a12, 0.015);

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

    // High-tech Lighting Schema
    const ambLight = new THREE.AmbientLight(0xffffff, 0.65);
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

    const hemiLight = new THREE.HemisphereLight(0x38bdf8, 0x1e1b4b, 0.55);
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

    // Mars Reddish-Brown Bedrock palette
    const colorLow = new THREE.Color(0x6e3b1f);
    const colorHigh = new THREE.Color(0xaf6132);

    for (let i = 0; i < posAttr.count; i++) {
      const localX = posAttr.getX(i);
      const localY = posAttr.getY(i);
      const worldX = localX;
      const worldY = localY + (yMin + yMax) / 2;

      // In comparison mode, show two visually separated copies of the exact
      // same physical course (A at -1.5 m, B at +1.5 m).
      const physicsX = this.terrainModel.course
        ? worldX-(worldX < 0 ? -1.5 : 1.5)
        : worldX;
      const surf = this.terrainModel.eval(physicsX, worldY);
      posAttr.setZ(i, surf.h);

      const ratio = (surf.h + 0.4) / 1.0;
      const c = colorLow.clone().lerp(colorHigh, Math.max(0, Math.min(1, ratio)));
      colors.push(c.r, c.g, c.b);
    }

    geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
    geometry.computeVertexNormals();

    const material = new THREE.MeshStandardMaterial({
      vertexColors: true,
      roughness: 0.90,
      metalness: 0.1,
      flatShading: true
    });

    this.terrainMesh = new THREE.Mesh(geometry, material);
    this.terrainMesh.position.set(0, (yMin + yMax) / 2, 0);
    this.terrainMesh.receiveShadow = true;
    this.scene.add(this.terrainMesh);

    // Subtle Sci-Fi Grid overlay
    const grid = new THREE.GridHelper(this.terrainModel.course ? 70 : 50, this.terrainModel.course ? 70 : 50, 0x06b6d4, 0x334155);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(0, (yMin+yMax)/2, -0.05);
    this.scene.add(grid);

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

    if (this.terrainModel.course) {
      const startY = this.terrainModel.course.startY;
      const startMaterial = new THREE.MeshBasicMaterial({
        color: 0x38bdf8, side: THREE.DoubleSide, transparent: true, opacity: 0.72
      });
      const startLine = new THREE.Mesh(new THREE.PlaneGeometry(6, 0.28), startMaterial);
      startLine.position.set(0, startY, this.terrainModel.eval(0, startY).h+0.025);
      this.scene.add(startLine);

      const ringGeometry = new THREE.RingGeometry(0.31, 0.36, 32);
      const ringMaterial = new THREE.MeshBasicMaterial({
        color: 0xf59e0b, side: THREE.DoubleSide, transparent: true, opacity: 0.78
      });
      for (const obstacle of this.terrainModel.course.obstacles) {
        for (const laneOffset of [-1.5, 1.5]) {
          const ring = new THREE.Mesh(ringGeometry, ringMaterial);
          ring.scale.set(obstacle.radiusX/0.36, obstacle.radiusY/0.36, 1);
          ring.position.set(obstacle.x+laneOffset, obstacle.y,
            this.terrainModel.eval(obstacle.x, obstacle.y).h+0.018);
          this.scene.add(ring);
        }
      }
    }

    // Render Boulders & Rocks
    if (this.terrainModel.rocks) {
      for (let rock of this.terrainModel.rocks) {
        const rockGeom = new THREE.DodecahedronGeometry(rock.r, 1);
        const rockMat = new THREE.MeshStandardMaterial({
          color: 0x8b5cf6, // Glowing violet accent boulder
          roughness: 0.8,
          metalness: 0.2
        });
        const rockMesh = new THREE.Mesh(rockGeom, rockMat);
        rockMesh.position.set(rock.x, rock.y, rock.h * 0.4 + (this.terrainModel.eval(rock.x, rock.y).h || 0));
        rockMesh.castShadow = true;
        rockMesh.receiveShadow = true;
        this.scene.add(rockMesh);
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
    const nodeGeom = new THREE.SphereGeometry(0.06, 18, 18);

    const createModelMeshes = (modelKey, colorTint) => {
      const m = this.models[modelKey];
      const nodeMat = new THREE.MeshStandardMaterial({ color: colorTint.node, metalness: 0.8, roughness: 0.2, emissive: colorTint.nodeE, emissiveIntensity: 0.5 });
      for (let i = 0; i < this.roverModel.nOuter; i++) {
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
    const contactGeometry = new THREE.SphereGeometry(0.085, 14, 14);
    for (let i = 0; i < this.roverModel.nOuter; i++) {
      const marker = new THREE.Mesh(contactGeometry, new THREE.MeshBasicMaterial({ color: 0x22c55e, transparent: true, opacity: 0.72 }));
      marker.visible = false;
      this.scene.add(marker);
      this.contactMarkers.push(marker);
    }

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

  createTextSprite(message) {
    const canvas = document.createElement('canvas');
    canvas.width = 512; canvas.height = 128;
    const context = canvas.getContext('2d');
    context.font = "Bold 36px monospace";
    context.fillStyle = "rgba(0,0,0,0.5)";
    context.fillRect(0,0,canvas.width,canvas.height);
    context.fillStyle = "rgba(16, 185, 129, 1.0)";
    context.textAlign = "center";
    context.fillText(message, 256, 75);
    const texture = new THREE.CanvasTexture(canvas);
    const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(spriteMaterial);
    sprite.scale.set(3.0, 0.75, 1.0);
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

      for (let i = 0; i < this.roverModel.nOuter; i++) {
        m.nodeMeshes[i].position.set(q[i][0] + offsetX, q[i][1], q[i][2]);
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
        item.mesh.scale.set(item.radius, len, item.radius);
        item.mesh.position.copy(pA.clone().add(pB).multiplyScalar(0.5));
        if (len > 1e-6) item.mesh.quaternion.setFromUnitVectors(yAxis, dir.normalize());

        if (this.options.showForceOverlay && diag.outerCableActuated && modelKey === 'B') {
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

    updateModel('A', dualSimState.simA, -1.5);
    updateModel('B', dualSimState.simB, 1.5);

    this.trailFrame++;
    if (this.trailFrame % 8 === 0) {
      for (const [modelKey, state, offsetX] of [
        ['A', dualSimState.simA, -1.5], ['B', dualSimState.simB, 1.5]
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
    const refPosB_World = [refPosB[0] + 1.5, refPosB[1], refPosB[2]];
    const payloadCore = dualSimState.simB.diag.corePosition || refPosB;
    this.comMarker.position.set(payloadCore[0]+1.5, payloadCore[1], payloadCore[2]);

    const contactSet = new Set(dualSimState.simB.diag.groundContactNodes || []);
    for (let i = 0; i < this.contactMarkers.length; i++) {
      const marker = this.contactMarkers[i];
      marker.visible = contactSet.has(i);
      if (marker.visible) {
        const position = dualSimState.simB.q[i];
        marker.position.set(position[0]+1.5, position[1], position[2]);
      }
    }

    const supportFace = dualSimState.simB.diag.supportFace || [];
    if (supportFace.length === 3) {
      const points = supportFace.map(node => {
        const position = dualSimState.simB.q[node];
        return new THREE.Vector3(position[0]+1.5, position[1], position[2]+0.006);
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
        return new THREE.Vector3(position[0]+1.5, position[1], position[2]+0.018);
      });
      const oldGeometry = this.tippingEdgeLine.geometry;
      this.tippingEdgeLine.geometry = new THREE.BufferGeometry().setFromPoints(points);
      oldGeometry.dispose();
      this.tippingEdgeLine.visible = true;
    } else {
      this.tippingEdgeLine.visible = false;
    }

    const stateStr = `${dualSimState.simB.diag.obstaclePhase || dualSimState.simB.diag.state || 'ROLLING'} · ${dualSimState.simB.diag.activeObstacleId || 'course'}`;
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
      const points = predictedPath.map(point => new THREE.Vector3(point[0]+1.5, point[1], point[2]+0.08));
      const oldGeometry = this.predictionLine.geometry;
      this.predictionLine.geometry = new THREE.BufferGeometry().setFromPoints(points);
      oldGeometry.dispose();
      this.predictionLine.computeLineDistances();
      this.predictionLine.visible = true;
    } else {
      this.predictionLine.visible = false;
    }

    if (this.options.cameraMode === 'chase') {
      const cx = (dualSimState.simA.diag.centroid[0] - 1.5 + refPosB_World[0]) / 2.0;
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
}
