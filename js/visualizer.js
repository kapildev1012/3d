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
    const Ny = 240;
    const xMin = -8, xMax = 8;
    const yMin = -4, yMax = 45;

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

      const surf = this.terrainModel.eval(worldX, worldY);
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
    const grid = new THREE.GridHelper(50, 50, 0x06b6d4, 0x334155);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(0, 20, -0.05);
    this.scene.add(grid);

    // Glowing Finish Line Gate Arch at Target Endpoint (y = 25.0m)
    const goalY = 25.0;
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

  createRoverObjects() {
    this.nodeMeshes = [];
    this.strutMeshes = [];
    this.outerCableMeshes = [];
    this.coreCableMeshes = [];

    const unitCylGeom = new THREE.CylinderGeometry(1, 1, 1, 12);

    // 1. Outer Strut Nodes (12 Titanium Nodes)
    const nodeGeom = new THREE.SphereGeometry(0.06, 18, 18);
    const nodeMat = new THREE.MeshStandardMaterial({
      color: 0x38bdf8, // Cyan node joints
      metalness: 0.8,
      roughness: 0.2,
      emissive: 0x0284c7,
      emissiveIntensity: 0.5
    });

    for (let i = 0; i < this.roverModel.nOuter; i++) {
      const nodeMesh = new THREE.Mesh(nodeGeom, nodeMat);
      nodeMesh.castShadow = true;
      this.scene.add(nodeMesh);
      this.nodeMeshes.push(nodeMesh);
    }

    // 2. Outer Compression Struts (6 Sleek Metallic Rods)
    for (let b = 0; b < this.roverModel.bars.length; b++) {
      const strutMat = new THREE.MeshStandardMaterial({
        color: 0x3b82f6, // Royal Blue compression strut
        metalness: 0.85,
        roughness: 0.15,
        emissive: 0x1d4ed8,
        emissiveIntensity: 0.3
      });
      const mesh = new THREE.Mesh(unitCylGeom, strutMat);
      mesh.castShadow = true;
      this.scene.add(mesh);
      this.strutMeshes.push({ mesh, radius: 0.035, material: strutMat });
    }

    // 3. Outer Pre-tensioned Tension Cables (24 Cables)
    for (let s = 0; s < this.roverModel.outerStrings.length; s++) {
      const cableMat = new THREE.MeshStandardMaterial({
        color: 0xef4444, // Red cable
        metalness: 0.4,
        roughness: 0.3,
        emissive: 0xdc2626,
        emissiveIntensity: 0.4
      });
      const mesh = new THREE.Mesh(unitCylGeom, cableMat);
      this.scene.add(mesh);
      this.outerCableMeshes.push({ mesh, radius: 0.012, material: cableMat });
    }

    // 4. CENTRAL PAYLOAD CORE (Radius R_core = 0.1D = 0.10 m)
    const coreGeom = new THREE.SphereGeometry(this.roverModel.R_core, 28, 28);
    const coreMat = new THREE.MeshPhysicalMaterial({
      color: 0x10b981, // Emerald green sensor payload core
      metalness: 0.7,
      roughness: 0.1,
      transmission: 0.3,
      emissive: 0x059669,
      emissiveIntensity: 0.6
    });
    this.coreMesh = new THREE.Mesh(coreGeom, coreMat);
    this.coreMesh.castShadow = true;
    this.scene.add(this.coreMesh);

    // Inner Core LED Halo
    const coreHaloGeom = new THREE.SphereGeometry(this.roverModel.R_core * 1.12, 16, 16);
    const coreHaloMat = new THREE.MeshBasicMaterial({
      color: 0x34d399,
      wireframe: true,
      transparent: true,
      opacity: 0.5
    });
    this.coreHaloMesh = new THREE.Mesh(coreHaloGeom, coreHaloMat);
    this.coreMesh.add(this.coreHaloMesh);

    // 5. Inner Core Radial Suspension Cables (12 Radial Cables)
    for (let i = 0; i < this.roverModel.nOuter; i++) {
      const coreCableMat = new THREE.MeshStandardMaterial({
        color: 0x10b981, // Emerald suspension cable
        metalness: 0.5,
        roughness: 0.2,
        emissive: 0x047857,
        emissiveIntensity: 0.5
      });
      const mesh = new THREE.Mesh(unitCylGeom, coreCableMat);
      this.scene.add(mesh);
      this.coreCableMeshes.push({ mesh, radius: 0.008, material: coreCableMat });
    }

    // Velocity Vector Indicator Arrow
    this.velArrow = new THREE.ArrowHelper(
      new THREE.Vector3(0, 1, 0),
      new THREE.Vector3(0, 0, 0),
      0.8,
      0xf59e0b
    );
    this.scene.add(this.velArrow);
  }

  createTrajectoryTrail() {
    this.trailPoints = [];
    const geom = new THREE.BufferGeometry();
    const mat = new THREE.LineBasicMaterial({
      color: 0x06b6d4,
      linewidth: 3
    });
    this.trailLine = new THREE.Line(geom, mat);
    this.scene.add(this.trailLine);
  }

  createGeometryCheckpointOverlay() {
    this.checkpointGroup = new THREE.Group();

    // Outer Bounding Sphere Wireframe (Diameter = 1.0D = 2.0m)
    const outerGeom = new THREE.SphereGeometry(this.roverModel.R_outer, 24, 24);
    const outerMat = new THREE.MeshBasicMaterial({
      color: 0x06b6d4,
      wireframe: true,
      transparent: true,
      opacity: 0.25
    });
    this.outerWireMesh = new THREE.Mesh(outerGeom, outerMat);
    this.checkpointGroup.add(this.outerWireMesh);

    // Inner Core Bounding Sphere Wireframe (Diameter = 0.1D = 0.2m)
    const innerGeom = new THREE.SphereGeometry(this.roverModel.R_core, 16, 16);
    const innerMat = new THREE.MeshBasicMaterial({
      color: 0x10b981,
      wireframe: true,
      transparent: true,
      opacity: 0.6
    });
    this.innerWireMesh = new THREE.Mesh(innerGeom, innerMat);
    this.checkpointGroup.add(this.innerWireMesh);

    this.checkpointGroup.visible = this.options.showGeometryCheckpoint;
    this.scene.add(this.checkpointGroup);
  }

  update(simState) {
    const { q, currentDiag, centroid } = simState;
    if (!q) return;

    const yAxis = new THREE.Vector3(0, 1, 0);

    // 1. Update Outer Node Positions
    for (let i = 0; i < this.roverModel.nOuter; i++) {
      this.nodeMeshes[i].position.set(q[i][0], q[i][1], q[i][2]);
    }

    // 2. Update Compression Struts
    for (let b = 0; b < this.roverModel.bars.length; b++) {
      const idxA = this.roverModel.bars[b][0];
      const idxB = this.roverModel.bars[b][1];
      const pA = new THREE.Vector3(...q[idxA]);
      const pB = new THREE.Vector3(...q[idxB]);
      const item = this.strutMeshes[b];

      const dir = new THREE.Vector3().subVectors(pB, pA);
      const len = dir.length();
      item.mesh.scale.set(item.radius, len, item.radius);
      item.mesh.position.copy(pA.clone().add(pB).multiplyScalar(0.5));
      if (len > 1e-6) item.mesh.quaternion.setFromUnitVectors(yAxis, dir.normalize());

      // Force color coding
      if (this.options.showForceOverlay && currentDiag.strutForces) {
        const comp = currentDiag.strutForces[b] || 0;
        if (comp > 50.0) {
          item.material.color.setHex(0xa855f7); // Glowing purple under heavy compression
          item.material.emissive.setHex(0x7e22ce);
        } else {
          item.material.color.setHex(0x3b82f6); // Royal blue normal
          item.material.emissive.setHex(0x1d4ed8);
        }
      }
    }

    // 3. Update Outer Tension Cables
    for (let s = 0; s < this.roverModel.outerStrings.length; s++) {
      const idxA = this.roverModel.outerStrings[s][0];
      const idxB = this.roverModel.outerStrings[s][1];
      const pA = new THREE.Vector3(...q[idxA]);
      const pB = new THREE.Vector3(...q[idxB]);
      const item = this.outerCableMeshes[s];

      const dir = new THREE.Vector3().subVectors(pB, pA);
      const len = dir.length();
      item.mesh.scale.set(item.radius, len, item.radius);
      item.mesh.position.copy(pA.clone().add(pB).multiplyScalar(0.5));
      if (len > 1e-6) item.mesh.quaternion.setFromUnitVectors(yAxis, dir.normalize());

      // Color coding for tension & active actuation
      if (this.options.showForceOverlay && currentDiag.outerCableActuated) {
        const isAct = currentDiag.outerCableActuated[s];
        const tension = currentDiag.outerCableForces ? currentDiag.outerCableForces[s] : 0;

        if (isAct) {
          item.material.color.setHex(0xf59e0b); // Glowing Gold/Amber when actively contracting
          item.material.emissive.setHex(0xd97706);
          item.material.emissiveIntensity = 0.9;
        } else if (tension > 100.0) {
          item.material.color.setHex(0x06b6d4); // Bright Cyan under high tension
          item.material.emissive.setHex(0x0891b2);
          item.material.emissiveIntensity = 0.7;
        } else {
          item.material.color.setHex(0xef4444); // Radiant red baseline pre-tension
          item.material.emissive.setHex(0xb91c1c);
          item.material.emissiveIntensity = 0.4;
        }
      }
    }

    // 4. Update Central Payload Core & Radial Suspension Cables
    const refPos = currentDiag.corePos ? currentDiag.corePos : (centroid || [0, 0, 0]);

    if (currentDiag.corePos) {
      const coreP = currentDiag.corePos;
      this.coreMesh.position.set(coreP[0], coreP[1], coreP[2]);

      // Suspension Cables connecting outer nodes to core anchors
      for (let i = 0; i < this.roverModel.nOuter; i++) {
        const pOuter = new THREE.Vector3(...q[i]);
        const anchorRel = this.roverModel.coreAnchors[i];
        const pAnchor = new THREE.Vector3(coreP[0] + anchorRel[0], coreP[1] + anchorRel[1], coreP[2] + anchorRel[2]);
        const item = this.coreCableMeshes[i];

        const dir = new THREE.Vector3().subVectors(pOuter, pAnchor);
        const len = dir.length();
        item.mesh.scale.set(item.radius, len, item.radius);
        item.mesh.position.copy(pOuter.clone().add(pAnchor).multiplyScalar(0.5));
        if (len > 1e-6) item.mesh.quaternion.setFromUnitVectors(yAxis, dir.normalize());
      }
    }

    // 5. Update Checkpoint Overlay Position
    if (refPos) {
      this.checkpointGroup.position.set(refPos[0], refPos[1], refPos[2]);
    }

    // 6. Update Velocity Vector Arrow
    if (currentDiag.velocityVector && refPos) {
      const [vx, vy, vz] = currentDiag.velocityVector;
      const speed = Math.sqrt(vx*vx + vy*vy + vz*vz);
      this.velArrow.position.set(refPos[0], refPos[1], refPos[2]);
      if (speed > 0.05) {
        this.velArrow.setDirection(new THREE.Vector3(vx/speed, vy/speed, vz/speed));
        this.velArrow.setLength(Math.min(1.5, speed * 0.8), 0.15, 0.08);
        this.velArrow.visible = true;
      } else {
        this.velArrow.visible = false;
      }
    }

    // 7. Update Trajectory Trail
    if (refPos) {
      const curPos = new THREE.Vector3(refPos[0], refPos[1], refPos[2]);
      if (this.trailPoints.length === 0 || curPos.distanceTo(this.trailPoints[this.trailPoints.length - 1]) > 0.20) {
        this.trailPoints.push(curPos);
        if (this.trailPoints.length > 500) this.trailPoints.shift();

        const positions = new Float32Array(this.trailPoints.length * 3);
        for (let i = 0; i < this.trailPoints.length; i++) {
          positions[i*3]     = this.trailPoints[i].x;
          positions[i*3 + 1] = this.trailPoints[i].y;
          positions[i*3 + 2] = this.trailPoints[i].z;
        }
        this.trailLine.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        this.trailLine.geometry.attributes.position.needsUpdate = true;
      }
    }

    // 8. Update Camera Controls & Chase Tracking
    if (refPos) {
      if (this.options.cameraMode === 'chase') {
        this.controls.target.set(refPos[0], refPos[1], refPos[2]);
        this.camera.position.set(refPos[0] - 2.8, refPos[1] - 4.5, refPos[2] + 2.5);
      } else if (this.options.cameraMode === 'top') {
        this.controls.target.set(refPos[0], refPos[1], 0);
        this.camera.position.set(refPos[0], refPos[1], 15);
      } else if (this.options.cameraMode === 'side') {
        this.controls.target.set(refPos[0], refPos[1], refPos[2]);
        this.camera.position.set(refPos[0] + 10, refPos[1], refPos[2]);
      }
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
