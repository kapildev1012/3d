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

// ── Procedural texturing helpers (photoreal Mars surface treatment) ──

/** Deterministic 2-D hash in [0, 1). */
function hash2(x, y) {
  const s = Math.sin(x*127.1+y*311.7)*43758.5453;
  return s-Math.floor(s);
}

/** Small seeded PRNG (mulberry32) for scenery that must not touch physics. */
function createLocalRng(seed) {
  let s = seed >>> 0;
  return function () {
    s = (s+0x6d2b79f5) | 0;
    let t = Math.imul(s ^ (s >>> 15), 1 | s);
    t = (t+Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0)/4294967296;
  };
}

/** Smooth value noise — the base layer for terrain colour variation. */
function valueNoise(x, y) {
  const xi = Math.floor(x);
  const yi = Math.floor(y);
  const tx = x-xi;
  const ty = y-yi;
  const sx = tx*tx*(3-2*tx);
  const sy = ty*ty*(3-2*ty);
  const h00 = hash2(xi, yi);
  const h10 = hash2(xi+1, yi);
  const h01 = hash2(xi, yi+1);
  const h11 = hash2(xi+1, yi+1);
  return h00*(1-sx)*(1-sy)+h10*sx*(1-sy)+h01*(1-sx)*sy+h11*sx*sy;
}

/** Value noise on a wrapping integer lattice so textures tile seamlessly. */
function periodicNoise(x, y, period) {
  const wrap = v => ((v % period)+period) % period;
  const xi = Math.floor(x);
  const yi = Math.floor(y);
  const tx = x-xi;
  const ty = y-yi;
  const sx = tx*tx*(3-2*tx);
  const sy = ty*ty*(3-2*ty);
  const h00 = hash2(wrap(xi), wrap(yi));
  const h10 = hash2(wrap(xi+1), wrap(yi));
  const h01 = hash2(wrap(xi), wrap(yi+1));
  const h11 = hash2(wrap(xi+1), wrap(yi+1));
  return h00*(1-sx)*(1-sy)+h10*sx*(1-sy)+h01*(1-sx)*sy+h11*sx*sy;
}

/** Tileable three-octave fBm for the generated surface textures. */
function tileFbm(u, v) {
  return 0.52*periodicNoise(u*6, v*6, 6)
    +0.30*periodicNoise(u*12, v*12, 12)
    +0.18*periodicNoise(u*24, v*24, 24);
}

let marsDetailTexturesCache = null;

/**
 * Generate (once per session) a tiled albedo + bump texture pair that gives
 * the ground its fine granular Martian regolith grain at any camera
 * distance. The albedo map is kept close to neutral grey with a faint warm
 * bias so vertex colours remain the dominant tone carrier.
 */
function generateMarsSurfaceTextures(renderer) {
  if (marsDetailTexturesCache) return marsDetailTexturesCache;
  const SIZE = 512;
  const makeCanvas = () => {
    const canvas = document.createElement('canvas');
    canvas.width = SIZE;
    canvas.height = SIZE;
    return canvas;
  };
  const albedoCanvas = makeCanvas();
  const bumpCanvas = makeCanvas();
  const albedoCtx = albedoCanvas.getContext('2d');
  const bumpCtx = bumpCanvas.getContext('2d');
  const albedo = albedoCtx.createImageData(SIZE, SIZE);
  const bump = bumpCtx.createImageData(SIZE, SIZE);

  // Perfectly tileable fBm on a wrapping lattice.
  for (let py = 0; py < SIZE; py++) {
    for (let px = 0; px < SIZE; px++) {
      const v = tileFbm(px/SIZE, py/SIZE);

      // Albedo: near-neutral multiplier (205–255) with a whisper of warmth.
      const lum = 205+50*v;
      const offset = (py*SIZE+px)*4;
      albedo.data[offset] = Math.min(255, lum*1.02);
      albedo.data[offset+1] = lum*0.985;
      albedo.data[offset+2] = lum*0.955;
      albedo.data[offset+3] = 255;

      // Bump: same field, higher contrast for crisp micro-relief.
      const relief = Math.max(0, Math.min(255, (v-0.18)*300));
      bump.data[offset] = relief;
      bump.data[offset+1] = relief;
      bump.data[offset+2] = relief;
      bump.data[offset+3] = 255;
    }
  }
  albedoCtx.putImageData(albedo, 0, 0);
  bumpCtx.putImageData(bump, 0, 0);

  const anisotropy = renderer.capabilities?.getMaxAnisotropy?.() || 4;
  const map = new THREE.CanvasTexture(albedoCanvas);
  map.wrapS = map.wrapT = THREE.RepeatWrapping;
  map.colorSpace = THREE.SRGBColorSpace;
  map.anisotropy = anisotropy;
  const bumpMap = new THREE.CanvasTexture(bumpCanvas);
  bumpMap.wrapS = bumpMap.wrapT = THREE.RepeatWrapping;
  bumpMap.anisotropy = anisotropy;
  marsDetailTexturesCache = { map, bumpMap };
  return marsDetailTexturesCache;
}

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
    // Bright butterscotch Mars daytime: a dust-loaded sky whose colour the
    // distance fog converges to, producing the seamless hazy horizon of the
    // reference photographs. No roads, tracks, or artificial ground marks —
    // the surface stays entirely natural.
    const isExpedition = (this.terrainModel.cfg.experimentId === 14);
    const dustySky = new THREE.Color(0xb2794e);
    this.scene.background = dustySky;
    this.scene.fog = new THREE.FogExp2(dustySky.getHex(),
      isExpedition ? 0.0013 : 0.010);

    // Camera (Z is UP in physics simulation)
    this.camera = new THREE.PerspectiveCamera(45, width / height, 0.1,
      isExpedition ? 2500 : 200);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(-4, -6, 4);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    // Cinematic filmic response: deep contrast under a harsh single sun.
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.08;
    this.renderer.shadowMap.enabled = true;
    // Stark midday shadows: plain PCF keeps contact edges crisp instead of
    // buttery soft, matching the overhead-sun reference photography.
    this.renderer.shadowMap.type = THREE.PCFShadowMap;

    this.container.innerHTML = '';
    this.container.appendChild(this.renderer.domElement);

    // Orbit Controls with Z-up constraint
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.maxPolarAngle = Math.PI / 2 + 0.05;
    this.controls.target.set(0, 3, 1.2);
    this.controls.update();

    // Zenith midday sun: intense overhead illumination with minimal
    // lateral shadow spread. Hard, short shadows fall directly beneath.
    const ambLight = new THREE.AmbientLight(0xffd9b8, 0.38);
    this.scene.add(ambLight);

    const sunLight = new THREE.DirectionalLight(0xfff6e8, 2.95);
    sunLight.position.set(0, 0, 35);
    sunLight.castShadow = true;
    sunLight.shadow.mapSize.width = 2048;
    sunLight.shadow.mapSize.height = 2048;
    // Tight ortho frustum around the tracked rover keeps shadow texel density
    // high, so stones and struts cast sharp contact shadows anywhere on the
    // km² field (the light rides along in updateDual).
    sunLight.shadow.camera.near = 1.0;
    sunLight.shadow.camera.far = 90;
    sunLight.shadow.camera.left = -11;
    sunLight.shadow.camera.right = 11;
    sunLight.shadow.camera.top = 11;
    sunLight.shadow.camera.bottom = -11;
    sunLight.shadow.bias = -0.00012;
    sunLight.shadow.normalBias = 0.012;
    sunLight.shadow.camera.updateProjectionMatrix();
    this.sunLight = sunLight;
    this.sunTarget = new THREE.Object3D();
    this.scene.add(sunLight);
    this.scene.add(sunLight.target);

    const hemiLight = new THREE.HemisphereLight(0xd9a06b, 0x47231a, 0.50);
    hemiLight.position.set(0, 0, 20);
    this.scene.add(hemiLight);

    // Visible zenith sun: a blazing core plus a soft dust halo pinned far away
    // along the light direction, placed nearly straight up for midday Mars.
    this.sunDirection = new THREE.Vector3(0.5, -0.5, 60).normalize();
    this.createSunSprites();
    this._sunScratch = new THREE.Vector3();

    // Handle Resize
    this.resizeObserver = new ResizeObserver(() => this.onResize());
    this.resizeObserver.observe(this.container);
  }

  createSunSprites() {
    const SIZE = 256;
    const makeGlowTexture = stops => {
      const canvas = document.createElement('canvas');
      canvas.width = canvas.height = SIZE;
      const ctx = canvas.getContext('2d');
      const gradient = ctx.createRadialGradient(SIZE/2, SIZE/2, 0, SIZE/2, SIZE/2, SIZE/2);
      for (const [stop, color] of stops) gradient.addColorStop(stop, color);
      ctx.fillStyle = gradient;
      ctx.fillRect(0, 0, SIZE, SIZE);
      const texture = new THREE.CanvasTexture(canvas);
      texture.colorSpace = THREE.SRGBColorSpace;
      return texture;
    };
    const discMaterial = new THREE.SpriteMaterial({
      map: makeGlowTexture([
        [0.00, 'rgba(255,253,244,1)'],
        [0.16, 'rgba(255,246,218,1)'],
        [0.38, 'rgba(255,228,168,0.92)'],
        [0.62, 'rgba(255,206,132,0.28)'],
        [1.00, 'rgba(255,196,120,0)']
      ]),
      transparent: true,
      fog: false,
      depthWrite: false,
      blending: THREE.AdditiveBlending
    });
    this.sunDisc = new THREE.Sprite(discMaterial);

    const haloMaterial = new THREE.SpriteMaterial({
      map: makeGlowTexture([
        [0.00, 'rgba(255,232,190,0.55)'],
        [0.35, 'rgba(255,214,158,0.20)'],
        [0.70, 'rgba(250,196,138,0.06)'],
        [1.00, 'rgba(245,188,128,0)']
      ]),
      transparent: true,
      fog: false,
      depthWrite: false,
      blending: THREE.AdditiveBlending
    });
    this.sunHalo = new THREE.Sprite(haloMaterial);

    this.scene.add(this.sunHalo);
    this.scene.add(this.sunDisc);
  }

  onResize() {
    const width = this.container.clientWidth;
    const height = this.container.clientHeight;
    if (width === 0 || height === 0) return;

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  // Sample the RENDERED terrain surface (position + partial slopes) at a
  // world point, by bilinear interpolation across the display grid built in
  // createTerrainMesh() (this.renderedSurface). The eye judges "floating"
  // against this triangulated mesh, not against the continuous analytic
  // field the physics solver walks on: on Level 14 the ~3 m vertex spacing
  // sags visibly below the analytic height between vertices, which is
  // exactly why scatter meshes drifted airborne. Falls back to the analytic
  // field outside the displayed extents.
  renderedSurfaceSample(worldX, worldY, modelType) {
    const fallback = () => {
      const physicsX = worldX-(modelType === 'adaptive' ? this.laneOffset : -this.laneOffset);
      const surf = this.terrainModel.eval(physicsX, worldY, modelType);
      return { h: surf.h || 0, sx: surf.dhdx || 0, sy: surf.dhdy || 0 };
    };
    const grid = this.renderedSurface;
    if (!grid || worldX < grid.xMin || worldX > grid.xMax || worldY < grid.yMin || worldY > grid.yMax) {
      return fallback();
    }
    const fx = (worldX-grid.xMin)/grid.sx;
    const fy = (worldY-grid.yMin)/grid.sy;
    const gx = Math.max(0, Math.min(grid.nx-2, Math.floor(fx)));
    const gy = Math.max(0, Math.min(grid.ny-2, Math.floor(fy)));
    const tx = Math.max(0, Math.min(1, fx-gx));
    const ty = Math.max(0, Math.min(1, fy-gy));
    const heights = grid.heights;
    const at = (cx, cy) => heights[cy*grid.nx+cx];
    const h00 = at(gx, gy), h10 = at(gx+1, gy), h01 = at(gx, gy+1), h11 = at(gx+1, gy+1);
    const hLow = h00*(1-tx)+h10*tx;
    const hHigh = h01*(1-tx)+h11*tx;
    const h = hLow*(1-ty)+hHigh*ty;
    // Partial slopes from the same bilinear patch (finite differences of the
    // piecewise-linear surface), used to tilt meshes onto the ground normal.
    const dx = grid.sx, dy = grid.sy;
    const sxLow = (h10-h00)/dx, sxHigh = (h11-h01)/dx;
    const sx = sxLow*(1-ty)+sxHigh*ty;
    const syLeft = (h01-h00)/dy, syRight = (h11-h10)/dy;
    const sy = syLeft*(1-tx)+syRight*tx;
    return { h, sx, sy };
  }

  createTerrainMesh() {
    const isKm = this.terrainModel.cfg.terrainLevel === 14;
    // Level 14 is a true 1 km × 1 km open-world field centred on the origin:
    // (x, y) ∈ [-500 m, +500 m] on both axes.
    const Nx = isKm ? 320 : 80;
    const Ny = this.terrainModel.course ? 320 : (isKm ? 320 : 240);
    const xMin = this.terrainModel.course ? -5 : (isKm ? -500 : -8);
    const xMax = this.terrainModel.course ? 5 : (isKm ? 500 : 8);
    const yMin = this.terrainModel.course ? 0 : (isKm ? -500 : -4);
    const yMax = this.terrainModel.course ? 70 : (isKm ? 500 : 45);

    const geometry = new THREE.PlaneGeometry(xMax - xMin, yMax - yMin, Nx, Ny);
    const posAttr = geometry.attributes.position;
    const colors = [];

    // Photo-matched granular Mars palette: iron-rich sand, sunlit ridges,
    // and darker exposed stone on steeper faces.
    const sandShadow = new THREE.Color(0x5a2d20);
    const sandMid = new THREE.Color(0x99563c);
    const sandSun = new THREE.Color(0xc47b55);
    const dustBed = new THREE.Color(0xc99a6f);   // fine sand & dust beds
    const dustDarkPool = new THREE.Color(0x43241a); // dark fine-grain basins
    const exposedRock = new THREE.Color(0x3e2924);
    const marshDark = new THREE.Color(0x2e3a25);   // boggy green-brown
    const marshLight = new THREE.Color(0x4a5438);   // lighter marsh edge

    // Rendered-surface grid: the exact height samples that draw this mesh,
    // kept for bilinear re-sampling so every scattered stone can be anchored
    // onto the surface actually visible on screen (never above it).
    const gridWidth = Nx+1;
    const gridHeight = Ny+1;
    const renderedHeights = new Float32Array(gridWidth*gridHeight);
    this.renderedSurface = {
      xMin, xMax, yMin, yMax,
      nx: gridWidth, ny: gridHeight,
      sx: (xMax-xMin)/Nx, sy: (yMax-yMin)/Ny,
      heights: renderedHeights
    };

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

      // Record the sample in the rendered-surface grid (row-major, clamped
      // index math keeps rounding robust at the borders).
      const gx = Math.max(0, Math.min(gridWidth-1,
        Math.round((worldX-xMin)/((xMax-xMin)/Nx))));
      const gy = Math.max(0, Math.min(gridHeight-1,
        Math.round((worldY-yMin)/((yMax-yMin)/Ny))));
      renderedHeights[gy*gridWidth+gx] = surf.h;

      const heightRatio = Math.max(0, Math.min(1, (surf.h+0.28)/0.85));
      const slope = Math.hypot(surf.dhdx, surf.dhdy);
      // Multi-octave natural variation: broad albedo drift across the field,
      // medium-scale dark-dust pooling in the lows, plus fine grain — the
      // untouched surface keeps zero artificial patterning.
      const macroDrift = valueNoise(physicsX*0.021+4.19, worldY*0.021+8.13);
      const mesoDust = valueNoise(physicsX*0.09+37.7, worldY*0.09+91.3);
      const grainHash = Math.sin(127.1*physicsX+311.7*worldY)*43758.5453;
      const granularNoise = grainHash-Math.floor(grainHash);
      const grain = 0.86+0.20*granularNoise;
      const c = sandShadow.clone().lerp(sandMid, Math.min(1, 0.30+1.1*heightRatio));
      c.lerp(sandSun, Math.max(0, Math.min(0.55, 0.35*heightRatio)));
      c.lerp(exposedRock, Math.max(0, Math.min(0.58, (slope-0.20)*0.85)));
      // Wind-winnowed bright dust sheets vs darker fine-grain basins.
      c.multiplyScalar(0.88+0.24*macroDrift);
      c.lerp(dustDarkPool, Math.max(0, mesoDust-0.60)*0.60);
      // Tint marsh zones with dark green-brown colour
      if (this.terrainModel.marshes && this.terrainModel.marshes.length > 0) {
        const marshInfo = this.terrainModel.marshAt(physicsX, worldY);
        if (marshInfo.inMarsh) {
          const marshBlend = Math.min(0.80, marshInfo.depth*0.9);
          const marshTint = marshDark.clone().lerp(marshLight, 1.0-marshInfo.depth);
          c.lerp(marshTint, marshBlend);
        }
      }
      // Fine sand & dust beds read as pale, low-friction sheets.
      if (this.terrainModel.sandPatches && this.terrainModel.sandPatches.length > 0) {
        const sandInfo = this.terrainModel.sandAt(physicsX, worldY);
        if (sandInfo.inSand) c.lerp(dustBed, Math.min(0.62, 0.62*sandInfo.depth));
      }
      c.multiplyScalar(Math.max(0.76, grain));
      colors.push(c.r, c.g, c.b);
    }

    geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
    geometry.computeVertexNormals();

    // Tiled regolith detail: the generated albedo multiplies the vertex
    // colours while the bump map adds crisp millimetre grain under the harsh
    // sun — "extreme texturing" without a texture download.
    const detail = generateMarsSurfaceTextures(this.renderer);
    const extentX = xMax-xMin;
    const extentY = yMax-yMin;
    const tileMetres = 3.2;
    detail.map.repeat.set(extentX/tileMetres, extentY/tileMetres);
    detail.bumpMap.repeat.set(extentX/tileMetres, extentY/tileMetres);

    const material = new THREE.MeshStandardMaterial({
      vertexColors: true,
      map: detail.map,
      bumpMap: detail.bumpMap,
      bumpScale: 0.45,
      roughness: 0.96,
      metalness: 0.0,
      flatShading: true
    });

    this.terrainMesh = new THREE.Mesh(geometry, material);
    this.terrainMesh.position.set(0, (yMin + yMax) / 2, 0);
    this.terrainMesh.receiveShadow = true;
    this.scene.add(this.terrainMesh);

    /**
     * Bilinear re-sample of the rendered mesh surface. Returns NaN outside
     * the drawn extent so callers can skip scenery that has no visible
     * ground beneath it (the definitive anti-floating guarantee).
     */
    this.renderedHeightAt = (worldX, worldY) => {
      const g = this.renderedSurface;
      if (!g) return NaN;
      const fx = (worldX-g.xMin)/g.sx;
      const fy = (worldY-g.yMin)/g.sy;
      if (fx < -0.001 || fy < -0.001 || fx > g.nx-1+0.001 || fy > g.ny-1+0.001) return NaN;
      const ix = Math.max(0, Math.min(g.nx-2, Math.floor(fx)));
      const iy = Math.max(0, Math.min(g.ny-2, Math.floor(fy)));
      const tx = Math.max(0, Math.min(1, fx-ix));
      const ty = Math.max(0, Math.min(1, fy-iy));
      const h00 = g.heights[iy*g.nx+ix];
      const h10 = g.heights[iy*g.nx+ix+1];
      const h01 = g.heights[(iy+1)*g.nx+ix];
      const h11 = g.heights[(iy+1)*g.nx+ix+1];
      return h00*(1-tx)*(1-ty)+h10*tx*(1-ty)+h01*(1-tx)*ty+h11*tx*ty;
    };

    /** Lane mapping identical to the mesh loop above. */
    this.laneSampleAt = (worldX, worldY) => {
      const lane = worldX < 0 ? -this.laneOffset : this.laneOffset;
      return {
        physicsX: worldX-lane,
        modelType: worldX >= 0 ? 'adaptive' : 'fixed'
      };
    };

    /**
     * Exact visible ground height under a world-space point: the lower of
     * the analytic collision heightfield and the rendered mesh surface, so
     * anchored objects can never hover above the drawn terrain.
     */
    this.groundZAt = (worldX, worldY) => {
      const { physicsX, modelType } = this.laneSampleAt(worldX, worldY);
      const analytic = this.terrainModel.eval(physicsX, worldY, modelType).h || 0;
      const rendered = this.renderedHeightAt(worldX, worldY);
      return Number.isFinite(rendered) ? Math.min(analytic, rendered) : analytic;
    };

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
    pillarLeft.position.set(-3.0, goalY, 1.75 + this.groundZAt(-3.0, goalY));
    gateGroup.add(pillarLeft);

    const pillarRight = new THREE.Mesh(pillarGeom, pillarMat);
    pillarRight.position.set(3.0, goalY, 1.75 + this.groundZAt(3.0, goalY));
    gateGroup.add(pillarRight);

    const archGeom = new THREE.CylinderGeometry(0.08, 0.08, 6.0, 16);
    const archBar = new THREE.Mesh(archGeom, pillarMat);
    archBar.rotation.z = Math.PI / 2;
    archBar.position.set(0, goalY, 3.5 + this.groundZAt(0, goalY));
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
    finishLine.position.set(0, goalY, 0.02 + this.groundZAt(0, goalY));
    gateGroup.add(finishLine);

    this.scene.add(gateGroup);

    // Explicit task markers for the monitored adaptive lane.
    const startYMarker = this.terrainModel.course
      ? this.terrainModel.course.startY-1.0 : 0;
    const goalTarget = this.terrainModel.cfg.targetDestination || [0, goalY];
    const taskMarkerGroup = new THREE.Group();
    const startMarkerGroundZ = this.groundZAt(this.laneOffset, startYMarker);
    const goalMarkerGroundZ = this.groundZAt(goalTarget[0]+this.laneOffset, goalTarget[1]);
    const startMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.16, 20, 20),
      new THREE.MeshStandardMaterial({ color: 0x38bdf8, emissive: 0x0284c7, emissiveIntensity: 1.0 })
    );
    startMarker.position.set(this.laneOffset, startYMarker, startMarkerGroundZ+0.18);
    taskMarkerGroup.add(startMarker);
    const goalMarker = new THREE.Mesh(
      new THREE.TorusGeometry(0.30, 0.055, 12, 32),
      new THREE.MeshStandardMaterial({ color: 0x22c55e, emissive: 0x16a34a, emissiveIntensity: 1.2 })
    );
    goalMarker.position.set(goalTarget[0]+this.laneOffset, goalTarget[1], goalMarkerGroundZ+0.08);
    taskMarkerGroup.add(goalMarker);
    const startLabel = this.createTextSprite('START A', 'rgba(56, 189, 248, 1)');
    startLabel.scale.set(1.4, 0.35, 1);
    startLabel.position.set(this.laneOffset, startYMarker, startMarkerGroundZ+0.62);
    taskMarkerGroup.add(startLabel);
    const goalLabel = this.createTextSprite('GOAL B', 'rgba(34, 197, 94, 1)');
    goalLabel.scale.set(1.4, 0.35, 1);
    goalLabel.position.set(goalTarget[0]+this.laneOffset, goalTarget[1], goalMarkerGroundZ+0.62);
    taskMarkerGroup.add(goalLabel);
    this.taskMarkerGroup = taskMarkerGroup;
    this.scene.add(taskMarkerGroup);

    if (this.terrainModel.course) {
      const startY = this.terrainModel.course.startY;
      const startMaterial = new THREE.MeshBasicMaterial({
        color: 0x38bdf8, side: THREE.DoubleSide, transparent: true, opacity: 0.72
      });
      const startLine = new THREE.Mesh(new THREE.PlaneGeometry(6, 0.28), startMaterial);
      startLine.position.set(0, startY, this.groundZAt(0, startY)+0.025);
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
            const modelType = laneOffset > 0 ? 'adaptive' : 'fixed';
            // Anchor onto the exact visible ground of this lane.
            const shardGroundZ = Math.min(
              this.groundZAt(localX+laneOffset, localY),
              this.renderedSurfaceSample(localX+laneOffset, localY, modelType).h);
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
            // Z-only rotation keeps every facet below the solid surface;
            // x/y tilt would lift corners above collision height.
            mesh.rotation.set(0, 0, angle);
            mesh.position.set(localX+laneOffset, localY, shardGroundZ-shardHeight);
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
          this.groundZAt(obstacle.x+this.laneOffset, obstacle.y)+0.06
        );
        this.scene.add(checkpointRing);
        const checkpointLabel = this.createTextSprite(`B${obstacleIndex+1}`, 'rgba(34, 211, 238, 1)');
        checkpointLabel.scale.set(0.72, 0.22, 1);
        checkpointLabel.position.set(
          obstacle.x+this.laneOffset,
          obstacle.y,
          this.groundZAt(obstacle.x+this.laneOffset, obstacle.y)+0.38
        );
        this.scene.add(checkpointLabel);
      }
    }

    // Level 14 expedition obstacle chain: Level-10-style solid obstacles at
    // expedition scale, drawn as shard clusters plus a cyan crest ring so the
    // adaptive lane can see its sequential checkpoint targets.
    if (this.terrainModel.expeditionObstacles?.length) {
      const severityScale = { small: 1.0, medium: 1.25, large: 1.5 };
      for (let obstacleIndex = 0; obstacleIndex < this.terrainModel.expeditionObstacles.length; obstacleIndex++) {
        const obstacle = this.terrainModel.expeditionObstacles[obstacleIndex];
        const severity = severityScale[obstacle.difficulty] || 1.0;
        for (const laneOffset of [-this.laneOffset, this.laneOffset]) {
          for (let shard = 0; shard < 5; shard++) {
            const angle = obstacle.yaw+shard*2.39996+obstacleIndex*0.47;
            const radial = shard === 0 ? 0 : 0.18+0.10*((shard+obstacleIndex)%3);
            const localX = obstacle.x+radial*obstacle.radiusX*Math.cos(angle);
            const localY = obstacle.y+radial*obstacle.radiusY*Math.sin(angle);
            const shardHeight = obstacle.height*(shard === 0 ? 0.34 : 0.18+0.035*((shard+2*obstacleIndex)%3));
            const expModelType = laneOffset > 0 ? 'adaptive' : 'fixed';
            // Anchor onto the rendered mesh surface, not the analytic field,
            // so shards never float above sagging display quads.
            const surface = this.renderedSurfaceSample(localX+laneOffset, localY, expModelType);
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
            mesh.scale.set(
              obstacle.radiusX*(shard === 0 ? 0.38 : 0.23+0.035*(shard%3)),
              obstacle.radiusY*(shard === 0 ? 0.31 : 0.20+0.03*((shard+1)%3)),
              shardHeight
            );
            mesh.rotation.set(0, 0, angle);
            mesh.position.set(localX+laneOffset, localY,
              Math.min(surface.h, this.groundZAt(localX+laneOffset, localY))-shardHeight*0.6);
            mesh.castShadow = true;
            this.scene.add(mesh);
          }
        }
        // Crest ring marks the scored checkpoint on the adaptive lane.
        const ring = new THREE.Mesh(
          new THREE.TorusGeometry(0.30*severity, 0.04, 10, 26),
          new THREE.MeshStandardMaterial({
            color: 0x22d3ee,
            emissive: 0x0891b2,
            emissiveIntensity: 1.35,
            roughness: 0.35
          })
        );
        ring.position.set(
          obstacle.x+this.laneOffset,
          obstacle.y,
          this.groundZAt(obstacle.x+this.laneOffset, obstacle.y)+0.06
        );
        this.scene.add(ring);
      }
    }

    // Level 14 expedition beacons: one glowing pylon every 50 m so a 1 km run
    // reads as a chain of reachable milestones instead of an endless slope.
    if (this.terrainModel.cfg.experimentId === 14 && !this.terrainModel.course) {
      const startY = this.terrainModel.cfg.courseStartY || 10;
      const goalY = this.terrainModel.cfg.targetGoalY || 800;
      const spacing = this.terrainModel.cfg.waypointSpacing || 50;
      const beaconGeometry = new THREE.CylinderGeometry(0.06, 0.10, 1.6, 8);
      for (let y = startY+spacing; y < goalY; y += spacing) {
        const index = Math.round((y-startY)/spacing)-1;
        const beaconGroundZ = this.groundZAt(this.laneOffset, y);
        const beacon = new THREE.Mesh(beaconGeometry, new THREE.MeshStandardMaterial({
          color: 0x38bdf8,
          emissive: 0x0284c7,
          emissiveIntensity: 1.1,
          roughness: 0.4
        }));
        beacon.position.set(this.laneOffset, y, beaconGroundZ+0.8);
        this.scene.add(beacon);
        const label = this.createTextSprite(`WP${index+1}`, 'rgba(56, 189, 248, 1)');
        label.scale.set(0.9, 0.26, 1);
        label.position.set(this.laneOffset, y, beaconGroundZ+1.85);
        this.scene.add(label);
      }
    }

    // ── Unified anchored surface scatter ──
    // Every stone in the world (shared Mars scenery, Level 14 pebble /
    // sharp-rock / boulder classes, Model B's private path stones and the
    // course grit) is spawned through one pipeline: exact heightfield
    // anchoring with a rendered-mesh clamp, ground-normal orientation and
    // size-class instanced batches.
    this.createScatterField();

    // Level 14 horizon: a ring of low, weathered ridge/cliff silhouettes at
    // the field fringe breaks the empty horizon exactly like the reference
    // photography. Pure scenery beyond the mission corridor — never
    // simulated — and fully fog-blended for atmospheric depth.
    if (isKm && !this.terrainModel.course) {
      const ridgeSeed = (this.terrainModel.cfg.seed ^ 0x0dd1d0d5) >>> 0;
      const ridgeRng = createLocalRng(ridgeSeed);
      const ridgeMaterials = [
        new THREE.MeshStandardMaterial({
          color: new THREE.Color(0x4a2b1f), roughness: 1.0, metalness: 0.0, flatShading: true
        }),
        new THREE.MeshStandardMaterial({
          color: new THREE.Color(0x6b4230), roughness: 1.0, metalness: 0.0, flatShading: true
        })
      ];
      const ridgeCount = 18;
      for (let i = 0; i < ridgeCount; i++) {
        const theta = (i/ridgeCount)*Math.PI*2+(ridgeRng()-0.5)*0.30;
        const radius = 458+ridgeRng()*30;
        const x = Math.max(-494, Math.min(494, Math.cos(theta)*radius));
        const y = Math.max(-494, Math.min(494, Math.sin(theta)*radius));
        const length = 150+ridgeRng()*190;
        const depth = 26+ridgeRng()*32;
        const height = 16+ridgeRng()*38;
        const geometry = new THREE.DodecahedronGeometry(1, 0);
        const ridge = new THREE.Mesh(geometry,
          ridgeMaterials[i%ridgeMaterials.length]);
        ridge.scale.set(length*0.5, depth, height);
        ridge.rotation.set(0, 0, theta+Math.PI/2+(ridgeRng()-0.5)*0.4);
        ridge.position.set(x, y, this.groundZAt(x, y)-height*0.45);
        this.scene.add(ridge);
      }
    }
  }

  // Size-class → low-poly geometry mapping for the scatter field.
  scatterClassFor(rock) {
    const r = Math.max(rock.rx || rock.r || 0.2, rock.ry || rock.r || 0.2);
    if (rock.sharp) return 'sharp';
    if (r >= 1.2) return 'boulder';
    if (r <= 0.16 && (rock.h ?? 0) <= 0.13) return 'pebble';
    return 'stone';
  }

  createScatterField() {
    const bothLanes = [-this.laneOffset, this.laneOffset];
    const items = [];
    const collect = (rocks, lanes) => {
      for (const rock of rocks || []) {
        for (const laneOffset of lanes) {
          const worldX = rock.x+laneOffset;
          // The two terrain copies meet at x=0. Do not draw a side-scene
          // stone after its lane offset carries it across that boundary,
          // where the mesh would have no matching physical height field.
          if ((laneOffset < 0 && worldX >= 0) || (laneOffset > 0 && worldX < 0)) continue;
          items.push({ rock, laneOffset, worldX });
        }
      }
    };
    collect(this.terrainModel.rocks, bothLanes);
    // Cosmetic micro gravel rides the exact same anchoring pipeline — every
    // chip is seated on the heightfield like a physical stone.
    collect(this.terrainModel.decorChips, bothLanes);
    if (this.terrainModel.course) {
      collect(this.terrainModel.bPathRocks, [this.laneOffset]);
      collect(this.terrainModel.courseGritRocks, bothLanes);
    }

    const geometries = {
      pebble: new THREE.IcosahedronGeometry(1, 0),
      sharp: new THREE.OctahedronGeometry(1, 0),
      stone: new THREE.DodecahedronGeometry(1, 0),
      boulder: new THREE.DodecahedronGeometry(1, 0)
    };
    // Irregular stones read as dark, weathered basalt against the bright
    // dust — high contrast makes every boulder pop with a hard shadow edge.
    const shadeDark = new THREE.Color(0x1f1410);
    const shadeLight = new THREE.Color(0x4c3126);
    const pebbleTint = new THREE.Color(0x5d3d2b);
    const boulderTint = new THREE.Color(0x140e0b);

    const groups = new Map();
    for (const { rock, laneOffset, worldX } of items) {
      const modelType = laneOffset > 0 ? 'adaptive' : 'fixed';
      // No drawn terrain beneath this stone (outside the rendered extent)?
      // Skip it entirely — that is how stones used to hover over the void.
      if (!Number.isFinite(this.renderedHeightAt?.(worldX, rock.y))) continue;
      const sample = this.renderedSurfaceSample(worldX, rock.y, modelType);
      // Exact anchor: the lower of the analytic collision crest and the
      // rendered mesh surface guarantees the base can never sit above the
      // visible ground, whatever the grid resolution does between vertices.
      const groundZ = this.groundZAt(worldX, rock.y);
      const cls = this.scatterClassFor(rock);
      const rx = Math.max(0.02, rock.rx || rock.r || 0.2);
      const ry = Math.max(0.02, rock.ry || rock.r || 0.2);
      const exposedRatio = cls === 'pebble' ? 0.78 : 0.72;
      const minHeight = cls === 'pebble' ? 0.03 : 0.05;
      const renderedHeight = Math.max(minHeight, exposedRatio*(rock.h || 0.06));

      let group = groups.get(cls);
      if (!group) {
        group = {
          geometry: geometries[cls],
          material: new THREE.MeshStandardMaterial({
            color: 0xffffff, roughness: 1.0, metalness: 0.0, flatShading: true
          }),
          transforms: []
        };
        groups.set(cls, group);
      }
      // Seat the centre below the visible surface: the top pokes out by
      // ~3/4 of the rendered height while the base stays buried even across
      // sagging quads, then tilt onto the local ground normal.
      const centerZ = groundZ-0.25*renderedHeight;
      const color = shadeDark.clone().lerp(shadeLight, 0.25+0.45*(rock.colorSeed ?? 0.5));
      if (cls === 'pebble') color.lerp(pebbleTint, 0.35);
      if (cls === 'boulder') color.lerp(boulderTint, 0.30);
      const position = new THREE.Vector3(worldX, rock.y, centerZ);
      const orientation = new THREE.Quaternion().setFromEuler(new THREE.Euler(
        Math.atan(sample.sy), -Math.atan(sample.sx), rock.yaw || 0, 'XYZ'));
      const scale = new THREE.Vector3(rx, ry, renderedHeight);
      group.transforms.push({ matrix: new THREE.Matrix4()
        .compose(position, orientation, scale), color });
    }

    for (const [cls, group] of groups) {
      const count = group.transforms.length;
      if (!count) continue;
      const mesh = new THREE.InstancedMesh(group.geometry, group.material, count);
      group.transforms.forEach((transform, index) => {
        mesh.setMatrixAt(index, transform.matrix);
        mesh.setColorAt(index, transform.color);
      });
      mesh.instanceMatrix.needsUpdate = true;
      if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      // Instances span the whole km²; never cull against the unit-geometry
      // bounding sphere.
      mesh.frustumCulled = false;
      mesh.name = `scatter-${cls}`;
      this.scene.add(mesh);
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

    // The sun rides along with the rover so the tight, sharp shadow frustum
    // always covers the action — even 450 m out on the open expedition.
    // Near-zenith offset keeps shadows short and stark, directly beneath.
    if (this.sunLight && this.sunTarget) {
      const focus = refPosB_World;
      this.sunTarget.position.set(focus[0], focus[1], focus[2]);
      this.sunLight.position.set(
        focus[0]+this.sunDirection.x*40,
        focus[1]+this.sunDirection.y*40,
        focus[2]+this.sunDirection.z*40);
      this.sunTarget.updateMatrixWorld();
    }

    // Pin the visible sun disc + halo far along the light direction from the
    // camera: an infinitely distant sun with constant angular size that real
    // ridges and terrain can still occlude.
    if (this.sunDisc && this.sunHalo && this.sunDirection) {
      const sunDistance = this.camera.far*0.80;
      const sunPosition = this._sunScratch.copy(this.camera.position)
        .addScaledVector(this.sunDirection, sunDistance);
      this.sunDisc.position.copy(sunPosition);
      this.sunHalo.position.copy(sunPosition);
      this.sunDisc.scale.setScalar(sunDistance*0.055);
      this.sunHalo.scale.setScalar(sunDistance*0.22);
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
