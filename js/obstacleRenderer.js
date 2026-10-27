/**
 * OBSTACLERENDERER.JS - three.js meshes for the solid obstacle field
 * (js/obstacleField.js), drawn identically in Model A's and Model B's lanes.
 *
 *   box       chamfered (bevelled) industrial crates / concrete blocks, ledges,
 *             walls and gate pillars, with subtle edge highlights
 *   rock      displaced, flat-shaded dark basalt dodecahedrons (Level 10 style),
 *             one per sphere-swept lobe of the collision shape
 *   cylinder  fallen pipes / rubble rollers
 *   ramp      inclined concrete wedges
 * All meshes cast and receive shadows. 'cross' obstacles (the ones a rover has
 * to crest) get a cyan checkpoint ring and a CP tag on Model B's lane, like the
 * Level 10 course. The physics shapes and the meshes come from the same numbers,
 * so what is drawn is what collides (rock facets are within a few mm of the lobe
 * spheres they represent).
 *
 * THREE is passed in so this module also loads in Node (tests) without three.js.
 */

const MATERIAL_SPECS = {
  crate: { color: 0x7a5a3a, roughness: 0.85, metalness: 0.05 },
  ledge: { color: 0x8d8a84, roughness: 0.95, metalness: 0.0 },
  wall: { color: 0x7d7a74, roughness: 0.95, metalness: 0.0 },
  pillar: { color: 0x9a968e, roughness: 0.9, metalness: 0.0 },
  roller: { color: 0x5d4a3c, roughness: 0.55, metalness: 0.45 },
  ramp: { color: 0x85817a, roughness: 0.95, metalness: 0.0 },
  boulder: { color: 0x2b211d, roughness: 1.0, metalness: 0.0 }
};

function seeded(seed) {
  let a = seed >>> 0;
  return () => { a = (a + 0x6D2B79F5) >>> 0; let t = Math.imul(a ^ (a >>> 15), a | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
}

function chamferedBox(THREE, o) {
  const r = Math.min(o.chamfer || 0.01, 0.4 * Math.min(o.hx, o.hy, o.hz));
  const shape = new THREE.Shape();
  const w = o.hx - r, d = o.hy - r;
  shape.moveTo(-w, -d); shape.lineTo(w, -d); shape.lineTo(w, d); shape.lineTo(-w, d); shape.lineTo(-w, -d);
  const geometry = new THREE.ExtrudeGeometry(shape, {
    depth: Math.max(1e-3, 2 * o.hz - 2 * r), bevelEnabled: true, bevelThickness: r, bevelSize: r, bevelSegments: 2, steps: 1, curveSegments: 4
  });
  geometry.translate(0, 0, -(o.hz - r));            // centre the extrusion on z = 0
  return geometry;
}

function rampGeometry(THREE, o) {
  const e = o.embed, H = o.height, L = o.ly, X = o.hx;
  // wedge in the obstacle frame (z measured from zBase): low edge at y = -L, high face at y = +L
  const v = [
    [-X, -L, 0], [X, -L, 0], [X, L, 0], [-X, L, 0],          // bottom
    [-X, -L, e], [X, -L, e], [X, L, e + H], [-X, L, e + H]   // top (inclined)
  ];
  const faces = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]];
  const positions = [];
  for (const f of faces) for (const k of f) positions.push(...v[k]);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.computeVertexNormals();
  return geometry;
}

function basaltLobe(THREE, radius, seed) {
  const geometry = new THREE.DodecahedronGeometry(radius, 1).toNonIndexed();
  const rnd = seeded(seed);
  const pos = geometry.attributes.position;
  // Deterministic inward-biased displacement keeps the facets inside the
  // collision sphere (|jag| <= 8% of r), so nothing visible floats off the solid.
  const cache = new Map();
  for (let i = 0; i < pos.count; i++) {
    const key = `${pos.getX(i).toFixed(4)},${pos.getY(i).toFixed(4)},${pos.getZ(i).toFixed(4)}`;
    if (!cache.has(key)) cache.set(key, 1 - 0.08 * rnd());
    const s = cache.get(key);
    pos.setXYZ(i, pos.getX(i) * s, pos.getY(i) * s, pos.getZ(i) * s);
  }
  geometry.computeVertexNormals();
  return geometry;
}

/**
 * Add the obstacle field to a scene.
 * @param {object} THREE three.js namespace
 * @param {object} scene THREE.Scene (or Group)
 * @param {Array} field obstacle list (lane-local)
 * @param {object} opts { laneOffsets: [xA, xB], adaptiveLaneOffset, createTextSprite(text, color) }
 * @returns {object} THREE.Group with all obstacle meshes (call dispose by removing it)
 */
export function addObstacleFieldMeshes(THREE, scene, field, opts = {}) {
  const group = new THREE.Group();
  group.name = 'obstacle-field';
  if (!field?.length) { scene.add(group); return group; }
  const lanes = opts.laneOffsets || [-1.5, 1.5];
  const materials = {};
  const material = template => {
    if (!materials[template]) {
      const spec = MATERIAL_SPECS[template] || MATERIAL_SPECS.crate;
      materials[template] = new THREE.MeshStandardMaterial({ ...spec, flatShading: template === 'boulder' });
    }
    return materials[template];
  };
  const edgeMaterial = new THREE.LineBasicMaterial({ color: 0xf1e6d2, transparent: true, opacity: 0.28 });

  for (const o of field) {
    for (const lane of lanes) {
      const node = new THREE.Group();
      node.position.set(o.x + lane, o.y, 0);
      node.rotation.z = o.yaw || 0;
      if (o.kind === 'box') {
        const geometry = chamferedBox(THREE, o);
        const mesh = new THREE.Mesh(geometry, material(o.template));
        mesh.position.z = o.cz;
        mesh.castShadow = true; mesh.receiveShadow = true;
        node.add(mesh);
        const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geometry, 35), edgeMaterial);
        edges.position.z = o.cz;
        node.add(edges);
        if (o.template === 'crate') {                    // plank bands on crates
          const band = new THREE.Mesh(new THREE.BoxGeometry(2 * o.hx * 1.002, 2 * o.hy * 1.002, 0.012), material('roller'));
          band.position.z = o.cz + 0.45 * o.hz;
          band.castShadow = true;
          node.add(band);
        }
      } else if (o.kind === 'rock') {
        o.lobes.forEach((lobe, k) => {
          const mesh = new THREE.Mesh(basaltLobe(THREE, lobe.r, (o.facetSeed || 1) + 7919 * k), material('boulder'));
          mesh.position.set(lobe.dx, lobe.dy, lobe.cz);
          mesh.rotation.set(0.3 * k, 0.7 * k, 1.1 * k);
          mesh.castShadow = true; mesh.receiveShadow = true;
          node.add(mesh);
        });
      } else if (o.kind === 'cylinder') {
        const mesh = new THREE.Mesh(new THREE.CylinderGeometry(o.radius, o.radius, 2 * o.halfLength, 20, 1), material('roller'));
        mesh.rotation.z = Math.PI / 2;                   // cylinder axis (y) -> obstacle x axis
        mesh.position.z = o.cz;
        mesh.castShadow = true; mesh.receiveShadow = true;
        node.add(mesh);
        for (const end of [-1, 1]) {                     // dark pipe bores
          const cap = new THREE.Mesh(new THREE.CircleGeometry(o.radius * 0.72, 16), new THREE.MeshBasicMaterial({ color: 0x15100d }));
          cap.rotation.y = end * Math.PI / 2;
          cap.position.set(end * (o.halfLength + 1e-3), 0, o.cz);
          node.add(cap);
        }
      } else if (o.kind === 'ramp') {
        const mesh = new THREE.Mesh(rampGeometry(THREE, o), material('ramp'));
        mesh.position.z = o.zBase;
        mesh.castShadow = true; mesh.receiveShadow = true;
        node.add(mesh);
      }
      group.add(node);
    }
    // Checkpoint ring + tag at the crest of every obstacle that must be crested,
    // on the adaptive lane (Model B), matching the Level 10 course markers.
    if (o.role === 'cross') {
      const lane = opts.adaptiveLaneOffset ?? lanes[lanes.length - 1];
      const ring = new THREE.Mesh(
        new THREE.TorusGeometry(0.22, 0.03, 10, 26),
        new THREE.MeshStandardMaterial({ color: 0x22d3ee, emissive: 0x0891b2, emissiveIntensity: 1.35, roughness: 0.35 })
      );
      ring.position.set(o.x + lane, o.y, o.top + 0.05);
      group.add(ring);
      if (opts.createTextSprite) {
        const label = opts.createTextSprite(`CP ${o.id}`, 'rgba(34, 211, 238, 1)');
        label.scale.set(0.72, 0.22, 1);
        label.position.set(o.x + lane, o.y, o.top + 0.36);
        group.add(label);
      }
    }
  }
  scene.add(group);
  return group;
}
