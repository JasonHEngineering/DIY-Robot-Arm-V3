// Mesh-accurate collision checking for the Hayley URDF using three-mesh-bvh.
//  - self collision between links that are not adjacent in the chain
//  - collision with the table top the base plate is bolted to
// Pairs already touching at the kinematic-zero pose (bearings, shared covers) are
// treated as "always touching" and ignored, like MoveIt's setup-assistant matrix.
import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

export const LINKS = ['base_link', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6'];
const SKIP_VISUALS = new Set(['hardware']);   // screws/standoffs: tiny, often modelled interpenetrating

export function owningLink(obj) {
  let o = obj;
  while (o && !o.isURDFLink) o = o.parent;
  return o;
}
function owningVisual(obj) {
  let o = obj;
  while (o && !o.isURDFVisual && !o.isURDFLink) o = o.parent;
  return o && o.isURDFVisual ? o : null;
}

export class CollisionModel {
  /** robot: a loaded URDFRobot at any pose (meshes loaded). */
  constructor(robot, { tableZ = -0.010 } = {}) {
    this.tableZ = tableZ;                         // base plate underside (m, robot frame)
    this.geo = {};
    this.box = {};
    robot.updateMatrixWorld(true);
    for (const name of LINKS) {
      const link = robot.links[name];
      const inv = link.matrixWorld.clone().invert();
      const parts = [];
      link.traverse(o => {
        if (!o.isMesh || owningLink(o) !== link) return;
        const vis = owningVisual(o);
        if (vis && SKIP_VISUALS.has(vis.name)) return;
        const g = new THREE.BufferGeometry();
        g.setAttribute('position', o.geometry.attributes.position.clone());
        if (o.geometry.index) g.setIndex(o.geometry.index.clone());
        g.applyMatrix4(new THREE.Matrix4().multiplyMatrices(inv, o.matrixWorld));
        parts.push(g.index ? g.toNonIndexed() : g);
      });
      if (!parts.length) continue;
      const merged = mergeGeometries(parts, false);
      merged.boundsTree = new MeshBVH(merged, { maxLeafTris: 8 });
      merged.computeBoundingBox();
      this.geo[name] = merged;
      this.box[name] = merged.boundingBox.clone();
    }
    this.names = LINKS.filter(n => this.geo[n]);
    this.ignored = new Set();
    for (let i = 0; i + 1 < LINKS.length; i++) this.ignored.add(key(LINKS[i], LINKS[i + 1]));
    this.alwaysTouching = [];
  }

  /** Call once with the robot at kinematic zero: pairs touching there are ignored from then on. */
  calibrate(robot) {
    const res = this.check(robot, { includeTable: false });
    for (const [a, b] of res.pairs) { this.ignored.add(key(a, b)); this.alwaysTouching.push(`${a}–${b}`); }
    return this.alwaysTouching;
  }

  /**
   * Check a robot instance at its current joint values.
   * returns { pairs: [[a,b],...], table: [link...], links: Set<link>, any: bool }
   */
  check(robot, { includeTable = true } = {}) {
    robot.updateMatrixWorld(true);
    const M = {}, W = {};
    for (const n of this.names) {
      M[n] = robot.links[n].matrixWorld;
      W[n] = this.box[n].clone().applyMatrix4(M[n]);
    }
    const pairs = [], table = [], links = new Set();
    const rel = new THREE.Matrix4(), inv = new THREE.Matrix4();
    for (let i = 0; i < this.names.length; i++) {
      const a = this.names[i];
      inv.copy(M[a]).invert();
      for (let j = i + 1; j < this.names.length; j++) {
        const b = this.names[j];
        if (this.ignored.has(key(a, b)) || !W[a].intersectsBox(W[b])) continue;
        rel.multiplyMatrices(inv, M[b]);
        if (this.geo[a].boundsTree.intersectsGeometry(this.geo[b], rel)) { pairs.push([a, b]); links.add(a); links.add(b); }
      }
    }
    if (includeTable) {
      // table slab in the robot (URDF) frame, mapped into each link's frame
      const slab = new THREE.Box3(new THREE.Vector3(-2, -2, this.tableZ - 1), new THREE.Vector3(2, 2, this.tableZ - 0.0005));
      const R = robot.matrixWorld;
      for (const n of this.names) {
        if (n === 'base_link') continue;
        rel.copy(M[n]).invert().multiply(R);
        if (this.geo[n].boundsTree.intersectsBox(slab, rel)) { table.push(n); links.add(n); }
      }
    }
    return { pairs, table, links, any: pairs.length + table.length > 0 };
  }
}
const key = (a, b) => (a < b ? a + '|' + b : b + '|' + a);

export function describe(res) {
  if (!res || !res.any) return 'no collision';
  const parts = res.pairs.map(([a, b]) => `${short(a)} ↔ ${short(b)}`).concat(res.table.map(n => `${short(n)} ↔ table`));
  return parts.join(', ');
}
const short = n => n === 'base_link' ? 'base' : n.replace('link', 'L');

/** Highlights colliding links of one robot instance by swapping materials. */
export class Highlighter {
  constructor(robot, hitMaterial) {
    this.items = [];
    robot.traverse(o => { if (o.isMesh) this.items.push({ mesh: o, link: owningLink(o)?.name, base: o.material }); });
    this.hit = hitMaterial;
    this.state = '';
  }
  rebase() { for (const it of this.items) it.base = it.mesh.material; }   // after a restyle
  apply(links) {
    const sig = [...(links || [])].sort().join(',');
    if (sig === this.state) return;
    this.state = sig;
    for (const it of this.items) it.mesh.material = links && links.has(it.link) ? this.hit : it.base;
  }
}
