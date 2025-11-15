import * as THREE from "three";
import {
  getEdgeVertexIndices,
  getBendingVertexIds,
} from "../geometry/triangle_utils.js";

import {
  vecSetDiff,
  vecSetCross,
  vecLengthSquared,
  vecDistSquared,
  vecAdd,
  vecCopy,
} from "../array.js";

import { vecScale } from "../array.js";

export class Cloth {
  constructor({ mesh, scene, attachVertexIds = [] }) {
    this.numVertices = mesh.vertices.length / 3;
    this.attachVertexIds = attachVertexIds;
    this.pos = new Float32Array(mesh.vertices);
    this.restPos = new Float32Array(mesh.vertices);
    this.prevPos = new Float32Array(mesh.vertices);
    this.edgeVertexIds = getEdgeVertexIndices(mesh.faceTriIds);
    this.triangleVertexIds = mesh.faceTriIds;
    this.bendingVertexIds = getBendingVertexIds(mesh.faceTriIds);
    this.vel = new Float32Array(3 * this.numVertices);

    this.constraintSolvers = [];
    this.colliderConstraintSolvers = [];

    this.grabId = -1;
    this.grabInvMass = 0.0;

    this.invMass = new Float32Array(this.numVertices);
    this.initInvMass(mesh.faceTriIds);

    // --------------------------------------------------------------------
    // visual edge mesh
    // --------------------------------------------------------------------
    let geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(this.pos, 3));
    geometry.setIndex(this.edgeVertexIds);
    const lineMaterial = new THREE.LineBasicMaterial({
      color: 0xff0000,
      linewidth: 2,
    });
    this.edgeMesh = new THREE.LineSegments(geometry, lineMaterial);
    this.edgeMesh.visible = false;
    scene.add(this.edgeMesh);

    // --------------------------------------------------------------------
    // visual tri mesh
    // --------------------------------------------------------------------
    geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(this.pos, 3));
    geometry.setIndex(mesh.faceTriIds);
    const visMaterial = new THREE.MeshPhongMaterial({
      color: 0xff0000,
      side: THREE.DoubleSide,
    });
    this.triMesh = new THREE.Mesh(geometry, visMaterial);
    // this.triMesh.visible = false;
    this.triMesh.castShadow = true;
    this.triMesh.userData = this; // for raycasting

    this.triMesh.layers.enable(1);
    scene.add(this.triMesh);
    geometry.computeVertexNormals();

    this.updateMeshes();
  }

  resetSimulation() {
    this.pos.set(this.restPos);
    this.prevPos.set(this.restPos);
    this.vel.fill(0.0);
  }

  setConstraintSolvers(solvers) {
    this.constraintSolvers = solvers;
  }

  setColliderConstraintSolvers(solvers) {
    this.colliderConstraintSolvers = solvers;
  }

  // --------------------------------------------------------------------
  // mass / rest-length init
  // --------------------------------------------------------------------
  initInvMass(triangleVertexIndices) {
    this.invMass.fill(0.0);

    const triangleCount = triangleVertexIndices.length / 3;
    const edge0 = [0.0, 0.0, 0.0];
    const edge1 = [0.0, 0.0, 0.0];
    const cross = [0.0, 0.0, 0.0];

    // 面積に基づいて頂点の質量を分配
    for (let triIndex = 0; triIndex < triangleCount; triIndex++) {
      const v0 = triangleVertexIndices[3 * triIndex + 0];
      const v1 = triangleVertexIndices[3 * triIndex + 1];
      const v2 = triangleVertexIndices[3 * triIndex + 2];

      vecSetDiff(edge0, 0, this.pos, v1, this.pos, v0);
      vecSetDiff(edge1, 0, this.pos, v2, this.pos, v0);
      vecSetCross(cross, 0, edge0, 0, edge1, 0);

      const triArea = 0.5 * Math.sqrt(vecLengthSquared(cross, 0));
      const pointInvMass = triArea > 0.0 ? 1.0 / (3.0 * triArea) : 0.0;

      this.invMass[v0] += pointInvMass;
      this.invMass[v1] += pointInvMass;
      this.invMass[v2] += pointInvMass;
    }

    // ------------------------------------------------------------------
    // attach: 上端両角の頂点を固定
    // ------------------------------------------------------------------
    for (const i of this.attachVertexIds) {
      this.invMass[i] = 0.0;
    }
  }

  // --------------------------------------------------------------------
  // time integration
  // --------------------------------------------------------------------
  preSolve(dt, gravity) {
    for (let i = 0; i < this.numVertices; i++) {
      if (this.invMass[i] === 0.0) continue;

      // v += g * dt
      vecAdd(this.vel, i, gravity, 0, dt);

      // x_prev = x
      vecCopy(this.prevPos, i, this.pos, i);

      // x += v * dt
      vecAdd(this.pos, i, this.vel, i, dt);

      // simple ground collision (y >= 0)
      const y = this.pos[3 * i + 1];
      if (y < 0.0) {
        vecCopy(this.pos, i, this.prevPos, i);
        this.pos[3 * i + 1] = 0.0;
      }
    }
  }

  solve(dt, colliders) {
    for (const solver of this.constraintSolvers) {
      solver.solve(this.pos, dt);
    }

    for (const solver of this.colliderConstraintSolvers) {
      for (const collider of colliders) {
        const { sphereCenter, sphereRadius } = collider;
        solver.solve(this.pos, dt, sphereCenter, sphereRadius);
      }
    }
  }

  postSolve(dt) {
    for (let i = 0; i < this.numVertices; i++) {
      if (this.invMass[i] === 0.0) continue;
      vecSetDiff(this.vel, i, this.pos, i, this.prevPos, i, 1.0 / dt);
    }

    for (let i = 0; i < this.numVertices; i++) {
      if (this.invMass[i] == 0.0) continue;
      vecScale(this.vel, i, 0.998);
    }
  }

  // --------------------------------------------------------------------
  // rendering updates
  // --------------------------------------------------------------------
  updateMeshes() {
    this.triMesh.geometry.computeVertexNormals();
    this.triMesh.geometry.attributes.position.needsUpdate = true;
    this.triMesh.geometry.computeBoundingSphere();
    this.edgeMesh.geometry.attributes.position.needsUpdate = true;
  }

  endFrame() {
    this.updateMeshes();
  }

  // --------------------------------------------------------------------
  // grabbing interaction
  // --------------------------------------------------------------------
  startGrab(pos) {
    const p = [pos.x, pos.y, pos.z];
    let minDistSq = Number.MAX_VALUE;
    this.grabId = -1;

    for (let i = 0; i < this.numVertices; i++) {
      const d2 = vecDistSquared(p, 0, this.pos, i);
      if (d2 < minDistSq) {
        minDistSq = d2;
        this.grabId = i;
      }
    }

    if (this.grabId >= 0) {
      this.grabInvMass = this.invMass[this.grabId];
      this.invMass[this.grabId] = 0.0;
      vecCopy(this.pos, this.grabId, p, 0);
    }
  }

  moveGrabbed(pos, vel) {
    if (this.grabId >= 0) {
      const p = [pos.x, pos.y, pos.z];
      vecCopy(this.pos, this.grabId, p, 0);
      this.endFrame();
    }
  }

  endGrab(pos, vel) {
    if (this.grabId >= 0) {
      this.invMass[this.grabId] = this.grabInvMass;
      const v = [vel.x, vel.y, vel.z];
      vecCopy(this.vel, this.grabId, v, 0);
      this.endFrame();
    }
    this.grabId = -1;
  }
}
