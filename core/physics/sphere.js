import * as THREE from "three";
import { vecAdd, vecCopy, vecScale, vecDistSquared } from "../array.js";

export class Sphere {
  constructor({
    radius = 0.5,
    center = [0.0, 1.0, 0.0],
    collisionMode = "outside",
    scene,
    invMass = 1.0, // 0.0 にすると完全固定
    color = 0x00ff00,
  }) {
    this.radius = radius;
    this.collisionMode = collisionMode;

    // 1頂点だけの「中心パーティクル」
    this.pos = new Float32Array(3);
    this.prevPos = new Float32Array(3);
    this.vel = new Float32Array(3);
    this.invMass = new Float32Array(1);
    this.invMass[0] = invMass;

    this.pos[0] = center[0];
    this.pos[1] = center[1];
    this.pos[2] = center[2];
    this.prevPos.set(this.pos);

    // 掴み操作用
    this.grabbed = false;
    this.grabInvMass = 0.0;

    // ------------------------------------------------------------------
    // visual sphere mesh
    // ------------------------------------------------------------------
    const geometry = new THREE.SphereGeometry(radius, 32, 16);
    const material = new THREE.MeshPhongMaterial({
      color,
      side: THREE.FrontSide,
    });

    this.sphereMesh = new THREE.Mesh(geometry, material);
    this.sphereMesh.castShadow = true;
    this.sphereMesh.userData = this; // for raycasting (Grabberと同じパターン)
    this.sphereMesh.layers.enable(1);
    this.sphereMesh.position.set(center[0], center[1], center[2]);
    scene.add(this.sphereMesh);
  }

  // --------------------------------------------------------------------
  // simulation lifecycle (Cloth とほぼ同じ構成)
  // --------------------------------------------------------------------
  resetSimulation() {
    this.pos.set(this.prevPos);
    this.vel.fill(0.0);
  }

  endFrame() {
    // 中心位置から Three.js mesh を更新
    this.sphereMesh.position.set(this.pos[0], this.pos[1], this.pos[2]);
    this.sphereMesh.updateMatrixWorld();
  }

  // --------------------------------------------------------------------
  // grabbing interaction (Cloth と同じインターフェース)
  // --------------------------------------------------------------------
  startGrab(pos) {
    // pos: THREE.Vector3 想定
    const p = [this.pos[0], this.pos[1], pos.z];

    // 一応「近いときだけ掴む」判定を入れておく（任意）
    const d2 = vecDistSquared(p, 0, this.pos, 0);
    const r2 = this.radius * this.radius * 4.0; // 半径の2倍以内ならOKくらいの雑判定
    if (d2 > r2) {
      this.grabbed = false;
      return;
    }

    this.grabbed = true;
    this.grabInvMass = this.invMass[0];
    this.invMass[0] = 0.0; // 掴んでいる間は固定

    vecCopy(this.pos, 0, p, 0);
    this.endFrame();
  }

  moveGrabbed(pos, vel) {
    if (!this.grabbed) return;

    const p = [this.pos[0], this.pos[1], pos.z];
    vecCopy(this.pos, 0, p, 0);
    this.endFrame();
  }

  endGrab(pos, vel) {
    if (!this.grabbed) return;

    this.invMass[0] = this.grabInvMass;

    // release 時の初速
    const v = [0, 0, vel.z];
    vecCopy(this.vel, 0, v, 0);

    this.grabbed = false;
    this.endFrame();
  }
}
