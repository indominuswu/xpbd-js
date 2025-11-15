import { ConstraintBase } from "../base.js";

export class SphereVertexCollisionConstraints extends ConstraintBase {
  constructor(context) {
    super(context);

    this.invMass = context.invMass;
    this.collisionMode = context.collisionMode;
    this.vertexIds = new Int32Array(context.vertexIds);
    this._grad = new Float32Array(3); // 一時バッファ（必要なら使う）
  }

  solve(pos, dt, sphereCenter, sphereRadius) {
    const cx = sphereCenter[0];
    const cy = sphereCenter[1];
    const cz = sphereCenter[2];
    const R = sphereRadius;

    const invMass = this.invMass;
    const count = this.vertexIds.length;

    for (let i = 0; i < count; i++) {
      const vid = this.vertexIds[i];
      const w = invMass[vid];
      if (w === 0.0) continue; // 固定点は無視

      const px = pos[3 * vid + 0];
      const py = pos[3 * vid + 1];
      const pz = pos[3 * vid + 2];

      let dx = px - cx;
      let dy = py - cy;
      let dz = pz - cz;

      const lenSq = dx * dx + dy * dy + dz * dz;
      if (lenSq === 0.0) {
        // 球の中心と完全一致しているのはかなりレアなのでスキップでもよいし、
        // 必要なら適当な法線方向に小さく押し出す処理をここに入れてもOK
        continue;
      }

      const dist = Math.sqrt(lenSq);

      // --- モードに応じて「違反かどうか」を判定 ---
      if (this.collisionMode === "outside") {
        // 外側モード: dist >= R が許される状態
        // → dist < R なら「内側に侵入している」ので補正が必要
        if (dist >= R) continue;
      } else {
        // inside モード: dist <= R が許される状態
        // → dist > R なら「外に出ている」ので補正が必要
        if (dist <= R) continue;
      }

      // 拘束: C(x) = |x - c| - R
      // outside モードでは C <= 0 が許される領域
      // inside モードでは C >= 0 が禁止領域
      // どちらの場合も「dist が R を越えてはいけない」ことを表すので、
      // 違反の場合は同じ C = dist - R で OK
      const C = dist - R;

      // grad C = (x - c) / |x - c|
      const invLen = 1.0 / dist;
      dx *= invLen;
      dy *= invLen;
      dz *= invLen;

      const wSum = w; // 球は無限質量 (w_sphere = 0) とみなす
      const lambda = -C / wSum;

      const s = lambda * w;

      // Δx = -λ * w * ∇C と同じ（符号は lambda の定義に吸収してある）
      pos[3 * vid + 0] += s * dx;
      pos[3 * vid + 1] += s * dy;
      pos[3 * vid + 2] += s * dz;
    }
  }
}
