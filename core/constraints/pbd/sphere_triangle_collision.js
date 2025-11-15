import { ConstraintBase } from "../base.js";

// 点 p と三角形 a,b,c の最近接点を返す
// 戻り値: { qx,qy,qz, u,v,w }  (u,v,w は a,b,c へのバリセントリック座標, u+v+w=1)
function closestPointOnTriangle(
  px,
  py,
  pz,
  ax,
  ay,
  az,
  bx,
  by,
  bz,
  cx,
  cy,
  cz
) {
  const abx = bx - ax;
  const aby = by - ay;
  const abz = bz - az;

  const acx = cx - ax;
  const acy = cy - ay;
  const acz = cz - az;

  const apx = px - ax;
  const apy = py - ay;
  const apz = pz - az;

  // 頂点 A 周りの領域チェック
  let d1 = abx * apx + aby * apy + abz * apz;
  let d2 = acx * apx + acy * apy + acz * apz;
  if (d1 <= 0.0 && d2 <= 0.0) {
    // 最近接点は A
    return { qx: ax, qy: ay, qz: az, u: 1.0, v: 0.0, w: 0.0 };
  }

  // 頂点 B 周り
  const bpx = px - bx;
  const bpy = py - by;
  const bpz = pz - bz;

  let d3 = abx * bpx + aby * bpy + abz * bpz;
  let d4 = acx * bpx + acy * bpy + acz * bpz;
  if (d3 >= 0.0 && d4 <= d3) {
    // 最近接点は B
    return { qx: bx, qy: by, qz: bz, u: 0.0, v: 1.0, w: 0.0 };
  }

  // 辺 AB 上
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    const v = d1 / (d1 - d3);
    const u = 1.0 - v;
    const qx = ax + v * abx;
    const qy = ay + v * aby;
    const qz = az + v * abz;
    return { qx, qy, qz, u, v, w: 0.0 };
  }

  // 頂点 C 周り
  const cpx = px - cx;
  const cpy = py - cy;
  const cpz = pz - cz;

  let d5 = abx * cpx + aby * cpy + abz * cpz;
  let d6 = acx * cpx + acy * cpy + acz * cpz;
  if (d6 >= 0.0 && d5 <= d6) {
    // 最近接点は C
    return { qx: cx, qy: cy, qz: cz, u: 0.0, v: 0.0, w: 1.0 };
  }

  // 辺 AC 上
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    const w = d2 / (d2 - d6);
    const u = 1.0 - w;
    const qx = ax + w * acx;
    const qy = ay + w * acy;
    const qz = az + w * acz;
    return { qx, qy, qz, u, v: 0.0, w };
  }

  // 辺 BC 上
  const va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0) {
    const w = (d4 - d3) / (d4 - d3 + (d5 - d6));
    const v = 1.0 - w;
    const qx = bx + w * (cx - bx);
    const qy = by + w * (cy - by);
    const qz = bz + w * (cz - bz);
    return { qx, qy, qz, u: 0.0, v, w };
  }

  // 面内部
  const denom = 1.0 / (va + vb + vc);
  const v = vb * denom;
  const w = vc * denom;
  const u = 1.0 - v - w;

  const qx = ax * u + bx * v + cx * w;
  const qy = ay * u + by * v + cy * w;
  const qz = az * u + bz * v + cz * w;

  return { qx, qy, qz, u, v, w };
}

export class SphereTriangleCollisionConstraints extends ConstraintBase {
  constructor(context) {
    super(context);

    this.invMass = context.invMass;
    this.triangleVertexIds = new Int32Array(context.triangleVertexIds); // 3つで1三角形
    this.collisionMode = context.collisionMode || "outside"; // "outside" or "inside"
  }

  solve(pos, dt, sphereCenter, sphereRadius) {
    const cx = sphereCenter[0];
    const cy = sphereCenter[1];
    const cz = sphereCenter[2];
    const R = sphereRadius;

    const invMass = this.invMass;
    const triangleVertexIds = this.triangleVertexIds;
    const triCount = triangleVertexIds.length / 3;

    for (let t = 0; t < triCount; t++) {
      const i0 = triangleVertexIds[3 * t + 0];
      const i1 = triangleVertexIds[3 * t + 1];
      const i2 = triangleVertexIds[3 * t + 2];

      const w0 = invMass[i0];
      const w1 = invMass[i1];
      const w2 = invMass[i2];

      // 全部固定されている三角形なら衝突しても動かせないので無視
      const wSumInv = w0 + w1 + w2;
      if (wSumInv === 0.0) {
        continue;
      }

      const ax = pos[3 * i0 + 0];
      const ay = pos[3 * i0 + 1];
      const az = pos[3 * i0 + 2];

      const bx = pos[3 * i1 + 0];
      const by = pos[3 * i1 + 1];
      const bz = pos[3 * i1 + 2];

      const cxv = pos[3 * i2 + 0];
      const cyv = pos[3 * i2 + 1];
      const czv = pos[3 * i2 + 2];

      // 球中心から見た三角形の最近接点 q
      const { qx, qy, qz, u, v, w } = closestPointOnTriangle(
        cx,
        cy,
        cz,
        ax,
        ay,
        az,
        bx,
        by,
        bz,
        cxv,
        cyv,
        czv
      );

      let dx = qx - cx;
      let dy = qy - cy;
      let dz = qz - cz;

      const lenSq = dx * dx + dy * dy + dz * dz;
      if (lenSq === 0.0) {
        // 球中心が三角形上に乗っているなどの非常にレアなケース
        // 必要ならここで適当な法線を使って押し出してもよい
        continue;
      }

      const dist = Math.sqrt(lenSq);

      // モードに応じて違反か判定
      if (this.collisionMode === "outside") {
        // 布は球の外側にいたい: dist >= R が許容
        // dist < R なら球にめり込んでいる
        if (dist >= R) continue;
      } else {
        // inside モード: 球の内側に閉じ込めたい (用途があれば)
        // dist <= R が許容、dist > R で外へ出ている
        if (dist <= R) continue;
      }

      // C = dist - R  (dist < R で負 → 侵入)
      const C = dist - R;

      // 法線方向
      const invLen = 1.0 / dist;
      dx *= invLen;
      dy *= invLen;
      dz *= invLen;

      // バリセントリックで「接触点に効く重み」を持たせる
      // ざっくり PBD: wEff = Σ (wi * alpha_i^2)
      const wEff = w0 * u * u + w1 * v * v + w2 * w * w;

      if (wEff === 0.0) continue;

      const lambda = -C / wEff;

      // 各頂点の修正量: Δxi = -λ * wi * αi * n
      const scale0 = lambda * w0 * u;
      const scale1 = lambda * w1 * v;
      const scale2 = lambda * w2 * w;

      pos[3 * i0 + 0] += scale0 * dx;
      pos[3 * i0 + 1] += scale0 * dy;
      pos[3 * i0 + 2] += scale0 * dz;

      pos[3 * i1 + 0] += scale1 * dx;
      pos[3 * i1 + 1] += scale1 * dy;
      pos[3 * i1 + 2] += scale1 * dz;

      pos[3 * i2 + 0] += scale2 * dx;
      pos[3 * i2 + 1] += scale2 * dy;
      pos[3 * i2 + 2] += scale2 * dz;
    }
  }
}
