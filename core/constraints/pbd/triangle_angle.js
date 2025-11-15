import { ConstraintBase } from "../base.js";

export class TriangleAngleConstraints extends ConstraintBase {
  constructor(context) {
    super(context);
    this.restPos = context.restPos;
    this.invMass = context.invMass;
    this.triangleVertexIds = new Int32Array(context.triangleVertexIds);
    this.stiffness = context.stiffness ?? 1.0;

    const numTris = this.triangleVertexIds.length / 3;

    // 各三角形の 3 内角ぶん cosθ を保存
    this.restCosAngles = new Float32Array(numTris * 3);
    this._buildRestAngles();

    // 一時バッファ
    this._u = new Float32Array(3);
    this._v = new Float32Array(3);
  }

  _buildRestAngles() {
    const numTris = this.triangleVertexIds.length / 3;

    for (let t = 0; t < numTris; t++) {
      const i0 = this.triangleVertexIds[3 * t + 0];
      const i1 = this.triangleVertexIds[3 * t + 1];
      const i2 = this.triangleVertexIds[3 * t + 2];

      // 3 つの頂点それぞれの内角 cosθ_rest を計算
      this.restCosAngles[3 * t + 0] = this._computeRestCosAngle(
        i0,
        i1,
        i2,
        this.restPos
      );
      this.restCosAngles[3 * t + 1] = this._computeRestCosAngle(
        i1,
        i2,
        i0,
        this.restPos
      );
      this.restCosAngles[3 * t + 2] = this._computeRestCosAngle(
        i2,
        i0,
        i1,
        this.restPos
      );
    }
  }

  _computeRestCosAngle(i, j, k, pos) {
    const ix = pos[3 * i + 0],
      iy = pos[3 * i + 1],
      iz = pos[3 * i + 2];
    const jx = pos[3 * j + 0],
      jy = pos[3 * j + 1],
      jz = pos[3 * j + 2];
    const kx = pos[3 * k + 0],
      ky = pos[3 * k + 1],
      kz = pos[3 * k + 2];

    const ux = jx - ix,
      uy = jy - iy,
      uz = jz - iz;
    const vx = kx - ix,
      vy = ky - iy,
      vz = kz - iz;

    const dot = ux * vx + uy * vy + uz * vz;
    const lenU = Math.hypot(ux, uy, uz);
    const lenV = Math.hypot(vx, vy, vz);

    if (lenU === 0.0 || lenV === 0.0) return 1.0; // 退避

    return dot / (lenU * lenV);
  }

  solve(pos, dt) {
    const k = this.stiffness;
    const numTris = this.triangleVertexIds.length / 3;
    const eps = 1e-8;

    for (let t = 0; t < numTris; t++) {
      const i0 = this.triangleVertexIds[3 * t + 0];
      const i1 = this.triangleVertexIds[3 * t + 1];
      const i2 = this.triangleVertexIds[3 * t + 2];

      // 三角形の 3 内角についてそれぞれ拘束を解く
      this._solveOneAngleConstraint(
        pos,
        i0,
        i1,
        i2,
        this.restCosAngles[3 * t + 0],
        k,
        eps
      );
      this._solveOneAngleConstraint(
        pos,
        i1,
        i2,
        i0,
        this.restCosAngles[3 * t + 1],
        k,
        eps
      );
      this._solveOneAngleConstraint(
        pos,
        i2,
        i0,
        i1,
        this.restCosAngles[3 * t + 2],
        k,
        eps
      );
    }
  }

  _solveOneAngleConstraint(pos, i, j, k, restCos, stiffness, eps) {
    const w0 = this.invMass[i];
    const w1 = this.invMass[j];
    const w2 = this.invMass[k];
    const wSum = w0 + w1 + w2;
    if (wSum === 0.0) {
      return;
    }

    const ix = pos[3 * i + 0],
      iy = pos[3 * i + 1],
      iz = pos[3 * i + 2];
    const jx = pos[3 * j + 0],
      jy = pos[3 * j + 1],
      jz = pos[3 * j + 2];
    const kx = pos[3 * k + 0],
      ky = pos[3 * k + 1],
      kz = pos[3 * k + 2];

    // u = xj - xi, v = xk - xi
    const ux = jx - ix,
      uy = jy - iy,
      uz = jz - iz;
    const vx = kx - ix,
      vy = ky - iy,
      vz = kz - iz;

    const lenU = Math.hypot(ux, uy, uz);
    const lenV = Math.hypot(vx, vy, vz);
    if (lenU < eps || lenV < eps) {
      return;
    }

    const dot = ux * vx + uy * vy + uz * vz;
    const invLenU = 1.0 / lenU;
    const invLenV = 1.0 / lenV;

    const cosTheta = dot * invLenU * invLenV;
    const C = cosTheta - restCos;

    // 変化が小さければスキップ
    if (Math.abs(C) < 1e-6) {
      return;
    }

    // ∂c/∂u, ∂c/∂v を計算
    const invLenU3 = invLenU * invLenU * invLenU;
    const invLenV3 = invLenV * invLenV * invLenV;

    // dC/du = v/(|u||v|) - (dot u)/( |u|^3 |v| )
    const factorUV = invLenU * invLenV;
    const factorU3V = dot * invLenU3 * invLenV;
    const dCdu_x = vx * factorUV - ux * factorU3V;
    const dCdu_y = vy * factorUV - uy * factorU3V;
    const dCdu_z = vz * factorUV - uz * factorU3V;

    // dC/dv = u/(|u||v|) - (dot v)/( |u| |v|^3 )
    const factorUV3 = dot * invLenU * invLenV3;
    const dCdv_x = ux * factorUV - vx * factorUV3;
    const dCdv_y = uy * factorUV - vy * factorUV3;
    const dCdv_z = uz * factorUV - vz * factorUV3;

    // ∇C_i, ∇C_j, ∇C_k
    const grad_jx = dCdu_x;
    const grad_jy = dCdu_y;
    const grad_jz = dCdu_z;

    const grad_kx = dCdv_x;
    const grad_ky = dCdv_y;
    const grad_kz = dCdv_z;

    const grad_ix = -grad_jx - grad_kx;
    const grad_iy = -grad_jy - grad_ky;
    const grad_iz = -grad_jz - grad_kz;

    const g0Len2 = grad_ix * grad_ix + grad_iy * grad_iy + grad_iz * grad_iz;
    const g1Len2 = grad_jx * grad_jx + grad_jy * grad_jy + grad_jz * grad_jz;
    const g2Len2 = grad_kx * grad_kx + grad_ky * grad_ky + grad_kz * grad_kz;

    const denom = Math.max(1e-3, w0 * g0Len2 + w1 * g1Len2 + w2 * g2Len2);
    if (denom < eps) {
      return;
    }

    const lambda = (-stiffness * C) / denom;

    // Δx = -λ w ∇C
    pos[3 * i + 0] += lambda * w0 * grad_ix;
    pos[3 * i + 1] += lambda * w0 * grad_iy;
    pos[3 * i + 2] += lambda * w0 * grad_iz;

    pos[3 * j + 0] += lambda * w1 * grad_jx;
    pos[3 * j + 1] += lambda * w1 * grad_jy;
    pos[3 * j + 2] += lambda * w1 * grad_jz;

    pos[3 * k + 0] += lambda * w2 * grad_kx;
    pos[3 * k + 1] += lambda * w2 * grad_ky;
    pos[3 * k + 2] += lambda * w2 * grad_kz;
  }
}
