// constraints/bending-isometric.js
import { triangleArea, cotAtVertex } from "../../geometry/triangle_utils.js";
import { ConstraintBase } from "../base.js";

export class IsometricBendingConstraints extends ConstraintBase {
  constructor(context) {
    super(context);
    this.restPos = context.restPos;
    this.invMass = context.invMass;
    this.bendingVertexIds = new Int32Array(context.bendingVertexIds);
    this.stiffness = context.stiffness;
    this.bendingCoeff = [];
    this._buildBendingCoeff();
  }

  _buildBendingCoeff() {
    this.bendingCoeff = [];

    for (let i = 0; i < this.bendingVertexIds.length / 4; i++) {
      const v0 = this.bendingVertexIds[4 * i + 0]; // shared edge vertex 0
      const v1 = this.bendingVertexIds[4 * i + 1]; // shared edge vertex 1
      const v2 = this.bendingVertexIds[4 * i + 2]; // opposite vertex (tri 0)
      const v3 = this.bendingVertexIds[4 * i + 3]; // opposite vertex (tri 1)

      const area0 = triangleArea(this.restPos, v1, v0, v2);
      const area1 = triangleArea(this.restPos, v1, v0, v3);
      const totalArea = Math.max(1e-15, area0 + area1);

      const cot01 = cotAtVertex(this.restPos, v0, v1, v2);
      const cot02 = cotAtVertex(this.restPos, v0, v1, v3);
      const cot03 = cotAtVertex(this.restPos, v1, v0, v2);
      const cot04 = cotAtVertex(this.restPos, v1, v0, v3);

      const c0 = cot03 + cot04;
      const c1 = cot01 + cot02;
      const c2 = -cot01 - cot03;
      const c3 = -cot02 - cot04;

      this.bendingCoeff.push({
        c: [c0, c1, c2, c3],
        scale: Math.sqrt(3 / (2 * totalArea)), // C, gradC 用
      });
    }
  }

  solve(pos, dt) {
    const eps = 1e-12;
    const k = this.stiffness;

    for (let i = 0; i < this.bendingVertexIds.length / 4; i++) {
      const v0 = this.bendingVertexIds[4 * i + 0];
      const v1 = this.bendingVertexIds[4 * i + 1];
      const v2 = this.bendingVertexIds[4 * i + 2];
      const v3 = this.bendingVertexIds[4 * i + 3];

      const { c, scale } = this.bendingCoeff[i];

      // S = Σ c_p x_p
      const Sx =
        c[0] * pos[3 * v0 + 0] +
        c[1] * pos[3 * v1 + 0] +
        c[2] * pos[3 * v2 + 0] +
        c[3] * pos[3 * v3 + 0];
      const Sy =
        c[0] * pos[3 * v0 + 1] +
        c[1] * pos[3 * v1 + 1] +
        c[2] * pos[3 * v2 + 1] +
        c[3] * pos[3 * v3 + 1];
      const Sz =
        c[0] * pos[3 * v0 + 2] +
        c[1] * pos[3 * v1 + 2] +
        c[2] * pos[3 * v2 + 2] +
        c[3] * pos[3 * v3 + 2];

      const Slen = Math.hypot(Sx, Sy, Sz);
      if (Slen < eps) continue;

      // C = scale * ||S||
      const C = scale * Slen;

      // gradC_i = scale * c_i * (S / ||S||)
      const nx = Sx / Slen;
      const ny = Sy / Slen;
      const nz = Sz / Slen;

      const g0x = scale * c[0] * nx;
      const g0y = scale * c[0] * ny;
      const g0z = scale * c[0] * nz;
      const g1x = scale * c[1] * nx;
      const g1y = scale * c[1] * ny;
      const g1z = scale * c[1] * nz;
      const g2x = scale * c[2] * nx;
      const g2y = scale * c[2] * ny;
      const g2z = scale * c[2] * nz;
      const g3x = scale * c[3] * nx;
      const g3y = scale * c[3] * ny;
      const g3z = scale * c[3] * nz;

      const w0 = this.invMass[v0];
      const w1 = this.invMass[v1];
      const w2 = this.invMass[v2];
      const w3 = this.invMass[v3];

      // XPBD では + alpha があったが PBD は純粋に wEff のみ
      const wEff =
        w0 * (scale * c[0]) * (scale * c[0]) +
        w1 * (scale * c[1]) * (scale * c[1]) +
        w2 * (scale * c[2]) * (scale * c[2]) +
        w3 * (scale * c[3]) * (scale * c[3]);

      if (wEff < eps) continue;

      // ★ PBD: stiffness を掛ける
      const lambda = (-k * C) / wEff;

      // 位置更新
      pos[3 * v0 + 0] += lambda * w0 * g0x;
      pos[3 * v0 + 1] += lambda * w0 * g0y;
      pos[3 * v0 + 2] += lambda * w0 * g0z;

      pos[3 * v1 + 0] += lambda * w1 * g1x;
      pos[3 * v1 + 1] += lambda * w1 * g1y;
      pos[3 * v1 + 2] += lambda * w1 * g1z;

      pos[3 * v2 + 0] += lambda * w2 * g2x;
      pos[3 * v2 + 1] += lambda * w2 * g2y;
      pos[3 * v2 + 2] += lambda * w2 * g2z;

      pos[3 * v3 + 0] += lambda * w3 * g3x;
      pos[3 * v3 + 1] += lambda * w3 * g3y;
      pos[3 * v3 + 2] += lambda * w3 * g3z;
    }
  }
}
