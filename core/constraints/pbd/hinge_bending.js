// constraints/bending-hinge.js
import { Vec3 } from "../../math.js";
import { ConstraintBase } from "../base.js";
import { computeRestBendingNormalAngles } from "../../geometry/triangle_utils.js";

export class HingeBendingConstraints extends ConstraintBase {
  constructor(context) {
    super(context);
    this.invMass = context.invMass;
    this.bendingVertexIds = new Int32Array(context.bendingVertexIds);
    this.stiffness = context.stiffness;
    this.restPos = context.restPos;
    this.restAngle = new Float32Array(this.bendingVertexIds.length / 4);
    this._buildRestAngles();
  }

  _buildRestAngles() {
    const eps = 1e-12;
    const numEdges = this.bendingVertexIds.length / 4;

    for (let i = 0; i < numEdges; i++) {
      const v0 = this.bendingVertexIds[4 * i + 0]; // p1
      const v1 = this.bendingVertexIds[4 * i + 1]; // p2
      const v2 = this.bendingVertexIds[4 * i + 2]; // p3
      const v3 = this.bendingVertexIds[4 * i + 3]; // p4

      const x1 = [
        this.restPos[3 * v0 + 0],
        this.restPos[3 * v0 + 1],
        this.restPos[3 * v0 + 2],
      ];
      const x2 = [
        this.restPos[3 * v1 + 0],
        this.restPos[3 * v1 + 1],
        this.restPos[3 * v1 + 2],
      ];
      const x3 = [
        this.restPos[3 * v2 + 0],
        this.restPos[3 * v2 + 1],
        this.restPos[3 * v2 + 2],
      ];
      const x4 = [
        this.restPos[3 * v3 + 0],
        this.restPos[3 * v3 + 1],
        this.restPos[3 * v3 + 2],
      ];

      // 相対系（p1 = 0）
      const p2 = Vec3.sub(x2, x1);
      const p3 = Vec3.sub(x3, x1);
      const p4 = Vec3.sub(x4, x1);

      const cross_p2p3 = Vec3.cross(p2, p3);
      const cross_p2p4 = Vec3.cross(p2, p4);
      const norm_cross_p2p3 = Vec3.norm(cross_p2p3);
      const norm_cross_p2p4 = Vec3.norm(cross_p2p4);

      const n1 = Vec3.mulScalar(
        cross_p2p3,
        1.0 / Math.max(norm_cross_p2p3, eps)
      );
      const n2 = Vec3.mulScalar(
        cross_p2p4,
        1.0 / Math.max(norm_cross_p2p4, eps)
      );

      const d = Vec3.dot(n1, n2);
      const clampedDot = Math.min(Math.max(d, -1.0), 1.0);
      const angle = Math.acos(clampedDot);
      this.restAngle[i] = angle;
    }
  }

  solve(pos, dt) {
    const eps = 1e-12;
    const numEdges = this.bendingVertexIds.length / 4;
    const k = this.stiffness;

    for (let i = 0; i < numEdges; i++) {
      const v0 = this.bendingVertexIds[4 * i + 0]; // p1
      const v1 = this.bendingVertexIds[4 * i + 1]; // p2
      const v2 = this.bendingVertexIds[4 * i + 2]; // p3
      const v3 = this.bendingVertexIds[4 * i + 3]; // p4

      const x1 = [pos[3 * v0 + 0], pos[3 * v0 + 1], pos[3 * v0 + 2]];
      const x2 = [pos[3 * v1 + 0], pos[3 * v1 + 1], pos[3 * v1 + 2]];
      const x3 = [pos[3 * v2 + 0], pos[3 * v2 + 1], pos[3 * v2 + 2]];
      const x4 = [pos[3 * v3 + 0], pos[3 * v3 + 1], pos[3 * v3 + 2]];

      // 相対系（p1 = 0）
      const p2 = Vec3.sub(x2, x1);
      const p3 = Vec3.sub(x3, x1);
      const p4 = Vec3.sub(x4, x1);

      const cross_p2p3 = Vec3.cross(p2, p3);
      const cross_p2p4 = Vec3.cross(p2, p4);
      const norm_cross_p2p3 = Vec3.norm(cross_p2p3);
      const norm_cross_p2p4 = Vec3.norm(cross_p2p4);

      const n1 = Vec3.mulScalar(
        cross_p2p3,
        1.0 / Math.max(norm_cross_p2p3, eps)
      );
      const n2 = Vec3.mulScalar(
        cross_p2p4,
        1.0 / Math.max(norm_cross_p2p4, eps)
      );

      const d = Vec3.dot(n1, n2);
      const clampedDot = Math.min(Math.max(d, -1.0), 1.0);
      const angle = Math.acos(clampedDot);

      // 勾配ベクトル q1..q4
      const q3 = Vec3.mulScalar(
        Vec3.add(Vec3.cross(p2, n2), Vec3.mulScalar(Vec3.cross(n1, p2), d)),
        1.0 / (norm_cross_p2p3 + eps)
      );
      const q4 = Vec3.mulScalar(
        Vec3.add(Vec3.cross(p2, n1), Vec3.mulScalar(Vec3.cross(n2, p2), d)),
        1.0 / (norm_cross_p2p4 + eps)
      );

      const t20 = Vec3.mulScalar(
        Vec3.add(Vec3.cross(p3, n2), Vec3.mulScalar(Vec3.cross(n1, p3), d)),
        1.0 / (norm_cross_p2p3 + eps)
      );
      const t21 = Vec3.mulScalar(
        Vec3.add(Vec3.cross(p4, n1), Vec3.mulScalar(Vec3.cross(n2, p4), d)),
        1.0 / (norm_cross_p2p4 + eps)
      );

      const q2 = Vec3.mulScalar(Vec3.add(t20, t21), -1.0);
      const q1 = Vec3.mulScalar(Vec3.add(Vec3.add(q2, q3), q4), -1.0);

      const w1 = this.invMass[v0];
      const w2 = this.invMass[v1];
      const w3 = this.invMass[v2];
      const w4 = this.invMass[v3];

      const norm_q1 = Vec3.norm(q1);
      const norm_q2 = Vec3.norm(q2);
      const norm_q3 = Vec3.norm(q3);
      const norm_q4 = Vec3.norm(q4);

      // XPBDの alpha を除去して純粋な PBD の denom に
      const denom =
        norm_q1 * norm_q1 * w1 +
        norm_q2 * norm_q2 * w2 +
        norm_q3 * norm_q3 * w3 +
        norm_q4 * norm_q4 * w4;

      const sinTerm = Math.sqrt(Math.max(0.0, 1.0 - clampedDot * clampedDot));
      const angleError = angle - this.restAngle[i];

      if (denom <= 1e-6) {
        continue;
      }

      // PBD: 硬さ k をここで掛ける
      // C = angle - restAngle, 係数 ~ k * C / denom
      const coeff = (k * angleError * sinTerm) / (denom + eps);

      const delta_p1 = Vec3.mulScalar(q1, -coeff * w1);
      const delta_p2 = Vec3.mulScalar(q2, -coeff * w2);
      const delta_p3 = Vec3.mulScalar(q3, -coeff * w3);
      const delta_p4 = Vec3.mulScalar(q4, -coeff * w4);

      pos[3 * v0 + 0] += delta_p1[0];
      pos[3 * v0 + 1] += delta_p1[1];
      pos[3 * v0 + 2] += delta_p1[2];

      pos[3 * v1 + 0] += delta_p2[0];
      pos[3 * v1 + 1] += delta_p2[1];
      pos[3 * v1 + 2] += delta_p2[2];

      pos[3 * v2 + 0] += delta_p3[0];
      pos[3 * v2 + 1] += delta_p3[1];
      pos[3 * v2 + 2] += delta_p3[2];

      pos[3 * v3 + 0] += delta_p4[0];
      pos[3 * v3 + 1] += delta_p4[1];
      pos[3 * v3 + 2] += delta_p4[2];
    }
  }
}
