import {
  vecSetDiff,
  vecLengthSquared,
  vecDistSquared,
  vecAdd,
  vecScale,
} from "../../array.js";
import { ConstraintBase } from "../base.js";

export class DiagonalBendingConstraints extends ConstraintBase {
  constructor(context) {
    super(context);
    this.restPos = context.restPos;
    this.invMass = context.invMass;
    this.bendingVertexIds = new Int32Array(context.bendingVertexIds);

    // PBD用パラメータ: 0〜1 を想定
    this.stiffness = context.stiffness;

    const quadCount = this.bendingVertexIds.length / 4;
    this.restLengths = new Float32Array(quadCount);

    this._grad = new Float32Array(3);

    this._buildRestLengths();
  }

  _buildRestLengths() {
    const quadCount = this.bendingVertexIds.length / 4;
    for (let i = 0; i < quadCount; i++) {
      const v2 = this.bendingVertexIds[4 * i + 2];
      const v3 = this.bendingVertexIds[4 * i + 3];
      this.restLengths[i] = Math.sqrt(
        vecDistSquared(this.restPos, v2, this.restPos, v3)
      );
    }
  }

  solve(pos, dt) {
    const quadCount = this.bendingVertexIds.length / 4;
    const k = this.stiffness;

    for (let i = 0; i < quadCount; i++) {
      const v2 = this.bendingVertexIds[4 * i + 2];
      const v3 = this.bendingVertexIds[4 * i + 3];

      const w0 = this.invMass[v2];
      const w1 = this.invMass[v3];
      const wSum = w0 + w1;
      if (wSum === 0.0) continue;

      // grad = (x0 - x1) / ||x0 - x1||
      vecSetDiff(this._grad, 0, pos, v2, pos, v3);
      const lenSq = vecLengthSquared(this._grad, 0);
      if (lenSq === 0.0) continue;

      const len = Math.sqrt(lenSq);
      vecScale(this._grad, 0, 1.0 / len);

      const restLen = this.restLengths[i];
      const C = len - restLen; // 斜め距離拘束: |x0 - x1| - L0 = 0

      // PBD: lambda に stiffness を掛けるだけ
      const lambda = (-k * C) / wSum;

      vecAdd(pos, v2, this._grad, 0, lambda * w0);
      vecAdd(pos, v3, this._grad, 0, -lambda * w1);
    }
  }
}
