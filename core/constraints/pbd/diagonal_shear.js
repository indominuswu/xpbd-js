import {
  vecSetDiff,
  vecLengthSquared,
  vecDistSquared,
  vecAdd,
  vecScale,
} from "../../array.js";
import { ConstraintBase } from "../base.js";

export class DiagonalShearConstraints extends ConstraintBase {
  constructor(context) {
    super(context);
    this.restPos = context.restPos;
    this.invMass = context.invMass;
    this.bendingVertexIds = new Int32Array(context.bendingVertexIds);

    // PBD用パラメータ: 0〜1 を想定
    this.stiffness = context.stiffness;

    const quadCount = this.bendingVertexIds.length / 4;
    // クアッド1つにつき対角線2本
    this.restLengths = new Float32Array(quadCount * 2);

    this._grad = new Float32Array(3);

    this._buildRestLengths();
  }

  _buildRestLengths() {
    const quadCount = this.bendingVertexIds.length / 4;

    for (let i = 0; i < quadCount; i++) {
      const v0 = this.bendingVertexIds[4 * i + 0];
      const v1 = this.bendingVertexIds[4 * i + 1];
      const v2 = this.bendingVertexIds[4 * i + 2];
      const v3 = this.bendingVertexIds[4 * i + 3];

      // 対角線1: v0 - v2
      this.restLengths[2 * i + 0] = Math.sqrt(
        vecDistSquared(this.restPos, v0, this.restPos, v2)
      );
      // 対角線2: v1 - v3
      this.restLengths[2 * i + 1] = Math.sqrt(
        vecDistSquared(this.restPos, v1, this.restPos, v3)
      );
    }
  }

  // 個々の対角線 (i0, i1) に対する1本分のPBD距離拘束
  _solveOneDiagonal(pos, i0, i1, restLen, stiffness) {
    const w0 = this.invMass[i0];
    const w1 = this.invMass[i1];
    const wSum = w0 + w1;
    if (wSum === 0.0) {
      return;
    }

    // grad = (x0 - x1) / ||x0 - x1||
    vecSetDiff(this._grad, 0, pos, i0, pos, i1);
    const lenSq = vecLengthSquared(this._grad, 0);
    if (lenSq === 0.0) {
      return;
    }

    const len = Math.sqrt(lenSq);
    vecScale(this._grad, 0, 1.0 / len);

    const C = len - restLen; // 距離拘束: |x0 - x1| - L0 = 0

    // PBD: lambda に stiffness を掛けるだけ
    const lambda = (-stiffness * C) / wSum;

    vecAdd(pos, i0, this._grad, 0, lambda * w0);
    vecAdd(pos, i1, this._grad, 0, -lambda * w1);
  }

  solve(pos, dt) {
    const quadCount = this.bendingVertexIds.length / 4;
    const k = this.stiffness;

    for (let i = 0; i < quadCount; i++) {
      const v0 = this.bendingVertexIds[4 * i + 0];
      const v1 = this.bendingVertexIds[4 * i + 1];
      const v2 = this.bendingVertexIds[4 * i + 2];
      const v3 = this.bendingVertexIds[4 * i + 3];

      // 対角線1: v0 - v2
      this._solveOneDiagonal(pos, v0, v2, this.restLengths[2 * i + 0], k);

      // 対角線2: v1 - v3
      this._solveOneDiagonal(pos, v1, v3, this.restLengths[2 * i + 1], k);
    }
  }
}
