import {
  vecSetDiff,
  vecLengthSquared,
  vecDistSquared,
  vecAdd,
  vecScale,
} from "../../array.js";
import { ConstraintBase } from "../base.js";

export class EdgeStretchingConstraints extends ConstraintBase {
  constructor(context) {
    super(context);
    this.restPos = context.restPos;
    this.invMass = context.invMass;
    this.edgeVertexIds = new Int32Array(context.edgeVertexIds);

    // PBD 用パラメータ：0〜1 を想定
    // 1.0 = 完全に距離を満たす, 0.0 = 何もしない
    this.stiffness = context.stiffness;

    // edge ごとの rest length
    this.restLengths = new Float32Array(this.edgeVertexIds.length / 2);
    this._buildRestLengths();
    this._grad = new Float32Array(3); // 一時バッファ
  }

  _buildRestLengths() {
    for (let i = 0; i < this.restLengths.length; i++) {
      const v0 = this.edgeVertexIds[2 * i + 0];
      const v1 = this.edgeVertexIds[2 * i + 1];
      this.restLengths[i] = Math.sqrt(
        vecDistSquared(this.restPos, v0, this.restPos, v1)
      );
    }
  }

  solve(pos, dt) {
    const k = this.stiffness;

    for (let i = 0; i < this.restLengths.length; i++) {
      const v0 = this.edgeVertexIds[2 * i + 0];
      const v1 = this.edgeVertexIds[2 * i + 1];

      const w0 = this.invMass[v0];
      const w1 = this.invMass[v1];
      const wSum = w0 + w1;
      if (wSum === 0.0) continue;

      // grad = (x0 - x1) / ||x0 - x1||
      vecSetDiff(this._grad, 0, pos, v0, pos, v1);
      const lenSq = vecLengthSquared(this._grad, 0);
      if (lenSq === 0.0) continue;

      const len = Math.sqrt(lenSq);
      vecScale(this._grad, 0, 1.0 / len); // 正規化

      const restLen = this.restLengths[i];
      const C = len - restLen; // 距離拘束: |x0 - x1| - L0 = 0

      // PBD のラグランジュ乗数（1回のソルバステップでの補正量）
      // 硬さ調整は C に対して単純に stiffness を掛ける形に
      const lambda = (-k * C) / wSum;

      // Δx0 = -λ * w0 * ∇C, Δx1 = -λ * w1 * ∇C
      vecAdd(pos, v0, this._grad, 0, lambda * w0);
      vecAdd(pos, v1, this._grad, 0, -lambda * w1);
    }
  }
}
