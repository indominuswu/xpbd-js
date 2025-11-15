export class ConstraintBase {
  /**
   * @param {Object} params
   * @param {Float32Array} params.pos
   *    頂点数 N に対して長さ 3N の「位置配列」(x,y,z が連続)
   *
   * @param {Float32Array} params.invMass
   *    頂点数 N に対して長さ N の「逆質量配列」
   *
   * @param {number} [params.stiffness=1.0]
   *    PBD 用の硬さ (0〜1)。反復ごとの修正量をスケールする。
   */
  constructor(context) {
    this.stiffness = context.stiffness || 1.0;
    this.compliance = context.compliance || 0.0;
  }
  solve(pos, dt) {}
  setStiffness(stiffness) {
    this.stiffness = stiffness;
  }
  setCompliance(compliance) {
    this.compliance = compliance;
  }
}

export class ColliderConstraintBase extends ConstraintBase {
  solve(pos, dt, sphereCenter, sphereRadius, collisionMode = "outside") {}
}

export class ConstraintContext {
  /**
   * @param {Object} params
   *
   * @param {Float32Array} params.restPos
   *    頂点IDを3要素ずつ使って参照する「位置配列」 (例: pos[3 * vid + 0])
   *
   * @param {Int32Array|number[]} params.vertexIds
   *    頂点ID配列。各頂点は単一のインデックスで表される。
   *
   * @param {Float32Array} params.invMass
   *    頂点ごとの逆質量 (0 なら固定)
   *
   * @param {Int32Array|number[]} params.edgeVertexIds
   *    エッジのための ID 配列。各エッジは [i0, i1] の **2要素**で構成される。
   *    例: 長さ 2 * numEdges の配列。
   *
   * @param {Int32Array|number[]} params.triangleVertexIds
   *    三角形角度拘束用のトライアングル ID 配列。
   *
   * @param {Int32Array|number[]} params.bendingVertexIds
   *    曲げ拘束（ヒンジ/アイソメトリック等）用の quad ID 配列。
   *    各 quad は [v0, v1, v2, v3] の **4要素**で構成される。
   *    例: 長さ 4 * numBendingConstraints の配列。
   *
   * @param {number} [params.stiffness=1.0]
   *    PBD 用の剛性係数（すべての拘束のデフォルトに使える）
   *
   * @param {number} [params.compliance=0.1]
   *    XPBD 用（将来的に必要なら使う）。PBD では無視しても良い。
   */
  constructor({
    restPos,
    invMass,
    vertexIds,
    edgeVertexIds,
    triangleVertexIds,
    bendingVertexIds,
    stiffness = 1.0,
    compliance = 0.1,
    collisionMode = "outside",
  }) {
    this.restPos = restPos;
    this.invMass = invMass;

    // 共通: 頂点ID配列
    this.vertexIds = vertexIds;

    // エッジ（stretching）用: [v0, v1, v0, v1, ...]
    this.edgeVertexIds = edgeVertexIds;

    // 三角形角度拘束用: [v0, v1, v2,  v0, v1, v2, ...]
    this.triangleVertexIds = triangleVertexIds;

    // 曲げ拘束用: [v0, v1, v2, v3,  v0, v1, v2, v3, ...]
    this.bendingVertexIds = bendingVertexIds;

    this.stiffness = stiffness;
    this.compliance = compliance; // XPBD 用
    this.collisionMode = collisionMode;
  }
}
