export function findTriangleNeighbors(triIds) {
  var edges = [];
  var numTris = triIds.length / 3;

  for (var tri = 0; tri < numTris; tri++) {
    for (var k = 0; k < 3; k++) {
      var v0 = triIds[3 * tri + k];
      var v1 = triIds[3 * tri + ((k + 1) % 3)];
      edges.push({
        vertexId0: Math.min(v0, v1),
        vertexId1: Math.max(v0, v1),
        edgeId: 3 * tri + k,
      });
    }
  }

  // sort so common edges are next to each other
  edges.sort((a, b) =>
    a.vertexId0 < b.vertexId0 ||
    (a.vertexId0 == b.vertexId0 && a.vertexId1 < b.vertexId1)
      ? -1
      : 1
  );

  const neighbors = new Float32Array(3 * numTris);
  neighbors.fill(-1); // open edge

  var nr = 0;
  while (nr < edges.length) {
    var e0 = edges[nr];
    nr++;
    if (nr < edges.length) {
      var e1 = edges[nr];
      if (e0.vertexId0 == e1.vertexId0 && e0.vertexId1 == e1.vertexId1) {
        neighbors[e0.edgeId] = e1.edgeId;
        neighbors[e1.edgeId] = e0.edgeId;
        nr++;
      }
    }
  }

  return neighbors;
}

export function computeRestBendingNormalAngles(pos, bendingVertexIds) {
  const numEdges = bendingVertexIds.length / 4;
  const restAngles = new Float32Array(numEdges);

  const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));

  for (let i = 0; i < numEdges; i++) {
    const v0 = bendingVertexIds[4 * i + 0]; // shared edge vertex 0
    const v1 = bendingVertexIds[4 * i + 1]; // shared edge vertex 1
    const v2 = bendingVertexIds[4 * i + 2]; // opposite vertex of tri 1
    const v3 = bendingVertexIds[4 * i + 3]; // opposite vertex of tri 2

    const x0 = pos[3 * v0],
      y0 = pos[3 * v0 + 1],
      z0 = pos[3 * v0 + 2];
    const x1 = pos[3 * v1],
      y1 = pos[3 * v1 + 1],
      z1 = pos[3 * v1 + 2];
    const x2 = pos[3 * v2],
      y2 = pos[3 * v2 + 1],
      z2 = pos[3 * v2 + 2];
    const x3 = pos[3 * v3],
      y3 = pos[3 * v3 + 1],
      z3 = pos[3 * v3 + 2];

    // 共有エッジ e = x1 - x0
    const ex = x1 - x0;
    const ey = y1 - y0;
    const ez = z1 - z0;

    // 2つの三角形の法線
    // 三角形1: (v2, v0, v1)
    const n1 = triangleNormal(x2, y2, z2, x0, y0, z0, x1, y1, z1);
    // 三角形2: (v3, v1, v0)
    const n2 = triangleNormal(x3, y3, z3, x1, y1, z1, x0, y0, z0);

    const n1x = n1[0],
      n1y = n1[1],
      n1z = n1[2];
    const n2x = n2[0],
      n2y = n2[1],
      n2z = n2[2];

    // cosθ
    let cosTheta = n1x * n2x + n1y * n2y + n1z * n2z;
    cosTheta = clamp(cosTheta, -1.0, 1.0);

    // 符号付きにするための三重積 sign = sign( e · (n1 × n2) )
    const mx = n1y * n2z - n1z * n2y;
    const my = n1z * n2x - n1x * n2z;
    const mz = n1x * n2y - n1y * n2x;
    const triple = ex * mx + ey * my + ez * mz;

    const sign = triple >= 0.0 ? 1.0 : -1.0;
    const angle = sign * Math.acos(cosTheta);

    // rest angle
    restAngles[i] = angle;
  }

  return restAngles;
}

export function triangleNormal(ax, ay, az, bx, by, bz, cx, cy, cz) {
  const ux = bx - ax,
    uy = by - ay,
    uz = bz - az;
  const vx = cx - ax,
    vy = cy - ay,
    vz = cz - az;
  let nx = uy * vz - uz * vy;
  let ny = uz * vx - ux * vz;
  let nz = ux * vy - uy * vx;
  const len = Math.hypot(nx, ny, nz) || 1.0;
  return [nx / len, ny / len, nz / len];
}

export function triangleArea(pos, ia, ib, ic) {
  const ax = pos[3 * ia],
    ay = pos[3 * ia + 1],
    az = pos[3 * ia + 2];
  const bx = pos[3 * ib],
    by = pos[3 * ib + 1],
    bz = pos[3 * ib + 2];
  const cx = pos[3 * ic],
    cy = pos[3 * ic + 1],
    cz = pos[3 * ic + 2];
  const ux = bx - ax,
    uy = by - ay,
    uz = bz - az;
  const vx = cx - ax,
    vy = cy - ay,
    vz = cz - az;
  const cx0 = uy * vz - uz * vy;
  const cx1 = uz * vx - ux * vz;
  const cx2 = ux * vy - uy * vx;
  return 0.5 * Math.hypot(cx0, cx1, cx2);
}

export function cotAtVertex(pos, ia, ib, ic) {
  // 各頂点の座標を取得
  const ax = pos[3 * ia],
    ay = pos[3 * ia + 1],
    az = pos[3 * ia + 2];
  const bx = pos[3 * ib],
    by = pos[3 * ib + 1],
    bz = pos[3 * ib + 2];
  const cx = pos[3 * ic],
    cy = pos[3 * ic + 1],
    cz = pos[3 * ic + 2];

  // ベクトル u = B - A, v = C - A
  const ux = bx - ax,
    uy = by - ay,
    uz = bz - az;
  const vx = cx - ax,
    vy = cy - ay,
    vz = cz - az;

  // 分子：内積 u · v
  const dot = ux * vx + uy * vy + uz * vz;

  // 分母：外積のノルム ||u × v||
  const cx0 = uy * vz - uz * vy;
  const cx1 = uz * vx - ux * vz;
  const cx2 = ux * vy - uy * vx;
  const den = Math.hypot(cx0, cx1, cx2); // = ||u × v||

  if (den < 1e-12) return 0.0; // 退化対策（面積0のとき）

  return dot / den;
}

export function getEdgeVertexIndices(faceTriIds) {
  const triangleCount = faceTriIds.length / 3;
  const neighborEdgeIndices = findTriangleNeighbors(faceTriIds);

  const edgeVertexIndices = [];

  for (let triId = 0; triId < triangleCount; triId++) {
    for (let k = 0; k < 3; k++) {
      const v0 = faceTriIds[3 * triId + k];
      const v1 = faceTriIds[3 * triId + ((k + 1) % 3)];

      const edgeIndex = 3 * triId + k;
      const neighborEdgeIndex = neighborEdgeIndices[edgeIndex];

      if (neighborEdgeIndex < 0 || v0 < v1) {
        edgeVertexIndices.push(v0, v1);
      }
    }
  }

  return edgeVertexIndices;
}

export function getBendingVertexIds(faceTriIds) {
  const triangleCount = faceTriIds.length / 3;
  const neighborEdgeIndices = findTriangleNeighbors(faceTriIds);

  const bendingVertexIds = [];

  for (let triId = 0; triId < triangleCount; triId++) {
    for (let k = 0; k < 3; k++) {
      const v0 = faceTriIds[3 * triId + k];
      const v1 = faceTriIds[3 * triId + ((k + 1) % 3)];

      const edgeIndex = 3 * triId + k;
      const neighborEdgeIndex = neighborEdgeIndices[edgeIndex];

      if (neighborEdgeIndex >= 0) {
        const neighborTriIndex = Math.floor(neighborEdgeIndex / 3);
        const neighborLocalEdgeIndex = neighborEdgeIndex % 3;

        const v2 = faceTriIds[3 * triId + ((k + 2) % 3)];
        const v3 =
          faceTriIds[3 * neighborTriIndex + ((neighborLocalEdgeIndex + 2) % 3)];

        bendingVertexIds.push(v0, v1, v2, v3);
      }
    }
  }

  return bendingVertexIds;
}
