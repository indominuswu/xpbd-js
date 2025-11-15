export function buildGridMesh({
  cols,
  rows,
  width,
  height,
  origin = [0, 0, 0],
  angle = 0.0,
}) {
  if (cols < 2 || rows < 2) {
    throw new Error("colsとrowsは2以上が必要です");
  }

  const [ox, oy, oz] = origin;
  const vertices = [];
  const faceTriIds = [];

  // セルサイズ
  const cellW = width / (cols - 1);
  const cellH = height / (rows - 1);

  // 中心揃えオフセット
  const xOffset = -width / 2;
  const yOffset = height / 2;

  // --- 頂点生成 ---
  const vertexIds = [];
  for (let j = 0; j < rows; j++) {
    for (let i = 0; i < cols; i++) {
      let x = +xOffset + i * cellW;
      let y = yOffset - j * cellH;
      let z = 0;

      if (angle !== 0) {
        const cosA = Math.cos(angle);
        const sinA = Math.sin(angle);

        const y2 = y * cosA - z * sinA;
        const z2 = y * sinA + z * cosA;

        y = y2;
        z = z2;
      }

      x += ox;
      y += oy;
      z += oz;
      vertices.push(x, y, z);
      vertexIds.push(vertices.length / 3 - 1);
    }
  }

  // --- 三角形生成 ---
  let p = false;
  for (let rowIdx = 0; rowIdx < rows - 1; rowIdx++) {
    for (let colIdx = 0; colIdx < cols - 1; colIdx++) {
      const v0 = rowIdx * cols + colIdx;
      const v1 = v0 + 1;
      const v2 = (rowIdx + 1) * cols + colIdx;
      const v3 = v2 + 1;

      if (p) {
        faceTriIds.push(v0, v2, v1);
        faceTriIds.push(v1, v2, v3);
        p = false;
      } else {
        faceTriIds.push(v0, v3, v1);
        faceTriIds.push(v2, v3, v0);
        p = true;
      }
    }
  }

  return { vertices, vertexIds, faceTriIds };
}
