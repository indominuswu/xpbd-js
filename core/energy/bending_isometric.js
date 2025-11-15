function bendingEnergyIsometric(pos, v0, v1, v2, v3) {
  // 左 T0=(X1,X0,X2)、右 T1=(X1,X0,X3) の面積
  const area0 = triangleArea(pos, v1, v0, v2);
  const area1 = triangleArea(pos, v1, v0, v3);
  const A = area0 + area1;
  if (A < 1e-15) {
    return { K: [0, 0, 0, 0], coeff: 0 };
  }

  const cot01 = cotAtVertex(pos, v0, v1, v2);
  const cot02 = cotAtVertex(pos, v0, v1, v3);
  const cot03 = cotAtVertex(pos, v1, v0, v2);
  const cot04 = cotAtVertex(pos, v1, v0, v3);
  const c = [cot03 + cot04, cot01 + cot02, -cot01 - cot03, -cot02 - cot04];
  const X = [
    [pos[3 * v0], pos[3 * v0 + 1], pos[3 * v0 + 2]],
    [pos[3 * v1], pos[3 * v1 + 1], pos[3 * v1 + 2]],
    [pos[3 * v2], pos[3 * v2 + 1], pos[3 * v2 + 2]],
    [pos[3 * v3], pos[3 * v3 + 1], pos[3 * v3 + 2]],
  ];
  var energy = 0.0;
  for (let p = 0; p < 4; p++) {
    for (let q = 0; q < 4; q++) {
      energy += c[p] * c[q] * _dot(X[p], X[q]);
    }
  }
  // const Sx =
  //   c[0] * pos[3 * v0] +
  //   c[1] * pos[3 * v1] +
  //   c[2] * pos[3 * v2] +
  //   c[3] * pos[3 * v3];
  // const Sy =
  //   c[0] * pos[3 * v0 + 1] +
  //   c[1] * pos[3 * v1 + 1] +
  //   c[2] * pos[3 * v2 + 1] +
  //   c[3] * pos[3 * v3 + 1];
  // const Sz =
  //   c[0] * pos[3 * v0 + 2] +
  //   c[1] * pos[3 * v1 + 2] +
  //   c[2] * pos[3 * v2 + 2] +
  //   c[3] * pos[3 * v3 + 2];
  // const Slen = Math.hypot(Sx, Sy, Sz);
  // const _energy = Slen * Slen;

  return (3 * energy) / A;
} // ヘルパ
