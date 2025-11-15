function bendingEnergyHinge(pos, v0, v1, v2, v3, theta0 = 0.0) {
  const x0 = [pos[3 * v0], pos[3 * v0 + 1], pos[3 * v0 + 2]];
  const x1 = [pos[3 * v1], pos[3 * v1 + 1], pos[3 * v1 + 2]];
  const x2 = [pos[3 * v2], pos[3 * v2 + 1], pos[3 * v2 + 2]];
  const x3 = [pos[3 * v3], pos[3 * v3 + 1], pos[3 * v3 + 2]];

  const e = _sub(x1, x0);
  const el = _norm(e);
  if (el < 1e-12) return 0.0;

  // 法線（無正規化→正規化）
  const n0p = _cross(_sub(x0, x1), _sub(x2, x1)); // (v0-v1) × (v2-v1)
  const n1p = _cross(_sub(x3, x1), _sub(x0, x1)); // (v3-v1) × (v0-v1)
  const a0 = 0.5 * _norm(n0p),
    a1 = 0.5 * _norm(n1p);
  const A = a0 + a1;
  if (A < 1e-15) return 0.0;

  const n0l = _norm(n0p),
    n1l = _norm(n1p);
  if (n0l < 1e-12 || n1l < 1e-12) return 0.0;
  const n0 = [n0p[0] / n0l, n0p[1] / n0l, n0p[2] / n0l];
  const n1 = [n1p[0] / n1l, n1p[1] / n1l, n1p[2] / n1l];

  // 符号付き二面角
  const enx = e[0] / el,
    eny = e[1] / el,
    enz = e[2] / el;
  const x = _dot(n0, n1);
  const cxn = _cross(n0, n1);
  const y = enx * cxn[0] + eny * cxn[1] + enz * cxn[2];
  const theta = Math.atan2(y, Math.max(-1, Math.min(1, x)));

  const dtheta = theta - theta0;
  // 離散シェル係数（同じ係数でソルバと整合）： 3|e|^2 / (2A)
  const coeff = (3.0 * el * el) / (2.0 * A);
  return coeff * dtheta * dtheta;
}
