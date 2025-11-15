export const Vec3 = {
  dot(a, b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  },
  sub(a, b) {
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  },
  add(a, b) {
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  },
  cross(a, b) {
    return [
      a[1] * b[2] - a[2] * b[1],
      a[2] * b[0] - a[0] * b[2],
      a[0] * b[1] - a[1] * b[0],
    ];
  },
  norm(a) {
    return Math.hypot(a[0], a[1], a[2]);
  },
  normalize(a) {
    const n = Vec3.norm(a);
    return n > 0 ? [a[0] / n, a[1] / n, a[2] / n] : [0, 0, 0];
  },
  mulScalar(a, s) {
    return [a[0] * s, a[1] * s, a[2] * s];
  },
  clamp(x, lo, hi) {
    return Math.max(lo, Math.min(hi, x));
  },
};
