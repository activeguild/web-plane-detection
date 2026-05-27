import { Point3D } from '../geometry/triangulation';

export type PlaneResult = {
  normal: number[];
  d: number;
  inliers: Point3D[];
};

export function detectPlane(
  points: Point3D[],
  threshold?: number,
  iterations: number = 200,
): PlaneResult | null {
  const n = points.length;
  if (n < 3) return null;

  // 閾値が未指定なら点群のスケールから自動設定
  // 中央値距離の 5% を閾値とする
  if (threshold === undefined) {
    const dists = points.map(p => Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z));
    const sorted = [...dists].sort((a, b) => a - b);
    const median = sorted[Math.floor(sorted.length / 2)];
    threshold = median * 0.05;
    console.log(`[SLAM] plane RANSAC auto-threshold: ${threshold.toFixed(4)} (median dist: ${median.toFixed(4)})`);
  }

  let bestInliers: number[] = [];
  let bestNormal: number[] = [0, 0, 0];
  let bestD = 0;

  for (let iter = 0; iter < iterations; iter++) {
    const i0 = Math.floor(Math.random() * n);
    let i1 = Math.floor(Math.random() * n);
    let i2 = Math.floor(Math.random() * n);
    if (i1 === i0) i1 = (i0 + 1) % n;
    if (i2 === i0 || i2 === i1) i2 = (i0 + 2) % n;

    const p0 = points[i0];
    const p1 = points[i1];
    const p2 = points[i2];

    const v1 = [p1.x - p0.x, p1.y - p0.y, p1.z - p0.z];
    const v2 = [p2.x - p0.x, p2.y - p0.y, p2.z - p0.z];

    const nx = v1[1] * v2[2] - v1[2] * v2[1];
    const ny = v1[2] * v2[0] - v1[0] * v2[2];
    const nz = v1[0] * v2[1] - v1[1] * v2[0];

    const len = Math.sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 1e-10) continue;

    const a = nx / len;
    const b = ny / len;
    const c = nz / len;
    const d = -(a * p0.x + b * p0.y + c * p0.z);

    const inliers: number[] = [];
    for (let i = 0; i < n; i++) {
      const dist = Math.abs(a * points[i].x + b * points[i].y + c * points[i].z + d);
      if (dist < threshold) {
        inliers.push(i);
      }
    }

    if (inliers.length > bestInliers.length) {
      bestInliers = inliers;
      bestNormal = [a, b, c];
      bestD = d;
    }
  }

  if (bestInliers.length < n * 0.3) {
    console.log(`[SLAM] plane detection failed: ${bestInliers.length}/${n} inliers`);
    return null;
  }

  const inlierPoints = bestInliers.map(i => points[i]);
  console.log(`[SLAM] plane detected: ${inlierPoints.length}/${n} inliers, normal=[${bestNormal.map(v => v.toFixed(3)).join(', ')}]`);

  return {
    normal: bestNormal,
    d: bestD,
    inliers: inlierPoints,
  };
}
