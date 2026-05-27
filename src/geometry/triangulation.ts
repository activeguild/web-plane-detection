import { Point2D } from '../features/orb';

export type Point3D = {
  x: number;
  y: number;
  z: number;
};

function svdSolve4x4(A: number[][]): number[] {
  const AtA: number[][] = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
  ];
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      let sum = 0;
      for (let k = 0; k < 4; k++) {
        sum += A[k][i] * A[k][j];
      }
      AtA[i][j] = sum;
    }
  }

  let traceSum = 0;
  for (let i = 0; i < 4; i++) traceSum += AtA[i][i];
  const shift = traceSum;

  const B: number[][] = [
    [shift - AtA[0][0], -AtA[0][1], -AtA[0][2], -AtA[0][3]],
    [-AtA[1][0], shift - AtA[1][1], -AtA[1][2], -AtA[1][3]],
    [-AtA[2][0], -AtA[2][1], shift - AtA[2][2], -AtA[2][3]],
    [-AtA[3][0], -AtA[3][1], -AtA[3][2], shift - AtA[3][3]],
  ];

  function solve(b: number[]): number[] {
    const aug: number[][] = B.map((row, i) => [...row, b[i]]);
    const n = 4;
    for (let col = 0; col < n; col++) {
      let maxRow = col;
      let maxVal = Math.abs(aug[col][col]);
      for (let row = col + 1; row < n; row++) {
        if (Math.abs(aug[row][col]) > maxVal) {
          maxVal = Math.abs(aug[row][col]);
          maxRow = row;
        }
      }
      [aug[col], aug[maxRow]] = [aug[maxRow], aug[col]];

      if (Math.abs(aug[col][col]) < 1e-12) continue;

      for (let row = col + 1; row < n; row++) {
        const factor = aug[row][col] / aug[col][col];
        for (let j = col; j <= n; j++) {
          aug[row][j] -= factor * aug[col][j];
        }
      }
    }

    const x = [0, 0, 0, 0];
    for (let i = n - 1; i >= 0; i--) {
      x[i] = aug[i][n];
      for (let j = i + 1; j < n; j++) {
        x[i] -= aug[i][j] * x[j];
      }
      if (Math.abs(aug[i][i]) > 1e-12) {
        x[i] /= aug[i][i];
      }
    }
    return x;
  }

  let v = [1, 1, 1, 1];
  for (let iter = 0; iter < 20; iter++) {
    const w = solve(v);
    let norm = 0;
    for (let i = 0; i < 4; i++) norm += w[i] * w[i];
    norm = Math.sqrt(norm);
    if (norm < 1e-15) break;
    for (let i = 0; i < 4; i++) v[i] = w[i] / norm;
  }

  return v;
}

function triangulateOne(
  x1: Point2D,
  x2: Point2D,
  P1: number[][],
  P2: number[][],
): Point3D | null {
  const A: number[][] = [
    [
      x1.x * P1[2][0] - P1[0][0],
      x1.x * P1[2][1] - P1[0][1],
      x1.x * P1[2][2] - P1[0][2],
      x1.x * P1[2][3] - P1[0][3],
    ],
    [
      x1.y * P1[2][0] - P1[1][0],
      x1.y * P1[2][1] - P1[1][1],
      x1.y * P1[2][2] - P1[1][2],
      x1.y * P1[2][3] - P1[1][3],
    ],
    [
      x2.x * P2[2][0] - P2[0][0],
      x2.x * P2[2][1] - P2[0][1],
      x2.x * P2[2][2] - P2[0][2],
      x2.x * P2[2][3] - P2[0][3],
    ],
    [
      x2.y * P2[2][0] - P2[1][0],
      x2.y * P2[2][1] - P2[1][1],
      x2.y * P2[2][2] - P2[1][2],
      x2.y * P2[2][3] - P2[1][3],
    ],
  ];

  const v = svdSolve4x4(A);

  if (Math.abs(v[3]) < 1e-10) return null;
  const x = v[0] / v[3];
  const y = v[1] / v[3];
  const z = v[2] / v[3];

  return { x, y, z };
}

function mat3x3MulVec(M: number[][], v: number[]): number[] {
  return [
    M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
    M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
    M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2],
  ];
}

export function triangulatePoints(
  prevPoints: Point2D[],
  currPoints: Point2D[],
  R: number[][],
  t: number[],
  cameraMatrix: number[][],
  inlierMask: boolean[],
): Point3D[] {
  const K = cameraMatrix;

  const P1: number[][] = [
    [K[0][0], K[0][1], K[0][2], 0],
    [K[1][0], K[1][1], K[1][2], 0],
    [K[2][0], K[2][1], K[2][2], 0],
  ];

  const P2: number[][] = [
    [
      K[0][0] * R[0][0] + K[0][1] * R[1][0] + K[0][2] * R[2][0],
      K[0][0] * R[0][1] + K[0][1] * R[1][1] + K[0][2] * R[2][1],
      K[0][0] * R[0][2] + K[0][1] * R[1][2] + K[0][2] * R[2][2],
      K[0][0] * t[0] + K[0][1] * t[1] + K[0][2] * t[2],
    ],
    [
      K[1][0] * R[0][0] + K[1][1] * R[1][0] + K[1][2] * R[2][0],
      K[1][0] * R[0][1] + K[1][1] * R[1][1] + K[1][2] * R[2][1],
      K[1][0] * R[0][2] + K[1][1] * R[1][2] + K[1][2] * R[2][2],
      K[1][0] * t[0] + K[1][1] * t[1] + K[1][2] * t[2],
    ],
    [
      K[2][0] * R[0][0] + K[2][1] * R[1][0] + K[2][2] * R[2][0],
      K[2][0] * R[0][1] + K[2][1] * R[1][1] + K[2][2] * R[2][1],
      K[2][0] * R[0][2] + K[2][1] * R[1][2] + K[2][2] * R[2][2],
      K[2][0] * t[0] + K[2][1] * t[1] + K[2][2] * t[2],
    ],
  ];

  const rawPoints: Point3D[] = [];
  const distances: number[] = [];

  for (let i = 0; i < prevPoints.length; i++) {
    if (!inlierMask[i]) continue;

    const pt = triangulateOne(prevPoints[i], currPoints[i], P1, P2);
    if (!pt) continue;

    if (pt.z <= 0) continue;

    const ptInCam2 = mat3x3MulVec(R, [pt.x, pt.y, pt.z]);
    ptInCam2[0] += t[0];
    ptInCam2[1] += t[1];
    ptInCam2[2] += t[2];
    if (ptInCam2[2] <= 0) continue;

    const dist = Math.sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    rawPoints.push(pt);
    distances.push(dist);
  }

  if (rawPoints.length === 0) return [];

  const sorted = [...distances].sort((a, b) => a - b);
  const median = sorted[Math.floor(sorted.length / 2)];
  const maxDist = median * 10;

  const filtered = rawPoints.filter((_, i) => distances[i] <= maxDist);
  console.log(`[SLAM] triangulated: ${filtered.length}/${rawPoints.length} points (median dist: ${median.toFixed(2)})`);
  return filtered;
}
