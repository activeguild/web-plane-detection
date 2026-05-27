# Phase 3: 基本行列推定と初期3D点群復元 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** カメラを動かすと自動的に基本行列を推定し、三角測量で3D点群を復元して鳥瞰図オーバーレイに表示する

**Architecture:** Phase 2 の追跡ループ上に初期化判定を追加。追跡点の平均移動量が閾値を超えたら `cv.findEssentialMat` + `cv.recoverPose` で R, t を推定し、DLT 三角測量で3D点群を復元。鳥瞰図（X-Z平面投影）をキャンバス右下にオーバーレイ描画。

**Tech Stack:** TypeScript, OpenCV.js (`@techstark/opencv-js`), Vite

---

## File Structure

| Action | Path | Responsibility |
|--------|------|----------------|
| Create | `src/camera/calibration.ts` | カメラ内部パラメータ管理 |
| Create | `src/geometry/essential.ts` | 基本行列推定 + R,t 復元 |
| Create | `src/geometry/triangulation.ts` | DLT 三角測量 |
| Create | `src/visualization/point-cloud.ts` | 3D点群の鳥瞰図描画 |
| Modify | `src/features/tracker.ts` | avgMotion を TrackResult に追加 |
| Modify | `src/main.ts` | 初期化フロー + 点群可視化統合 |

---

### Task 1: カメラキャリブレーションモジュール

**Files:**
- Create: `src/camera/calibration.ts`

- [ ] **Step 1: `src/camera/calibration.ts` を作成**

```typescript
import cv from '@techstark/opencv-js';

export class CameraCalibration {
  readonly fx: number;
  readonly fy: number;
  readonly cx: number;
  readonly cy: number;

  constructor(imageWidth: number, imageHeight: number) {
    this.fx = imageWidth * 0.9;
    this.fy = imageWidth * 0.9;
    this.cx = imageWidth / 2;
    this.cy = imageHeight / 2;
  }

  getCameraMatrix(): number[][] {
    return [
      [this.fx, 0, this.cx],
      [0, this.fy, this.cy],
      [0, 0, 1],
    ];
  }

  getCameraMatrixAsMat(): cv.Mat {
    const K = cv.matFromArray(3, 3, cv.CV_64FC1, [
      this.fx, 0, this.cx,
      0, this.fy, this.cy,
      0, 0, 1,
    ]);
    return K;
  }

  getFocalLength(): number {
    return this.fx;
  }

  getPrincipalPoint(): { x: number; y: number } {
    return { x: this.cx, y: this.cy };
  }
}
```

- [ ] **Step 2: Commit**

```bash
git add src/camera/calibration.ts
git commit -m "feat: add camera calibration module with approximate intrinsics"
```

---

### Task 2: 基本行列推定モジュール

**Files:**
- Create: `src/geometry/essential.ts`

- [ ] **Step 1: `src/geometry/essential.ts` を作成**

```typescript
import cv from '@techstark/opencv-js';
import { Point2D } from '../features/orb';

export type PoseResult = {
  R: number[][];
  t: number[];
  inlierCount: number;
  inlierMask: boolean[];
};

function matToArray2D(mat: cv.Mat, rows: number, cols: number): number[][] {
  const result: number[][] = [];
  for (let r = 0; r < rows; r++) {
    const row: number[] = [];
    for (let c = 0; c < cols; c++) {
      row.push(mat.doubleAt(r, c));
    }
    result.push(row);
  }
  return result;
}

function matToArray1D(mat: cv.Mat, length: number): number[] {
  const result: number[] = [];
  for (let i = 0; i < length; i++) {
    result.push(mat.doubleAt(i, 0));
  }
  return result;
}

export function estimatePose(
  prevPoints: Point2D[],
  currPoints: Point2D[],
  focalLength: number,
  principalPoint: { x: number; y: number },
): PoseResult | null {
  const n = prevPoints.length;
  if (n < 8) return null;

  // Point2D[] → cv.Mat (N×1, CV_32FC2)
  const pts1 = new cv.Mat(n, 1, cv.CV_32FC2);
  const pts2 = new cv.Mat(n, 1, cv.CV_32FC2);
  const data1 = pts1.data32F;
  const data2 = pts2.data32F;
  for (let i = 0; i < n; i++) {
    data1[i * 2] = prevPoints[i].x;
    data1[i * 2 + 1] = prevPoints[i].y;
    data2[i * 2] = currPoints[i].x;
    data2[i * 2 + 1] = currPoints[i].y;
  }

  const pp = new cv.Point(principalPoint.x, principalPoint.y);
  const mask = new cv.Mat();

  let E: cv.Mat;
  try {
    E = cv.findEssentialMat(pts1, pts2, focalLength, pp, cv.RANSAC, 0.999, 1.0, mask);
  } catch (e) {
    pts1.delete();
    pts2.delete();
    mask.delete();
    console.error('[SLAM] findEssentialMat failed:', e);
    return null;
  }

  // インライア数チェック
  const maskData = mask.data;
  let inlierCount = 0;
  const inlierMask: boolean[] = [];
  for (let i = 0; i < n; i++) {
    const isInlier = maskData[i] === 1;
    inlierMask.push(isInlier);
    if (isInlier) inlierCount++;
  }

  if (inlierCount < n * 0.3) {
    E.delete();
    pts1.delete();
    pts2.delete();
    mask.delete();
    console.log(`[SLAM] too few inliers: ${inlierCount}/${n}`);
    return null;
  }

  // R, t を復元
  const R = new cv.Mat();
  const t = new cv.Mat();
  const recoverMask = new cv.Mat();

  try {
    cv.recoverPose(E, pts1, pts2, R, t, focalLength, pp, recoverMask);
  } catch (e) {
    E.delete();
    pts1.delete();
    pts2.delete();
    mask.delete();
    R.delete();
    t.delete();
    recoverMask.delete();
    console.error('[SLAM] recoverPose failed:', e);
    return null;
  }

  const result: PoseResult = {
    R: matToArray2D(R, 3, 3),
    t: matToArray1D(t, 3),
    inlierCount,
    inlierMask,
  };

  E.delete();
  pts1.delete();
  pts2.delete();
  mask.delete();
  R.delete();
  t.delete();
  recoverMask.delete();

  console.log(`[SLAM] pose estimated: ${inlierCount}/${n} inliers`);
  return result;
}
```

- [ ] **Step 2: Commit**

```bash
git add src/geometry/essential.ts
git commit -m "feat: add essential matrix estimation with findEssentialMat + recoverPose"
```

---

### Task 3: 三角測量モジュール

**Files:**
- Create: `src/geometry/triangulation.ts`

- [ ] **Step 1: `src/geometry/triangulation.ts` を作成**

```typescript
import { Point2D } from '../features/orb';

export type Point3D = {
  x: number;
  y: number;
  z: number;
};

// 4×4 行列の SVD で最小特異値の右特異ベクトルを求める (Jacobi 反復法)
function svdSolve4x4(A: number[][]): number[] {
  // A^T * A を計算 (4×4)
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

  // べき乗法の逆反復で最小固有値の固有ベクトルを求める
  // まず AtA の対角和でシフト量を決める
  let traceSum = 0;
  for (let i = 0; i < 4; i++) traceSum += AtA[i][i];
  const shift = traceSum; // 大きめのシフト

  // (AtA - shift * I) の逆行列を使った逆反復法
  // shift * I - AtA を作って、それで連立方程式を解く
  const B: number[][] = [
    [shift - AtA[0][0], -AtA[0][1], -AtA[0][2], -AtA[0][3]],
    [-AtA[1][0], shift - AtA[1][1], -AtA[1][2], -AtA[1][3]],
    [-AtA[2][0], -AtA[2][1], shift - AtA[2][2], -AtA[2][3]],
    [-AtA[3][0], -AtA[3][1], -AtA[3][2], shift - AtA[3][3]],
  ];

  // ガウスの消去法で B^{-1} * v を計算する関数
  function solve(b: number[]): number[] {
    const aug: number[][] = B.map((row, i) => [...row, b[i]]);
    const n = 4;
    for (let col = 0; col < n; col++) {
      // ピボット選択
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

  // 逆反復法: 20回反復
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
  // DLT: 4×4 行列 A を構築
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

  // 同次座標 → 3D座標
  if (Math.abs(v[3]) < 1e-10) return null;
  const x = v[0] / v[3];
  const y = v[1] / v[3];
  const z = v[2] / v[3];

  return { x, y, z };
}

// 3×3 行列と 3×1 ベクトルの積
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

  // P1 = K × [I | 0]
  const P1: number[][] = [
    [K[0][0], K[0][1], K[0][2], 0],
    [K[1][0], K[1][1], K[1][2], 0],
    [K[2][0], K[2][1], K[2][2], 0],
  ];

  // P2 = K × [R | t]
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

    // cheirality check: カメラ1の前方 (z > 0)
    if (pt.z <= 0) continue;

    // cheirality check: カメラ2の前方
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

  // 外れ値フィルタ: 中央値の10倍超を除外
  const sorted = [...distances].sort((a, b) => a - b);
  const median = sorted[Math.floor(sorted.length / 2)];
  const maxDist = median * 10;

  const filtered = rawPoints.filter((_, i) => distances[i] <= maxDist);
  console.log(`[SLAM] triangulated: ${filtered.length}/${rawPoints.length} points (median dist: ${median.toFixed(2)})`);
  return filtered;
}
```

- [ ] **Step 2: Commit**

```bash
git add src/geometry/triangulation.ts
git commit -m "feat: add DLT triangulation with cheirality check and outlier filter"
```

---

### Task 4: 点群可視化モジュール

**Files:**
- Create: `src/visualization/point-cloud.ts`

- [ ] **Step 1: `src/visualization/point-cloud.ts` を作成**

```typescript
import { Point3D } from '../geometry/triangulation';

export class PointCloudView {
  private ctx: CanvasRenderingContext2D;
  private viewSize: number = 200;
  private originX: number;
  private originY: number;

  constructor(ctx: CanvasRenderingContext2D, canvasWidth: number, canvasHeight: number) {
    this.ctx = ctx;
    this.originX = canvasWidth - this.viewSize - 10;
    this.originY = canvasHeight - this.viewSize - 10;
  }

  draw(points: Point3D[], cameraT?: number[]): void {
    const ctx = this.ctx;
    const ox = this.originX;
    const oy = this.originY;
    const size = this.viewSize;

    // 半透明背景
    ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
    ctx.fillRect(ox, oy, size, size);

    // 枠線
    ctx.strokeStyle = '#444';
    ctx.lineWidth = 1;
    ctx.strokeRect(ox, oy, size, size);

    // ラベル
    ctx.fillStyle = '#888';
    ctx.font = '10px sans-serif';
    ctx.fillText('Bird\'s Eye (X-Z)', ox + 4, oy + 12);

    if (points.length === 0) return;

    // X-Z 平面のスケーリング（点群の範囲に合わせる）
    let minX = Infinity, maxX = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;
    for (const p of points) {
      if (p.x < minX) minX = p.x;
      if (p.x > maxX) maxX = p.x;
      if (p.z < minZ) minZ = p.z;
      if (p.z > maxZ) maxZ = p.z;
    }

    // カメラ位置も範囲に含める
    if (cameraT) {
      if (cameraT[0] < minX) minX = cameraT[0];
      if (cameraT[0] > maxX) maxX = cameraT[0];
      if (cameraT[2] < minZ) minZ = cameraT[2];
      if (cameraT[2] > maxZ) maxZ = cameraT[2];
    }

    // 原点（カメラ1）も含める
    if (0 < minX) minX = 0;
    if (0 > maxX) maxX = 0;
    if (0 < minZ) minZ = 0;
    if (0 > maxZ) maxZ = 0;

    const rangeX = maxX - minX || 1;
    const rangeZ = maxZ - minZ || 1;
    const margin = 20;
    const drawSize = size - margin * 2;
    const scale = Math.min(drawSize / rangeX, drawSize / rangeZ);

    const toScreenX = (x: number) => ox + margin + (x - minX) * scale;
    const toScreenY = (z: number) => oy + margin + (z - minZ) * scale;

    // 3D点群（白い点）
    ctx.fillStyle = '#ffffff';
    for (const p of points) {
      const sx = toScreenX(p.x);
      const sy = toScreenY(p.z);
      ctx.fillRect(sx - 1, sy - 1, 2, 2);
    }

    // カメラ1（原点、緑の三角）
    this.drawCamera(toScreenX(0), toScreenY(0), '#00ff00');

    // カメラ2（推定位置、黄色の三角）
    if (cameraT) {
      this.drawCamera(toScreenX(cameraT[0]), toScreenY(cameraT[2]), '#ffff00');
    }
  }

  private drawCamera(sx: number, sy: number, color: string): void {
    const ctx = this.ctx;
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(sx, sy - 5);
    ctx.lineTo(sx - 4, sy + 3);
    ctx.lineTo(sx + 4, sy + 3);
    ctx.closePath();
    ctx.fill();
  }
}
```

- [ ] **Step 2: Commit**

```bash
git add src/visualization/point-cloud.ts
git commit -m "feat: add bird's-eye view point cloud visualization"
```

---

### Task 5: tracker.ts に avgMotion を追加

**Files:**
- Modify: `src/features/tracker.ts`

- [ ] **Step 1: `TrackResult` に `avgMotion` を追加し、`process()` で計算する**

`src/features/tracker.ts` の `TrackResult` 型を変更:

```typescript
export type TrackResult = {
  points: Point2D[];
  prevPoints: Point2D[];
  count: number;
  avgMotion: number;
};
```

初回 return（行31-35付近）に `avgMotion: 0` を追加:

```typescript
      return {
        points: this.prevPoints,
        prevPoints: this.prevPoints,
        count: this.prevPoints.length,
        avgMotion: 0,
      };
```

最終 return（行106-110付近）の直前に平均移動量の計算を追加し、return に含める:

```typescript
    // 平均移動量を計算
    let totalMotion = 0;
    let motionCount = 0;
    for (let i = 0; i < trackedPoints.length; i++) {
      const dx = trackedPoints[i].x - trackedPrevPoints[i].x;
      const dy = trackedPoints[i].y - trackedPrevPoints[i].y;
      const dist = Math.sqrt(dx * dx + dy * dy);
      // 新規追加点（prev === curr）はスキップ
      if (dist > 0) {
        totalMotion += dist;
        motionCount++;
      }
    }
    const avgMotion = motionCount > 0 ? totalMotion / motionCount : 0;

    return {
      points: trackedPoints,
      prevPoints: trackedPrevPoints,
      count: trackedPoints.length,
      avgMotion,
    };
```

- [ ] **Step 2: Commit**

```bash
git add src/features/tracker.ts
git commit -m "feat: add avgMotion to TrackResult for initialization trigger"
```

---

### Task 6: main.ts に初期化フロー + 点群可視化を統合

**Files:**
- Modify: `src/main.ts`

- [ ] **Step 1: `src/main.ts` を以下に置き換え**

```typescript
import cv from '@techstark/opencv-js';
import { initCamera } from './camera/capture';
import { CameraCalibration } from './camera/calibration';
import { OrbDetector } from './features/orb';
import { FeatureTracker } from './features/tracker';
import { estimatePose } from './geometry/essential';
import { triangulatePoints, Point3D } from './geometry/triangulation';
import { PointCloudView } from './visualization/point-cloud';

function waitForOpenCv(): Promise<void> {
  return new Promise((resolve) => {
    if (cv.Mat) {
      resolve();
      return;
    }
    cv.onRuntimeInitialized = () => resolve();
  });
}

async function main() {
  const video = document.getElementById('video') as HTMLVideoElement;
  const canvas = document.getElementById('canvas') as HTMLCanvasElement;
  const loading = document.getElementById('loading') as HTMLDivElement;
  const ctx = canvas.getContext('2d')!;

  console.log('[SLAM] main() started');

  loading.textContent = 'OpenCV.js を読み込み中...';
  await waitForOpenCv();
  console.log('[SLAM] OpenCV.js ready');

  loading.textContent = 'カメラを起動中...';
  await initCamera(video);
  const w = video.videoWidth;
  const h = video.videoHeight;
  console.log(`[SLAM] camera ready: ${w}x${h}`);
  canvas.width = w;
  canvas.height = h;

  loading.style.display = 'none';

  // モジュール初期化
  const calibration = new CameraCalibration(w, h);
  const orb = new OrbDetector(500);
  const tracker = new FeatureTracker(orb, 200);
  const pointCloudView = new PointCloudView(ctx, w, h);

  const offscreen = document.createElement('canvas');
  offscreen.width = w;
  offscreen.height = h;
  const offCtx = offscreen.getContext('2d')!;

  // 初期化状態
  let initialized = false;
  let points3D: Point3D[] = [];
  let cameraT: number[] = [];
  const MOTION_THRESHOLD = 50;

  console.log('[SLAM] starting tracking loop');

  let frameCount = 0;
  function processFrame() {
    try {
      // フレーム取得
      offCtx.drawImage(video, 0, 0, w, h);
      const imageData = offCtx.getImageData(0, 0, w, h);
      const frame = cv.matFromImageData(imageData);

      // 追跡
      const result = tracker.process(frame);
      frame.delete();

      // 描画: 映像
      ctx.drawImage(video, 0, 0, w, h);

      // 描画: 移動ベクトル（赤い線）
      ctx.strokeStyle = '#ff0000';
      ctx.lineWidth = 1.5;
      for (let i = 0; i < result.count; i++) {
        const prev = result.prevPoints[i];
        const curr = result.points[i];
        if (prev.x !== curr.x || prev.y !== curr.y) {
          ctx.beginPath();
          ctx.moveTo(prev.x, prev.y);
          ctx.lineTo(curr.x, curr.y);
          ctx.stroke();
        }
      }

      // 描画: 特徴点マーカー（緑の円）
      ctx.fillStyle = '#00ff00';
      for (let i = 0; i < result.count; i++) {
        const p = result.points[i];
        ctx.beginPath();
        ctx.arc(p.x, p.y, 3, 0, Math.PI * 2);
        ctx.fill();
      }

      // 初期化判定
      if (!initialized && result.avgMotion > MOTION_THRESHOLD && result.count >= 30) {
        console.log(`[SLAM] initialization trigger: avgMotion=${result.avgMotion.toFixed(1)}, points=${result.count}`);

        const pose = estimatePose(
          result.prevPoints,
          result.points,
          calibration.getFocalLength(),
          calibration.getPrincipalPoint(),
        );

        if (pose) {
          points3D = triangulatePoints(
            result.prevPoints,
            result.points,
            pose.R,
            pose.t,
            calibration.getCameraMatrix(),
            pose.inlierMask,
          );

          if (points3D.length > 10) {
            cameraT = pose.t;
            initialized = true;
            console.log(`[SLAM] initialized! ${points3D.length} 3D points`);
          } else {
            console.log(`[SLAM] not enough 3D points: ${points3D.length}`);
          }
        }
      }

      // 点群可視化
      if (initialized) {
        pointCloudView.draw(points3D, cameraT);
      }

      frameCount++;
      if (frameCount === 1) console.log(`[SLAM] first frame: ${result.count} points`);
    } catch (e) {
      console.error('[SLAM] processFrame error:', e);
    }
    requestAnimationFrame(processFrame);
  }

  requestAnimationFrame(processFrame);
}

main().catch((err) => {
  console.error('初期化エラー:', err);
  const loading = document.getElementById('loading');
  if (loading) loading.textContent = `エラー: ${err.message}`;
});
```

- [ ] **Step 2: 動作確認**

Run: `npm run dev` (または `pnpm vite`)

Expected:
- カメラ映像 + 追跡点（緑マーカー + 赤ベクトル）は Phase 2 と同じ
- カメラを十分に動かすと、右下に鳥瞰図オーバーレイが出現
- 白い点が3D点群、緑の三角がカメラ1、黄色の三角がカメラ2

- [ ] **Step 3: Commit**

```bash
git add src/main.ts
git commit -m "feat: integrate initialization flow with 3D point cloud visualization"
```

---

## Summary

| Task | 内容 | 依存 |
|------|------|------|
| 1 | カメラキャリブレーション | なし |
| 2 | 基本行列推定 | なし |
| 3 | 三角測量 | なし |
| 4 | 点群可視化 | Task 3 (Point3D 型) |
| 5 | tracker に avgMotion 追加 | なし |
| 6 | main.ts 統合 | Task 1-5 すべて |

Task 1, 2, 3, 5 は独立しており並列実行可能。Task 4 は Task 3 の `Point3D` 型に依存。Task 6 は全タスク完了後。
