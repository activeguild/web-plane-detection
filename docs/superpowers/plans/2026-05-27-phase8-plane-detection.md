# Phase 8: RANSAC 平面検出と可視化 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 初期化時の3D点群から RANSAC で平面を検出し、カメラ映像上に半透明ポリゴンでオーバーレイ描画する

**Architecture:** `detectPlane()` で3D点群から RANSAC 平面フィッティング。インライア点の凸包を計算し、毎フレーム PnP 姿勢で2D投影して Canvas 2D で半透明ポリゴンを描画。平面検出は初期化時に1回のみ。

**Tech Stack:** TypeScript, Canvas 2D API

---

## File Structure

| Action | Path | Responsibility |
|--------|------|----------------|
| Create | `src/plane/ransac.ts` | RANSAC 平面検出 |
| Create | `src/visualization/plane-overlay.ts` | 平面の2D投影 + 凸包 + ポリゴン描画 |
| Modify | `src/main.ts` | 平面検出 + オーバーレイを統合 |

---

### Task 1: RANSAC 平面検出モジュール

**Files:**
- Create: `src/plane/ransac.ts`

- [ ] **Step 1: `src/plane/ransac.ts` を作成**

```typescript
import { Point3D } from '../geometry/triangulation';

export type PlaneResult = {
  normal: number[];
  d: number;
  inliers: Point3D[];
};

export function detectPlane(
  points: Point3D[],
  threshold: number = 0.02,
  iterations: number = 200,
): PlaneResult | null {
  const n = points.length;
  if (n < 3) return null;

  let bestInliers: number[] = [];
  let bestNormal: number[] = [0, 0, 0];
  let bestD = 0;

  for (let iter = 0; iter < iterations; iter++) {
    // ランダムに3点を選択
    const i0 = Math.floor(Math.random() * n);
    let i1 = Math.floor(Math.random() * n);
    let i2 = Math.floor(Math.random() * n);
    if (i1 === i0) i1 = (i0 + 1) % n;
    if (i2 === i0 || i2 === i1) i2 = (i0 + 2) % n;

    const p0 = points[i0];
    const p1 = points[i1];
    const p2 = points[i2];

    // 2つのベクトル
    const v1 = [p1.x - p0.x, p1.y - p0.y, p1.z - p0.z];
    const v2 = [p2.x - p0.x, p2.y - p0.y, p2.z - p0.z];

    // 外積で法線を計算
    const nx = v1[1] * v2[2] - v1[2] * v2[1];
    const ny = v1[2] * v2[0] - v1[0] * v2[2];
    const nz = v1[0] * v2[1] - v1[1] * v2[0];

    const len = Math.sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 1e-10) continue;

    // 正規化
    const a = nx / len;
    const b = ny / len;
    const c = nz / len;
    const d = -(a * p0.x + b * p0.y + c * p0.z);

    // インライア判定
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

  // 全点の30%以上がインライアでなければ失敗
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
```

- [ ] **Step 2: Commit**

```bash
git add src/plane/ransac.ts
git commit -m "feat: add RANSAC plane detection from 3D point cloud"
```

---

### Task 2: 平面オーバーレイ描画モジュール

**Files:**
- Create: `src/visualization/plane-overlay.ts`

- [ ] **Step 1: `src/visualization/plane-overlay.ts` を作成**

```typescript
import { Point3D } from '../geometry/triangulation';

export class PlaneOverlay {
  private ctx: CanvasRenderingContext2D;

  constructor(ctx: CanvasRenderingContext2D) {
    this.ctx = ctx;
  }

  draw(inliers: Point3D[], R: number[][], t: number[], K: number[][]): void {
    // 3D → 2D 投影
    const projected: { x: number; y: number }[] = [];
    for (const p of inliers) {
      // P_cam = R × P_world + t
      const cx = R[0][0] * p.x + R[0][1] * p.y + R[0][2] * p.z + t[0];
      const cy = R[1][0] * p.x + R[1][1] * p.y + R[1][2] * p.z + t[1];
      const cz = R[2][0] * p.x + R[2][1] * p.y + R[2][2] * p.z + t[2];

      // カメラの後ろはスキップ
      if (cz <= 0) continue;

      // p_img = K × P_cam
      const u = (K[0][0] * cx + K[0][2] * cz) / cz;
      const v = (K[1][1] * cy + K[1][2] * cz) / cz;

      projected.push({ x: u, y: v });
    }

    if (projected.length < 3) return;

    // 凸包 (Graham scan)
    const hull = this.convexHull(projected);
    if (hull.length < 3) return;

    const ctx = this.ctx;

    // 半透明ポリゴン
    ctx.fillStyle = 'rgba(0, 120, 255, 0.3)';
    ctx.beginPath();
    ctx.moveTo(hull[0].x, hull[0].y);
    for (let i = 1; i < hull.length; i++) {
      ctx.lineTo(hull[i].x, hull[i].y);
    }
    ctx.closePath();
    ctx.fill();

    // 輪郭線
    ctx.strokeStyle = 'rgba(0, 120, 255, 0.7)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(hull[0].x, hull[0].y);
    for (let i = 1; i < hull.length; i++) {
      ctx.lineTo(hull[i].x, hull[i].y);
    }
    ctx.closePath();
    ctx.stroke();
  }

  private convexHull(points: { x: number; y: number }[]): { x: number; y: number }[] {
    const pts = [...points].sort((a, b) => a.x - b.x || a.y - b.y);
    const n = pts.length;
    if (n <= 2) return pts;

    function cross(O: { x: number; y: number }, A: { x: number; y: number }, B: { x: number; y: number }): number {
      return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
    }

    // 下側凸包
    const lower: { x: number; y: number }[] = [];
    for (const p of pts) {
      while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) {
        lower.pop();
      }
      lower.push(p);
    }

    // 上側凸包
    const upper: { x: number; y: number }[] = [];
    for (let i = n - 1; i >= 0; i--) {
      while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], pts[i]) <= 0) {
        upper.pop();
      }
      upper.push(pts[i]);
    }

    // 最初と最後は重複するので除去
    lower.pop();
    upper.pop();

    return lower.concat(upper);
  }
}
```

- [ ] **Step 2: Commit**

```bash
git add src/visualization/plane-overlay.ts
git commit -m "feat: add plane overlay with convex hull projection"
```

---

### Task 3: main.ts に平面検出 + オーバーレイを統合

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
import { estimatePosePnP } from './geometry/pnp';
import { triangulatePoints, Point3D } from './geometry/triangulation';
import { detectPlane, PlaneResult } from './plane/ransac';
import { SlamMap } from './slam/map';
import { PlaneOverlay } from './visualization/plane-overlay';
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
  const slamMap = new SlamMap();
  const pointCloudView = new PointCloudView(ctx, w, h);
  const planeOverlay = new PlaneOverlay(ctx);
  const K = calibration.getCameraMatrixAsMat();
  const Karray = calibration.getCameraMatrix();

  const offscreen = document.createElement('canvas');
  offscreen.width = w;
  offscreen.height = h;
  const offCtx = offscreen.getContext('2d')!;

  // 状態
  let initialized = false;
  let points3D: Point3D[] = [];
  let planeResult: PlaneResult | null = null;
  let currentR: number[][] | null = null;
  let currentT: number[] | null = null;
  const trajectory: { x: number; z: number }[] = [{ x: 0, z: 0 }];
  const MOTION_THRESHOLD = 15;

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

      if (!initialized) {
        // --- 未初期化: ホモグラフィ初期化 ---
        if (frameCount % 30 === 0) {
          console.log(`[SLAM] avgMotion=${result.avgMotion.toFixed(1)}, count=${result.count}`);
        }

        if (result.avgMotion > MOTION_THRESHOLD && result.count >= 30) {
          console.log(`[SLAM] initialization trigger: avgMotion=${result.avgMotion.toFixed(1)}`);

          const pose = estimatePose(result.prevPoints, result.points, K);

          if (pose) {
            const inlierIds: number[] = [];
            const inlierPrev: { x: number; y: number; id: number }[] = [];
            const inlierCurr: { x: number; y: number; id: number }[] = [];
            for (let i = 0; i < result.count; i++) {
              if (pose.inlierMask[i]) {
                inlierIds.push(result.ids[i]);
                inlierPrev.push(result.prevPoints[i]);
                inlierCurr.push(result.points[i]);
              }
            }

            points3D = triangulatePoints(
              inlierPrev, inlierCurr,
              pose.R, pose.t,
              Karray,
              inlierIds.map(() => true),
            );

            if (points3D.length > 10) {
              slamMap.register(inlierIds.slice(0, points3D.length), points3D);
              trajectory.push({ x: pose.t[0], z: pose.t[2] });
              currentR = pose.R;
              currentT = pose.t;
              initialized = true;
              console.log(`[SLAM] initialized! ${points3D.length} 3D points`);

              // 平面検出（初期化時に1回）
              planeResult = detectPlane(points3D);
            }
          }
        }
      } else {
        // --- 初期化済み: PnP 追跡 ---
        const { points3D: matched3D, points2D: matched2D } = slamMap.get3D2DPairs(result.ids, result.points);

        if (matched3D.length >= 6) {
          const pnpResult = estimatePosePnP(matched3D, matched2D, K);
          if (pnpResult) {
            currentR = pnpResult.R;
            currentT = pnpResult.t;
            trajectory.push({ x: pnpResult.t[0], z: pnpResult.t[2] });
            if (frameCount % 30 === 0) {
              console.log(`[SLAM] PnP: ${pnpResult.inlierCount}/${matched3D.length} inliers`);
            }
          }
        } else if (frameCount % 60 === 0) {
          console.log(`[SLAM] PnP: not enough matches (${matched3D.length})`);
        }
      }

      // 平面オーバーレイ描画
      if (planeResult && currentR && currentT) {
        planeOverlay.draw(planeResult.inliers, currentR, currentT, Karray);
      }

      // 点群 + 軌跡の可視化
      if (initialized) {
        pointCloudView.draw(points3D, trajectory);
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

Run: `pnpm vite` (または `npm run dev`)

Expected:
- Phase 4 と同じ追跡 + 鳥瞰図
- 初期化成功後、カメラ映像上に半透明の青いポリゴンが表示される
- カメラを動かすとポリゴンが姿勢に追従して移動・変形する
- vConsole に `[SLAM] plane detected: X/Y inliers` ログが表示される

- [ ] **Step 3: Commit**

```bash
git add src/main.ts
git commit -m "feat: integrate RANSAC plane detection with camera overlay"
```

---

## Summary

| Task | 内容 | 依存 |
|------|------|------|
| 1 | RANSAC 平面検出 | なし |
| 2 | 平面オーバーレイ描画 | なし |
| 3 | main.ts 統合 | Task 1, 2 |

Task 1 と Task 2 は独立で並列実行可能。Task 3 は全タスク完了後。
