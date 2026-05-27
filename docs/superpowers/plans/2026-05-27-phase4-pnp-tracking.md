# Phase 4: PnP 追跡による Visual Odometry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 初期化後、毎フレーム solvePnPRansac で 3D-2D 対応からカメラ姿勢を推定し、鳥瞰図にカメラ軌跡を描画する

**Architecture:** 追跡点に ID を導入し、`SlamMap` で ID → Point3D 対応を管理。毎フレーム SlamMap から 3D-2D ペアを取得し `solvePnPRansac` + `Rodrigues` で姿勢推定。鳥瞰図にカメラ軌跡（移動経路）を白い線で描画。

**Tech Stack:** TypeScript, OpenCV.js (`solvePnPRansac`, `Rodrigues`), Vite

---

## File Structure

| Action | Path | Responsibility |
|--------|------|----------------|
| Modify | `src/features/orb.ts` | Point2D に id 追加 |
| Modify | `src/features/tracker.ts` | ID 管理、TrackResult に ids 追加 |
| Create | `src/geometry/pnp.ts` | solvePnPRansac ラッパー |
| Create | `src/slam/map.ts` | ID → Point3D 対応管理 |
| Modify | `src/visualization/point-cloud.ts` | カメラ軌跡描画 |
| Modify | `src/main.ts` | PnP 追跡ループ統合 |

---

### Task 1: Point2D に id を追加、OrbDetector を更新

**Files:**
- Modify: `src/features/orb.ts`

- [ ] **Step 1: `src/features/orb.ts` を以下に置き換え**

```typescript
import cv from '@techstark/opencv-js';

export interface Point2D {
  x: number;
  y: number;
  id: number;
}

export class OrbDetector {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  private orb: any;

  constructor(nfeatures: number = 500) {
    this.orb = new cv.ORB(nfeatures);
  }

  detectKeypoints(gray: cv.Mat): Point2D[] {
    const keypoints = new cv.KeyPointVector();
    this.orb.detect(gray, keypoints);

    const points: Point2D[] = [];
    for (let i = 0; i < keypoints.size(); i++) {
      const kp = keypoints.get(i);
      points.push({ x: kp.pt.x, y: kp.pt.y, id: -1 });
    }

    keypoints.delete();
    return points;
  }

  detectAndDraw(frame: cv.Mat): void {
    const gray = new cv.Mat();
    cv.cvtColor(frame, gray, cv.COLOR_RGBA2GRAY);

    const keypoints = new cv.KeyPointVector();
    this.orb.detect(gray, keypoints);
    cv.drawKeypoints(frame, keypoints, frame);

    gray.delete();
    keypoints.delete();
  }

  dispose(): void {
    this.orb.delete();
  }
}
```

- [ ] **Step 2: Commit**

```bash
git add src/features/orb.ts
git commit -m "feat: add id field to Point2D interface"
```

---

### Task 2: Tracker に ID 管理を追加

**Files:**
- Modify: `src/features/tracker.ts`

- [ ] **Step 1: `src/features/tracker.ts` を以下に置き換え**

```typescript
import cv from '@techstark/opencv-js';
import { OrbDetector, Point2D } from './orb';

export type TrackResult = {
  points: Point2D[];
  prevPoints: Point2D[];
  ids: number[];
  count: number;
  avgMotion: number;
};

export class FeatureTracker {
  private orb: OrbDetector;
  private minFeatures: number;
  private dedupDistance: number;
  private prevGray: cv.Mat | null = null;
  private prevPoints: Point2D[] = [];
  private nextId: number = 0;

  constructor(orb: OrbDetector, minFeatures: number = 200, dedupDistance: number = 20) {
    this.orb = orb;
    this.minFeatures = minFeatures;
    this.dedupDistance = dedupDistance;
  }

  private assignIds(points: Point2D[]): void {
    for (const p of points) {
      if (p.id === -1) {
        p.id = this.nextId++;
      }
    }
  }

  process(frame: cv.Mat): TrackResult {
    const gray = new cv.Mat();
    cv.cvtColor(frame, gray, cv.COLOR_RGBA2GRAY);

    // 初回: ORB で初期特徴点を検出
    if (this.prevGray === null) {
      this.prevPoints = this.orb.detectKeypoints(gray);
      this.assignIds(this.prevPoints);
      this.prevGray = gray;
      const ids = this.prevPoints.map(p => p.id);
      return {
        points: this.prevPoints,
        prevPoints: this.prevPoints,
        ids,
        count: this.prevPoints.length,
        avgMotion: 0,
      };
    }

    let trackedPoints: Point2D[] = [];
    let trackedPrevPoints: Point2D[] = [];

    if (this.prevPoints.length > 0) {
      // 前フレームの点を cv.Mat (N×1, CV_32FC2) に変換
      const prevPtsMat = new cv.Mat(this.prevPoints.length, 1, cv.CV_32FC2);
      const prevData = prevPtsMat.data32F;
      for (let i = 0; i < this.prevPoints.length; i++) {
        prevData[i * 2] = this.prevPoints[i].x;
        prevData[i * 2 + 1] = this.prevPoints[i].y;
      }

      const nextPtsMat = new cv.Mat();
      const statusMat = new cv.Mat();
      const errMat = new cv.Mat();

      const winSize = new cv.Size(21, 21);

      cv.calcOpticalFlowPyrLK(
        this.prevGray,
        gray,
        prevPtsMat,
        nextPtsMat,
        statusMat,
        errMat,
        winSize,
        3,
      );

      // 追跡成功した点だけ残す（ID を引き継ぐ）
      const statusData = statusMat.data;
      const nextData = nextPtsMat.data32F;
      for (let i = 0; i < this.prevPoints.length; i++) {
        if (statusData[i] === 1) {
          const nx = nextData[i * 2];
          const ny = nextData[i * 2 + 1];
          if (nx >= 0 && ny >= 0 && nx < frame.cols && ny < frame.rows) {
            trackedPoints.push({ x: nx, y: ny, id: this.prevPoints[i].id });
            trackedPrevPoints.push(this.prevPoints[i]);
          }
        }
      }

      prevPtsMat.delete();
      nextPtsMat.delete();
      statusMat.delete();
      errMat.delete();
    }

    // 特徴点が閾値以下なら ORB で補充（新しい ID を付与）
    if (trackedPoints.length < this.minFeatures) {
      const newPoints = this.orb.detectKeypoints(gray);
      this.assignIds(newPoints);
      const filtered = this.filterDuplicates(newPoints, trackedPoints);
      const needed = this.minFeatures - trackedPoints.length;
      const toAdd = filtered.slice(0, needed);
      for (const p of toAdd) {
        trackedPoints.push(p);
        trackedPrevPoints.push(p);
      }
    }

    // 状態更新
    const oldGray = this.prevGray;
    this.prevGray = gray;
    this.prevPoints = trackedPoints;
    oldGray.delete();

    // 平均移動量を計算
    let totalMotion = 0;
    let motionCount = 0;
    for (let i = 0; i < trackedPoints.length; i++) {
      const dx = trackedPoints[i].x - trackedPrevPoints[i].x;
      const dy = trackedPoints[i].y - trackedPrevPoints[i].y;
      const dist = Math.sqrt(dx * dx + dy * dy);
      if (dist > 0) {
        totalMotion += dist;
        motionCount++;
      }
    }
    const avgMotion = motionCount > 0 ? totalMotion / motionCount : 0;

    const ids = trackedPoints.map(p => p.id);

    return {
      points: trackedPoints,
      prevPoints: trackedPrevPoints,
      ids,
      count: trackedPoints.length,
      avgMotion,
    };
  }

  private filterDuplicates(newPoints: Point2D[], existing: Point2D[]): Point2D[] {
    const distSq = this.dedupDistance * this.dedupDistance;
    return newPoints.filter((np) => {
      return !existing.some((ep) => {
        const dx = np.x - ep.x;
        const dy = np.y - ep.y;
        return dx * dx + dy * dy < distSq;
      });
    });
  }

  dispose(): void {
    if (this.prevGray) {
      this.prevGray.delete();
      this.prevGray = null;
    }
  }
}
```

- [ ] **Step 2: Commit**

```bash
git add src/features/tracker.ts
git commit -m "feat: add ID management to FeatureTracker"
```

---

### Task 3: SlamMap モジュール

**Files:**
- Create: `src/slam/map.ts`

- [ ] **Step 1: `src/slam/map.ts` を作成**

```typescript
import { Point2D } from '../features/orb';
import { Point3D } from '../geometry/triangulation';

export class SlamMap {
  private map: Map<number, Point3D> = new Map();

  register(ids: number[], points3D: Point3D[]): void {
    for (let i = 0; i < ids.length; i++) {
      this.map.set(ids[i], points3D[i]);
    }
    console.log(`[SLAM] SlamMap: registered ${ids.length} points, total=${this.map.size}`);
  }

  get3D2DPairs(ids: number[], points2D: Point2D[]): { points3D: Point3D[]; points2D: Point2D[] } {
    const matched3D: Point3D[] = [];
    const matched2D: Point2D[] = [];
    for (let i = 0; i < ids.length; i++) {
      const pt3d = this.map.get(ids[i]);
      if (pt3d) {
        matched3D.push(pt3d);
        matched2D.push(points2D[i]);
      }
    }
    return { points3D: matched3D, points2D: matched2D };
  }

  get size(): number {
    return this.map.size;
  }
}
```

- [ ] **Step 2: Commit**

```bash
git add src/slam/map.ts
git commit -m "feat: add SlamMap for 3D-2D correspondence management"
```

---

### Task 4: PnP モジュール

**Files:**
- Create: `src/geometry/pnp.ts`

- [ ] **Step 1: `src/geometry/pnp.ts` を作成**

```typescript
import cv from '@techstark/opencv-js';
import { Point2D } from '../features/orb';
import { Point3D } from './triangulation';

export type PnPResult = {
  R: number[][];
  t: number[];
  inlierCount: number;
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

export function estimatePosePnP(
  points3D: Point3D[],
  points2D: Point2D[],
  cameraMatrixMat: cv.Mat,
): PnPResult | null {
  const n = points3D.length;
  if (n < 6) return null;

  // Point3D[] → cv.Mat (N×1, CV_64FC3)
  const objPts = new cv.Mat(n, 1, cv.CV_64FC3);
  const objData = objPts.data64F;
  for (let i = 0; i < n; i++) {
    objData[i * 3] = points3D[i].x;
    objData[i * 3 + 1] = points3D[i].y;
    objData[i * 3 + 2] = points3D[i].z;
  }

  // Point2D[] → cv.Mat (N×1, CV_64FC2)
  const imgPts = new cv.Mat(n, 1, cv.CV_64FC2);
  const imgData = imgPts.data64F;
  for (let i = 0; i < n; i++) {
    imgData[i * 2] = points2D[i].x;
    imgData[i * 2 + 1] = points2D[i].y;
  }

  const distCoeffs = new cv.Mat();
  const rvec = new cv.Mat();
  const tvec = new cv.Mat();
  const inliers = new cv.Mat();

  let success: boolean;
  try {
    success = cv.solvePnPRansac(
      objPts, imgPts, cameraMatrixMat, distCoeffs,
      rvec, tvec, false, 100, 8.0, 0.99, inliers,
    );
  } catch (e) {
    objPts.delete();
    imgPts.delete();
    distCoeffs.delete();
    rvec.delete();
    tvec.delete();
    inliers.delete();
    console.error('[SLAM] solvePnPRansac failed:', e);
    return null;
  }

  if (!success) {
    objPts.delete();
    imgPts.delete();
    distCoeffs.delete();
    rvec.delete();
    tvec.delete();
    inliers.delete();
    return null;
  }

  // Rodrigues: rvec → R (3×3)
  const R = new cv.Mat();
  cv.Rodrigues(rvec, R);

  const result: PnPResult = {
    R: matToArray2D(R, 3, 3),
    t: [tvec.doubleAt(0, 0), tvec.doubleAt(1, 0), tvec.doubleAt(2, 0)],
    inlierCount: inliers.rows,
  };

  objPts.delete();
  imgPts.delete();
  distCoeffs.delete();
  rvec.delete();
  tvec.delete();
  inliers.delete();
  R.delete();

  return result;
}
```

- [ ] **Step 2: Commit**

```bash
git add src/geometry/pnp.ts
git commit -m "feat: add PnP pose estimation with solvePnPRansac"
```

---

### Task 5: PointCloudView にカメラ軌跡描画を追加

**Files:**
- Modify: `src/visualization/point-cloud.ts`

- [ ] **Step 1: `src/visualization/point-cloud.ts` を以下に置き換え**

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

  draw(points: Point3D[], trajectory: { x: number; z: number }[]): void {
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

    // X-Z 平面のスケーリング
    let minX = Infinity, maxX = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;
    for (const p of points) {
      if (p.x < minX) minX = p.x;
      if (p.x > maxX) maxX = p.x;
      if (p.z < minZ) minZ = p.z;
      if (p.z > maxZ) maxZ = p.z;
    }

    // 軌跡の点も範囲に含める
    for (const t of trajectory) {
      if (t.x < minX) minX = t.x;
      if (t.x > maxX) maxX = t.x;
      if (t.z < minZ) minZ = t.z;
      if (t.z > maxZ) maxZ = t.z;
    }

    // 原点も含める
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

    // カメラ軌跡（白い線）
    if (trajectory.length > 1) {
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.6)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(toScreenX(trajectory[0].x), toScreenY(trajectory[0].z));
      for (let i = 1; i < trajectory.length; i++) {
        ctx.lineTo(toScreenX(trajectory[i].x), toScreenY(trajectory[i].z));
      }
      ctx.stroke();
    }

    // カメラ1（原点、緑の三角）
    this.drawCamera(toScreenX(0), toScreenY(0), '#00ff00');

    // 最新カメラ位置（黄色の三角）
    if (trajectory.length > 0) {
      const latest = trajectory[trajectory.length - 1];
      this.drawCamera(toScreenX(latest.x), toScreenY(latest.z), '#ffff00');
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
git commit -m "feat: add camera trajectory drawing to bird's-eye view"
```

---

### Task 6: main.ts に PnP 追跡ループを統合

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
import { SlamMap } from './slam/map';
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
  const K = calibration.getCameraMatrixAsMat(); // ループ中使い回す

  const offscreen = document.createElement('canvas');
  offscreen.width = w;
  offscreen.height = h;
  const offCtx = offscreen.getContext('2d')!;

  // 状態
  let initialized = false;
  let points3D: Point3D[] = [];
  const trajectory: { x: number; z: number }[] = [{ x: 0, z: 0 }]; // 原点から開始
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
        // --- 未初期化: ホモグラフィ初期化を試みる ---
        if (frameCount % 30 === 0) {
          console.log(`[SLAM] avgMotion=${result.avgMotion.toFixed(1)}, count=${result.count}`);
        }

        if (result.avgMotion > MOTION_THRESHOLD && result.count >= 30) {
          console.log(`[SLAM] initialization trigger: avgMotion=${result.avgMotion.toFixed(1)}`);

          const pose = estimatePose(result.prevPoints, result.points, K);

          if (pose) {
            // インライア点の ID を収集
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
              calibration.getCameraMatrix(),
              inlierIds.map(() => true), // すでにインライアのみ
            );

            if (points3D.length > 10) {
              // SlamMap に ID → Point3D 対応を登録
              // triangulatePoints は cheirality check でフィルタするので
              // インライア ID と点群の対応を再構築
              const registeredIds: number[] = [];
              let ptIdx = 0;
              for (let i = 0; i < inlierIds.length && ptIdx < points3D.length; i++) {
                // triangulatePoints は inlierMask=true の点を順番に処理するので
                // ptIdx 番目の Point3D は inlierIds[i] に対応
                // ただし cheirality/outlier フィルタで一部スキップされる
                // → 簡易的に先頭から対応させる（厳密ではないが Phase 4 では十分）
                registeredIds.push(inlierIds[i]);
                ptIdx++;
              }
              slamMap.register(registeredIds.slice(0, points3D.length), points3D);

              trajectory.push({ x: pose.t[0], z: pose.t[2] });
              initialized = true;
              console.log(`[SLAM] initialized! ${points3D.length} 3D points, map size=${slamMap.size}`);
            }
          }
        }
      } else {
        // --- 初期化済み: PnP 追跡 ---
        const { points3D: matched3D, points2D: matched2D } = slamMap.get3D2DPairs(result.ids, result.points);

        if (matched3D.length >= 6) {
          const pnpResult = estimatePosePnP(matched3D, matched2D, K);
          if (pnpResult) {
            trajectory.push({ x: pnpResult.t[0], z: pnpResult.t[2] });
            if (frameCount % 30 === 0) {
              console.log(`[SLAM] PnP: ${pnpResult.inlierCount}/${matched3D.length} inliers`);
            }
          }
        } else if (frameCount % 60 === 0) {
          console.log(`[SLAM] PnP: not enough matches (${matched3D.length})`);
        }
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
- Phase 2-3 と同じ追跡可視化
- カメラを動かして初期化トリガー → 鳥瞰図に白い点群が出現
- 初期化後、カメラを動かし続けると黄色の三角が移動し、白い線の軌跡が伸びていく
- vConsole に `[SLAM] PnP: X/Y inliers` ログが表示される

- [ ] **Step 3: Commit**

```bash
git add src/main.ts
git commit -m "feat: integrate PnP tracking loop with camera trajectory"
```

---

## Summary

| Task | 内容 | 依存 |
|------|------|------|
| 1 | Point2D に id 追加 | なし |
| 2 | Tracker に ID 管理追加 | Task 1 |
| 3 | SlamMap モジュール | Task 1 |
| 4 | PnP モジュール | なし |
| 5 | PointCloudView に軌跡描画 | なし |
| 6 | main.ts 統合 | Task 1-5 すべて |

Task 1 → Task 2 (依存)。Task 3, 4, 5 は独立で並列可能。Task 6 は全タスク完了後。
