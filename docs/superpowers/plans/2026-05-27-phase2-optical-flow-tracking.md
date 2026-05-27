# Phase 2: オプティカルフロー追跡 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 特徴点をフレーム間で Lucas-Kanade オプティカルフローで追跡し、追跡マーカー + 移動ベクトルを描画する

**Architecture:** `FeatureTracker` が前後フレームのグレースケール画像を保持し、`cv.calcOpticalFlowPyrLK` で追跡。追跡失敗点を除外し、点数が閾値以下になったら `OrbDetector` で補充。描画は Canvas 2D API に移行し、映像とオーバーレイを分離。

**Tech Stack:** TypeScript, OpenCV.js (`@techstark/opencv-js`), Vite

---

## File Structure

| Action | Path | Responsibility |
|--------|------|----------------|
| Modify | `src/features/orb.ts` | `detectKeypoints()` メソッド追加 |
| Create | `src/features/tracker.ts` | LK追跡 + 特徴点管理 |
| Modify | `src/main.ts` | Tracker ベースの処理ループ + Canvas 2D 描画 |

---

### Task 1: OrbDetector に detectKeypoints メソッドを追加

**Files:**
- Modify: `src/features/orb.ts`

- [ ] **Step 1: `detectKeypoints` メソッドを追加**

`src/features/orb.ts` を以下の内容に置き換え:

```typescript
import cv from '@techstark/opencv-js';

export interface Point2D {
  x: number;
  y: number;
}

export class OrbDetector {
  private orb: cv.ORB;

  constructor(nfeatures: number = 500) {
    this.orb = new cv.ORB(nfeatures);
  }

  detectKeypoints(gray: cv.Mat): Point2D[] {
    const keypoints = new cv.KeyPointVector();
    this.orb.detect(gray, keypoints);

    const points: Point2D[] = [];
    for (let i = 0; i < keypoints.size(); i++) {
      const kp = keypoints.get(i);
      points.push({ x: kp.pt.x, y: kp.pt.y });
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
git commit -m "feat: add detectKeypoints method to OrbDetector"
```

---

### Task 2: FeatureTracker モジュールを作成

**Files:**
- Create: `src/features/tracker.ts`

- [ ] **Step 1: `src/features/tracker.ts` を作成**

```typescript
import cv from '@techstark/opencv-js';
import { OrbDetector, Point2D } from './orb';

export type TrackResult = {
  points: Point2D[];
  prevPoints: Point2D[];
  count: number;
};

export class FeatureTracker {
  private orb: OrbDetector;
  private minFeatures: number;
  private dedupDistance: number;
  private prevGray: cv.Mat | null = null;
  private prevPoints: Point2D[] = [];

  constructor(orb: OrbDetector, minFeatures: number = 200, dedupDistance: number = 20) {
    this.orb = orb;
    this.minFeatures = minFeatures;
    this.dedupDistance = dedupDistance;
  }

  process(frame: cv.Mat): TrackResult {
    const gray = new cv.Mat();
    cv.cvtColor(frame, gray, cv.COLOR_RGBA2GRAY);

    // 初回: ORB で初期特徴点を検出
    if (this.prevGray === null) {
      this.prevPoints = this.orb.detectKeypoints(gray);
      this.prevGray = gray;
      return {
        points: this.prevPoints,
        prevPoints: this.prevPoints,
        count: this.prevPoints.length,
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

      // 追跡成功した点だけ残す
      const statusData = statusMat.data;
      const nextData = nextPtsMat.data32F;
      for (let i = 0; i < this.prevPoints.length; i++) {
        if (statusData[i] === 1) {
          const nx = nextData[i * 2];
          const ny = nextData[i * 2 + 1];
          // 画像外に出た点は除外
          if (nx >= 0 && ny >= 0 && nx < frame.cols && ny < frame.rows) {
            trackedPoints.push({ x: nx, y: ny });
            trackedPrevPoints.push(this.prevPoints[i]);
          }
        }
      }

      prevPtsMat.delete();
      nextPtsMat.delete();
      statusMat.delete();
      errMat.delete();
    }

    // 特徴点が閾値以下なら ORB で補充
    if (trackedPoints.length < this.minFeatures) {
      const newPoints = this.orb.detectKeypoints(gray);
      const filtered = this.filterDuplicates(newPoints, trackedPoints);
      const needed = this.minFeatures - trackedPoints.length;
      const toAdd = filtered.slice(0, needed);
      for (const p of toAdd) {
        trackedPoints.push(p);
        trackedPrevPoints.push(p); // 新規点は移動ベクトルなし
      }
    }

    // 状態更新
    const oldGray = this.prevGray;
    this.prevGray = gray;
    this.prevPoints = trackedPoints;
    oldGray.delete();

    return {
      points: trackedPoints,
      prevPoints: trackedPrevPoints,
      count: trackedPoints.length,
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
git commit -m "feat: add FeatureTracker with LK optical flow tracking"
```

---

### Task 3: main.ts を Tracker ベースの処理ループに書き換え

**Files:**
- Modify: `src/main.ts`

- [ ] **Step 1: `src/main.ts` を以下の内容に置き換え**

```typescript
import cv from '@techstark/opencv-js';
import { initCamera } from './camera/capture';
import { OrbDetector } from './features/orb';
import { FeatureTracker } from './features/tracker';

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
  console.log('[SLAM] waiting for OpenCV.js...');
  await waitForOpenCv();
  console.log('[SLAM] OpenCV.js ready');

  loading.textContent = 'カメラを起動中...';
  console.log('[SLAM] initializing camera...');
  await initCamera(video);
  const w = video.videoWidth;
  const h = video.videoHeight;
  console.log(`[SLAM] camera ready: ${w}x${h}`);
  canvas.width = w;
  canvas.height = h;

  loading.style.display = 'none';

  // ORB 検出器 + Tracker
  const orb = new OrbDetector(500);
  const tracker = new FeatureTracker(orb, 200);
  console.log('[SLAM] tracker created');

  // offscreen canvas でフレーム取得
  const offscreen = document.createElement('canvas');
  offscreen.width = w;
  offscreen.height = h;
  const offCtx = offscreen.getContext('2d')!;

  console.log('[SLAM] starting tracking loop');

  let frameCount = 0;
  function processFrame() {
    try {
      // 映像フレーム取得
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
        // 同じ点（新規追加）はスキップ
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

      frameCount++;
      if (frameCount === 1) console.log(`[SLAM] first frame: ${result.count} points tracked`);
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

Expected: カメラ映像上に緑の円（追跡中の特徴点）と赤い線（移動ベクトル）が表示される。カメラを動かすと線が伸びる方向が変わる。静止していると線はほぼ表示されない。

- [ ] **Step 3: Commit**

```bash
git add src/main.ts
git commit -m "feat: integrate FeatureTracker with Canvas 2D rendering"
```

---

## Summary

| Task | 内容 | 依存 |
|------|------|------|
| 1 | OrbDetector に detectKeypoints 追加 | なし |
| 2 | FeatureTracker モジュール作成 | Task 1 |
| 3 | main.ts を Tracker ベースに書き換え | Task 1, 2 |

Task 1 → Task 2 → Task 3 の順序で実行。
