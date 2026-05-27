# Phase 1: ORB特徴点リアルタイム抽出・可視化 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** カメラ映像上にORB特徴点をリアルタイム描画するブラウザデモを作成する

**Architecture:** Vite + TypeScript でビルド。`getUserMedia()` で背面カメラ映像を取得し、OpenCV.js (`@techstark/opencv-js`) の ORB でフレームごとに特徴点を検出、`cv.drawKeypoints()` + `cv.imshow()` で canvas 1枚に描画する。

**Tech Stack:** TypeScript, Vite, OpenCV.js (`@techstark/opencv-js`), npm

---

## File Structure

| Action | Path | Responsibility |
|--------|------|----------------|
| Create | `package.json` | npm dependencies & scripts |
| Create | `tsconfig.json` | TypeScript config |
| Create | `vite.config.ts` | Vite config (HTTPS, host) |
| Create | `index.html` | エントリーHTML (canvas + hidden video) |
| Create | `src/main.ts` | 初期化・処理ループ |
| Create | `src/camera/capture.ts` | getUserMedia ラッパー |
| Create | `src/features/orb.ts` | ORB特徴点抽出・描画 |

---

### Task 1: プロジェクト初期化 (Vite + TypeScript)

**Files:**
- Create: `package.json`
- Create: `tsconfig.json`
- Create: `vite.config.ts`
- Create: `index.html`
- Create: `src/main.ts` (placeholder)

- [ ] **Step 1: npm init + Vite & TypeScript インストール**

```bash
cd /Users/j1ngzoue/projects/web-plane-detection
npm init -y
npm install --save-dev vite typescript
npm install @techstark/opencv-js
npm install --save-dev @vitejs/plugin-basic-ssl
```

- [ ] **Step 2: tsconfig.json を作成**

```json
{
  "compilerOptions": {
    "target": "ES2020",
    "module": "ESNext",
    "moduleResolution": "bundler",
    "strict": true,
    "esModuleInterop": true,
    "skipLibCheck": true,
    "forceConsistentCasingInFileNames": true,
    "outDir": "./dist",
    "sourceMap": true
  },
  "include": ["src"]
}
```

- [ ] **Step 3: vite.config.ts を作成**

```typescript
import { defineConfig } from 'vite';
import basicSsl from '@vitejs/plugin-basic-ssl';

export default defineConfig({
  plugins: [basicSsl()],
  server: {
    host: true,
  },
});
```

- [ ] **Step 4: index.html を作成**

```html
<!DOCTYPE html>
<html lang="ja">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Web Plane Detection - Phase 1</title>
    <style>
      * { margin: 0; padding: 0; box-sizing: border-box; }
      body { background: #000; overflow: hidden; }
      #canvas { display: block; width: 100vw; height: 100vh; object-fit: cover; }
      #video { display: none; }
      #loading {
        position: fixed; inset: 0;
        display: flex; align-items: center; justify-content: center;
        color: #fff; font-family: sans-serif; font-size: 1.2rem;
        background: #000;
      }
    </style>
  </head>
  <body>
    <div id="loading">OpenCV.js を読み込み中...</div>
    <video id="video" autoplay playsinline></video>
    <canvas id="canvas"></canvas>
    <script type="module" src="/src/main.ts"></script>
  </body>
</html>
```

- [ ] **Step 5: src/main.ts に placeholder を作成**

```typescript
console.log('Phase 1: ORB Feature Extraction');
```

- [ ] **Step 6: package.json に scripts を追加**

`package.json` の `"scripts"` を以下に変更:

```json
{
  "scripts": {
    "dev": "vite",
    "build": "tsc && vite build",
    "preview": "vite preview"
  }
}
```

- [ ] **Step 7: 動作確認**

Run: `npm run dev`

Expected: Vite の開発サーバーが起動し、ブラウザで「OpenCV.js を読み込み中...」が表示される。コンソールに `Phase 1: ORB Feature Extraction` が出力される。

Ctrl+C でサーバーを停止。

- [ ] **Step 8: Commit**

```bash
git add package.json package-lock.json tsconfig.json vite.config.ts index.html src/main.ts
git commit -m "feat: initialize Vite + TypeScript project for Phase 1"
```

---

### Task 2: カメラキャプチャモジュール

**Files:**
- Create: `src/camera/capture.ts`

- [ ] **Step 1: src/camera/capture.ts を作成**

```typescript
export async function initCamera(video: HTMLVideoElement): Promise<void> {
  const stream = await navigator.mediaDevices.getUserMedia({
    video: {
      facingMode: 'environment',
      width: { ideal: 640 },
      height: { ideal: 480 },
    },
    audio: false,
  });

  video.srcObject = stream;

  return new Promise((resolve) => {
    video.onloadedmetadata = () => {
      video.play();
      resolve();
    };
  });
}
```

- [ ] **Step 2: main.ts からカメラを初期化して映像が取れることを確認**

`src/main.ts` を以下に置き換え:

```typescript
import { initCamera } from './camera/capture';

async function main() {
  const video = document.getElementById('video') as HTMLVideoElement;
  const canvas = document.getElementById('canvas') as HTMLCanvasElement;
  const loading = document.getElementById('loading') as HTMLDivElement;

  // カメラ初期化
  await initCamera(video);
  canvas.width = video.videoWidth;
  canvas.height = video.videoHeight;

  loading.style.display = 'none';

  // 仮: canvas に映像だけ描画して確認
  const ctx = canvas.getContext('2d')!;
  function draw() {
    ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
    requestAnimationFrame(draw);
  }
  requestAnimationFrame(draw);
}

main().catch((err) => {
  console.error('初期化エラー:', err);
  const loading = document.getElementById('loading');
  if (loading) loading.textContent = `エラー: ${err.message}`;
});
```

- [ ] **Step 3: 動作確認**

Run: `npm run dev`

Expected: ブラウザでカメラ映像が canvas 上にリアルタイム表示される。モバイル実機では背面カメラが使用される。

Ctrl+C でサーバーを停止。

- [ ] **Step 4: Commit**

```bash
git add src/camera/capture.ts src/main.ts
git commit -m "feat: add camera capture module with getUserMedia"
```

---

### Task 3: ORB特徴点抽出モジュール

**Files:**
- Create: `src/features/orb.ts`

- [ ] **Step 1: src/features/orb.ts を作成**

```typescript
import cv from '@techstark/opencv-js';

export class OrbDetector {
  private orb: cv.ORB;

  constructor(nfeatures: number = 500) {
    this.orb = new cv.ORB(nfeatures);
  }

  detectAndDraw(frame: cv.Mat): void {
    const gray = new cv.Mat();
    cv.cvtColor(frame, gray, cv.COLOR_RGBA2GRAY);

    const keypoints = new cv.KeyPointVector();
    const descriptors = new cv.Mat();

    this.orb.detect(gray, keypoints);

    cv.drawKeypoints(frame, keypoints, frame);

    gray.delete();
    keypoints.delete();
    descriptors.delete();
  }

  dispose(): void {
    this.orb.delete();
  }
}
```

- [ ] **Step 2: Commit**

```bash
git add src/features/orb.ts
git commit -m "feat: add ORB feature detector module"
```

---

### Task 4: メインループ統合 — OpenCV.js ロード + ORB 処理ループ

**Files:**
- Modify: `src/main.ts`

- [ ] **Step 1: src/main.ts を OpenCV.js + ORB 統合版に書き換え**

```typescript
import cv from '@techstark/opencv-js';
import { initCamera } from './camera/capture';
import { OrbDetector } from './features/orb';

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

  // OpenCV.js WASM ロード待ち
  loading.textContent = 'OpenCV.js を読み込み中...';
  await waitForOpenCv();

  // カメラ初期化
  loading.textContent = 'カメラを起動中...';
  await initCamera(video);
  canvas.width = video.videoWidth;
  canvas.height = video.videoHeight;

  loading.style.display = 'none';

  // ORB 検出器
  const detector = new OrbDetector(500);

  // VideoCapture
  const cap = new cv.VideoCapture(video);
  const frame = new cv.Mat(video.videoHeight, video.videoWidth, cv.CV_8UC4);

  function processFrame() {
    cap.read(frame);
    detector.detectAndDraw(frame);
    cv.imshow(canvas, frame);
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

Run: `npm run dev`

Expected: カメラ映像上に緑色の ORB 特徴点マーカーがリアルタイムで描画される。特徴の多いテクスチャ（本の表紙、キーボードなど）に向けるとマーカーが多く表示される。

Ctrl+C でサーバーを停止。

- [ ] **Step 3: Commit**

```bash
git add src/main.ts
git commit -m "feat: integrate OpenCV.js ORB detection with camera feed"
```

---

### Task 5: .gitignore とクリーンアップ

**Files:**
- Create: `.gitignore`

- [ ] **Step 1: .gitignore を作成**

```
node_modules/
dist/
.superpowers/
*.local
```

- [ ] **Step 2: Commit**

```bash
git add .gitignore
git commit -m "chore: add .gitignore"
```

---

## Summary

| Task | 内容 | 依存 |
|------|------|------|
| 1 | プロジェクト初期化 (Vite + TS) | なし |
| 2 | カメラキャプチャモジュール | Task 1 |
| 3 | ORB特徴点抽出モジュール | Task 1 |
| 4 | メインループ統合 | Task 2, 3 |
| 5 | .gitignore とクリーンアップ | Task 4 |

Task 2 と Task 3 は独立しており並列実行可能。
