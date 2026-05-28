# 3Dモデル配置機能 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 検出平面の中心に Three.js でキューブを配置し、カメラ姿勢に追従してARレンダリングする

**Architecture:** Three.js の `WebGLRenderer(alpha: true)` でカメラ映像 canvas の上に透過オーバーレイ。`PerspectiveCamera` のパラメータをカメラ行列 K から設定。毎フレーム PnP の R, t で Three.js カメラを更新。

**Tech Stack:** TypeScript, Three.js, Vite

---

## File Structure

| Action | Path | Responsibility |
|--------|------|----------------|
| Create | `src/rendering/ar-scene.ts` | Three.js シーン管理 + モデル配置 |
| Modify | `index.html` | Three.js 用 canvas 追加 |
| Modify | `src/main.ts` | ArScene 統合 |

---

### Task 1: Three.js インストール + index.html 更新

**Files:**
- Modify: `package.json`
- Modify: `index.html`

- [ ] **Step 1: Three.js をインストール**

```bash
npm install three --cache "$TMPDIR/npm-cache"
```

npm が壊れている場合は `pnpm add three` を使用。

- [ ] **Step 2: index.html に gl-canvas を追加**

`index.html` を以下に置き換え:

```html
<!DOCTYPE html>
<html lang="ja">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Web Plane Detection</title>
    <style>
      * { margin: 0; padding: 0; box-sizing: border-box; }
      body { background: #000; overflow: hidden; }
      #canvas {
        display: block; width: 100vw; height: 100vh; object-fit: cover;
        position: absolute; top: 0; left: 0;
      }
      #gl-canvas {
        display: block; width: 100vw; height: 100vh;
        position: absolute; top: 0; left: 0;
        pointer-events: none;
      }
      #video { display: none; }
      #loading {
        position: fixed; inset: 0; z-index: 10;
        display: flex; flex-direction: column; align-items: center; justify-content: center;
        color: #fff; font-family: sans-serif; font-size: 1.2rem;
        background: #000; gap: 16px;
      }
      #imu-btn {
        display: none;
        padding: 12px 24px;
        font-size: 1rem;
        background: #0078ff;
        color: #fff;
        border: none;
        border-radius: 8px;
        cursor: pointer;
        font-family: sans-serif;
      }
      #imu-btn:active { background: #005acc; }
    </style>
  </head>
  <body>
    <div id="loading">
      <span id="loading-text">OpenCV.js を読み込み中...</span>
      <button id="imu-btn">モーションセンサーを許可</button>
    </div>
    <video id="video" autoplay playsinline></video>
    <canvas id="canvas"></canvas>
    <canvas id="gl-canvas"></canvas>
    <script src="https://unpkg.com/vconsole@latest/dist/vconsole.min.js"></script>
    <script>new window.VConsole();</script>
    <script type="module" src="/src/main.ts"></script>
  </body>
</html>
```

- [ ] **Step 3: Commit**

```bash
git add package.json package-lock.json index.html
git commit -m "feat: add Three.js dependency and gl-canvas overlay"
```

---

### Task 2: ArScene モジュール

**Files:**
- Create: `src/rendering/ar-scene.ts`

- [ ] **Step 1: `src/rendering/ar-scene.ts` を作成**

```typescript
import * as THREE from 'three';
import { Point3D } from '../geometry/triangulation';

export class ArScene {
  private renderer: THREE.WebGLRenderer;
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private cube: THREE.Mesh | null = null;
  private _isModelPlaced = false;

  constructor(glCanvas: HTMLCanvasElement, width: number, height: number, K: number[][]) {
    // レンダラー（背景透過）
    this.renderer = new THREE.WebGLRenderer({
      canvas: glCanvas,
      alpha: true,
      antialias: true,
    });
    this.renderer.setSize(width, height);
    this.renderer.setClearColor(0x000000, 0);

    // シーン
    this.scene = new THREE.Scene();

    // ライト
    const ambient = new THREE.AmbientLight(0xffffff, 0.6);
    this.scene.add(ambient);
    const directional = new THREE.DirectionalLight(0xffffff, 0.8);
    directional.position.set(1, 2, 3);
    this.scene.add(directional);

    // カメラ（K から fov を計算）
    const fy = K[1][1];
    const fov = 2 * Math.atan(height / (2 * fy)) * (180 / Math.PI);
    this.camera = new THREE.PerspectiveCamera(fov, width / height, 0.01, 1000);
  }

  placeModel(planeInliers: Point3D[], planeNormal: number[]): void {
    if (this.cube) return; // 既に配置済み

    // 重心を計算
    let cx = 0, cy = 0, cz = 0;
    for (const p of planeInliers) {
      cx += p.x; cy += p.y; cz += p.z;
    }
    cx /= planeInliers.length;
    cy /= planeInliers.length;
    cz /= planeInliers.length;

    // キューブサイズ: インライア点の広がりの 10%
    let maxSpread = 0;
    for (const p of planeInliers) {
      const dx = p.x - cx;
      const dy = p.y - cy;
      const dz = p.z - cz;
      const dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
      if (dist > maxSpread) maxSpread = dist;
    }
    const cubeSize = Math.max(maxSpread * 0.1, 0.05);

    // キューブ作成
    const geometry = new THREE.BoxGeometry(cubeSize, cubeSize, cubeSize);
    const material = new THREE.MeshStandardMaterial({
      color: 0x00aaff,
      metalness: 0.3,
      roughness: 0.7,
    });
    this.cube = new THREE.Mesh(geometry, material);
    this.cube.position.set(cx, cy, cz);

    // 平面法線に合わせて回転（キューブの底面が平面に接するように）
    // キューブの Y 軸を平面法線に合わせる
    const up = new THREE.Vector3(planeNormal[0], planeNormal[1], planeNormal[2]).normalize();
    const defaultUp = new THREE.Vector3(0, 1, 0);
    const quat = new THREE.Quaternion().setFromUnitVectors(defaultUp, up);
    this.cube.quaternion.copy(quat);

    // 平面上に底面が接するように半分だけ持ち上げる
    this.cube.position.x += up.x * cubeSize / 2;
    this.cube.position.y += up.y * cubeSize / 2;
    this.cube.position.z += up.z * cubeSize / 2;

    this.scene.add(this.cube);
    this._isModelPlaced = true;

    console.log(`[AR] cube placed at (${cx.toFixed(2)}, ${cy.toFixed(2)}, ${cz.toFixed(2)}), size=${cubeSize.toFixed(3)}`);
  }

  render(R: number[][], t: number[]): void {
    if (!this._isModelPlaced) return;

    // OpenCV → Three.js 座標変換
    // OpenCV: X右, Y下, Z奥
    // Three.js: X右, Y上, Z手前
    // 変換: Y と Z を反転
    const mat = new THREE.Matrix4();
    mat.set(
      R[0][0],  -R[0][1],  -R[0][2],  t[0],
      -R[1][0],  R[1][1],   R[1][2],  -t[1],
      -R[2][0],  R[2][1],   R[2][2],  -t[2],
      0,         0,          0,         1,
    );

    // viewMatrix の逆行列 = カメラのワールド変換
    const viewMatrixInverse = mat.clone().invert();
    this.camera.matrixAutoUpdate = false;
    this.camera.matrix.copy(viewMatrixInverse);
    this.camera.matrixWorldNeedsUpdate = true;

    this.renderer.render(this.scene, this.camera);
  }

  get isModelPlaced(): boolean {
    return this._isModelPlaced;
  }
}
```

- [ ] **Step 2: Commit**

```bash
git add src/rendering/ar-scene.ts
git commit -m "feat: add ArScene with Three.js cube placement on detected plane"
```

---

### Task 3: main.ts に ArScene を統合

**Files:**
- Modify: `src/main.ts`

- [ ] **Step 1: import に ArScene を追加**

`src/main.ts` の import セクションに追加:

```typescript
import { ArScene } from './rendering/ar-scene';
```

- [ ] **Step 2: ArScene を初期化**

モジュール初期化セクション（`const mapper = new Mapper(...)` の後）に追加:

```typescript
  const glCanvas = document.getElementById('gl-canvas') as HTMLCanvasElement;
  glCanvas.width = w;
  glCanvas.height = h;
  const arScene = new ArScene(glCanvas, w, h, Karray);
```

- [ ] **Step 3: 平面検出成功時に placeModel を呼ぶ**

平面検出の行 (`planeResult = detectPlane(...)`) の後に追加:

```typescript
              // 平面上にキューブを配置
              if (planeResult) {
                arScene.placeModel(planeResult.inliers, planeResult.normal);
              }
```

- [ ] **Step 4: 毎フレームの描画に ArScene.render を追加**

既存の平面オーバーレイ描画の後に追加:

```typescript
      // AR レンダリング
      if (arScene.isModelPlaced && currentR && currentT) {
        arScene.render(currentR, currentT);
      }
```

- [ ] **Step 5: 動作確認**

Run: `pnpm vite`

Expected:
- カメラを動かして初期化 → 平面検出
- 検出平面の中心に青いキューブが表示される
- カメラを動かすとキューブが平面上に固定されているように見える

- [ ] **Step 6: Commit**

```bash
git add src/main.ts
git commit -m "feat: integrate ArScene for 3D cube rendering on detected plane"
```

---

## Summary

| Task | 内容 | 依存 |
|------|------|------|
| 1 | Three.js インストール + index.html | なし |
| 2 | ArScene モジュール | Task 1 |
| 3 | main.ts 統合 | Task 1, 2 |
