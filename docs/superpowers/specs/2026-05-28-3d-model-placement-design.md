# 3Dモデル配置機能: 検出平面上にキューブを表示

## 概要

検出した平面の中心に Three.js でキューブを配置し、カメラ姿勢に追従してレンダリングする。カメラ映像の上に Three.js のレンダリング結果を透過オーバーレイで重ねる。

## ゴール

- 平面検出成功時、検出平面の中心にキューブが自動配置される
- カメラを動かすとキューブが平面上に固定されているように見える（AR 体験）

## ファイル構成

```
src/
├── rendering/
│   └── ar-scene.ts          # 新規: Three.js シーン管理 + モデル配置
├── main.ts                  # 変更: ArScene 統合
index.html                   # 変更: Three.js canvas 追加
package.json                 # 変更: three を依存に追加
```

## モジュール設計

### `src/rendering/ar-scene.ts` (新規)

Three.js のシーン・カメラ・レンダラーを管理し、検出平面上にモデルを配置する。

**インターフェース:**

```typescript
class ArScene {
  constructor(glCanvas: HTMLCanvasElement, width: number, height: number, K: number[][])
  placeModel(planeInliers: Point3D[], planeNormal: number[]): void
  render(R: number[][], t: number[]): void
  get isModelPlaced(): boolean
}
```

**Three.js 構成:**

- `WebGLRenderer`: `alpha: true` で背景透過。`glCanvas` を直接渡す
- `PerspectiveCamera`: カメラ行列 K から fov を算出: `fov = 2 * atan(height / (2 * fy)) * 180 / PI`
- `Scene`: `AmbientLight` + `DirectionalLight` + キューブ (`BoxGeometry` + `MeshStandardMaterial`)
- キューブのサイズ: 平面インライア点の広がりの 10% 程度（自動スケーリング）

**カメラ姿勢の適用:**

Three.js のカメラ座標系と OpenCV の座標系は異なる:
- OpenCV: X右、Y下、Z奥
- Three.js: X右、Y上、Z手前

変換: Three.js の viewMatrix = `[R|t]` に Y, Z 軸の反転を適用。

```
Three.js camera matrix = [
  R[0][0],  -R[0][1],  -R[0][2],  t[0],
  -R[1][0],  R[1][1],   R[1][2],  -t[1],
  -R[2][0],  R[2][1],   R[2][2],  -t[2],
  0,         0,          0,         1
]
```

### `index.html` (変更)

Three.js 用の canvas を追加:

```html
<canvas id="canvas"></canvas>
<canvas id="gl-canvas"></canvas>
```

CSS で `gl-canvas` を `canvas` の上に絶対配置。`pointer-events: none` で操作を透過。

### `src/main.ts` (変更)

- `three` のインストール (`npm install three`)
- `ArScene` を初期化
- 平面検出成功時に `arScene.placeModel()` を呼ぶ
- 毎フレーム PnP 成功時に `arScene.render(R, t)` を呼ぶ

## データフロー

```
平面検出成功:
  planeResult.inliers → 重心計算 → ArScene.placeModel(inliers, normal)
  → キューブを重心位置に配置、法線方向に合わせて回転

毎フレーム (PnP 成功):
  R, t → ArScene.render(R, t)
  → Three.js カメラの position/rotation を更新
  → WebGLRenderer.render() → gl-canvas に描画
  → カメラ映像 canvas の上にオーバーレイ
```

## 実装上のポイント

### Three.js のインストール

```bash
npm install three
npm install --save-dev @types/three
```

`@types/three` は不要の場合もある（three には型定義が含まれている）。

### near/far クリッピング

3D 点群のスケールに合わせて `near` と `far` を設定。`near: 0.01`, `far: 1000` で十分。

### キューブのサイズ

平面インライア点の XZ 平面での広がり（バウンディングボックスの短辺）の 10% をキューブの辺の長さにする。

## スコープ外

- GLTF モデルの読み込み
- タップで配置位置を選択
- 複数モデルの配置
- シャドウ / 環境光マッピング
