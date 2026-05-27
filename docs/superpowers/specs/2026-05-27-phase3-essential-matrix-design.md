# Phase 3: 基本行列推定と初期3D点群復元

## 概要

Phase 2 の追跡点ペアから基本行列 E を推定し、カメラの回転 R と並進 t を復元する。対応点を三角測量して初期3D点群を構築し、鳥瞰図で可視化する。

## ゴール

- カメラを十分に動かすと自動的に初期化が実行され、3D点群が鳥瞰図オーバーレイに表示される

## ファイル構成

```
src/
├── camera/
│   ├── capture.ts           # 既存（変更なし）
│   └── calibration.ts       # 新規: カメラ内部パラメータ管理
├── features/
│   ├── orb.ts               # 既存（変更なし）
│   └── tracker.ts           # 変更: 対応点ペアの参照フレーム保持
├── geometry/
│   ├── essential.ts         # 新規: 基本行列推定 + R,t 復元
│   └── triangulation.ts     # 新規: 三角測量で3D点群生成
├── visualization/
│   └── point-cloud.ts       # 新規: 3D点群の2D簡易プロジェクション描画
└── main.ts                  # 変更: 初期化フロー + 点群可視化を統合
```

## モジュール設計

### `src/camera/calibration.ts` (新規)

カメラ内部パラメータを管理する。

- `fx = fy = width * 0.9`, `cx = width / 2`, `cy = height / 2` で近似
- `getCameraMatrix()`: 3×3 のカメラ行列を `number[][]` で返す
- `getCameraMatrixAsMat()`: OpenCV.js の `cv.Mat` (3×3, CV_64FC1) として返す
- `getFocalLength()`: fx の値を返す（`findEssentialMat` に必要）
- `getPrincipalPoint()`: `{ x: cx, y: cy }` を返す（`findEssentialMat` に必要）

### `src/geometry/essential.ts` (新規)

2フレーム間の対応点から基本行列を推定し、R, t を復元する。

**インターフェース:**

```typescript
type PoseResult = {
  R: number[][];      // 3×3 回転行列
  t: number[];        // 3×1 並進ベクトル
  inlierCount: number;
  inlierMask: boolean[];
};

function estimatePose(
  prevPoints: Point2D[],
  currPoints: Point2D[],
  focalLength: number,
  principalPoint: { x: number; y: number },
): PoseResult | null;
```

- `cv.findEssentialMat` で E を推定（RANSAC、閾値 1.0）
- `cv.recoverPose` で E から R, t を復元
- インライア数が少なすぎる場合（全体の 30% 未満）は `null` を返す
- cv.Mat の結果を `number[][]` / `number[]` に変換して返す（OpenCV.js 依存を閉じ込める）

### `src/geometry/triangulation.ts` (新規)

対応点と R, t から3D点群を三角測量で復元する。DLT (Direct Linear Transform) 法で自前実装。

**インターフェース:**

```typescript
type Point3D = {
  x: number;
  y: number;
  z: number;
};

function triangulatePoints(
  prevPoints: Point2D[],
  currPoints: Point2D[],
  R: number[][],
  t: number[],
  cameraMatrix: number[][],
  inlierMask: boolean[],
): Point3D[];
```

- フレーム1 の投影行列 P1 = K × [I | 0]
- フレーム2 の投影行列 P2 = K × [R | t]
- 各対応点ペアに対して DLT で 3D 座標を計算
- cheirality check: 両カメラの前方にある点のみ採用（z > 0）
- 外れ値フィルタ: カメラからの距離が極端に大きい点（中央値の 10 倍超）を除外

### `src/visualization/point-cloud.ts` (新規)

3D点群をキャンバス上の鳥瞰図オーバーレイとして描画する。

**インターフェース:**

```typescript
class PointCloudView {
  constructor(ctx: CanvasRenderingContext2D, canvasWidth: number, canvasHeight: number)
  draw(points: Point3D[], cameraR?: number[][], cameraT?: number[]): void
}
```

- キャンバス右下に 200×200px の半透明背景領域を描画
- X-Z平面（鳥瞰図）に3D点群をプロット（白い点）
- カメラ位置を緑の三角で表示
- 自動スケーリング（点群の範囲に合わせて表示倍率を調整）

### `src/features/tracker.ts` (変更)

`TrackResult` に「参照フレームからの累積移動量」を追加して、初期化トリガーの判定に使う。

- `TrackResult` に `avgMotion: number` を追加（全追跡点の平均移動距離）

### `src/main.ts` (変更)

初期化フローを追加:

1. Phase 2 の追跡ループを継続
2. `result.avgMotion > 50` で初期化トリガー
3. `estimatePose()` で R, t を推定
4. 成功したら `triangulatePoints()` で3D点群を復元
5. `PointCloudView.draw()` で鳥瞰図を描画
6. 初期化済みフラグで再初期化を防止

## データフロー

```
追跡ループ中:
  tracker.process(frame) → TrackResult { points, prevPoints, avgMotion }
       ↓
  avgMotion > 50px ?
    NO  → 追跡可視化のみ（Phase 2 と同じ）
    YES → 初期化開始:
      1. estimatePose(prevPoints, points, focal, pp) → PoseResult { R, t, inlierMask }
      2. PoseResult が null → 初期化失敗、追跡継続
      3. triangulatePoints(prevPoints, points, R, t, K, inlierMask) → Point3D[]
      4. PointCloudView.draw(points3D, R, t) → 鳥瞰図オーバーレイ
      5. 初期化済みフラグ = true
```

## 実装上のポイント

### findEssentialMat のパラメータ

```typescript
cv.findEssentialMat(
  pts1,           // cv.Mat (N×1, CV_32FC2)
  pts2,           // cv.Mat (N×1, CV_32FC2)
  focal,          // number (fx)
  pp,             // cv.Point (cx, cy)
  cv.RANSAC,      // method
  0.999,          // probability
  1.0,            // threshold (pixels)
  mask,           // output inlier mask
);
```

### DLT 三角測量

各対応点 (x1, x2) に対して 4×4 の行列 A を構築:
```
A[0] = x1.x * P1[2] - P1[0]
A[1] = x1.y * P1[2] - P1[1]
A[2] = x2.x * P2[2] - P2[0]
A[3] = x2.y * P2[2] - P2[1]
```
A の SVD の最小特異値に対応するベクトルが同次座標。w で割って 3D 座標を得る。

SVD は小さな 4×4 行列なので自前実装（Jacobi法）またはシンプルな反復法で実装可能。

### メモリ管理

- `findEssentialMat` / `recoverPose` に渡す cv.Mat は関数内で生成・解放
- 結果は JavaScript のプレーンオブジェクトに変換して返す（OpenCV.js 依存を `essential.ts` 内に閉じ込める）

## スコープ外

- PnP による継続フレームの姿勢推定（Phase 4）
- IMU 融合（Phase 5-6）
- バンドル調整（Phase 7）
- 3D点群のインタラクティブ表示（Three.js 等）
