# Phase 4: PnP 追跡による Visual Odometry

## 概要

Phase 3 の初期化（3D点群 + 初期姿勢）後、毎フレーム `solvePnPRansac` で 3D-2D 対応からカメラ姿勢を推定する。追跡点に ID を導入し、3D点との対応を管理する。カメラ軌跡を鳥瞰図に表示。

## ゴール

- 初期化後、カメラを動かすと鳥瞰図にカメラの移動軌跡がリアルタイムで描画される
- 3D点群は初期化時のまま固定（更新しない）

## 最終プロジェクトゴール

検出した平面上に任意の3Dモデルを配置・表示する WebAR エンジン。

## ファイル構成

```
src/
├── camera/
│   ├── capture.ts           # 既存（変更なし）
│   └── calibration.ts       # 既存（変更なし）
├── features/
│   ├── orb.ts               # 変更: Point2D に id フィールド追加
│   └── tracker.ts           # 変更: 追跡点に ID を付与・管理
├── geometry/
│   ├── essential.ts         # 既存（変更なし）
│   ├── triangulation.ts     # 既存（変更なし）
│   └── pnp.ts               # 新規: solvePnPRansac ラッパー
├── slam/
│   └── map.ts               # 新規: 3D-2D 対応管理
├── visualization/
│   └── point-cloud.ts       # 変更: カメラ軌跡の描画を追加
└── main.ts                  # 変更: 初期化後 → PnP 追跡ループ
```

## モジュール設計

### `src/features/orb.ts` (変更)

`Point2D` に `id` フィールドを追加:

```typescript
export interface Point2D {
  x: number;
  y: number;
  id: number;
}
```

`detectKeypoints` は `id: -1`（未割り当て）で返す。ID の付与は Tracker が担当。

### `src/features/tracker.ts` (変更)

- 内部カウンター `nextId` で各点にユニーク ID を付与
- 追跡成功した点は前フレームの ID を引き継ぐ
- ORB 補充で追加された点は新しい ID を受け取る
- `TrackResult` に `ids: number[]` を追加

```typescript
export type TrackResult = {
  points: Point2D[];
  prevPoints: Point2D[];
  ids: number[];
  count: number;
  avgMotion: number;
};
```

### `src/slam/map.ts` (新規)

3D点と追跡点 ID の対応を管理。

```typescript
class SlamMap {
  register(ids: number[], points3D: Point3D[]): void
  get3D2DPairs(ids: number[], points2D: Point2D[]): { points3D: Point3D[]; points2D: Point2D[] }
  get size(): number
}
```

- `register`: 初期化時に ID → Point3D の対応を一括登録
- `get3D2DPairs`: 現フレームの追跡点 ID から、対応する 3D 点がある点だけを抽出してペアで返す

### `src/geometry/pnp.ts` (新規)

`solvePnPRansac` のラッパー。

```typescript
type PnPResult = {
  R: number[][];
  t: number[];
  inlierCount: number;
};

function estimatePosePnP(
  points3D: Point3D[],
  points2D: Point2D[],
  cameraMatrixMat: cv.Mat,
): PnPResult | null;
```

- 3D点と2D点のペアから `solvePnPRansac` でカメラ姿勢を推定
- Rodrigues ベクトル (rvec) → 回転行列 R への変換は `cv.Rodrigues` を使用
- 対応点が 6 未満なら null を返す
- cv.Mat の生成と解放を関数内で完結

### `src/visualization/point-cloud.ts` (変更)

`draw` メソッドにカメラ軌跡の引数を追加:

```typescript
draw(points: Point3D[], trajectory: { x: number; z: number }[]): void
```

- 軌跡は白い線で繋いで表示
- 最新のカメラ位置は黄色の三角で表示
- 3D点群は白い点のまま

### `src/main.ts` (変更)

初期化後の状態遷移:

1. **未初期化**: Phase 2 の追跡ループ + avgMotion による初期化判定
2. **初期化時**: ホモグラフィ → R,t → 三角測量 → SlamMap に登録
3. **初期化済み**: 毎フレーム PnP 追跡
   - SlamMap から 3D-2D ペア取得
   - `estimatePosePnP` で姿勢推定
   - 成功 → カメラ軌跡に追加
   - 失敗 → 前フレームの姿勢を維持

## データフロー

```
初期化済み後の毎フレーム:
  1. tracker.process(frame) → TrackResult { points, ids, avgMotion, ... }
  2. slamMap.get3D2DPairs(ids, points) → { points3D, points2D }
  3. 対応が 6 点以上 → estimatePosePnP(points3D, points2D, K) → PnPResult
  4. 成功 → trajectory.push({ x: t[0], z: t[2] })
  5. pointCloudView.draw(points3D, trajectory)
```

## 実装上のポイント

### solvePnPRansac のパラメータ

```typescript
cv.solvePnPRansac(
  objectPoints,    // cv.Mat (N×1, CV_64FC3) - 3D points
  imagePoints,     // cv.Mat (N×1, CV_64FC2) - 2D points
  cameraMatrix,    // cv.Mat (3×3, CV_64FC1)
  distCoeffs,      // cv.Mat (空) - 歪み係数なし
  rvec,            // output: 回転ベクトル
  tvec,            // output: 並進ベクトル
  false,           // useExtrinsicGuess
  100,             // iterationsCount
  8.0,             // reprojectionError
  0.99,            // confidence
  inliers,         // output: インライアインデックス
);
```

### Rodrigues 変換

`solvePnPRansac` は回転を Rodrigues ベクトル (3×1) で返す。`cv.Rodrigues(rvec, R)` で 3×3 回転行列に変換。

### メモリ管理

- PnP に渡す cv.Mat は `estimatePosePnP` 内で生成・解放
- CameraMatrix は main.ts で1回だけ生成し、ループ中は使い回す（毎フレーム delete しない）

## スコープ外

- 3D点群の継続更新（新規追跡点の三角測量）
- IMU 融合（Phase 5-6）
- バンドル調整（Phase 7）
- 平面検出（Phase 8）
- 3Dモデルの描画
