# Phase 8: RANSAC 平面検出と可視化

## 概要

Phase 3-4 で得た3D点群から RANSAC で平面を1枚検出し、カメラ映像上に半透明ポリゴンでオーバーレイ表示する。

## ゴール

- 初期化後、3D点群から平面を自動検出し、カメラ映像上に検出平面の範囲を半透明ポリゴンで描画する
- カメラを動かすと PnP 姿勢に追従してオーバーレイが更新される

## ファイル構成

```
src/
├── plane/
│   └── ransac.ts              # 新規: RANSAC 平面検出
├── visualization/
│   ├── point-cloud.ts         # 既存（変更なし）
│   └── plane-overlay.ts       # 新規: 平面の2D投影描画
└── main.ts                    # 変更: 平面検出 + オーバーレイ描画を統合
```

## モジュール設計

### `src/plane/ransac.ts` (新規)

3D点群から RANSAC で最大の平面を検出する。

**インターフェース:**

```typescript
type PlaneResult = {
  normal: number[];     // 平面の法線ベクトル [a, b, c] (正規化済み)
  d: number;            // ax + by + cz + d = 0 の d
  inliers: Point3D[];   // 平面上のインライア点
};

function detectPlane(
  points: Point3D[],
  threshold: number = 0.02,
  iterations: number = 100,
): PlaneResult | null;
```

**アルゴリズム:**
1. `iterations` 回繰り返す:
   - ランダムに3点を選択
   - 3点から平面 (法線 n, 距離 d) を計算（外積）
   - 全点について平面までの距離を計算、`threshold` 以内の点をインライアとする
   - インライア数が最大なら更新
2. 最大インライア数が全点の 30% 以上なら成功、そうでなければ null

### `src/visualization/plane-overlay.ts` (新規)

検出平面のインライア点を画像座標に投影し、凸包を半透明ポリゴンで描画する。

**インターフェース:**

```typescript
class PlaneOverlay {
  constructor(ctx: CanvasRenderingContext2D)
  draw(inliers: Point3D[], R: number[][], t: number[], K: number[][]): void
}
```

**処理:**
1. 各インライア点を R, t, K でカメラ画像座標に投影: `p_img = K × (R × P + t)`
2. 投影点の凸包を計算（Graham scan）
3. 凸包を半透明の青色ポリゴン (`rgba(0, 120, 255, 0.3)`) で描画
4. 凸包の輪郭線も描画（`rgba(0, 120, 255, 0.7)`）

### `src/main.ts` (変更)

- 初期化成功時、3D点群に対して `detectPlane()` を1回実行
- 平面検出成功 → `PlaneResult` を保持
- 毎フレームの描画で PnP 姿勢を使って `PlaneOverlay.draw()` を呼ぶ
- PnP が失敗したフレームは前回の姿勢でオーバーレイを描画

## データフロー

```
初期化成功時:
  points3D → detectPlane(points3D) → PlaneResult { normal, d, inliers }

毎フレーム (初期化済み + 平面検出済み):
  PnP → R, t
  PlaneOverlay.draw(planeResult.inliers, R, t, K) → 映像上にポリゴン
```

## 実装上のポイント

### 3D→2D 投影

```
P_cam = R × P_world + t       // ワールド座標 → カメラ座標
p = K × P_cam                  // カメラ座標 → 画像座標（同次）
u = p[0] / p[2], v = p[1] / p[2]  // 正規化
```

投影時に P_cam.z <= 0 の点（カメラの後ろ）はスキップ。

### Graham scan 凸包

投影された 2D 点群の凸包を計算する。O(n log n) で十分高速。

### 平面検出のタイミング

3D点群が固定なので初期化時に1回だけ実行。毎フレーム再検出は不要。

## スコープ外

- 複数平面の検出
- IMU 重力方向による平面フィルタ（Phase 5 後に追加）
- 平面上への3Dモデル配置
- 平面の追跡更新（新規3D点による平面の拡張）
