# Phase 5: IMU 取得・正規化・重力フィルタ

## 概要

`DeviceMotionEvent` からIMUデータ（加速度・角速度）を取得し、iOS/Android の差分を吸収して正規化する。重力ベクトルを画面上に可視化し、平面検出に重力方向フィルタを追加して水平面のみを検出する。

## ゴール

- IMU データの取得と iOS/Android 正規化
- 画面左下に重力ベクトルの方向を示すインジケータを表示
- 平面検出で重力方向に垂直な平面（水平面）のみを採用するフィルタ

## ファイル構成

```
src/
├── imu/
│   ├── sensor.ts              # 新規: DeviceMotionEvent ラッパー
│   └── normalize.ts           # 新規: iOS/Android 差分吸収・重力ベクトル算出
├── plane/
│   └── ransac.ts              # 変更: 重力方向フィルタを追加
├── visualization/
│   └── gravity-indicator.ts   # 新規: 重力ベクトル矢印の描画
└── main.ts                    # 変更: IMU 初期化 + 重力表示 + 平面フィルタ統合
```

## モジュール設計

### `src/imu/sensor.ts` (新規)

DeviceMotionEvent のラッパー。iOS のパーミッション処理を含む。

**インターフェース:**

```typescript
type ImuData = {
  acceleration: { x: number; y: number; z: number };   // 重力込み加速度 (m/s²)
  rotationRate: { alpha: number; beta: number; gamma: number }; // 角速度 (rad/s)
  timestamp: number;
};

class ImuSensor {
  start(callback: (data: ImuData) => void): Promise<void>
  stop(): void
  get isAvailable(): boolean
}
```

- `start()`: iOS では `DeviceMotionEvent.requestPermission()` を呼ぶ（ユーザー操作起因でないと動かない）
- `accelerationIncludingGravity` を使用（重力込みの加速度 = 重力ベクトルの推定に利用）
- コールバックは `devicemotion` イベントごとに呼ばれる（通常60Hz）

### `src/imu/normalize.ts` (新規)

iOS と Android の差分を吸収して統一フォーマットに変換する。

**インターフェース:**

```typescript
function detectPlatform(): 'ios' | 'android' | 'unknown'
function normalizeImuData(event: DeviceMotionEvent, platform: 'ios' | 'android' | 'unknown'): ImuData
```

**正規化ルール:**
- iOS: `rotationRate` は度/秒 → ラジアン/秒に変換 (`× Math.PI / 180`)
- iOS: `accelerationIncludingGravity` の符号が Android と逆（重力が正の向き） → 符号を反転して統一
- Android: そのまま使用
- `timestamp`: `performance.now()` で統一

### `src/visualization/gravity-indicator.ts` (新規)

画面左下に重力方向を示す小さなインジケータを描画。

**インターフェース:**

```typescript
class GravityIndicator {
  constructor(ctx: CanvasRenderingContext2D)
  draw(gravity: { x: number; y: number; z: number }): void
}
```

- 50×50px の円形エリアを画面左下に描画
- 重力ベクトルの X, Z 成分を 2D 投影して矢印で表示（端末の傾き方向が分かる）
- 背景は半透明の黒、矢印はシアン

### `src/plane/ransac.ts` (変更)

`detectPlane` に重力フィルタを追加。

**変更:**

```typescript
function detectPlane(
  points: Point3D[],
  threshold?: number,
  iterations?: number,
  gravityVector?: { x: number; y: number; z: number },
): PlaneResult | null;
```

- `gravityVector` が指定された場合、検出した平面の法線と重力方向の角度を計算
- 角度差が 30 度以内なら水平面として採用
- 角度差が 30 度を超えたら、次に良いインライア数の平面を試す（最大 iterations 内で）
- `gravityVector` が null/undefined なら従来通りフィルタなし

### `src/main.ts` (変更)

- IMU の初期化ボタンを追加（iOS のパーミッション要求はユーザー操作起因が必要）
- IMU データの最新値を保持
- `GravityIndicator.draw()` を毎フレーム呼び出し
- `detectPlane()` に `gravityVector` を渡す

**iOS パーミッション対応:**

`index.html` にボタンを追加するか、`loading` div をタップイベントのトリガーにする。ユーザーがタップしたタイミングで `DeviceMotionEvent.requestPermission()` を呼ぶ。

## 実装上のポイント

### iOS パーミッション

```typescript
// iOS 13+ ではユーザー操作起因でしか requestPermission を呼べない
if (typeof DeviceMotionEvent.requestPermission === 'function') {
  const permission = await DeviceMotionEvent.requestPermission();
  if (permission !== 'granted') throw new Error('IMU permission denied');
}
```

### 重力ベクトルの取得

`accelerationIncludingGravity` は端末が静止しているとき重力加速度（約 9.8 m/s²）を返す。これがそのまま重力方向のベクトルになる。正規化して単位ベクトルにして使用。

### 重力と平面法線の角度比較

```
cos(θ) = |dot(normal, gravity)| / (|normal| × |gravity|)
θ < 30° なら水平面
```

`dot` の絶対値を使う（法線の向きは表裏どちらでも良い）。

## スコープ外

- EKF による疎結合融合（Phase 6）
- カメラフレームと IMU の時刻同期
- 角速度データの利用（Phase 6 で使用）
- IMU バイアス補正
