# Phase 2: オプティカルフローによる特徴点追跡・動きの可視化

## 概要

Phase 1 で実装した ORB 特徴点抽出の上に、Lucas-Kanade オプティカルフロー (`cv.calcOpticalFlowPyrLK`) による追跡を追加する。毎フレーム ORB 再検出する代わりに、前フレームの特徴点を追跡し、追跡失敗で減った分だけ ORB で補充する。

## ゴール

- 特徴点をフレーム間で追跡し、追跡中の点にマーカー + 移動ベクトル（前フレームからの線）を描画する

## ファイル構成

```
src/
├── camera/
│   └── capture.ts           # 既存（変更なし）
├── features/
│   ├── orb.ts               # 既存（変更なし、Tracker から呼ばれる）
│   └── tracker.ts           # 新規: LK追跡 + 特徴点管理
└── main.ts                  # 変更: Tracker を使った処理ループに差し替え
```

## モジュール設計

### `src/features/tracker.ts` (新規)

**責務:**
- 前フレーム・現フレームのグレースケール画像を保持
- `cv.calcOpticalFlowPyrLK` で特徴点をフレーム間追跡
- ステータスベクトルで追跡失敗点を除外
- 特徴点数が閾値 (200) を下回ったら `OrbDetector` で新規追加
- 既存の追跡点と近すぎる新規点は重複排除（距離閾値: 20px）

**インターフェース:**

```typescript
class FeatureTracker {
  constructor(orb: OrbDetector, minFeatures: number = 200)
  process(frame: cv.Mat): TrackResult
  dispose(): void
}

type TrackResult = {
  points: cv.Point[]       // 現フレームの特徴点座標
  prevPoints: cv.Point[]   // 前フレームの特徴点座標（移動ベクトル描画用）
  count: number            // 追跡中の点数
}
```

### `src/features/orb.ts` (既存・変更なし)

`FeatureTracker` から特徴点補充時に呼び出される。`detectAndDraw` ではなく `detect` のみ使うため、キーポイント座標だけ返す用途で利用。現状の `detectAndDraw` はそのまま残し、Tracker からは内部の `orb.detect()` を直接使うか、新しいメソッドを追加する。

→ `OrbDetector` に `detectKeypoints(gray: cv.Mat): cv.Point[]` メソッドを追加し、キーポイント座標の配列を返す。`detectAndDraw` は Phase 1 互換として残す。

### `src/main.ts` (変更)

- `OrbDetector` + `FeatureTracker` を初期化
- 毎フレーム: offscreen canvas → cv.Mat → `tracker.process(frame)` で追跡
- `TrackResult` を Canvas 2D API で描画（映像 + マーカー + 移動ベクトル）
- `cv.imshow` は使わず、Canvas 2D API で映像と描画を統合

## データフロー

```
毎フレーム:
  1. offscreen canvas → cv.Mat (RGBA) → グレースケール変換
  2. 前フレームなし → OrbDetector で初期特徴点検出 → 前フレームとして保存
  3. 前フレームあり → calcOpticalFlowPyrLK(prevGray, gray, prevPoints) で追跡
  4. ステータスベクトルで追跡失敗点を除外
  5. 残った点数 < 200 → OrbDetector で補充（既存点から20px以内の重複を除外）
  6. TrackResult { points, prevPoints, count } を返す
  7. 現フレームのグレースケールを前フレームとして保存
```

## 描画方式

Canvas 2D API で描画（Phase 1 の `cv.imshow` から変更）:

1. `ctx.drawImage(video, ...)` で映像描画
2. 各 `points[i]` に緑の円（半径 3px）でマーカー
3. `prevPoints[i]` → `points[i]` に赤い線（移動ベクトル）

Phase 1 では offscreen canvas → cv.Mat → cv.imshow でフレームを canvas に描画していたが、Phase 2 では映像とオーバーレイを分離して Canvas 2D API で描画する。OpenCV.js は計算のみに使用。

## 実装上のポイント

### calcOpticalFlowPyrLK のパラメータ

```typescript
cv.calcOpticalFlowPyrLK(
  prevGray,          // 前フレーム（グレースケール）
  gray,              // 現フレーム（グレースケール）
  prevPts,           // 前フレームの特徴点 (cv.Mat, N×1, CV_32FC2)
  nextPts,           // 出力: 現フレームの特徴点位置
  status,            // 出力: 追跡成功=1, 失敗=0
  err,               // 出力: エラー値
  winSize,           // 探索窓 (21, 21)
  maxLevel,          // ピラミッドレベル数: 3
);
```

### メモリ管理

- `prevGray` は前フレームとして保持するため、毎フレーム delete しない（次フレームで上書き時に前の分を delete）
- `calcOpticalFlowPyrLK` の出力 (`nextPts`, `status`, `err`) は毎フレーム delete
- `frame` (RGBA) とフレーム内のグレースケールは毎フレーム delete

### 特徴点の cv.Mat 形式

`calcOpticalFlowPyrLK` は特徴点を `cv.Mat` (N×1, CV_32FC2) で受け取る。`OrbDetector` から得た `KeyPoint` の座標を `Float32Array` に変換して `cv.Mat` に詰める必要がある。

## スコープ外

- 追跡軌跡（過去Nフレーム分のトレイル）表示
- パフォーマンス情報（FPS等）の表示
- 姿勢推定（Phase 3-4）
- IMU 融合（Phase 5-6）
