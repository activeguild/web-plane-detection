# ブラウザ向け Visual-Inertial SLAM 実装プロジェクト

## プロジェクト概要

スマートフォンのブラウザ上で動作する Visual-Inertial SLAM (VI-SLAM) をスクラッチ実装する。最終目標は**カメラとモーションセンサーを使った平面検出**で、AR Quick Look / Scene Viewer の代替となる独自のWebARエンジンの基盤を作る。

### なぜ自前実装か

- WebXR Device API は iOS Safari が未対応で実質Android専用
- 8th Wall は商用ライセンス料が高い
- AR.js / MindAR はマーカーベース中心で任意平面検出に不向き
- → クロスプラットフォームで動く独自SLAMコアが必要

## 技術スタック

- **言語**: TypeScript
- **画像処理**: OpenCV.js (WASM) — 特徴点抽出・追跡のみ任せる
- **数値計算**: 自前実装中心 (姿勢推定・最適化・平面検出)
- **センサー入力**:
  - カメラ: `getUserMedia()`
  - IMU: `DeviceMotionEvent` / `DeviceOrientationEvent`
- **対象環境**: モバイルブラウザ (iOS Safari, Android Chrome)

## システム構成

```
カメラ映像 ──┐
            ├─→ 特徴点抽出 → 特徴点追跡 → 姿勢推定 ─┐
IMU      ──┘                          ↑          ├─→ 3D点群 → 平面検出
                                    IMU融合      │
                                                 ↓
                                          バンドル調整
                                          (誤差最小化)
```

## 実装パイプライン

### 1. カメラキャリブレーション

- 内部パラメータ (fx, fy, cx, cy, 歪み係数) の取得が必要
- スマホブラウザでは機種別の正確な値が取れない
- **方針**: 視野角を仮定した近似値で開始 (`fx ≈ fy ≈ 画像幅 × 0.9`)
- 将来的に機種別キャリブレーション値のデータベースを持つ

### 2. 特徴点抽出と追跡

- **抽出**: ORB (OpenCV.js の `cv.ORB_create()`)
  - モバイルでのリアルタイム性 (30fps) を考えると ORB 一択
  - バイナリ記述子でマッチングも高速
- **追跡**: Lucas-Kanade オプティカルフロー (`cv.calcOpticalFlowPyrLK`)
  - 毎フレーム抽出ではなく追跡で済ませる
  - 一定間隔で新規特徴点を追加してリフレッシュ

### 3. 姿勢推定 (Visual Odometry)

**初期化 (最初の2フレーム)**:
1. 5点アルゴリズム + RANSAC で基本行列 E を推定
2. E を分解して回転 R と並進 t を取得
3. 4通りの解から正しいものを選ぶ (cheirality check)
4. 三角測量で初期3D点群を構築

**継続フレーム**:
- PnP問題として解く (`cv.solvePnPRansac`)
- 3D-2D対応から現在の姿勢を求める

### 4. IMU融合

**取得コード**:
```javascript
await DeviceMotionEvent.requestPermission(); // iOSは必須
window.addEventListener('devicemotion', (e) => {
  // e.acceleration: 重力除いた加速度 (m/s²)
  // e.accelerationIncludingGravity: 重力込み
  // e.rotationRate: 角速度
});
```

**注意点**:
- iOS の `rotationRate` は **度/秒**、Android は実装次第 (要正規化)
- 端末座標系の軸の向きがブラウザ・OSで微妙に違う
- サンプリングレート (通常60Hz) は保証なし
- カメラフレームと IMU のタイムスタンプ同期は `performance.now()` で揃える

**融合方式**: **疎結合 (Loose Coupling)** から開始
- カメラと IMU で別々に姿勢推定 → EKF で統合
- 実装が楽。後で密結合に発展可能

### 5. バンドル調整

- 直近 N フレーム (5〜10) だけを最適化する**ローカルバンドル調整**
- 全体最適化はブラウザでは重すぎる
- 自前 Levenberg-Marquardt 実装 (小規模なら可能)
- スパース行列ソルバが必要なら問題サイズを抑えて対処

### 6. 平面検出 (最終ゴール)

- **RANSAC 平面フィッティング** で 3D点群から平面を抽出
- **重力方向の活用**: IMU の重力ベクトルに垂直な平面 (床・机) に絞る
  - 検出が高速・安定する
  - ARKit / ARCore もこの戦略
- 検出した平面の点を除外して再 RANSAC で複数平面対応

## 実装順序 (重要)

挫折しないために、各段階で動くものを作りながら進める。

1. **Phase 1**: OpenCV.js でカメラ映像 → 特徴点抽出 → 可視化
2. **Phase 2**: オプティカルフローで追跡 → 動きの可視化
3. **Phase 3**: 2フレーム間の基本行列推定 → 初期3D点群復元
4. **Phase 4**: PnP で継続追跡 → Visual Odometry 完成
5. **Phase 5**: IMU 取得・正規化・比較
6. **Phase 6**: EKF で疎結合融合 (VI-Odometry)
7. **Phase 7**: ローカルバンドル調整
8. **Phase 8**: RANSAC 平面検出 → ゴール

各 Phase で必ずデモが動く状態にする。Phase 1〜4 まではカメラのみ (Visual SLAM)、IMU は Phase 5 から。

## 参考実装・文献

- **AlvaAR** (https://github.com/alanross/AlvaAR)
  - TypeScript + WASM のブラウザ向け SLAM 実装
  - 最も近い既存実装。コードを参考にする
- **ORB-SLAM3** (C++)
  - 教科書的実装、論文も読みやすい
- **VINS-Mono** (C++)
  - VI-SLAM のデファクト
  - 論文: "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator"
- **Multiple View Geometry in Computer Vision** (Hartley & Zisserman)
  - エピポーラ幾何の聖典
- **State Estimation for Robotics** (Barfoot)
  - SLAM の数学的基礎、無料 PDF 配布あり

## ディレクトリ構成 (案)

```
src/
├── camera/
│   ├── capture.ts          # getUserMedia ラッパー
│   └── calibration.ts      # 内部パラメータ管理
├── imu/
│   ├── sensor.ts           # DeviceMotion ラッパー
│   └── normalize.ts        # iOS/Android 差分吸収
├── features/
│   ├── orb.ts              # OpenCV.js ORB ラッパー
│   └── tracker.ts          # Lucas-Kanade 追跡
├── geometry/
│   ├── essential.ts        # 基本行列 + 5点アルゴリズム
│   ├── triangulation.ts    # 三角測量
│   └── pnp.ts              # PnP 姿勢推定
├── fusion/
│   ├── ekf.ts              # 拡張カルマンフィルタ
│   └── timesync.ts         # カメラ・IMU タイムスタンプ同期
├── optimization/
│   ├── lm.ts               # Levenberg-Marquardt
│   └── ba.ts               # ローカルバンドル調整
├── plane/
│   └── ransac.ts           # RANSAC 平面検出
└── slam/
    └── pipeline.ts         # 全体パイプライン統合
```

## 開発上の注意

- **モバイル実機テスト必須**: PC のデバッグでは IMU やカメラ挙動が再現できない
- **iOS 13+ のパーミッション**: `DeviceMotionEvent.requestPermission()` はユーザー操作起因でしか呼べない
- **HTTPS 必須**: `getUserMedia()` は HTTP では動かない (localhost 除く)
- **OpenCV.js の WASM ロード**: 数MBあるので初回ロード時間に注意
- **デバッグ可視化を最優先**: 特徴点・追跡線・3D点群・平面を画面に重ね描きする仕組みを早めに作る

## 現状

設計段階。Phase 1 から着手予定。
