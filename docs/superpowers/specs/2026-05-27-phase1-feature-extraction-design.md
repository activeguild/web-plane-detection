# Phase 1: カメラ映像からORB特徴点をリアルタイム抽出・可視化

## 概要

スマートフォンブラウザ上でカメラ映像を取得し、OpenCV.js の ORB アルゴリズムで特徴点を抽出、映像上にリアルタイム描画するデモを作成する。VI-SLAM パイプラインの最初のステップ。

## ゴール

- カメラ映像の上に ORB 特徴点のマーカーをリアルタイム描画する

## 技術スタック

- **パッケージマネージャ**: npm
- **ビルド**: Vite + TypeScript
- **OpenCV.js**: `@techstark/opencv-js` (npm パッケージ)
- **対象環境**: モバイルブラウザ (iOS Safari, Android Chrome)

## プロジェクト構成

```
web-plane-detection/
├── index.html
├── package.json
├── tsconfig.json
├── vite.config.ts
└── src/
    ├── main.ts              # エントリーポイント（初期化・ループ起動）
    ├── camera/
    │   └── capture.ts       # getUserMedia ラッパー
    └── features/
        └── orb.ts           # ORB特徴点抽出 + 描画
```

CLAUDE.md のディレクトリ構成案に沿い、Phase 1 で必要なものだけ作成する。Phase 2 以降で `tracker.ts`, `geometry/` 等を追加。

## データフロー

```
getUserMedia() → <video> (非表示)
       ↓
requestAnimationFrame ループ:
  1. cv.VideoCapture でフレーム取得 → cv.Mat (RGBA)
  2. RGBA → グレースケール変換
  3. cv.ORB_create() で特徴点検出
  4. cv.drawKeypoints() でフレーム上に描画
  5. cv.imshow() で <canvas> に出力
  6. cv.Mat を delete() で解放（メモリリーク防止）
```

## モジュール設計

### `src/camera/capture.ts`

- `getUserMedia()` で背面カメラ (`facingMode: 'environment'`) を取得
- `<video>` 要素にストリームを接続
- 映像の準備完了を Promise で返す

### `src/features/orb.ts`

- `cv.ORB_create()` で ORB 検出器を生成
- グレースケール画像からキーポイントとディスクリプタを抽出
- `cv.drawKeypoints()` でフレーム上に描画

### `src/main.ts`

- OpenCV.js の WASM ロード完了を待機
- カメラ初期化
- `requestAnimationFrame` で毎フレーム処理ループを実行
  - フレーム取得 → ORB 抽出 → 描画 → canvas 出力 → Mat 解放

## 実装上のポイント

### OpenCV.js WASM ロード

`@techstark/opencv-js` の Promise ベースのロードを使用して、WASM 準備完了まで待機する。

### メモリ管理

`cv.Mat` は毎フレーム生成されるため、必ず `delete()` で解放する。忘れると WASM ヒープが枯渇してクラッシュする。

### モバイル対応

- カメラは `facingMode: 'environment'`（背面カメラ）を指定
- canvas サイズはビューポートに合わせる

### HTTPS 対応

`getUserMedia()` は HTTPS が必須（localhost 除く）。Vite の `--host` で LAN 公開し、実機テストには `@vitejs/plugin-basic-ssl` または mkcert プラグインで自己署名証明書を使用する。

## スコープ外

- パフォーマンス情報（FPS等）の表示 — 必要になったら追加
- 特徴点追跡（Phase 2）
- 姿勢推定（Phase 3-4）
- IMU 融合（Phase 5-6）
