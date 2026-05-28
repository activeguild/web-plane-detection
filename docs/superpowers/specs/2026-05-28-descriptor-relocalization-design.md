# ディスクリプタベース再ローカライゼーション

## 概要

PnP マッチ不足時に ORB ディスクリプタ + BFMatcher で SlamMap の 3D 点と現フレームの特徴点をマッチングし、PnP 姿勢推定を復旧する。LK 追跡はそのまま維持し、フォールバックとしてディスクリプタマッチングを使う。

## ゴール

- PnP 追跡がロストしても、ディスクリプタマッチングで自動復旧
- カメラを動かしてもオブジェクトが平面上に固定されて見える安定した AR 体験

## ファイル構成

```
src/
├── features/
│   └── orb.ts               # 変更: detectWithDescriptors() 追加
├── slam/
│   ├── map.ts               # 変更: ディスクリプタ保存 + ディスクリプタベース relocalize
│   └── mapper.ts            # 変更: 新規点のディスクリプタも保存
└── main.ts                  # 変更: 再ローカライゼーション呼び出し更新
```

## モジュール変更

### `src/features/orb.ts` (変更)

`detectWithDescriptors()` メソッドを追加:

```typescript
type OrbResult = {
  keypoints: Point2D[];
  descriptors: cv.Mat;  // N×32, CV_8UC1（呼び出し側で delete 必要）
};

detectWithDescriptors(gray: cv.Mat): OrbResult
```

- `orb.detectAndCompute(gray, mask, keypoints, descriptors)` を使用
- キーポイント座標を Point2D[] に変換、ディスクリプタは cv.Mat のまま返す
- 呼び出し側が descriptors.delete() を担当

### `src/slam/map.ts` (変更)

3D 点にディスクリプタを紐付け:

```typescript
type MapEntry = {
  point3D: Point3D;
  descriptor: Uint8Array;  // 32バイト（ORB ディスクリプタ1行分のコピー）
};
```

`register()` にディスクリプタ引数を追加:
```typescript
register(ids: number[], points3D: Point3D[], descriptors?: cv.Mat): void
```

`relocalize()` をディスクリプタベースに書き換え:
```typescript
relocalize(
  keypoints: Point2D[],
  descriptors: cv.Mat,
  K: cv.Mat,
): { R: number[][]; t: number[]; matchCount: number } | null
```

- SlamMap 全エントリのディスクリプタを cv.Mat に集約
- `BFMatcher(NORM_HAMMING, crossCheck=true).match()` で照合
- Hamming 距離 < 60 のマッチのみ採用
- マッチした 3D-2D ペアが 6 以上あれば `solvePnPRansac` で姿勢推定
- 成功なら R, t, matchCount を返す

### `src/slam/mapper.ts` (変更)

`expandMap()` で三角測量した新規点のディスクリプタも SlamMap に登録:
- expandMap 呼び出し時に現フレームのディスクリプタを受け取り、新規点に対応するディスクリプタを SlamMap.register に渡す

### `src/main.ts` (変更)

PnP マッチ不足時:
1. 現フレームでグレースケール画像を取得（既に tracker 内で変換済みだが、再取得が必要）
2. `orb.detectWithDescriptors(gray)` で ORB 検出
3. `slamMap.relocalize(keypoints, descriptors, K)` でディスクリプタマッチング + PnP
4. 成功 → `currentR`, `currentT` を更新、AR レンダリング継続

## 実装上のポイント

### BFMatcher の使い方

```typescript
const bf = new cv.BFMatcher(cv.NORM_HAMMING, true); // crossCheck=true
const matches = new cv.DMatchVector();
bf.match(queryDesc, trainDesc, matches);

for (let i = 0; i < matches.size(); i++) {
  const m = matches.get(i);
  if (m.distance < 60) {
    // queryIdx → 現フレームのキーポイント
    // trainIdx → SlamMap のエントリ
  }
}
matches.delete();
bf.delete();
```

### ディスクリプタの保存形式

cv.Mat は WASM メモリを使うので長期保持は避ける。各エントリに `Uint8Array(32)` としてコピーして保存。relocalize 時に一括で cv.Mat に戻す。

### パフォーマンス

BFMatcher は O(N×M) なので SlamMap が大きくなると遅くなる。対策:
- SlamMap のサイズ上限を設ける（例: 最新 500 エントリのみ保持）
- relocalize は毎フレームではなく PnP 失敗時のみ実行

## スコープ外

- キーフレーム管理
- ループクロージング
- IMU 融合（Phase 6 EKF）
