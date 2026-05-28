# Phase 7: 新規点の三角測量による SlamMap 拡張

## 概要

PnP 追跡中に 3D 対応のない追跡点を三角測量して SlamMap に追加し、PnP マッチが枯渇しないようにする。AR として継続的なトラッキングを可能にする。

## ゴール

- PnP マッチ数が閾値（50）以下になったら、3D 対応のない追跡点を三角測量して SlamMap に追加
- PnP が止まらず、カメラを動かし続けても追跡が継続する

## ファイル構成

```
src/
├── slam/
│   ├── map.ts               # 既存（変更なし）
│   └── mapper.ts            # 新規: 新規点の三角測量 + SlamMap 追加
└── main.ts                  # 変更: mapper を統合
```

## モジュール設計

### `src/slam/mapper.ts` (新規)

PnP で姿勢が取れている状態で、3D 対応のない追跡点を三角測量して SlamMap に登録する。

**三角測量には2フレーム間の視差が必要。** そのため Mapper は前回の expand 実行時の姿勢と未登録点を保持し、次回 expand 時にその間の対応で三角測量する。

**インターフェース:**

```typescript
class Mapper {
  constructor(slamMap: SlamMap, cameraMatrix: number[][])

  // 3D 対応のない追跡点を三角測量して SlamMap に登録
  // 戻り値: 追加した3D点の配列
  expandMap(
    ids: number[],
    points: Point2D[],
    R: number[][],
    t: number[],
  ): Point3D[]
}
```

**内部状態:**
- `prevUnmatchedIds: number[]` — 前回の未登録点の ID
- `prevUnmatchedPoints: Point2D[]` — 前回の未登録点の 2D 座標
- `prevR: number[][]`, `prevT: number[]` — 前回の姿勢

**処理フロー:**
1. `ids` から SlamMap に未登録の点を抽出（未登録 = 3D 対応がない点）
2. 前回データがなければ → 今回の未登録点と姿勢を保存して終了
3. 前回データがあれば → 前回と今回で共通の ID を持つ未登録点を探す
4. 共通の点ペアに対して `triangulatePoints` で三角測量
5. cheirality check + 外れ値フィルタを通った点を SlamMap に登録
6. 今回の未登録点と姿勢を保存（次回用）

### `src/main.ts` (変更)

- `Mapper` を初期化
- PnP 成功後、マッチ数 < 50 なら `mapper.expandMap()` を呼ぶ
- 返された新規 Point3D を `points3D` 配列に追加（鳥瞰図表示用）

## データフロー

```
PnP 追跡ループ中:
  1. PnP 成功 → currentR, currentT
  2. マッチ数 < 50 ?
    YES → mapper.expandMap(ids, points, currentR, currentT)
      → Mapper が前回保存した姿勢・点と今回の姿勢・点で三角測量
      → 新規 Point3D を SlamMap に登録
      → points3D に追加
    NO → スキップ（十分なマッチがある）
```

## 実装上のポイント

### 前回・今回の共通 ID マッチング

前回の未登録 ID と今回の未登録 ID の共通部分を探す。`Set` を使って O(n) で照合。共通 ID の点だけが三角測量の対象。

### 三角測量の流用

既存の `triangulatePoints` をそのまま使う。投影行列を前回の姿勢（P1 = K[R_prev|t_prev]）と今回の姿勢（P2 = K[R_curr|t_curr]）で構築。

ただし `triangulatePoints` は P1 = K[I|0] を前提としているので、**Mapper 内で正規化座標系に変換してから三角測量**するか、投影行列を直接渡す方式に変更する。

→ シンプルに: 前回姿勢を基準座標にして、今回姿勢との相対 R, t を計算。`R_rel = R_curr × R_prev^T`, `t_rel = t_curr - R_rel × t_prev`。これを `triangulatePoints` に渡す。三角測量後の点は前回姿勢の座標系なので、ワールド座標に戻す: `P_world = R_prev^T × (P_local - t_prev)`。

## スコープ外

- Levenberg-Marquardt による最適化（精度向上は将来のタスク）
- キーフレーム管理
- ループクロージング
