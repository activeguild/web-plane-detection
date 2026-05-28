# Phase 7: SlamMap 拡張 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** PnP マッチが枯渇しないよう、3D 対応のない追跡点を三角測量して SlamMap に継続的に追加する

**Architecture:** `Mapper` が前回と今回の未登録点・姿勢を保持し、共通 ID の点ペアを三角測量して SlamMap に登録。マッチ数 < 50 で発動。既存の `triangulatePoints` を流用し、相対姿勢で三角測量後ワールド座標に変換。

**Tech Stack:** TypeScript

---

## File Structure

| Action | Path | Responsibility |
|--------|------|----------------|
| Modify | `src/slam/map.ts` | `has(id)` メソッド追加 |
| Create | `src/slam/mapper.ts` | 新規点の三角測量 + SlamMap 追加 |
| Modify | `src/main.ts` | Mapper 統合 |

---

### Task 1: SlamMap に has メソッド追加

**Files:**
- Modify: `src/slam/map.ts`

- [ ] **Step 1: `has` メソッドを追加**

`src/slam/map.ts` の `SlamMap` クラスに以下を追加（`get size()` の前に）:

```typescript
  has(id: number): boolean {
    return this.map.has(id);
  }
```

- [ ] **Step 2: Commit**

```bash
git add src/slam/map.ts
git commit -m "feat: add has() method to SlamMap"
```

---

### Task 2: Mapper モジュール

**Files:**
- Create: `src/slam/mapper.ts`

- [ ] **Step 1: `src/slam/mapper.ts` を作成**

```typescript
import { Point2D } from '../features/orb';
import { triangulatePoints, Point3D } from '../geometry/triangulation';
import { SlamMap } from './map';

// 3×3 行列の転置
function transpose(M: number[][]): number[][] {
  return [
    [M[0][0], M[1][0], M[2][0]],
    [M[0][1], M[1][1], M[2][1]],
    [M[0][2], M[1][2], M[2][2]],
  ];
}

// 3×3 行列の積
function mul3x3(A: number[][], B: number[][]): number[][] {
  const C: number[][] = [[0,0,0],[0,0,0],[0,0,0]];
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      for (let k = 0; k < 3; k++) {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return C;
}

// 3×3 行列 × 3ベクトル
function mulMV(M: number[][], v: number[]): number[] {
  return [
    M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
    M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
    M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2],
  ];
}

export class Mapper {
  private slamMap: SlamMap;
  private cameraMatrix: number[][];

  // 前回の expand 時のデータ
  private prevIds: number[] | null = null;
  private prevPoints: Map<number, Point2D> | null = null;
  private prevR: number[][] | null = null;
  private prevT: number[] | null = null;

  constructor(slamMap: SlamMap, cameraMatrix: number[][]) {
    this.slamMap = slamMap;
    this.cameraMatrix = cameraMatrix;
  }

  expandMap(
    ids: number[],
    points: Point2D[],
    R: number[][],
    t: number[],
  ): Point3D[] {
    // 未登録の点を抽出
    const unmatchedIds: number[] = [];
    const unmatchedPoints = new Map<number, Point2D>();
    for (let i = 0; i < ids.length; i++) {
      if (!this.slamMap.has(ids[i])) {
        unmatchedIds.push(ids[i]);
        unmatchedPoints.set(ids[i], points[i]);
      }
    }

    // 前回データがなければ保存して終了
    if (this.prevIds === null || this.prevPoints === null || this.prevR === null || this.prevT === null) {
      this.prevIds = unmatchedIds;
      this.prevPoints = unmatchedPoints;
      this.prevR = R;
      this.prevT = t;
      console.log(`[SLAM] Mapper: stored ${unmatchedIds.length} unmatched points for next expand`);
      return [];
    }

    // 前回と今回で共通の ID を探す
    const prevSet = new Set(this.prevIds);
    const commonIds: number[] = [];
    for (const id of unmatchedIds) {
      if (prevSet.has(id) && this.prevPoints.has(id)) {
        commonIds.push(id);
      }
    }

    if (commonIds.length < 8) {
      // 共通点が少なすぎる → データを更新して終了
      this.prevIds = unmatchedIds;
      this.prevPoints = unmatchedPoints;
      this.prevR = R;
      this.prevT = t;
      console.log(`[SLAM] Mapper: only ${commonIds.length} common unmatched points, need 8+`);
      return [];
    }

    // 前回と今回の対応点を構築
    const prevPts: Point2D[] = [];
    const currPts: Point2D[] = [];
    for (const id of commonIds) {
      prevPts.push(this.prevPoints.get(id)!);
      currPts.push(unmatchedPoints.get(id)!);
    }

    // 相対姿勢を計算: R_rel = R_curr × R_prev^T, t_rel = t_curr - R_rel × t_prev
    const Rpt = transpose(this.prevR);
    const Rrel = mul3x3(R, Rpt);
    const RrelTprev = mulMV(Rrel, this.prevT);
    const trel = [
      t[0] - RrelTprev[0],
      t[1] - RrelTprev[1],
      t[2] - RrelTprev[2],
    ];

    // 三角測量（前回姿勢の座標系で）
    const allTrue = commonIds.map(() => true);
    const localPoints = triangulatePoints(
      prevPts, currPts,
      Rrel, trel,
      this.cameraMatrix,
      allTrue,
    );

    if (localPoints.length === 0) {
      this.prevIds = unmatchedIds;
      this.prevPoints = unmatchedPoints;
      this.prevR = R;
      this.prevT = t;
      console.log('[SLAM] Mapper: triangulation produced 0 points');
      return [];
    }

    // ローカル座標（前回姿勢基準）→ ワールド座標に変換
    // P_world = R_prev^T × (P_local - t_prev)
    // ただし triangulatePoints は P1=[I|0] 基準なので P_local はそのまま前回カメラ座標
    // P_world = R_prev^T × P_local + (R_prev^T × (-t_prev)) は不正確
    // 正確: P_cam_prev = P_local, P_world = R_prev^T × (P_cam_prev - t_prev)
    const worldPoints: Point3D[] = [];
    for (const p of localPoints) {
      const shifted = [p.x - this.prevT[0], p.y - this.prevT[1], p.z - this.prevT[2]];
      const pw = mulMV(Rpt, shifted);
      worldPoints.push({ x: pw[0], y: pw[1], z: pw[2] });
    }

    // SlamMap に登録（commonIds の先頭から localPoints.length 分）
    const registeredIds = commonIds.slice(0, localPoints.length);
    this.slamMap.register(registeredIds, worldPoints);

    console.log(`[SLAM] Mapper: expanded ${worldPoints.length} new 3D points (from ${commonIds.length} common)`);

    // データを更新
    this.prevIds = unmatchedIds;
    this.prevPoints = unmatchedPoints;
    this.prevR = R;
    this.prevT = t;

    return worldPoints;
  }
}
```

- [ ] **Step 2: Commit**

```bash
git add src/slam/mapper.ts
git commit -m "feat: add Mapper for expanding SlamMap with new triangulated points"
```

---

### Task 3: main.ts に Mapper を統合

**Files:**
- Modify: `src/main.ts`

- [ ] **Step 1: import に Mapper を追加**

`src/main.ts` の先頭の import に追加:

```typescript
import { Mapper } from './slam/mapper';
```

- [ ] **Step 2: Mapper を初期化**

モジュール初期化セクション（`const K = calibration.getCameraMatrixAsMat();` の後）に追加:

```typescript
  const mapper = new Mapper(slamMap, Karray);
```

- [ ] **Step 3: PnP 追跡ブロックに Mapper 呼び出しを追加**

PnP 追跡の `else` ブロック内で、PnP 成功後にマッチ数チェック + expandMap を追加。

既存の PnP ブロック:

```typescript
      } else {
        const { points3D: matched3D, points2D: matched2D } = slamMap.get3D2DPairs(result.ids, result.points);

        if (matched3D.length >= 6) {
          const pnpResult = estimatePosePnP(matched3D, matched2D, K);
          if (pnpResult) {
            currentR = pnpResult.R;
            currentT = pnpResult.t;
            trajectory.push({ x: pnpResult.t[0], z: pnpResult.t[2] });
            if (frameCount % 30 === 0) {
              console.log(`[SLAM] PnP: ${pnpResult.inlierCount}/${matched3D.length} inliers`);
            }
          }
        } else if (frameCount % 60 === 0) {
          console.log(`[SLAM] PnP: not enough matches (${matched3D.length})`);
        }
      }
```

これを以下に置き換え:

```typescript
      } else {
        const { points3D: matched3D, points2D: matched2D } = slamMap.get3D2DPairs(result.ids, result.points);

        if (matched3D.length >= 6) {
          const pnpResult = estimatePosePnP(matched3D, matched2D, K);
          if (pnpResult) {
            currentR = pnpResult.R;
            currentT = pnpResult.t;
            trajectory.push({ x: pnpResult.t[0], z: pnpResult.t[2] });
            if (frameCount % 30 === 0) {
              console.log(`[SLAM] PnP: ${pnpResult.inlierCount}/${matched3D.length} inliers, map=${slamMap.size}`);
            }

            // マッチ数が少なくなったら新規点を三角測量して追加
            if (matched3D.length < 50 && currentR && currentT) {
              const newPts = mapper.expandMap(result.ids, result.points, currentR, currentT);
              for (const p of newPts) {
                points3D.push(p);
              }
            }
          }
        } else if (frameCount % 60 === 0) {
          console.log(`[SLAM] PnP: not enough matches (${matched3D.length}), map=${slamMap.size}`);
        }
      }
```

- [ ] **Step 4: 動作確認**

Run: `pnpm vite`

Expected:
- 初期化後、PnP 追跡が継続
- マッチ数が 50 以下になると `[SLAM] Mapper: expanded N new 3D points` がログに表示
- SlamMap のサイズが増加し、PnP マッチが補充される
- 鳥瞰図に新規の白い点が追加される
- カメラを動かし続けても黄色い三角が止まらない（または止まるまでの時間が大幅に延びる）

- [ ] **Step 5: Commit**

```bash
git add src/main.ts
git commit -m "feat: integrate Mapper to expand SlamMap during PnP tracking"
```

---

## Summary

| Task | 内容 | 依存 |
|------|------|------|
| 1 | SlamMap に has() 追加 | なし |
| 2 | Mapper モジュール | Task 1 |
| 3 | main.ts 統合 | Task 1, 2 |

Task 1 → Task 2 → Task 3 の順序。
