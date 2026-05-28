import { Point2D } from '../features/orb';
import { triangulatePoints, Point3D } from '../geometry/triangulation';
import { SlamMap } from './map';

function transpose(M: number[][]): number[][] {
  return [
    [M[0][0], M[1][0], M[2][0]],
    [M[0][1], M[1][1], M[2][1]],
    [M[0][2], M[1][2], M[2][2]],
  ];
}

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
    const unmatchedIds: number[] = [];
    const unmatchedPoints = new Map<number, Point2D>();
    for (let i = 0; i < ids.length; i++) {
      if (!this.slamMap.has(ids[i])) {
        unmatchedIds.push(ids[i]);
        unmatchedPoints.set(ids[i], points[i]);
      }
    }

    if (this.prevIds === null || this.prevPoints === null || this.prevR === null || this.prevT === null) {
      this.prevIds = unmatchedIds;
      this.prevPoints = unmatchedPoints;
      this.prevR = R;
      this.prevT = t;
      console.log(`[SLAM] Mapper: stored ${unmatchedIds.length} unmatched points for next expand`);
      return [];
    }

    const prevSet = new Set(this.prevIds);
    const commonIds: number[] = [];
    for (const id of unmatchedIds) {
      if (prevSet.has(id) && this.prevPoints.has(id)) {
        commonIds.push(id);
      }
    }

    if (commonIds.length < 8) {
      this.prevIds = unmatchedIds;
      this.prevPoints = unmatchedPoints;
      this.prevR = R;
      this.prevT = t;
      console.log(`[SLAM] Mapper: only ${commonIds.length} common unmatched points, need 8+`);
      return [];
    }

    const prevPts: Point2D[] = [];
    const currPts: Point2D[] = [];
    for (const id of commonIds) {
      prevPts.push(this.prevPoints.get(id)!);
      currPts.push(unmatchedPoints.get(id)!);
    }

    const Rpt = transpose(this.prevR);
    const Rrel = mul3x3(R, Rpt);
    const RrelTprev = mulMV(Rrel, this.prevT);
    const trel = [
      t[0] - RrelTprev[0],
      t[1] - RrelTprev[1],
      t[2] - RrelTprev[2],
    ];

    // ベースラインチェック: 相対並進が小さすぎる場合はスキップ（データは保持しない → 蓄積させる）
    const baseline = Math.sqrt(trel[0]*trel[0] + trel[1]*trel[1] + trel[2]*trel[2]);
    if (baseline < 0.01) {
      // prevデータは更新しない（もっと動いてから三角測量する）
      return [];
    }

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

    const worldPoints: Point3D[] = [];
    for (const p of localPoints) {
      const shifted = [p.x - this.prevT[0], p.y - this.prevT[1], p.z - this.prevT[2]];
      const pw = mulMV(Rpt, shifted);
      worldPoints.push({ x: pw[0], y: pw[1], z: pw[2] });
    }

    const registeredIds = commonIds.slice(0, localPoints.length);
    this.slamMap.register(registeredIds, worldPoints);

    console.log(`[SLAM] Mapper: expanded ${worldPoints.length} new 3D points (from ${commonIds.length} common)`);

    this.prevIds = unmatchedIds;
    this.prevPoints = unmatchedPoints;
    this.prevR = R;
    this.prevT = t;

    return worldPoints;
  }
}
