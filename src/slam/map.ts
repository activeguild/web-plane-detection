import { Point2D } from '../features/orb';
import { Point3D } from '../geometry/triangulation';

export class SlamMap {
  private map: Map<number, Point3D> = new Map();

  register(ids: number[], points3D: Point3D[]): void {
    for (let i = 0; i < ids.length; i++) {
      this.map.set(ids[i], points3D[i]);
    }
    console.log(`[SLAM] SlamMap: registered ${ids.length} points, total=${this.map.size}`);
  }

  get3D2DPairs(ids: number[], points2D: Point2D[]): { points3D: Point3D[]; points2D: Point2D[] } {
    const matched3D: Point3D[] = [];
    const matched2D: Point2D[] = [];
    for (let i = 0; i < ids.length; i++) {
      const pt3d = this.map.get(ids[i]);
      if (pt3d) {
        matched3D.push(pt3d);
        matched2D.push(points2D[i]);
      }
    }
    return { points3D: matched3D, points2D: matched2D };
  }

  // 再ローカライゼーション: 3D点を最後の姿勢で2Dに再投影し、
  // 現在の追跡点と最近傍マッチングして ID を再割り当てする
  relocalize(
    ids: number[],
    points2D: Point2D[],
    R: number[][],
    t: number[],
    K: number[][],
    maxDist: number = 100,
  ): number {
    // 全 3D 点を 2D に投影
    const projected: { mapId: number; u: number; v: number }[] = [];
    for (const [mapId, pt3d] of this.map) {
      const cx = R[0][0]*pt3d.x + R[0][1]*pt3d.y + R[0][2]*pt3d.z + t[0];
      const cy = R[1][0]*pt3d.x + R[1][1]*pt3d.y + R[1][2]*pt3d.z + t[1];
      const cz = R[2][0]*pt3d.x + R[2][1]*pt3d.y + R[2][2]*pt3d.z + t[2];
      if (cz <= 0) continue;

      const u = (K[0][0] * cx + K[0][2] * cz) / cz;
      const v = (K[1][1] * cy + K[1][2] * cz) / cz;
      projected.push({ mapId, u, v });
    }

    if (projected.length === 0) return 0;

    // 現在の追跡点のうち、SlamMap に未登録のものだけ対象
    const unmatchedIndices: number[] = [];
    for (let i = 0; i < ids.length; i++) {
      if (!this.map.has(ids[i])) {
        unmatchedIndices.push(i);
      }
    }

    // 投影点と追跡点の最近傍マッチング
    const maxDistSq = maxDist * maxDist;
    const usedMapIds = new Set<number>();
    let matchCount = 0;

    for (const idx of unmatchedIndices) {
      const px = points2D[idx].x;
      const py = points2D[idx].y;
      let bestDist = maxDistSq;
      let bestMapId = -1;

      for (const proj of projected) {
        if (usedMapIds.has(proj.mapId)) continue;
        const dx = px - proj.u;
        const dy = py - proj.v;
        const distSq = dx * dx + dy * dy;
        if (distSq < bestDist) {
          bestDist = distSq;
          bestMapId = proj.mapId;
        }
      }

      if (bestMapId >= 0) {
        // 追跡点の ID で既存の 3D 点を再登録
        const pt3d = this.map.get(bestMapId)!;
        this.map.set(ids[idx], pt3d);
        usedMapIds.add(bestMapId);
        matchCount++;
      }
    }

    if (matchCount > 0) {
      console.log(`[SLAM] relocalized: ${matchCount} points re-matched`);
    }
    return matchCount;
  }

  has(id: number): boolean {
    return this.map.has(id);
  }

  get size(): number {
    return this.map.size;
  }
}
