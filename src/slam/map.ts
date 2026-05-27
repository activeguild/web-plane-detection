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

  get size(): number {
    return this.map.size;
  }
}
