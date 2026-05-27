import { Point3D } from '../geometry/triangulation';

export class PlaneOverlay {
  private ctx: CanvasRenderingContext2D;

  constructor(ctx: CanvasRenderingContext2D) {
    this.ctx = ctx;
  }

  private logCount = 0;

  draw(inliers: Point3D[], R: number[][], t: number[], K: number[][]): void {
    const projected: { x: number; y: number }[] = [];
    let behindCount = 0;
    for (const p of inliers) {
      const cx = R[0][0] * p.x + R[0][1] * p.y + R[0][2] * p.z + t[0];
      const cy = R[1][0] * p.x + R[1][1] * p.y + R[1][2] * p.z + t[1];
      const cz = R[2][0] * p.x + R[2][1] * p.y + R[2][2] * p.z + t[2];

      if (cz <= 0) { behindCount++; continue; }

      const u = (K[0][0] * cx + K[0][2] * cz) / cz;
      const v = (K[1][1] * cy + K[1][2] * cz) / cz;

      projected.push({ x: u, y: v });
    }

    if (this.logCount < 5) {
      console.log(`[SLAM] PlaneOverlay: ${projected.length} projected, ${behindCount} behind, t=[${t.map(v=>v.toFixed(3))}]`);
      if (projected.length > 0) {
        const p0 = projected[0];
        console.log(`[SLAM] PlaneOverlay: first point (${p0.x.toFixed(1)}, ${p0.y.toFixed(1)})`);
      }
      this.logCount++;
    }

    if (projected.length < 3) return;

    const hull = this.convexHull(projected);
    if (hull.length < 3) return;

    const ctx = this.ctx;

    ctx.fillStyle = 'rgba(0, 120, 255, 0.3)';
    ctx.beginPath();
    ctx.moveTo(hull[0].x, hull[0].y);
    for (let i = 1; i < hull.length; i++) {
      ctx.lineTo(hull[i].x, hull[i].y);
    }
    ctx.closePath();
    ctx.fill();

    ctx.strokeStyle = 'rgba(0, 120, 255, 0.7)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(hull[0].x, hull[0].y);
    for (let i = 1; i < hull.length; i++) {
      ctx.lineTo(hull[i].x, hull[i].y);
    }
    ctx.closePath();
    ctx.stroke();
  }

  private convexHull(points: { x: number; y: number }[]): { x: number; y: number }[] {
    const pts = [...points].sort((a, b) => a.x - b.x || a.y - b.y);
    const n = pts.length;
    if (n <= 2) return pts;

    function cross(O: { x: number; y: number }, A: { x: number; y: number }, B: { x: number; y: number }): number {
      return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
    }

    const lower: { x: number; y: number }[] = [];
    for (const p of pts) {
      while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) {
        lower.pop();
      }
      lower.push(p);
    }

    const upper: { x: number; y: number }[] = [];
    for (let i = n - 1; i >= 0; i--) {
      while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], pts[i]) <= 0) {
        upper.pop();
      }
      upper.push(pts[i]);
    }

    lower.pop();
    upper.pop();

    return lower.concat(upper);
  }
}
