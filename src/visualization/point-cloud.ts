import { Point3D } from '../geometry/triangulation';

export class PointCloudView {
  private ctx: CanvasRenderingContext2D;
  private viewSize: number = 200;
  private originX: number;
  private originY: number;

  constructor(ctx: CanvasRenderingContext2D, canvasWidth: number, canvasHeight: number) {
    this.ctx = ctx;
    this.originX = canvasWidth - this.viewSize - 10;
    this.originY = canvasHeight - this.viewSize - 10;
  }

  draw(points: Point3D[], trajectory: { x: number; z: number }[]): void {
    const ctx = this.ctx;
    const ox = this.originX;
    const oy = this.originY;
    const size = this.viewSize;

    ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
    ctx.fillRect(ox, oy, size, size);

    ctx.strokeStyle = '#444';
    ctx.lineWidth = 1;
    ctx.strokeRect(ox, oy, size, size);

    ctx.fillStyle = '#888';
    ctx.font = '10px sans-serif';
    ctx.fillText('Bird\'s Eye (X-Z)', ox + 4, oy + 12);

    if (points.length === 0) return;

    let minX = Infinity, maxX = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;
    for (const p of points) {
      if (p.x < minX) minX = p.x;
      if (p.x > maxX) maxX = p.x;
      if (p.z < minZ) minZ = p.z;
      if (p.z > maxZ) maxZ = p.z;
    }

    for (const t of trajectory) {
      if (t.x < minX) minX = t.x;
      if (t.x > maxX) maxX = t.x;
      if (t.z < minZ) minZ = t.z;
      if (t.z > maxZ) maxZ = t.z;
    }

    if (0 < minX) minX = 0;
    if (0 > maxX) maxX = 0;
    if (0 < minZ) minZ = 0;
    if (0 > maxZ) maxZ = 0;

    const rangeX = maxX - minX || 1;
    const rangeZ = maxZ - minZ || 1;
    const margin = 20;
    const drawSize = size - margin * 2;
    const scale = Math.min(drawSize / rangeX, drawSize / rangeZ);

    const toScreenX = (x: number) => ox + margin + (x - minX) * scale;
    const toScreenY = (z: number) => oy + margin + (z - minZ) * scale;

    ctx.fillStyle = '#ffffff';
    for (const p of points) {
      const sx = toScreenX(p.x);
      const sy = toScreenY(p.z);
      ctx.fillRect(sx - 1, sy - 1, 2, 2);
    }

    if (trajectory.length > 1) {
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.6)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(toScreenX(trajectory[0].x), toScreenY(trajectory[0].z));
      for (let i = 1; i < trajectory.length; i++) {
        ctx.lineTo(toScreenX(trajectory[i].x), toScreenY(trajectory[i].z));
      }
      ctx.stroke();
    }

    this.drawCamera(toScreenX(0), toScreenY(0), '#00ff00');

    if (trajectory.length > 0) {
      const latest = trajectory[trajectory.length - 1];
      this.drawCamera(toScreenX(latest.x), toScreenY(latest.z), '#ffff00');
    }
  }

  private drawCamera(sx: number, sy: number, color: string): void {
    const ctx = this.ctx;
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(sx, sy - 5);
    ctx.lineTo(sx - 4, sy + 3);
    ctx.lineTo(sx + 4, sy + 3);
    ctx.closePath();
    ctx.fill();
  }
}
