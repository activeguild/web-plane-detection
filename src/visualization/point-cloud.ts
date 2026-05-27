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

  draw(points: Point3D[], cameraT?: number[]): void {
    const ctx = this.ctx;
    const ox = this.originX;
    const oy = this.originY;
    const size = this.viewSize;

    // 半透明背景
    ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
    ctx.fillRect(ox, oy, size, size);

    // 枠線
    ctx.strokeStyle = '#444';
    ctx.lineWidth = 1;
    ctx.strokeRect(ox, oy, size, size);

    // ラベル
    ctx.fillStyle = '#888';
    ctx.font = '10px sans-serif';
    ctx.fillText('Bird\'s Eye (X-Z)', ox + 4, oy + 12);

    if (points.length === 0) return;

    // X-Z 平面のスケーリング（点群の範囲に合わせる）
    let minX = Infinity, maxX = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;
    for (const p of points) {
      if (p.x < minX) minX = p.x;
      if (p.x > maxX) maxX = p.x;
      if (p.z < minZ) minZ = p.z;
      if (p.z > maxZ) maxZ = p.z;
    }

    // カメラ位置も範囲に含める
    if (cameraT) {
      if (cameraT[0] < minX) minX = cameraT[0];
      if (cameraT[0] > maxX) maxX = cameraT[0];
      if (cameraT[2] < minZ) minZ = cameraT[2];
      if (cameraT[2] > maxZ) maxZ = cameraT[2];
    }

    // 原点（カメラ1）も含める
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

    // 3D点群（白い点）
    ctx.fillStyle = '#ffffff';
    for (const p of points) {
      const sx = toScreenX(p.x);
      const sy = toScreenY(p.z);
      ctx.fillRect(sx - 1, sy - 1, 2, 2);
    }

    // カメラ1（原点、緑の三角）
    this.drawCamera(toScreenX(0), toScreenY(0), '#00ff00');

    // カメラ2（推定位置、黄色の三角）
    if (cameraT) {
      this.drawCamera(toScreenX(cameraT[0]), toScreenY(cameraT[2]), '#ffff00');
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
