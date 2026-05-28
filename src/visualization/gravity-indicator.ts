export class GravityIndicator {
  private ctx: CanvasRenderingContext2D;
  private size = 50;
  private ox: number;
  private oy: number;

  constructor(ctx: CanvasRenderingContext2D, _canvasHeight: number) {
    this.ctx = ctx;
    this.ox = 10 + this.size;
    this.oy = 10 + this.size;
  }

  draw(gravity: { x: number; y: number; z: number }): void {
    const ctx = this.ctx;
    const cx = this.ox;
    const cy = this.oy;
    const r = this.size / 2;

    ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = '#444';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = '#888';
    ctx.font = '8px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('G', cx, cy - r + 10);

    const mag = Math.sqrt(gravity.x * gravity.x + gravity.y * gravity.y + gravity.z * gravity.z);
    if (mag < 0.01) return;

    const gx = gravity.x / mag;
    const gz = gravity.z / mag;

    const arrowLen = r * 0.7;
    const endX = cx + gx * arrowLen;
    const endY = cy + gz * arrowLen;

    ctx.strokeStyle = '#00ffff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(endX, endY);
    ctx.stroke();

    const headLen = 6;
    const angle = Math.atan2(gz, gx);
    ctx.fillStyle = '#00ffff';
    ctx.beginPath();
    ctx.moveTo(endX, endY);
    ctx.lineTo(
      endX - headLen * Math.cos(angle - 0.4),
      endY - headLen * Math.sin(angle - 0.4),
    );
    ctx.lineTo(
      endX - headLen * Math.cos(angle + 0.4),
      endY - headLen * Math.sin(angle + 0.4),
    );
    ctx.closePath();
    ctx.fill();

    ctx.textAlign = 'left';
  }
}
