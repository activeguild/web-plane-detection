import cv from '@techstark/opencv-js';

export class CameraCalibration {
  readonly fx: number;
  readonly fy: number;
  readonly cx: number;
  readonly cy: number;

  constructor(imageWidth: number, imageHeight: number) {
    this.fx = imageWidth * 0.9;
    this.fy = imageWidth * 0.9;
    this.cx = imageWidth / 2;
    this.cy = imageHeight / 2;
  }

  getCameraMatrix(): number[][] {
    return [
      [this.fx, 0, this.cx],
      [0, this.fy, this.cy],
      [0, 0, 1],
    ];
  }

  getCameraMatrixAsMat(): cv.Mat {
    const K = cv.matFromArray(3, 3, cv.CV_64FC1, [
      this.fx, 0, this.cx,
      0, this.fy, this.cy,
      0, 0, 1,
    ]);
    return K;
  }

  getFocalLength(): number {
    return this.fx;
  }

  getPrincipalPoint(): { x: number; y: number } {
    return { x: this.cx, y: this.cy };
  }
}
