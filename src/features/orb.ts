import cv from '@techstark/opencv-js';

export interface Point2D {
  x: number;
  y: number;
  id: number;
}

export class OrbDetector {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  private orb: any;

  constructor(nfeatures: number = 500) {
    this.orb = new cv.ORB(nfeatures);
  }

  // キーポイント + ディスクリプタを返す（呼び出し側で descriptors.delete() 必須）
  detectWithDescriptors(gray: cv.Mat): { keypoints: Point2D[]; descriptors: cv.Mat } {
    const kpVec = new cv.KeyPointVector();
    const desc = new cv.Mat();
    const mask = new cv.Mat();
    this.orb.detect(gray, kpVec);
    this.orb.compute(gray, kpVec, desc);
    mask.delete();

    const keypoints: Point2D[] = [];
    for (let i = 0; i < kpVec.size(); i++) {
      const kp = kpVec.get(i);
      keypoints.push({ x: kp.pt.x, y: kp.pt.y, id: -1 });
    }

    kpVec.delete();
    return { keypoints, descriptors: desc };
  }

  detectKeypoints(gray: cv.Mat): Point2D[] {
    const keypoints = new cv.KeyPointVector();
    this.orb.detect(gray, keypoints);

    const points: Point2D[] = [];
    for (let i = 0; i < keypoints.size(); i++) {
      const kp = keypoints.get(i);
      points.push({ x: kp.pt.x, y: kp.pt.y, id: -1 });
    }

    keypoints.delete();
    return points;
  }

  detectAndDraw(frame: cv.Mat): void {
    const gray = new cv.Mat();
    cv.cvtColor(frame, gray, cv.COLOR_RGBA2GRAY);

    const keypoints = new cv.KeyPointVector();
    this.orb.detect(gray, keypoints);
    cv.drawKeypoints(frame, keypoints, frame);

    gray.delete();
    keypoints.delete();
  }

  dispose(): void {
    this.orb.delete();
  }
}
