import cv from '@techstark/opencv-js';

export class OrbDetector {
  private orb: cv.ORB;

  constructor(nfeatures: number = 500) {
    this.orb = new cv.ORB(nfeatures);
  }

  detectAndDraw(frame: cv.Mat): void {
    const gray = new cv.Mat();
    cv.cvtColor(frame, gray, cv.COLOR_RGBA2GRAY);

    const keypoints = new cv.KeyPointVector();
    const descriptors = new cv.Mat();

    this.orb.detect(gray, keypoints);

    cv.drawKeypoints(frame, keypoints, frame);

    gray.delete();
    keypoints.delete();
    descriptors.delete();
  }

  dispose(): void {
    this.orb.delete();
  }
}
