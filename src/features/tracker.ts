import cv from '@techstark/opencv-js';
import { OrbDetector, Point2D } from './orb';

export type TrackResult = {
  points: Point2D[];
  prevPoints: Point2D[];
  count: number;
};

export class FeatureTracker {
  private orb: OrbDetector;
  private minFeatures: number;
  private dedupDistance: number;
  private prevGray: cv.Mat | null = null;
  private prevPoints: Point2D[] = [];

  constructor(orb: OrbDetector, minFeatures: number = 200, dedupDistance: number = 20) {
    this.orb = orb;
    this.minFeatures = minFeatures;
    this.dedupDistance = dedupDistance;
  }

  process(frame: cv.Mat): TrackResult {
    const gray = new cv.Mat();
    cv.cvtColor(frame, gray, cv.COLOR_RGBA2GRAY);

    // 初回: ORB で初期特徴点を検出
    if (this.prevGray === null) {
      this.prevPoints = this.orb.detectKeypoints(gray);
      this.prevGray = gray;
      return {
        points: this.prevPoints,
        prevPoints: this.prevPoints,
        count: this.prevPoints.length,
      };
    }

    let trackedPoints: Point2D[] = [];
    let trackedPrevPoints: Point2D[] = [];

    if (this.prevPoints.length > 0) {
      // 前フレームの点を cv.Mat (N×1, CV_32FC2) に変換
      const prevPtsMat = new cv.Mat(this.prevPoints.length, 1, cv.CV_32FC2);
      const prevData = prevPtsMat.data32F;
      for (let i = 0; i < this.prevPoints.length; i++) {
        prevData[i * 2] = this.prevPoints[i].x;
        prevData[i * 2 + 1] = this.prevPoints[i].y;
      }

      const nextPtsMat = new cv.Mat();
      const statusMat = new cv.Mat();
      const errMat = new cv.Mat();

      const winSize = new cv.Size(21, 21);

      cv.calcOpticalFlowPyrLK(
        this.prevGray,
        gray,
        prevPtsMat,
        nextPtsMat,
        statusMat,
        errMat,
        winSize,
        3,
      );

      // 追跡成功した点だけ残す
      const statusData = statusMat.data;
      const nextData = nextPtsMat.data32F;
      for (let i = 0; i < this.prevPoints.length; i++) {
        if (statusData[i] === 1) {
          const nx = nextData[i * 2];
          const ny = nextData[i * 2 + 1];
          // 画像外に出た点は除外
          if (nx >= 0 && ny >= 0 && nx < frame.cols && ny < frame.rows) {
            trackedPoints.push({ x: nx, y: ny });
            trackedPrevPoints.push(this.prevPoints[i]);
          }
        }
      }

      prevPtsMat.delete();
      nextPtsMat.delete();
      statusMat.delete();
      errMat.delete();
    }

    // 特徴点が閾値以下なら ORB で補充
    if (trackedPoints.length < this.minFeatures) {
      const newPoints = this.orb.detectKeypoints(gray);
      const filtered = this.filterDuplicates(newPoints, trackedPoints);
      const needed = this.minFeatures - trackedPoints.length;
      const toAdd = filtered.slice(0, needed);
      for (const p of toAdd) {
        trackedPoints.push(p);
        trackedPrevPoints.push(p); // 新規点は移動ベクトルなし
      }
    }

    // 状態更新
    const oldGray = this.prevGray;
    this.prevGray = gray;
    this.prevPoints = trackedPoints;
    oldGray.delete();

    return {
      points: trackedPoints,
      prevPoints: trackedPrevPoints,
      count: trackedPoints.length,
    };
  }

  private filterDuplicates(newPoints: Point2D[], existing: Point2D[]): Point2D[] {
    const distSq = this.dedupDistance * this.dedupDistance;
    return newPoints.filter((np) => {
      return !existing.some((ep) => {
        const dx = np.x - ep.x;
        const dy = np.y - ep.y;
        return dx * dx + dy * dy < distSq;
      });
    });
  }

  dispose(): void {
    if (this.prevGray) {
      this.prevGray.delete();
      this.prevGray = null;
    }
  }
}
