import cv from '@techstark/opencv-js';
import { OrbDetector, Point2D } from './orb';

export type TrackResult = {
  points: Point2D[];
  prevPoints: Point2D[];
  ids: number[];
  count: number;
  avgMotion: number;
};

export class FeatureTracker {
  private orb: OrbDetector;
  private minFeatures: number;
  private dedupDistance: number;
  private prevGray: cv.Mat | null = null;
  private prevPoints: Point2D[] = [];
  private nextId: number = 0;

  constructor(orb: OrbDetector, minFeatures: number = 200, dedupDistance: number = 20) {
    this.orb = orb;
    this.minFeatures = minFeatures;
    this.dedupDistance = dedupDistance;
  }

  private assignIds(points: Point2D[]): void {
    for (const p of points) {
      if (p.id === -1) {
        p.id = this.nextId++;
      }
    }
  }

  process(frame: cv.Mat): TrackResult {
    const gray = new cv.Mat();
    cv.cvtColor(frame, gray, cv.COLOR_RGBA2GRAY);

    if (this.prevGray === null) {
      this.prevPoints = this.orb.detectKeypoints(gray);
      this.assignIds(this.prevPoints);
      this.prevGray = gray;
      const ids = this.prevPoints.map(p => p.id);
      return {
        points: this.prevPoints,
        prevPoints: this.prevPoints,
        ids,
        count: this.prevPoints.length,
        avgMotion: 0,
      };
    }

    let trackedPoints: Point2D[] = [];
    let trackedPrevPoints: Point2D[] = [];

    if (this.prevPoints.length > 0) {
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
        this.prevGray, gray, prevPtsMat, nextPtsMat,
        statusMat, errMat, winSize, 3,
      );

      const statusData = statusMat.data;
      const nextData = nextPtsMat.data32F;
      for (let i = 0; i < this.prevPoints.length; i++) {
        if (statusData[i] === 1) {
          const nx = nextData[i * 2];
          const ny = nextData[i * 2 + 1];
          if (nx >= 0 && ny >= 0 && nx < frame.cols && ny < frame.rows) {
            trackedPoints.push({ x: nx, y: ny, id: this.prevPoints[i].id });
            trackedPrevPoints.push(this.prevPoints[i]);
          }
        }
      }

      prevPtsMat.delete();
      nextPtsMat.delete();
      statusMat.delete();
      errMat.delete();
    }

    if (trackedPoints.length < this.minFeatures) {
      const newPoints = this.orb.detectKeypoints(gray);
      this.assignIds(newPoints);
      const filtered = this.filterDuplicates(newPoints, trackedPoints);
      const needed = this.minFeatures - trackedPoints.length;
      const toAdd = filtered.slice(0, needed);
      for (const p of toAdd) {
        trackedPoints.push(p);
        trackedPrevPoints.push(p);
      }
    }

    const oldGray = this.prevGray;
    this.prevGray = gray;
    this.prevPoints = trackedPoints;
    oldGray.delete();

    let totalMotion = 0;
    let motionCount = 0;
    for (let i = 0; i < trackedPoints.length; i++) {
      const dx = trackedPoints[i].x - trackedPrevPoints[i].x;
      const dy = trackedPoints[i].y - trackedPrevPoints[i].y;
      const dist = Math.sqrt(dx * dx + dy * dy);
      if (dist > 0) {
        totalMotion += dist;
        motionCount++;
      }
    }
    const avgMotion = motionCount > 0 ? totalMotion / motionCount : 0;

    const ids = trackedPoints.map(p => p.id);

    return {
      points: trackedPoints,
      prevPoints: trackedPrevPoints,
      ids,
      count: trackedPoints.length,
      avgMotion,
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
