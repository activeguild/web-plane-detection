import cv from '@techstark/opencv-js';
import { Point2D } from '../features/orb';

export type PlanarTrackResult = {
  success: boolean;
  H: number[][] | null;
  matchCount: number;
  inlierCount: number;
};

export class PlanarTracker {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  private orb: any;
  private refDescriptors: cv.Mat | null = null;
  private refKeypoints: Point2D[] = [];
  private _isInitialized = false;

  // ホモグラフィの時間的平滑化
  private prevH: number[][] | null = null;
  private readonly smoothAlpha = 0.3; // 新フレーム 30%, 前フレーム 70%

  constructor(nfeatures: number = 1000) {
    this.orb = new cv.ORB(nfeatures);
  }

  setReference(gray: cv.Mat): number {
    if (this.refDescriptors) {
      this.refDescriptors.delete();
    }

    const kpVec = new cv.KeyPointVector();
    const desc = new cv.Mat();
    this.orb.detect(gray, kpVec);
    this.orb.compute(gray, kpVec, desc);

    this.refKeypoints = [];
    for (let i = 0; i < kpVec.size(); i++) {
      const kp = kpVec.get(i);
      this.refKeypoints.push({ x: kp.pt.x, y: kp.pt.y, id: i });
    }
    kpVec.delete();

    this.refDescriptors = desc;
    this._isInitialized = true;
    this.prevH = null;

    console.log(`[Track] reference set: ${this.refKeypoints.length} keypoints`);
    return this.refKeypoints.length;
  }

  track(gray: cv.Mat): PlanarTrackResult {
    if (!this._isInitialized || !this.refDescriptors) {
      return { success: false, H: null, matchCount: 0, inlierCount: 0 };
    }

    // 現フレームの ORB
    const kpVec = new cv.KeyPointVector();
    const desc = new cv.Mat();
    this.orb.detect(gray, kpVec);
    this.orb.compute(gray, kpVec, desc);

    const currKeypoints: Point2D[] = [];
    for (let i = 0; i < kpVec.size(); i++) {
      const kp = kpVec.get(i);
      currKeypoints.push({ x: kp.pt.x, y: kp.pt.y, id: i });
    }
    kpVec.delete();

    if (desc.rows < 8 || this.refDescriptors.rows < 8) {
      desc.delete();
      return { success: false, H: null, matchCount: 0, inlierCount: 0 };
    }

    // knnMatch + Lowe's ratio test (crossCheck=false)
    const bf = new cv.BFMatcher(cv.NORM_HAMMING, false);
    const matchesVec = new cv.DMatchVectorVector();
    try {
      bf.knnMatch(desc, this.refDescriptors, matchesVec, 2);
    } catch (e) {
      bf.delete();
      desc.delete();
      matchesVec.delete();
      return { success: false, H: null, matchCount: 0, inlierCount: 0 };
    }

    // Lowe's ratio test: 1番目 / 2番目 < 0.75
    const goodMatches: { queryIdx: number; trainIdx: number }[] = [];
    for (let i = 0; i < matchesVec.size(); i++) {
      const knn = matchesVec.get(i);
      if (knn.size() >= 2) {
        const m1 = knn.get(0);
        const m2 = knn.get(1);
        if (m1.distance < 0.75 * m2.distance) {
          goodMatches.push({ queryIdx: m1.queryIdx, trainIdx: m1.trainIdx });
        }
      }
    }
    matchesVec.delete();
    bf.delete();
    desc.delete();

    if (goodMatches.length < 10) {
      return { success: false, H: null, matchCount: goodMatches.length, inlierCount: 0 };
    }

    // マッチした点のペアを構築
    const n = goodMatches.length;
    const srcPts = new cv.Mat(n, 1, cv.CV_32FC2);
    const dstPts = new cv.Mat(n, 1, cv.CV_32FC2);
    const srcData = srcPts.data32F;
    const dstData = dstPts.data32F;

    for (let i = 0; i < n; i++) {
      const m = goodMatches[i];
      const refPt = this.refKeypoints[m.trainIdx];
      const curPt = currKeypoints[m.queryIdx];
      srcData[i * 2] = refPt.x;
      srcData[i * 2 + 1] = refPt.y;
      dstData[i * 2] = curPt.x;
      dstData[i * 2 + 1] = curPt.y;
    }

    // findHomography (RANSAC)
    const mask = new cv.Mat();
    let H: cv.Mat;
    try {
      H = cv.findHomography(srcPts, dstPts, cv.RANSAC, 3.0, mask);
    } catch (e) {
      srcPts.delete();
      dstPts.delete();
      mask.delete();
      return { success: false, H: null, matchCount: n, inlierCount: 0 };
    }

    srcPts.delete();
    dstPts.delete();

    let inlierCount = 0;
    for (let i = 0; i < mask.rows; i++) {
      if (mask.data[i] === 1) inlierCount++;
    }
    mask.delete();

    if (inlierCount < 8 || H.rows !== 3 || H.cols !== 3) {
      H.delete();
      return { success: false, H: null, matchCount: n, inlierCount };
    }

    // cv.Mat → number[][]
    const Harr: number[][] = [];
    for (let r = 0; r < 3; r++) {
      const row: number[] = [];
      for (let c = 0; c < 3; c++) {
        row.push(H.doubleAt(r, c));
      }
      Harr.push(row);
    }
    H.delete();

    // 適応的平滑化: H の変化量に応じて alpha を調整
    let smoothed: number[][];
    if (this.prevH === null) {
      smoothed = Harr;
    } else {
      // H の変化量を計算（各要素の差の二乗和）
      let diff = 0;
      for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
          const d = Harr[i][j] - this.prevH[i][j];
          diff += d * d;
        }
      }
      // 変化が大きい → alpha 高く（追従優先）、小さい → alpha 低く（安定優先）
      const alpha = Math.min(0.9, Math.max(0.15, diff * 50));
      smoothed = Harr.map((row, i) =>
        row.map((v, j) => alpha * v + (1 - alpha) * this.prevH![i][j])
      );
    }
    this.prevH = smoothed;

    return { success: true, H: smoothed, matchCount: n, inlierCount };
  }

  get isInitialized(): boolean {
    return this._isInitialized;
  }

  dispose(): void {
    if (this.refDescriptors) {
      this.refDescriptors.delete();
      this.refDescriptors = null;
    }
  }
}
