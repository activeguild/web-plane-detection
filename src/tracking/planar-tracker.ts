import cv from '@techstark/opencv-js';
import { Point2D } from '../features/orb';

export type PlanarTrackResult = {
  success: boolean;
  H: number[][] | null;        // 3×3 ホモグラフィ行列 (参照→現フレーム)
  matchCount: number;
  inlierCount: number;
};

export class PlanarTracker {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  private orb: any;
  private refDescriptors: cv.Mat | null = null;
  private refKeypoints: Point2D[] = [];
  private _isInitialized = false;

  constructor(nfeatures: number = 1000) {
    this.orb = new cv.ORB(nfeatures);
  }

  // 参照フレームを設定
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

    console.log(`[Track] reference set: ${this.refKeypoints.length} keypoints`);
    return this.refKeypoints.length;
  }

  // 現フレームを参照フレームとマッチングしてホモグラフィを計算
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

    // BFMatcher
    const bf = new cv.BFMatcher(cv.NORM_HAMMING, true);
    const matches = new cv.DMatchVector();
    try {
      bf.match(desc, this.refDescriptors, matches);
    } catch (e) {
      bf.delete();
      desc.delete();
      matches.delete();
      return { success: false, H: null, matchCount: 0, inlierCount: 0 };
    }

    // 距離でフィルタ + ソート
    const goodMatches: { queryIdx: number; trainIdx: number; distance: number }[] = [];
    for (let i = 0; i < matches.size(); i++) {
      const m = matches.get(i);
      if (m.distance < 70) {
        goodMatches.push({ queryIdx: m.queryIdx, trainIdx: m.trainIdx, distance: m.distance });
      }
    }
    goodMatches.sort((a, b) => a.distance - b.distance);
    matches.delete();
    bf.delete();
    desc.delete();

    if (goodMatches.length < 10) {
      return { success: false, H: null, matchCount: goodMatches.length, inlierCount: 0 };
    }

    // マッチした点のペアを構築
    const n = Math.min(goodMatches.length, 200); // 上位200件
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

    // インライア数
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

    return { success: true, H: Harr, matchCount: n, inlierCount };
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
