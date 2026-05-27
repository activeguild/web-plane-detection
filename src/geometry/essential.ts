import cv from '@techstark/opencv-js';
import { Point2D } from '../features/orb';

export type PoseResult = {
  R: number[][];
  t: number[];
  inlierCount: number;
  inlierMask: boolean[];
};

function matToArray2D(mat: cv.Mat, rows: number, cols: number): number[][] {
  const result: number[][] = [];
  for (let r = 0; r < rows; r++) {
    const row: number[] = [];
    for (let c = 0; c < cols; c++) {
      row.push(mat.doubleAt(r, c));
    }
    result.push(row);
  }
  return result;
}

function matToArray1D(mat: cv.Mat, length: number): number[] {
  const result: number[] = [];
  for (let i = 0; i < length; i++) {
    result.push(mat.doubleAt(i, 0));
  }
  return result;
}

export function estimatePose(
  prevPoints: Point2D[],
  currPoints: Point2D[],
  focalLength: number,
  principalPoint: { x: number; y: number },
): PoseResult | null {
  const n = prevPoints.length;
  if (n < 8) return null;

  // Point2D[] → cv.Mat (N×1, CV_32FC2)
  const pts1 = new cv.Mat(n, 1, cv.CV_32FC2);
  const pts2 = new cv.Mat(n, 1, cv.CV_32FC2);
  const data1 = pts1.data32F;
  const data2 = pts2.data32F;
  for (let i = 0; i < n; i++) {
    data1[i * 2] = prevPoints[i].x;
    data1[i * 2 + 1] = prevPoints[i].y;
    data2[i * 2] = currPoints[i].x;
    data2[i * 2 + 1] = currPoints[i].y;
  }

  const pp = new cv.Point(principalPoint.x, principalPoint.y);
  const mask = new cv.Mat();

  let E: cv.Mat;
  try {
    E = cv.findEssentialMat(pts1, pts2, focalLength, pp, cv.RANSAC, 0.999, 1.0, mask);
  } catch (e) {
    pts1.delete();
    pts2.delete();
    mask.delete();
    console.error('[SLAM] findEssentialMat failed:', e);
    return null;
  }

  // インライア数チェック
  const maskData = mask.data;
  let inlierCount = 0;
  const inlierMask: boolean[] = [];
  for (let i = 0; i < n; i++) {
    const isInlier = maskData[i] === 1;
    inlierMask.push(isInlier);
    if (isInlier) inlierCount++;
  }

  if (inlierCount < n * 0.3) {
    E.delete();
    pts1.delete();
    pts2.delete();
    mask.delete();
    console.log(`[SLAM] too few inliers: ${inlierCount}/${n}`);
    return null;
  }

  // R, t を復元
  const R = new cv.Mat();
  const t = new cv.Mat();
  const recoverMask = new cv.Mat();

  try {
    cv.recoverPose(E, pts1, pts2, R, t, focalLength, pp, recoverMask);
  } catch (e) {
    E.delete();
    pts1.delete();
    pts2.delete();
    mask.delete();
    R.delete();
    t.delete();
    recoverMask.delete();
    console.error('[SLAM] recoverPose failed:', e);
    return null;
  }

  const result: PoseResult = {
    R: matToArray2D(R, 3, 3),
    t: matToArray1D(t, 3),
    inlierCount,
    inlierMask,
  };

  E.delete();
  pts1.delete();
  pts2.delete();
  mask.delete();
  R.delete();
  t.delete();
  recoverMask.delete();

  console.log(`[SLAM] pose estimated: ${inlierCount}/${n} inliers`);
  return result;
}
