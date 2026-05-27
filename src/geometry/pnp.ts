import cv from '@techstark/opencv-js';
import { Point2D } from '../features/orb';
import { Point3D } from './triangulation';

export type PnPResult = {
  R: number[][];
  t: number[];
  inlierCount: number;
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

export function estimatePosePnP(
  points3D: Point3D[],
  points2D: Point2D[],
  cameraMatrixMat: cv.Mat,
): PnPResult | null {
  const n = points3D.length;
  if (n < 6) return null;

  const objPts = new cv.Mat(n, 1, cv.CV_64FC3);
  const objData = objPts.data64F;
  for (let i = 0; i < n; i++) {
    objData[i * 3] = points3D[i].x;
    objData[i * 3 + 1] = points3D[i].y;
    objData[i * 3 + 2] = points3D[i].z;
  }

  const imgPts = new cv.Mat(n, 1, cv.CV_64FC2);
  const imgData = imgPts.data64F;
  for (let i = 0; i < n; i++) {
    imgData[i * 2] = points2D[i].x;
    imgData[i * 2 + 1] = points2D[i].y;
  }

  const distCoeffs = new cv.Mat();
  const rvec = new cv.Mat();
  const tvec = new cv.Mat();
  const inliers = new cv.Mat();

  let success: boolean;
  try {
    success = cv.solvePnPRansac(
      objPts, imgPts, cameraMatrixMat, distCoeffs,
      rvec, tvec, false, 100, 8.0, 0.99, inliers,
    );
  } catch (e) {
    objPts.delete();
    imgPts.delete();
    distCoeffs.delete();
    rvec.delete();
    tvec.delete();
    inliers.delete();
    console.error('[SLAM] solvePnPRansac failed:', e);
    return null;
  }

  if (!success) {
    objPts.delete();
    imgPts.delete();
    distCoeffs.delete();
    rvec.delete();
    tvec.delete();
    inliers.delete();
    return null;
  }

  const R = new cv.Mat();
  cv.Rodrigues(rvec, R);

  const result: PnPResult = {
    R: matToArray2D(R, 3, 3),
    t: [tvec.doubleAt(0, 0), tvec.doubleAt(1, 0), tvec.doubleAt(2, 0)],
    inlierCount: inliers.rows,
  };

  objPts.delete();
  imgPts.delete();
  distCoeffs.delete();
  rvec.delete();
  tvec.delete();
  inliers.delete();
  R.delete();

  return result;
}
