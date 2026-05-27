import cv from '@techstark/opencv-js';
import { Point2D } from '../features/orb';

export type PoseResult = {
  R: number[][];
  t: number[];
  inlierCount: number;
  inlierMask: boolean[];
};

// 3×3 行列の積
function mul3x3(A: number[][], B: number[][]): number[][] {
  const C: number[][] = [[0,0,0],[0,0,0],[0,0,0]];
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      for (let k = 0; k < 3; k++) {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  return C;
}

// 3×3 行列の転置
function transpose3x3(A: number[][]): number[][] {
  return [
    [A[0][0], A[1][0], A[2][0]],
    [A[0][1], A[1][1], A[2][1]],
    [A[0][2], A[1][2], A[2][2]],
  ];
}

// カメラ内部行列の逆行列
function invertK(K: number[][]): number[][] {
  const fx = K[0][0], fy = K[1][1], cx = K[0][2], cy = K[1][2];
  return [
    [1/fx, 0, -cx/fx],
    [0, 1/fy, -cy/fy],
    [0, 0, 1],
  ];
}

// 3×3 対称行列の Jacobi 固有値分解
// 戻り値: { eigenvalues: number[], eigenvectors: number[][] (列ベクトル) }
function jacobiEigen3x3(M: number[][]): { eigenvalues: number[]; eigenvectors: number[][] } {
  // コピー
  const A = M.map(r => [...r]);
  // V = 単位行列
  const V = [[1,0,0],[0,1,0],[0,0,1]];

  for (let iter = 0; iter < 100; iter++) {
    // 最大の非対角要素を見つける
    let maxVal = 0, p = 0, q = 1;
    for (let i = 0; i < 3; i++) {
      for (let j = i + 1; j < 3; j++) {
        if (Math.abs(A[i][j]) > maxVal) {
          maxVal = Math.abs(A[i][j]);
          p = i; q = j;
        }
      }
    }
    if (maxVal < 1e-12) break;

    // 回転角
    const theta = 0.5 * Math.atan2(2 * A[p][q], A[p][p] - A[q][q]);
    const c = Math.cos(theta);
    const s = Math.sin(theta);

    // A を更新
    const App = A[p][p], Aqq = A[q][q], Apq = A[p][q];
    A[p][p] = c*c*App + 2*c*s*Apq + s*s*Aqq;
    A[q][q] = s*s*App - 2*c*s*Apq + c*c*Aqq;
    A[p][q] = 0; A[q][p] = 0;

    for (let i = 0; i < 3; i++) {
      if (i === p || i === q) continue;
      const Aip = A[i][p], Aiq = A[i][q];
      A[i][p] = c*Aip + s*Aiq; A[p][i] = A[i][p];
      A[i][q] = -s*Aip + c*Aiq; A[q][i] = A[i][q];
    }

    // V を更新
    for (let i = 0; i < 3; i++) {
      const Vip = V[i][p], Viq = V[i][q];
      V[i][p] = c*Vip + s*Viq;
      V[i][q] = -s*Vip + c*Viq;
    }
  }

  return {
    eigenvalues: [A[0][0], A[1][1], A[2][2]],
    eigenvectors: V, // V[i][j] = i番目の行ベクトル、j番目の固有ベクトル
  };
}

// ホモグラフィ H からカメラの R, t の候補を求める
// H_normalized = K^-1 * H * K で正規化した H を分解
// 参考: Malis & Vargas (2007) "Deeper understanding of the homography decomposition for vision-based control"
function decomposeHomography(
  H: number[][],
  K: number[][],
): { R: number[][]; t: number[]; n: number[] }[] {
  const Kinv = invertK(K);
  // H_norm = K^-1 * H * K ... ではなく K^-1 * H
  // ホモグラフィは H ∝ K(R + t*n^T/d)K^-1 なので
  // K^-1 * H * K^-1^-1 = K^-1 * H ではない
  // 正しくは: H_norm = K^-1 * H * K  (ただし H は画像座標系)
  // しかし findHomography は画像座標で H を返すので
  // H_norm = K^-1 * H (右側の K は元の正規化座標→画像変換に対応)
  // 実際: H ∝ R + t*n^T/d (正規化座標系で)
  // なので H_norm = K^-1 * H * K^-1^(-1) は不要
  // H_norm = K^-1 * H とすれば H_norm ∝ R + t*n^T/d ... これも違う
  // 正: pixel座標の H に対して、H_norm = K^-1 * H * K が正規化座標のホモグラフィ
  // ではない。H は p2 ∝ H * p1 (pixel) で、p = K * [R+tn^T/d] * K^-1 * p1_pixel
  // よって H = K * (R + t*n^T/d) * K^-1 → K^-1 * H * K = R + t*n^T/d
  const Hn = mul3x3(mul3x3(Kinv, H), K);

  // SVD of Hn: sigma1 >= sigma2 >= sigma3 を求めるため Hn^T * Hn の固有値分解
  const HtH = mul3x3(transpose3x3(Hn), Hn);
  const { eigenvalues } = jacobiEigen3x3(HtH);

  // 固有値をソート (降順)
  const indices = [0, 1, 2].sort((a, b) => eigenvalues[b] - eigenvalues[a]);
  const s = indices.map(i => Math.sqrt(Math.max(eigenvalues[i], 0)));

  // sigma2 で正規化
  if (s[1] < 1e-10) return [];
  const sigma2 = s[1];
  const Hn_norm = Hn.map(row => row.map(v => v / sigma2));

  // 簡易分解: sigma1 ≈ sigma2 ≈ sigma3 なら純粋回転
  // それ以外なら R + t*n^T/d 分解
  // SVD ベースの厳密分解の代わりに、簡易的に R ≈ Hn_norm とする
  // (d が大きい = 平面が遠い場合、t*n^T/d → 0 で H ≈ K*R*K^-1)

  // det で符号調整
  function det3x3(M: number[][]): number {
    return M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1])
         - M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0])
         + M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);
  }

  det3x3(Hn_norm); // 符号確認用 (値は polar decomposition で自動調整)

  // R の推定: Hn_norm の最も近い回転行列 (polar decomposition)
  // R = Hn_norm * (Hn_norm^T * Hn_norm)^{-1/2}
  // 簡易版: Hn_norm を直接 SVD して U*Vt を R とする
  const HnTHn = mul3x3(transpose3x3(Hn_norm), Hn_norm);
  const eig = jacobiEigen3x3(HnTHn);
  // 固有値は polar decomposition の sqrtInv 構築に使用

  // (Hn^T * Hn)^{-1/2} を構築
  const sqrtInv: number[][] = [[0,0,0],[0,0,0],[0,0,0]];
  for (let i = 0; i < 3; i++) {
    const lam = Math.max(eig.eigenvalues[i], 1e-10);
    const invSqrt = 1 / Math.sqrt(lam);
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        sqrtInv[r][c] += invSqrt * eig.eigenvectors[r][i] * eig.eigenvectors[c][i];
      }
    }
  }

  let R = mul3x3(Hn_norm, sqrtInv);

  // det(R) = 1 を保証
  if (det3x3(R) < 0) {
    R = R.map(row => row.map(v => -v));
  }

  // t の推定: t = (H_norm - R) * n / d
  // n が不明なので、簡易的に t = H_norm の並進成分として
  // H_norm の3列目 - R の3列目 が t に比例
  const t = [
    Hn_norm[0][2] - R[0][2],
    Hn_norm[1][2] - R[1][2],
    Hn_norm[2][2] - R[2][2],
  ];

  // t を正規化
  const tNorm = Math.sqrt(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]);
  if (tNorm > 1e-10) {
    t[0] /= tNorm;
    t[1] /= tNorm;
    t[2] /= tNorm;
  }

  return [
    { R, t, n: [0, 0, 1] },
    { R, t: t.map(v => -v), n: [0, 0, -1] },
  ];
}

// cheirality check
function countInFront(
  pts1: Point2D[], pts2: Point2D[],
  R: number[][], t: number[],
  K: number[][], inlierMask: boolean[],
): number {
  const Kinv = invertK(K);
  let count = 0;

  for (let i = 0; i < pts1.length; i++) {
    if (!inlierMask[i]) continue;

    const x1 = Kinv[0][0] * pts1[i].x + Kinv[0][2];
    const y1 = Kinv[1][1] * pts1[i].y + Kinv[1][2];
    const x2 = Kinv[0][0] * pts2[i].x + Kinv[0][2];
    const y2 = Kinv[1][1] * pts2[i].y + Kinv[1][2];

    // 簡易三角測量で z 成分のみチェック
    const p2row0 = [R[0][0] - x2*R[2][0], R[0][1] - x2*R[2][1], R[0][2] - x2*R[2][2]];
    const p2row1 = [R[1][0] - y2*R[2][0], R[1][1] - y2*R[2][1], R[1][2] - y2*R[2][2]];
    const b0 = x2*t[2] - t[0];
    const b1 = y2*t[2] - t[1];

    const denom0 = p2row0[0]*x1 + p2row0[1]*y1 + p2row0[2];
    const denom1 = p2row1[0]*x1 + p2row1[1]*y1 + p2row1[2];

    let z1: number;
    if (Math.abs(denom0) > Math.abs(denom1)) {
      z1 = b0 / denom0;
    } else if (Math.abs(denom1) > 1e-10) {
      z1 = b1 / denom1;
    } else {
      continue;
    }

    if (z1 <= 0) continue;

    const z2 = R[2][0]*x1*z1 + R[2][1]*y1*z1 + R[2][2]*z1 + t[2];
    if (z2 <= 0) continue;

    count++;
  }
  return count;
}

export function estimatePose(
  prevPoints: Point2D[],
  currPoints: Point2D[],
  cameraMatrixMat: cv.Mat,
): PoseResult | null {
  const n = prevPoints.length;
  if (n < 8) return null;

  // Point2D[] → cv.Mat (N×1, CV_64FC2)
  const pts1 = new cv.Mat(n, 1, cv.CV_64FC2);
  const pts2 = new cv.Mat(n, 1, cv.CV_64FC2);
  const data1 = pts1.data64F;
  const data2 = pts2.data64F;
  for (let i = 0; i < n; i++) {
    data1[i * 2] = prevPoints[i].x;
    data1[i * 2 + 1] = prevPoints[i].y;
    data2[i * 2] = currPoints[i].x;
    data2[i * 2 + 1] = currPoints[i].y;
  }

  const mask = new cv.Mat();

  let H: cv.Mat;
  try {
    H = cv.findHomography(pts1, pts2, cv.RANSAC, 3.0, mask);
  } catch (e) {
    pts1.delete();
    pts2.delete();
    mask.delete();
    console.error('[SLAM] findHomography failed:', e);
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

  pts1.delete();
  pts2.delete();
  mask.delete();

  if (inlierCount < n * 0.3) {
    H.delete();
    console.log(`[SLAM] too few inliers: ${inlierCount}/${n}`);
    return null;
  }

  // H (cv.Mat 3×3) → number[][]
  const Harray: number[][] = [];
  for (let r = 0; r < 3; r++) {
    const row: number[] = [];
    for (let c = 0; c < 3; c++) {
      row.push(H.doubleAt(r, c));
    }
    Harray.push(row);
  }
  H.delete();

  // カメラ行列
  const K: number[][] = [];
  for (let r = 0; r < 3; r++) {
    const row: number[] = [];
    for (let c = 0; c < 3; c++) {
      row.push(cameraMatrixMat.doubleAt(r, c));
    }
    K.push(row);
  }

  // ホモグラフィ分解 → R, t 候補
  const candidates = decomposeHomography(Harray, K);
  if (candidates.length === 0) {
    console.log('[SLAM] homography decomposition failed');
    return null;
  }

  // cheirality check で最良の候補を選択
  let bestR = candidates[0].R;
  let bestT = candidates[0].t;
  let bestCount = 0;

  for (const c of candidates) {
    const cnt = countInFront(prevPoints, currPoints, c.R, c.t, K, inlierMask);
    if (cnt > bestCount) {
      bestCount = cnt;
      bestR = c.R;
      bestT = c.t;
    }
  }

  if (bestCount < 10) {
    console.log(`[SLAM] cheirality check failed: only ${bestCount} points in front`);
    return null;
  }

  console.log(`[SLAM] pose estimated: ${inlierCount}/${n} inliers, ${bestCount} in front`);
  return {
    R: bestR,
    t: bestT,
    inlierCount,
    inlierMask,
  };
}
