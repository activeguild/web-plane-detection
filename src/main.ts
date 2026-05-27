import cv from '@techstark/opencv-js';
import { initCamera } from './camera/capture';
import { CameraCalibration } from './camera/calibration';
import { OrbDetector } from './features/orb';
import { FeatureTracker } from './features/tracker';
import { estimatePose } from './geometry/essential';
import { estimatePosePnP } from './geometry/pnp';
import { triangulatePoints, Point3D } from './geometry/triangulation';
import { detectPlane, PlaneResult } from './plane/ransac';
import { SlamMap } from './slam/map';
import { PlaneOverlay } from './visualization/plane-overlay';
import { PointCloudView } from './visualization/point-cloud';

function waitForOpenCv(): Promise<void> {
  return new Promise((resolve) => {
    if (cv.Mat) {
      resolve();
      return;
    }
    cv.onRuntimeInitialized = () => resolve();
  });
}

async function main() {
  const video = document.getElementById('video') as HTMLVideoElement;
  const canvas = document.getElementById('canvas') as HTMLCanvasElement;
  const loading = document.getElementById('loading') as HTMLDivElement;
  const ctx = canvas.getContext('2d')!;

  console.log('[SLAM] main() started');

  loading.textContent = 'OpenCV.js を読み込み中...';
  await waitForOpenCv();
  console.log('[SLAM] OpenCV.js ready');

  loading.textContent = 'カメラを起動中...';
  await initCamera(video);
  const w = video.videoWidth;
  const h = video.videoHeight;
  console.log(`[SLAM] camera ready: ${w}x${h}`);
  canvas.width = w;
  canvas.height = h;

  loading.style.display = 'none';

  // モジュール初期化
  const calibration = new CameraCalibration(w, h);
  const orb = new OrbDetector(500);
  const tracker = new FeatureTracker(orb, 200);
  const slamMap = new SlamMap();
  const pointCloudView = new PointCloudView(ctx, w, h);
  const planeOverlay = new PlaneOverlay(ctx);
  const K = calibration.getCameraMatrixAsMat();
  const Karray = calibration.getCameraMatrix();

  const offscreen = document.createElement('canvas');
  offscreen.width = w;
  offscreen.height = h;
  const offCtx = offscreen.getContext('2d')!;

  // 状態
  let initialized = false;
  let points3D: Point3D[] = [];
  let planeResult: PlaneResult | null = null;
  let currentR: number[][] | null = null;
  let currentT: number[] | null = null;
  const trajectory: { x: number; z: number }[] = [{ x: 0, z: 0 }];
  const MOTION_THRESHOLD = 15;

  console.log('[SLAM] starting tracking loop');

  let frameCount = 0;
  function processFrame() {
    try {
      // フレーム取得
      offCtx.drawImage(video, 0, 0, w, h);
      const imageData = offCtx.getImageData(0, 0, w, h);
      const frame = cv.matFromImageData(imageData);

      // 追跡
      const result = tracker.process(frame);
      frame.delete();

      // 描画: 映像
      ctx.drawImage(video, 0, 0, w, h);

      // 描画: 移動ベクトル（赤い線）
      ctx.strokeStyle = '#ff0000';
      ctx.lineWidth = 1.5;
      for (let i = 0; i < result.count; i++) {
        const prev = result.prevPoints[i];
        const curr = result.points[i];
        if (prev.x !== curr.x || prev.y !== curr.y) {
          ctx.beginPath();
          ctx.moveTo(prev.x, prev.y);
          ctx.lineTo(curr.x, curr.y);
          ctx.stroke();
        }
      }

      // 描画: 特徴点マーカー（緑の円）
      ctx.fillStyle = '#00ff00';
      for (let i = 0; i < result.count; i++) {
        const p = result.points[i];
        ctx.beginPath();
        ctx.arc(p.x, p.y, 3, 0, Math.PI * 2);
        ctx.fill();
      }

      if (!initialized) {
        // --- 未初期化: ホモグラフィ初期化 ---
        if (frameCount % 30 === 0) {
          console.log(`[SLAM] avgMotion=${result.avgMotion.toFixed(1)}, count=${result.count}`);
        }

        if (result.avgMotion > MOTION_THRESHOLD && result.count >= 30) {
          console.log(`[SLAM] initialization trigger: avgMotion=${result.avgMotion.toFixed(1)}`);

          const pose = estimatePose(result.prevPoints, result.points, K);

          if (pose) {
            const inlierIds: number[] = [];
            const inlierPrev: { x: number; y: number; id: number }[] = [];
            const inlierCurr: { x: number; y: number; id: number }[] = [];
            for (let i = 0; i < result.count; i++) {
              if (pose.inlierMask[i]) {
                inlierIds.push(result.ids[i]);
                inlierPrev.push(result.prevPoints[i]);
                inlierCurr.push(result.points[i]);
              }
            }

            points3D = triangulatePoints(
              inlierPrev, inlierCurr,
              pose.R, pose.t,
              Karray,
              inlierIds.map(() => true),
            );

            if (points3D.length > 10) {
              slamMap.register(inlierIds.slice(0, points3D.length), points3D);
              trajectory.push({ x: pose.t[0], z: pose.t[2] });
              currentR = pose.R;
              currentT = pose.t;
              initialized = true;
              console.log(`[SLAM] initialized! ${points3D.length} 3D points`);

              // 平面検出（初期化時に1回）
              planeResult = detectPlane(points3D);
            }
          }
        }
      } else {
        // --- 初期化済み: PnP 追跡 ---
        const { points3D: matched3D, points2D: matched2D } = slamMap.get3D2DPairs(result.ids, result.points);

        if (matched3D.length >= 6) {
          const pnpResult = estimatePosePnP(matched3D, matched2D, K);
          if (pnpResult) {
            currentR = pnpResult.R;
            currentT = pnpResult.t;
            trajectory.push({ x: pnpResult.t[0], z: pnpResult.t[2] });
            if (frameCount % 30 === 0) {
              console.log(`[SLAM] PnP: ${pnpResult.inlierCount}/${matched3D.length} inliers`);
            }
          }
        } else if (frameCount % 60 === 0) {
          console.log(`[SLAM] PnP: not enough matches (${matched3D.length})`);
        }
      }

      // 平面オーバーレイ描画
      if (planeResult && currentR && currentT) {
        planeOverlay.draw(planeResult.inliers, currentR, currentT, Karray);
      }

      // 点群 + 軌跡の可視化
      if (initialized) {
        pointCloudView.draw(points3D, trajectory);
      }

      frameCount++;
      if (frameCount === 1) console.log(`[SLAM] first frame: ${result.count} points`);
    } catch (e) {
      console.error('[SLAM] processFrame error:', e);
    }
    requestAnimationFrame(processFrame);
  }

  requestAnimationFrame(processFrame);
}

main().catch((err) => {
  console.error('初期化エラー:', err);
  const loading = document.getElementById('loading');
  if (loading) loading.textContent = `エラー: ${err.message}`;
});
