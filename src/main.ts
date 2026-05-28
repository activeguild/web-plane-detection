import cv from '@techstark/opencv-js';
import { initCamera } from './camera/capture';
import { CameraCalibration } from './camera/calibration';
import { OrbDetector } from './features/orb';
import { FeatureTracker } from './features/tracker';
import { estimatePose } from './geometry/essential';
import { estimatePosePnP } from './geometry/pnp';
import { triangulatePoints, Point3D } from './geometry/triangulation';
import { ImuSensor } from './imu/sensor';
import { ImuData } from './imu/normalize';
import { detectPlane, PlaneResult } from './plane/ransac';
import { SlamMap } from './slam/map';
import { Mapper } from './slam/mapper';
import { GravityIndicator } from './visualization/gravity-indicator';
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
  const loadingText = document.getElementById('loading-text') as HTMLSpanElement;
  const imuBtn = document.getElementById('imu-btn') as HTMLButtonElement;
  const ctx = canvas.getContext('2d')!;

  console.log('[SLAM] main() started');

  loadingText.textContent = 'OpenCV.js を読み込み中...';
  await waitForOpenCv();
  console.log('[SLAM] OpenCV.js ready');

  loadingText.textContent = 'カメラを起動中...';
  await initCamera(video);
  const w = video.videoWidth;
  const h = video.videoHeight;
  console.log(`[SLAM] camera ready: ${w}x${h}`);
  canvas.width = w;
  canvas.height = h;

  // IMU 初期化
  const imuSensor = new ImuSensor();
  let latestImu: ImuData | null = null;
  let gravity: { x: number; y: number; z: number } | null = null;

  const startImu = async () => {
    try {
      let imuLogCount = 0;
      await imuSensor.start((data) => {
        latestImu = data;
        if (imuLogCount < 3) {
          console.log(`[IMU] data: accel=(${data.acceleration.x.toFixed(2)}, ${data.acceleration.y.toFixed(2)}, ${data.acceleration.z.toFixed(2)})`);
          imuLogCount++;
        }
        const a = 0.8;
        if (gravity === null) {
          gravity = { ...data.acceleration };
        } else {
          gravity.x = a * gravity.x + (1 - a) * data.acceleration.x;
          gravity.y = a * gravity.y + (1 - a) * data.acceleration.y;
          gravity.z = a * gravity.z + (1 - a) * data.acceleration.z;
        }
      });
      console.log('[SLAM] IMU started');
    } catch (e) {
      console.warn('[SLAM] IMU not available:', e);
    }
  };

  const DME = DeviceMotionEvent as any;
  if (typeof DME.requestPermission === 'function') {
    loadingText.textContent = 'モーションセンサーの許可が必要です';
    imuBtn.style.display = 'block';
    await new Promise<void>((resolve) => {
      imuBtn.addEventListener('click', async () => {
        imuBtn.style.display = 'none';
        loadingText.textContent = '許可を取得中...';
        await startImu();
        resolve();
      }, { once: true });
    });
  } else {
    await startImu();
  }

  loading.style.display = 'none';

  // モジュール初期化
  const calibration = new CameraCalibration(w, h);
  const orb = new OrbDetector(500);
  const tracker = new FeatureTracker(orb, 200);
  const slamMap = new SlamMap();
  const pointCloudView = new PointCloudView(ctx, w, h);
  const planeOverlay = new PlaneOverlay(ctx);
  const gravityIndicator = new GravityIndicator(ctx, h);
  const K = calibration.getCameraMatrixAsMat();
  const Karray = calibration.getCameraMatrix();
  const mapper = new Mapper(slamMap, Karray);

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
      offCtx.drawImage(video, 0, 0, w, h);
      const imageData = offCtx.getImageData(0, 0, w, h);
      const frame = cv.matFromImageData(imageData);

      const result = tracker.process(frame);
      frame.delete();

      ctx.drawImage(video, 0, 0, w, h);

      // 描画: 移動ベクトル
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

      // 描画: 特徴点マーカー
      ctx.fillStyle = '#00ff00';
      for (let i = 0; i < result.count; i++) {
        const p = result.points[i];
        ctx.beginPath();
        ctx.arc(p.x, p.y, 3, 0, Math.PI * 2);
        ctx.fill();
      }

      if (!initialized) {
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

              // 平面検出（重力フィルタ付き）
              planeResult = detectPlane(points3D, undefined, 200, gravity ?? undefined);
            }
          }
        }
      } else {
        // 毎フレーム再ローカライゼーション（ID → 3D 対応を最新に保つ）
        if (currentR && currentT) {
          slamMap.relocalize(result.ids, result.points, currentR, currentT, Karray);
        }

        const { points3D: matched3D, points2D: matched2D } = slamMap.get3D2DPairs(result.ids, result.points);

        if (matched3D.length >= 4) {
          const pnpResult = estimatePosePnP(matched3D, matched2D, K);
          if (pnpResult) {
            currentR = pnpResult.R;
            currentT = pnpResult.t;
            trajectory.push({ x: pnpResult.t[0], z: pnpResult.t[2] });
            if (frameCount % 30 === 0) {
              console.log(`[SLAM] PnP: ${pnpResult.inlierCount}/${matched3D.length} inliers, map=${slamMap.size}`);
            }

            // 新規点を三角測量してマップを拡張
            const newPts = mapper.expandMap(result.ids, result.points, currentR, currentT);
            for (const p of newPts) {
              points3D.push(p);
            }
          }
        } else if (frameCount % 30 === 0) {
          console.log(`[SLAM] PnP: matches=${matched3D.length}, map=${slamMap.size}`);
        }
      }

      // 平面オーバーレイ
      if (planeResult && currentR && currentT) {
        planeOverlay.draw(planeResult.inliers, currentR, currentT, Karray);
      }

      // 点群 + 軌跡
      if (initialized) {
        pointCloudView.draw(points3D, trajectory);
      }

      // 重力インジケータ
      if (gravity) {
        gravityIndicator.draw(gravity);
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
