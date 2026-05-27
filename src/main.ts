import cv from '@techstark/opencv-js';
import { initCamera } from './camera/capture';
import { CameraCalibration } from './camera/calibration';
import { OrbDetector } from './features/orb';
import { FeatureTracker } from './features/tracker';
import { estimatePose } from './geometry/essential';
import { triangulatePoints, Point3D } from './geometry/triangulation';
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
  const pointCloudView = new PointCloudView(ctx, w, h);

  const offscreen = document.createElement('canvas');
  offscreen.width = w;
  offscreen.height = h;
  const offCtx = offscreen.getContext('2d')!;

  // 初期化状態
  let initialized = false;
  let points3D: Point3D[] = [];
  let cameraT: number[] = [];
  const MOTION_THRESHOLD = 50;

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

      // 初期化判定
      if (!initialized && result.avgMotion > MOTION_THRESHOLD && result.count >= 30) {
        console.log(`[SLAM] initialization trigger: avgMotion=${result.avgMotion.toFixed(1)}, points=${result.count}`);

        const pose = estimatePose(
          result.prevPoints,
          result.points,
          calibration.getFocalLength(),
          calibration.getPrincipalPoint(),
        );

        if (pose) {
          points3D = triangulatePoints(
            result.prevPoints,
            result.points,
            pose.R,
            pose.t,
            calibration.getCameraMatrix(),
            pose.inlierMask,
          );

          if (points3D.length > 10) {
            cameraT = pose.t;
            initialized = true;
            console.log(`[SLAM] initialized! ${points3D.length} 3D points`);
          } else {
            console.log(`[SLAM] not enough 3D points: ${points3D.length}`);
          }
        }
      }

      // 点群可視化
      if (initialized) {
        pointCloudView.draw(points3D, cameraT);
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
