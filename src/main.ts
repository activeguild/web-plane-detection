import cv from '@techstark/opencv-js';
import { initCamera } from './camera/capture';
import { CameraCalibration } from './camera/calibration';
import { ImuSensor } from './imu/sensor';
import { ImuData } from './imu/normalize';
import { PlanarTracker } from './tracking/planar-tracker';
import { GravityIndicator } from './visualization/gravity-indicator';
import { ArScene } from './rendering/ar-scene';

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

  // IMU
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

  // モジュール初期化
  const calibration = new CameraCalibration(w, h);
  const Karray = calibration.getCameraMatrix();
  const tracker = new PlanarTracker(1000);
  const gravityIndicator = new GravityIndicator(ctx, h);

  const glCanvas = document.getElementById('gl-canvas') as HTMLCanvasElement;
  glCanvas.width = w;
  glCanvas.height = h;
  const arScene = new ArScene(glCanvas, w, h, Karray);

  const offscreen = document.createElement('canvas');
  offscreen.width = w;
  offscreen.height = h;
  const offCtx = offscreen.getContext('2d')!;

  // 状態
  let referenceSet = false;
  let frameCount = 0;
  const INIT_DELAY = 60; // 60フレーム（約2秒）待ってから参照フレームを取得

  loading.style.display = 'none';

  console.log('[SLAM] starting tracking loop');

  function processFrame() {
    try {
      offCtx.drawImage(video, 0, 0, w, h);
      const imageData = offCtx.getImageData(0, 0, w, h);
      const frame = cv.matFromImageData(imageData);
      const gray = new cv.Mat();
      cv.cvtColor(frame, gray, cv.COLOR_RGBA2GRAY);
      frame.delete();

      // 映像描画
      ctx.drawImage(video, 0, 0, w, h);

      if (!referenceSet) {
        // 参照フレーム取得待ち
        frameCount++;
        if (frameCount >= INIT_DELAY) {
          const kpCount = tracker.setReference(gray);
          if (kpCount >= 50) {
            referenceSet = true;
            arScene.placeModel();
            console.log(`[SLAM] reference frame set, tracking started`);
          } else {
            frameCount = 0;
            console.log(`[SLAM] not enough keypoints (${kpCount}), retrying...`);
          }
        }
        // カウントダウンをキャンバスに描画
        if (!referenceSet) {
          const remaining = Math.ceil((INIT_DELAY - frameCount) / 30);
          ctx.fillStyle = 'rgba(0, 0, 0, 0.5)';
          ctx.fillRect(0, h / 2 - 30, w, 60);
          ctx.fillStyle = '#fff';
          ctx.font = '20px sans-serif';
          ctx.textAlign = 'center';
          ctx.fillText(`平面に向けて静止してください... ${remaining}`, w / 2, h / 2 + 7);
        }
      } else {
        // 平面追跡
        const result = tracker.track(gray);

        if (result.success && result.H) {
          arScene.renderFromHomography(result.H);

          // H で参照フレームの中心点を現フレームに変換して 2D マーカーを描画
          const H = result.H;
          const refX = w / 2, refY = h / 2;
          const denom = H[2][0] * refX + H[2][1] * refY + H[2][2];
          if (Math.abs(denom) > 1e-6) {
            const curX = (H[0][0] * refX + H[0][1] * refY + H[0][2]) / denom;
            const curY = (H[1][0] * refX + H[1][1] * refY + H[1][2]) / denom;
            // 赤い十字マーカー
            ctx.strokeStyle = '#ff0000';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.moveTo(curX - 15, curY); ctx.lineTo(curX + 15, curY);
            ctx.moveTo(curX, curY - 15); ctx.lineTo(curX, curY + 15);
            ctx.stroke();
            // 四角の枠（参照フレームの中心付近の正方形を変換）
            const size = 40;
            const corners = [
              [refX - size, refY - size],
              [refX + size, refY - size],
              [refX + size, refY + size],
              [refX - size, refY + size],
            ];
            ctx.strokeStyle = '#00ff00';
            ctx.lineWidth = 2;
            ctx.beginPath();
            for (let i = 0; i < 4; i++) {
              const [sx, sy] = corners[i];
              const d = H[2][0] * sx + H[2][1] * sy + H[2][2];
              const cx2 = (H[0][0] * sx + H[0][1] * sy + H[0][2]) / d;
              const cy2 = (H[1][0] * sx + H[1][1] * sy + H[1][2]) / d;
              if (i === 0) ctx.moveTo(cx2, cy2); else ctx.lineTo(cx2, cy2);
            }
            ctx.closePath();
            ctx.stroke();
          }

          if (frameCount % 60 === 0) {
            console.log(`[Track] matches=${result.matchCount}, inliers=${result.inlierCount}`);
          }
        } else if (frameCount % 30 === 0) {
          console.log(`[Track] lost: matches=${result.matchCount}, inliers=${result.inlierCount}`);
        }
      }

      // 重力インジケータ
      if (gravity) {
        gravityIndicator.draw(gravity);
      }

      gray.delete();
      frameCount++;
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
