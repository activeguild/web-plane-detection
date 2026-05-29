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
          const H = result.H;

          // H で点を変換するヘルパー
          const transformPt = (px: number, py: number): [number, number] | null => {
            const d = H[2][0] * px + H[2][1] * py + H[2][2];
            if (Math.abs(d) < 1e-6) return null;
            return [
              (H[0][0] * px + H[0][1] * py + H[0][2]) / d,
              (H[1][0] * px + H[1][1] * py + H[1][2]) / d,
            ];
          };

          // キューブの底面（参照フレームの画面中央に正方形）
          const cx = w / 2, cy = h / 2;
          const size = 35;
          const bottom = [
            transformPt(cx - size, cy - size),
            transformPt(cx + size, cy - size),
            transformPt(cx + size, cy + size),
            transformPt(cx - size, cy + size),
          ];

          // 上面: 底面を上にオフセット（H のローカルスケールで高さを調整）
          const heightOffset = size * 1.5;
          const top = [
            transformPt(cx - size, cy - size - heightOffset),
            transformPt(cx + size, cy - size - heightOffset),
            transformPt(cx + size, cy + size - heightOffset),
            transformPt(cx - size, cy + size - heightOffset),
          ];

          // 全点が有効かチェック
          if (bottom.every(p => p !== null) && top.every(p => p !== null)) {
            const b = bottom as [number, number][];
            const t2 = top as [number, number][];

            // 底面（青い半透明）
            ctx.fillStyle = 'rgba(0, 120, 255, 0.4)';
            ctx.beginPath();
            ctx.moveTo(b[0][0], b[0][1]);
            for (let i = 1; i < 4; i++) ctx.lineTo(b[i][0], b[i][1]);
            ctx.closePath();
            ctx.fill();

            // 上面（明るい青の半透明）
            ctx.fillStyle = 'rgba(80, 180, 255, 0.5)';
            ctx.beginPath();
            ctx.moveTo(t2[0][0], t2[0][1]);
            for (let i = 1; i < 4; i++) ctx.lineTo(t2[i][0], t2[i][1]);
            ctx.closePath();
            ctx.fill();

            // 側面（4面）
            ctx.fillStyle = 'rgba(0, 100, 220, 0.3)';
            for (let i = 0; i < 4; i++) {
              const j = (i + 1) % 4;
              ctx.beginPath();
              ctx.moveTo(b[i][0], b[i][1]);
              ctx.lineTo(b[j][0], b[j][1]);
              ctx.lineTo(t2[j][0], t2[j][1]);
              ctx.lineTo(t2[i][0], t2[i][1]);
              ctx.closePath();
              ctx.fill();
            }

            // エッジ線
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
            ctx.lineWidth = 2;
            // 底面エッジ
            ctx.beginPath();
            ctx.moveTo(b[0][0], b[0][1]);
            for (let i = 1; i < 4; i++) ctx.lineTo(b[i][0], b[i][1]);
            ctx.closePath();
            ctx.stroke();
            // 上面エッジ
            ctx.beginPath();
            ctx.moveTo(t2[0][0], t2[0][1]);
            for (let i = 1; i < 4; i++) ctx.lineTo(t2[i][0], t2[i][1]);
            ctx.closePath();
            ctx.stroke();
            // 縦エッジ
            ctx.beginPath();
            for (let i = 0; i < 4; i++) {
              ctx.moveTo(b[i][0], b[i][1]);
              ctx.lineTo(t2[i][0], t2[i][1]);
            }
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
