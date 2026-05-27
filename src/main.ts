import cv from '@techstark/opencv-js';
import { initCamera } from './camera/capture';
import { OrbDetector } from './features/orb';

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

  console.log('[SLAM] main() started');

  // OpenCV.js WASM ロード待ち
  loading.textContent = 'OpenCV.js を読み込み中...';
  console.log('[SLAM] waiting for OpenCV.js...');
  await waitForOpenCv();
  console.log('[SLAM] OpenCV.js ready');

  // カメラ初期化
  loading.textContent = 'カメラを起動中...';
  console.log('[SLAM] initializing camera...');
  await initCamera(video);
  const w = video.videoWidth;
  const h = video.videoHeight;
  console.log(`[SLAM] camera ready: ${w}x${h}`);
  canvas.width = w;
  canvas.height = h;

  loading.style.display = 'none';

  // ORB 検出器
  console.log('[SLAM] creating ORB detector...');
  const detector = new OrbDetector(500);
  console.log('[SLAM] ORB detector created');

  // Canvas 2D で映像フレームを取得する方式
  // cv.VideoCapture はモバイルでサイズ不一致エラーが出るため回避
  const offscreen = document.createElement('canvas');
  offscreen.width = w;
  offscreen.height = h;
  const offCtx = offscreen.getContext('2d')!;

  console.log('[SLAM] starting processing loop');

  let frameCount = 0;
  function processFrame() {
    try {
      // video → offscreen canvas → ImageData → cv.Mat
      offCtx.drawImage(video, 0, 0, w, h);
      const imageData = offCtx.getImageData(0, 0, w, h);
      const frame = cv.matFromImageData(imageData);

      detector.detectAndDraw(frame);
      cv.imshow(canvas, frame);
      frame.delete();

      frameCount++;
      if (frameCount === 1) console.log('[SLAM] first frame processed');
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
