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

  // OpenCV.js WASM ロード待ち
  loading.textContent = 'OpenCV.js を読み込み中...';
  await waitForOpenCv();

  // カメラ初期化
  loading.textContent = 'カメラを起動中...';
  await initCamera(video);
  canvas.width = video.videoWidth;
  canvas.height = video.videoHeight;

  loading.style.display = 'none';

  // ORB 検出器
  const detector = new OrbDetector(500);

  // VideoCapture
  const cap = new cv.VideoCapture(video);
  const frame = new cv.Mat(video.videoHeight, video.videoWidth, cv.CV_8UC4);

  function processFrame() {
    cap.read(frame);
    detector.detectAndDraw(frame);
    cv.imshow(canvas, frame);
    requestAnimationFrame(processFrame);
  }

  requestAnimationFrame(processFrame);
}

main().catch((err) => {
  console.error('初期化エラー:', err);
  const loading = document.getElementById('loading');
  if (loading) loading.textContent = `エラー: ${err.message}`;
});
