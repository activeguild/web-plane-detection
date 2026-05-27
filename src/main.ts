import { initCamera } from './camera/capture';

async function main() {
  const video = document.getElementById('video') as HTMLVideoElement;
  const canvas = document.getElementById('canvas') as HTMLCanvasElement;
  const loading = document.getElementById('loading') as HTMLDivElement;

  // カメラ初期化
  await initCamera(video);
  canvas.width = video.videoWidth;
  canvas.height = video.videoHeight;

  loading.style.display = 'none';

  // 仮: canvas に映像だけ描画して確認
  const ctx = canvas.getContext('2d')!;
  function draw() {
    ctx.drawImage(video, 0, 0, canvas.width, canvas.height);
    requestAnimationFrame(draw);
  }
  requestAnimationFrame(draw);
}

main().catch((err) => {
  console.error('初期化エラー:', err);
  const loading = document.getElementById('loading');
  if (loading) loading.textContent = `エラー: ${err.message}`;
});
