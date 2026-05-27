export async function initCamera(video: HTMLVideoElement): Promise<void> {
  const stream = await navigator.mediaDevices.getUserMedia({
    video: {
      facingMode: 'environment',
      width: { ideal: 640 },
      height: { ideal: 480 },
    },
    audio: false,
  });

  video.srcObject = stream;

  return new Promise((resolve) => {
    video.onloadedmetadata = async () => {
      await video.play();
      // videoWidth/Height が確定するまで待つ
      const waitForSize = () => {
        if (video.videoWidth > 0 && video.videoHeight > 0) {
          resolve();
        } else {
          requestAnimationFrame(waitForSize);
        }
      };
      waitForSize();
    };
  });
}
