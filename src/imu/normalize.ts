export type ImuData = {
  acceleration: { x: number; y: number; z: number };
  rotationRate: { alpha: number; beta: number; gamma: number };
  timestamp: number;
};

export function detectPlatform(): 'ios' | 'android' | 'unknown' {
  const ua = navigator.userAgent.toLowerCase();
  if (/iphone|ipad|ipod/.test(ua)) return 'ios';
  if (/android/.test(ua)) return 'android';
  return 'unknown';
}

export function normalizeImuData(
  event: DeviceMotionEvent,
  platform: 'ios' | 'android' | 'unknown',
): ImuData {
  const accel = event.accelerationIncludingGravity;
  let ax = accel?.x ?? 0;
  let ay = accel?.y ?? 0;
  let az = accel?.z ?? 0;

  if (platform === 'ios') {
    ax = -ax;
    ay = -ay;
    az = -az;
  }

  const rot = event.rotationRate;
  let alpha = rot?.alpha ?? 0;
  let beta = rot?.beta ?? 0;
  let gamma = rot?.gamma ?? 0;

  if (platform === 'ios') {
    const deg2rad = Math.PI / 180;
    alpha *= deg2rad;
    beta *= deg2rad;
    gamma *= deg2rad;
  }

  return {
    acceleration: { x: ax, y: ay, z: az },
    rotationRate: { alpha, beta, gamma },
    timestamp: performance.now(),
  };
}
