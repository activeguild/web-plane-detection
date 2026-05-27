import { ImuData, normalizeImuData, detectPlatform } from './normalize';

export class ImuSensor {
  private platform: 'ios' | 'android' | 'unknown';
  private callback: ((data: ImuData) => void) | null = null;
  private handler: ((event: DeviceMotionEvent) => void) | null = null;
  private _isAvailable = false;

  constructor() {
    this.platform = detectPlatform();
  }

  async start(callback: (data: ImuData) => void): Promise<void> {
    this.callback = callback;

    const DME = DeviceMotionEvent as any;
    if (typeof DME.requestPermission === 'function') {
      const permission = await DME.requestPermission();
      if (permission !== 'granted') {
        console.warn('[IMU] permission denied');
        return;
      }
    }

    this.handler = (event: DeviceMotionEvent) => {
      if (this.callback) {
        const data = normalizeImuData(event, this.platform);
        this.callback(data);
      }
    };

    window.addEventListener('devicemotion', this.handler);
    this._isAvailable = true;
    console.log(`[IMU] started (platform: ${this.platform})`);
  }

  stop(): void {
    if (this.handler) {
      window.removeEventListener('devicemotion', this.handler);
      this.handler = null;
    }
    this.callback = null;
    this._isAvailable = false;
  }

  get isAvailable(): boolean {
    return this._isAvailable;
  }
}
