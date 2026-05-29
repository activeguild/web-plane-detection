import * as THREE from 'three';

export class ArScene {
  private renderer: THREE.WebGLRenderer;
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private cube: THREE.Mesh | null = null;
  private _isModelPlaced = false;

  // 姿勢平滑化
  private smoothQuat: THREE.Quaternion | null = null;
  private smoothPos: THREE.Vector3 | null = null;
  private readonly smoothAlpha = 0.1;
  private logCount = 0;

  private K: number[][];
  private Kinv: number[][];

  constructor(glCanvas: HTMLCanvasElement, width: number, height: number, K: number[][]) {
    this.renderer = new THREE.WebGLRenderer({
      canvas: glCanvas,
      alpha: true,
      antialias: true,
      preserveDrawingBuffer: true,
    });
    this.renderer.setSize(width, height);
    this.renderer.setClearColor(0x000000, 0);

    this.scene = new THREE.Scene();

    const ambient = new THREE.AmbientLight(0xffffff, 0.6);
    this.scene.add(ambient);
    const directional = new THREE.DirectionalLight(0xffffff, 0.8);
    directional.position.set(1, 2, 3);
    this.scene.add(directional);

    const fy = K[1][1];
    const fov = 2 * Math.atan(height / (2 * fy)) * (180 / Math.PI);
    this.camera = new THREE.PerspectiveCamera(fov, width / height, 0.01, 1000);

    this.K = K;
    this.Kinv = [
      [1/K[0][0], 0, -K[0][2]/K[0][0]],
      [0, 1/K[1][1], -K[1][2]/K[1][1]],
      [0, 0, 1],
    ];
  }

  placeModel(): void {
    if (this.cube) return;

    // キューブをワールド原点の Z=1 平面上に配置
    // (参照フレームのカメラ座標系で z=1 の平面)
    const cubeSize = 0.1;
    const geometry = new THREE.BoxGeometry(cubeSize, cubeSize, cubeSize);
    const material = new THREE.MeshStandardMaterial({
      color: 0x00aaff,
      metalness: 0.3,
      roughness: 0.7,
    });
    this.cube = new THREE.Mesh(geometry, material);

    // Three.js 座標: OpenCV の (0, 0, 1) → Three.js の (0, 0, -1)
    this.cube.position.set(0, 0, -1);

    this.scene.add(this.cube);
    this._isModelPlaced = true;
    console.log('[AR] cube placed at origin (0, 0, -1) in Three.js coords');
  }

  // ホモグラフィ H (参照フレーム→現フレーム) からカメラ姿勢を計算して描画
  renderFromHomography(H: number[][]): void {
    if (!this._isModelPlaced) return;

    // H_norm = K^-1 × H × K (正規化座標系のホモグラフィ)
    // H_norm ≈ R + t × n^T (平面が z=1 の場合、n=[0,0,1], d=1)
    // → H_norm の列を取り出して R, t を近似
    const Hn = this.mul3x3(this.mul3x3(this.Kinv, H), this.K);

    // H_norm の列ベクトル
    const h1 = [Hn[0][0], Hn[1][0], Hn[2][0]];
    const h2 = [Hn[0][1], Hn[1][1], Hn[2][1]];
    const h3 = [Hn[0][2], Hn[1][2], Hn[2][2]];

    // lambda = 1 / |h1| でスケール正規化
    const lambda = 1 / Math.sqrt(h1[0]*h1[0] + h1[1]*h1[1] + h1[2]*h1[2]);

    // R の列ベクトル
    const r1 = h1.map(v => v * lambda);
    const r2 = h2.map(v => v * lambda);
    // r3 = r1 × r2 (外積で直交性を保証)
    const r3 = [
      r1[1]*r2[2] - r1[2]*r2[1],
      r1[2]*r2[0] - r1[0]*r2[2],
      r1[0]*r2[1] - r1[1]*r2[0],
    ];
    // h3 = r3 + t/d なので、t/d = h3*lambda - r3
    const h3scaled = h3.map(v => v * lambda);
    const t = [
      h3scaled[0] - r3[0],
      h3scaled[1] - r3[1],
      h3scaled[2] - r3[2],
    ];

    if (this.logCount < 10) {
      console.log(`[AR] t=(${t[0].toFixed(4)}, ${t[1].toFixed(4)}, ${t[2].toFixed(4)})`);
      this.logCount++;
    }

    // R (ワールド→カメラ, OpenCV座標系)
    const R = [
      [r1[0], r2[0], r3[0]],
      [r1[1], r2[1], r3[1]],
      [r1[2], r2[2], r3[2]],
    ];

    // R^T (カメラ→ワールド回転)
    const Rt = [
      [R[0][0], R[1][0], R[2][0]],
      [R[0][1], R[1][1], R[2][1]],
      [R[0][2], R[1][2], R[2][2]],
    ];

    // カメラ位置 = -R^T × t
    const camPos = [
      -(Rt[0][0]*t[0] + Rt[0][1]*t[1] + Rt[0][2]*t[2]),
      -(Rt[1][0]*t[0] + Rt[1][1]*t[1] + Rt[1][2]*t[2]),
      -(Rt[2][0]*t[0] + Rt[2][1]*t[1] + Rt[2][2]*t[2]),
    ];

    // OpenCV → Three.js: F = diag(1, -1, -1)
    // F × Rt × F
    const camToWorldMat = new THREE.Matrix4();
    camToWorldMat.set(
       Rt[0][0], -Rt[0][1], -Rt[0][2],  camPos[0],
      -Rt[1][0],  Rt[1][1],  Rt[1][2], -camPos[1],
      -Rt[2][0],  Rt[2][1],  Rt[2][2], -camPos[2],
       0,         0,          0,         1,
    );

    const newPos = new THREE.Vector3();
    const newQuat = new THREE.Quaternion();
    const newScale = new THREE.Vector3();
    camToWorldMat.decompose(newPos, newQuat, newScale);

    // 平滑化
    if (this.smoothQuat === null || this.smoothPos === null) {
      this.smoothQuat = newQuat.clone();
      this.smoothPos = newPos.clone();
    } else {
      const jumpDist = this.smoothPos.distanceTo(newPos);
      if (jumpDist < 3.0) {
        this.smoothQuat.slerp(newQuat, this.smoothAlpha);
        this.smoothPos.lerp(newPos, this.smoothAlpha);
      }
    }

    const smoothMat = new THREE.Matrix4();
    smoothMat.compose(this.smoothPos, this.smoothQuat, new THREE.Vector3(1, 1, 1));

    this.camera.matrixAutoUpdate = false;
    this.camera.matrix.copy(smoothMat);
    this.camera.matrixWorldNeedsUpdate = true;

    this.renderer.render(this.scene, this.camera);
  }

  private mul3x3(A: number[][], B: number[][]): number[][] {
    const C: number[][] = [[0,0,0],[0,0,0],[0,0,0]];
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        for (let k = 0; k < 3; k++) {
          C[i][j] += A[i][k] * B[k][j];
        }
      }
    }
    return C;
  }

  get isModelPlaced(): boolean {
    return this._isModelPlaced;
  }
}
