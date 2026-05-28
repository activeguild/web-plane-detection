import * as THREE from 'three';
import { Point3D } from '../geometry/triangulation';

export class ArScene {
  private renderer: THREE.WebGLRenderer;
  private scene: THREE.Scene;
  private camera: THREE.PerspectiveCamera;
  private cube: THREE.Mesh | null = null;
  private _isModelPlaced = false;

  constructor(glCanvas: HTMLCanvasElement, width: number, height: number, K: number[][]) {
    this.renderer = new THREE.WebGLRenderer({
      canvas: glCanvas,
      alpha: true,
      antialias: true,
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
  }

  placeModel(planeInliers: Point3D[], planeNormal: number[]): void {
    if (this.cube) return;

    let cx = 0, cy = 0, cz = 0;
    for (const p of planeInliers) {
      cx += p.x; cy += p.y; cz += p.z;
    }
    cx /= planeInliers.length;
    cy /= planeInliers.length;
    cz /= planeInliers.length;

    let maxSpread = 0;
    for (const p of planeInliers) {
      const dx = p.x - cx;
      const dy = p.y - cy;
      const dz = p.z - cz;
      const dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
      if (dist > maxSpread) maxSpread = dist;
    }
    const cubeSize = Math.max(maxSpread * 0.1, 0.05);

    const geometry = new THREE.BoxGeometry(cubeSize, cubeSize, cubeSize);
    const material = new THREE.MeshStandardMaterial({
      color: 0x00aaff,
      metalness: 0.3,
      roughness: 0.7,
    });
    this.cube = new THREE.Mesh(geometry, material);
    this.cube.position.set(cx, cy, cz);

    const up = new THREE.Vector3(planeNormal[0], planeNormal[1], planeNormal[2]).normalize();
    const defaultUp = new THREE.Vector3(0, 1, 0);
    const quat = new THREE.Quaternion().setFromUnitVectors(defaultUp, up);
    this.cube.quaternion.copy(quat);

    this.cube.position.x += up.x * cubeSize / 2;
    this.cube.position.y += up.y * cubeSize / 2;
    this.cube.position.z += up.z * cubeSize / 2;

    this.scene.add(this.cube);
    this._isModelPlaced = true;

    console.log(`[AR] cube placed at (${cx.toFixed(2)}, ${cy.toFixed(2)}, ${cz.toFixed(2)}), size=${cubeSize.toFixed(3)}`);
  }

  render(R: number[][], t: number[]): void {
    if (!this._isModelPlaced) return;

    const mat = new THREE.Matrix4();
    mat.set(
      R[0][0],  -R[0][1],  -R[0][2],  t[0],
      -R[1][0],  R[1][1],   R[1][2],  -t[1],
      -R[2][0],  R[2][1],   R[2][2],  -t[2],
      0,         0,          0,         1,
    );

    const viewMatrixInverse = mat.clone().invert();
    this.camera.matrixAutoUpdate = false;
    this.camera.matrix.copy(viewMatrixInverse);
    this.camera.matrixWorldNeedsUpdate = true;

    this.renderer.render(this.scene, this.camera);
  }

  get isModelPlaced(): boolean {
    return this._isModelPlaced;
  }
}
