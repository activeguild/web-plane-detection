import cv from '@techstark/opencv-js';
import { Point2D } from '../features/orb';
import { Point3D } from '../geometry/triangulation';

type MapEntry = {
  point3D: Point3D;
  descriptor: Uint8Array | null; // 32バイト ORB ディスクリプタ
};

export class SlamMap {
  private map: Map<number, MapEntry> = new Map();
  private maxSize = 500;

  register(ids: number[], points3D: Point3D[], descriptors?: cv.Mat): void {
    for (let i = 0; i < ids.length; i++) {
      let desc: Uint8Array | null = null;
      if (descriptors && i < descriptors.rows) {
        desc = new Uint8Array(32);
        for (let j = 0; j < 32; j++) {
          desc[j] = descriptors.data[i * descriptors.cols + j];
        }
      }
      this.map.set(ids[i], { point3D: points3D[i], descriptor: desc });
    }

    // サイズ上限を超えたら古いエントリを削除
    if (this.map.size > this.maxSize) {
      const keys = Array.from(this.map.keys());
      const toDelete = keys.slice(0, keys.length - this.maxSize);
      for (const k of toDelete) {
        this.map.delete(k);
      }
    }

    console.log(`[SLAM] SlamMap: registered ${ids.length} points, total=${this.map.size}`);
  }

  get3D2DPairs(ids: number[], points2D: Point2D[]): { points3D: Point3D[]; points2D: Point2D[] } {
    const matched3D: Point3D[] = [];
    const matched2D: Point2D[] = [];
    for (let i = 0; i < ids.length; i++) {
      const entry = this.map.get(ids[i]);
      if (entry) {
        matched3D.push(entry.point3D);
        matched2D.push(points2D[i]);
      }
    }
    return { points3D: matched3D, points2D: matched2D };
  }

  // ディスクリプタベース再ローカライゼーション
  // 現フレームの ORB 特徴量と SlamMap のディスクリプタを BFMatcher で照合し、
  // マッチした 3D-2D ペアを返す
  descriptorMatch(
    keypoints: Point2D[],
    descriptors: cv.Mat,
  ): { points3D: Point3D[]; points2D: Point2D[] } {
    // SlamMap からディスクリプタ付きエントリを集約
    const entries: { point3D: Point3D; descriptor: Uint8Array }[] = [];
    for (const entry of this.map.values()) {
      if (entry.descriptor) {
        entries.push({ point3D: entry.point3D, descriptor: entry.descriptor });
      }
    }

    if (entries.length < 6 || descriptors.rows < 6) {
      return { points3D: [], points2D: [] };
    }

    // SlamMap ディスクリプタを cv.Mat に変換
    const trainDesc = new cv.Mat(entries.length, 32, cv.CV_8UC1);
    for (let i = 0; i < entries.length; i++) {
      for (let j = 0; j < 32; j++) {
        trainDesc.data[i * 32 + j] = entries[i].descriptor[j];
      }
    }

    // BFMatcher
    const bf = new cv.BFMatcher(cv.NORM_HAMMING, true);
    const matches = new cv.DMatchVector();

    try {
      bf.match(descriptors, trainDesc, matches);
    } catch (e) {
      console.error('[SLAM] BFMatcher failed:', e);
      bf.delete();
      trainDesc.delete();
      matches.delete();
      return { points3D: [], points2D: [] };
    }

    // Hamming 距離 < 60 のマッチのみ採用
    const matched3D: Point3D[] = [];
    const matched2D: Point2D[] = [];
    for (let i = 0; i < matches.size(); i++) {
      const m = matches.get(i);
      if (m.distance < 60) {
        matched3D.push(entries[m.trainIdx].point3D);
        matched2D.push(keypoints[m.queryIdx]);
      }
    }

    matches.delete();
    bf.delete();
    trainDesc.delete();

    return { points3D: matched3D, points2D: matched2D };
  }

  has(id: number): boolean {
    return this.map.has(id);
  }

  get size(): number {
    return this.map.size;
  }
}
