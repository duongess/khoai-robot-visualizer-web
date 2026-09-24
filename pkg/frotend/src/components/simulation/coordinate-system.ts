import { WorldBounds, TerrainPoint } from '../../types/simulation';
import { DEFAULT_SCENE_CONFIG } from '../../lib/default-scene-config';

// Bootstrap fallback only; a running canvas replaces this with backend
// workspace telemetry. Keeping it derived avoids a second hard-coded world.
export const DEFAULT_WORLD_BOUNDS: WorldBounds = { ...DEFAULT_SCENE_CONFIG.workspace! };

export interface CanvasTransform {
  scale: number;
  offsetX: number;
  offsetY: number;
  width: number;
  height: number;
  bounds: WorldBounds;
}

/**
 * Calculates a fixed-aspect-ratio canvas transform fitting the complete workspace.
 * Uses a uniform scale factor so objects and mechanisms are never distorted,
 * and leaves padding around the simulation perimeter.
 */
export function getCanvasTransform(
  canvasWidth: number,
  canvasHeight: number,
  bounds: WorldBounds = DEFAULT_WORLD_BOUNDS,
  padding: number = 24
): CanvasTransform {
  const worldW = bounds.maxX - bounds.minX;
  const worldH = bounds.maxY - bounds.minY;

  const availW = Math.max(10, canvasWidth - padding * 2);
  const availH = Math.max(10, canvasHeight - padding * 2);

  const scale = Math.min(availW / worldW, availH / worldH);

  // Center the world inside the canvas
  const renderedW = worldW * scale;
  const renderedH = worldH * scale;

  const offsetX = (canvasWidth - renderedW) / 2;
  const offsetY = (canvasHeight - renderedH) / 2;

  return {
    scale,
    offsetX,
    offsetY,
    width: canvasWidth,
    height: canvasHeight,
    bounds,
  };
}

/**
 * Explicit conversion from World Coordinates (meters, +X right, +Y up)
 * to Canvas Coordinates (pixels, +X right, +Y down).
 */
export function worldToCanvas(
  pos: { x: number; y: number },
  transform: CanvasTransform
): { x: number; y: number } {
  const { scale, offsetX, offsetY, height, bounds } = transform;
  const cx = offsetX + (pos.x - bounds.minX) * scale;
  const cy = height - offsetY - (pos.y - bounds.minY) * scale;
  return { x: cx, y: cy };
}

/**
 * Explicit conversion from Canvas Coordinates (pixels)
 * to World Coordinates (meters).
 */
export function canvasToWorld(
  pos: { x: number; y: number },
  transform: CanvasTransform
): { x: number; y: number } {
  const { scale, offsetX, offsetY, height, bounds } = transform;
  const wx = bounds.minX + (pos.x - offsetX) / scale;
  const wy = bounds.minY + (height - offsetY - pos.y) / scale;
  return { x: wx, y: wy };
}

/**
 * Linear interpolation of terrain surface height Y at any world X coordinate.
 */
export function getTerrainHeightAt(x: number, points: TerrainPoint[]): number {
  if (!points || points.length === 0) return 0.25;
  if (points.length === 1) return points[0].y;

  // Ensure points are sorted by x
  const sorted = [...points].sort((a, b) => a.x - b.x);

  if (x <= sorted[0].x) return sorted[0].y;
  if (x >= sorted[sorted.length - 1].x) return sorted[sorted.length - 1].y;

  for (let i = 0; i < sorted.length - 1; i++) {
    const p1 = sorted[i];
    const p2 = sorted[i + 1];
    if (x >= p1.x && x <= p2.x) {
      const dx = p2.x - p1.x;
      if (dx <= 0.0001) return p1.y;
      const t = (x - p1.x) / dx;
      return p1.y + t * (p2.y - p1.y);
    }
  }

  return sorted[sorted.length - 1].y;
}
