import { TerrainPoint } from '../../types/simulation';
import { CanvasTransform, worldToCanvas, canvasToWorld, getTerrainHeightAt } from './coordinate-system';

export interface DrawTerrainOptions {
  points: TerrainPoint[];
  transform: CanvasTransform;
  ctx: CanvasRenderingContext2D;
  isPaused: boolean;
  selectedPointId?: string | null;
  hoveredPointId?: string | null;
}

export const DEFAULT_TERRAIN_POINTS: TerrainPoint[] = [
  { id: 't-1', x: 0.0, y: 0.35 },
  { id: 't-2', x: 1.5, y: 0.35 },
  { id: 't-3', x: 3.0, y: 0.55 },
  { id: 't-4', x: 4.5, y: 0.25 },
  { id: 't-5', x: 6.0, y: 0.25 },
];

export const DEFAULT_FLAT_TERRAIN_POINTS: TerrainPoint[] = [
  { id: 't-1', x: 0.0, y: 0.3 },
  { id: 't-2', x: 2.0, y: 0.3 },
  { id: 't-3', x: 4.0, y: 0.3 },
  { id: 't-4', x: 6.0, y: 0.3 },
];

export function drawEditableTerrain({
  points,
  transform,
  ctx,
  isPaused,
  selectedPointId = null,
  hoveredPointId = null,
}: DrawTerrainOptions): void {
  if (!points || points.length < 2) return;

  const sortedPoints = [...points].sort((a, b) => a.x - b.x);
  const { width, height } = transform;

  ctx.save();

  // 1. Fill polygon below the terrain line down to the bottom of the canvas
  ctx.beginPath();
  const firstCanvas = worldToCanvas(sortedPoints[0], transform);
  ctx.moveTo(firstCanvas.x, firstCanvas.y);

  for (let i = 1; i < sortedPoints.length; i++) {
    const cp = worldToCanvas(sortedPoints[i], transform);
    ctx.lineTo(cp.x, cp.y);
  }

  const lastCanvas = worldToCanvas(sortedPoints[sortedPoints.length - 1], transform);
  ctx.lineTo(lastCanvas.x, height);
  ctx.lineTo(firstCanvas.x, height);
  ctx.closePath();

  // Gradient bedrock fill
  const terrainGrad = ctx.createLinearGradient(0, Math.min(firstCanvas.y, lastCanvas.y), 0, height);
  terrainGrad.addColorStop(0, '#132038');
  terrainGrad.addColorStop(0.3, '#0e172a');
  terrainGrad.addColorStop(1, '#060a14');
  ctx.fillStyle = terrainGrad;
  ctx.fill();

  // Subtle soil texture / diagonal hatching
  ctx.save();
  ctx.clip(); // clip to terrain polygon
  ctx.strokeStyle = 'rgba(2, 132, 199, 0.08)';
  ctx.lineWidth = 1;
  const hatchSpacing = 18;
  for (let x = -height; x < width + height; x += hatchSpacing) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x + height, height);
    ctx.stroke();
  }
  ctx.restore();

  // 2. Continuous Surface Line
  ctx.strokeStyle = '#0284c7';
  ctx.lineWidth = 2.5;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  ctx.beginPath();
  ctx.moveTo(firstCanvas.x, firstCanvas.y);
  for (let i = 1; i < sortedPoints.length; i++) {
    const cp = worldToCanvas(sortedPoints[i], transform);
    ctx.lineTo(cp.x, cp.y);
  }
  ctx.stroke();

  // Surface top glow line
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(firstCanvas.x, firstCanvas.y - 1);
  for (let i = 1; i < sortedPoints.length; i++) {
    const cp = worldToCanvas(sortedPoints[i], transform);
    ctx.lineTo(cp.x, cp.y - 1);
  }
  ctx.stroke();

  // 3. Control Points (rendered only when paused or for selected/hovered cues)
  if (isPaused) {
    sortedPoints.forEach((pt, index) => {
      const cp = worldToCanvas(pt, transform);
      const isSelected = pt.id === selectedPointId;
      const isHovered = pt.id === hoveredPointId;
      const isBoundary = index === 0 || index === sortedPoints.length - 1;

      // Outer ring for selected or hovered
      if (isSelected || isHovered) {
        ctx.beginPath();
        ctx.arc(cp.x, cp.y, isSelected ? 11 : 9, 0, Math.PI * 2);
        ctx.fillStyle = isSelected ? 'rgba(56, 189, 248, 0.25)' : 'rgba(56, 189, 248, 0.15)';
        ctx.fill();
        ctx.strokeStyle = isSelected ? '#38bdf8' : '#7dd3fc';
        ctx.lineWidth = 2;
        ctx.stroke();
      }

      // Point node
      ctx.beginPath();
      ctx.arc(cp.x, cp.y, isSelected ? 6 : 5, 0, Math.PI * 2);
      ctx.fillStyle = isSelected ? '#38bdf8' : isBoundary ? '#94a3b8' : '#0ea5e9';
      ctx.fill();
      ctx.strokeStyle = '#0f172a';
      ctx.lineWidth = 1.5;
      ctx.stroke();

      // Coordinate tooltip badge on hovered or selected
      if (isHovered || isSelected) {
        ctx.fillStyle = '#f8fafc';
        ctx.font = '9px ui-monospace, monospace';
        ctx.textAlign = 'center';
        ctx.fillText(`(${pt.x.toFixed(2)}m, ${pt.y.toFixed(2)}m)`, cp.x, cp.y - 14);
      }
    });
  }

  ctx.restore();
}

/**
 * Hit-test a terrain control point within pixel threshold (e.g. 14px)
 */
export function hitTestTerrainPoint(
  canvasPos: { x: number; y: number },
  points: TerrainPoint[],
  transform: CanvasTransform,
  thresholdPx: number = 14
): TerrainPoint | null {
  for (const pt of points) {
    const cp = worldToCanvas(pt, transform);
    const dist = Math.hypot(canvasPos.x - cp.x, canvasPos.y - cp.y);
    if (dist <= thresholdPx) {
      return pt;
    }
  }
  return null;
}

/**
 * Hit-test proximity to the terrain line itself (for double-click point insertion)
 */
export function hitTestTerrainSurface(
  canvasPos: { x: number; y: number },
  points: TerrainPoint[],
  transform: CanvasTransform,
  thresholdPx: number = 16
): { worldX: number; worldY: number; insertIndex: number } | null {
  const worldPos = canvasToWorld(canvasPos, transform);
  const sorted = [...points].sort((a, b) => a.x - b.x);

  if (worldPos.x < sorted[0].x || worldPos.x > sorted[sorted.length - 1].x) {
    return null;
  }

  const terrainY = getTerrainHeightAt(worldPos.x, sorted);
  const terrainCanvas = worldToCanvas({ x: worldPos.x, y: terrainY }, transform);

  const distY = Math.abs(canvasPos.y - terrainCanvas.y);
  if (distY <= thresholdPx) {
    // Find segment to insert into
    let insertIndex = 1;
    for (let i = 0; i < sorted.length - 1; i++) {
      if (worldPos.x >= sorted[i].x && worldPos.x <= sorted[i + 1].x) {
        insertIndex = i + 1;
        break;
      }
    }
    return {
      worldX: Number(worldPos.x.toFixed(2)),
      worldY: Number(terrainY.toFixed(2)),
      insertIndex,
    };
  }

  return null;
}
