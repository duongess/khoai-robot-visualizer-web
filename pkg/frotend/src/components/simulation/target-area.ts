import { TerrainPoint } from '../../types/simulation';
import { CanvasTransform, worldToCanvas, getTerrainHeightAt } from './coordinate-system';

export interface DrawTargetAreaOptions {
  targetX: number;
  widthM: number;
  terrainPoints: TerrainPoint[];
  transform: CanvasTransform;
  ctx: CanvasRenderingContext2D;
  isPaused: boolean;
  isHovered?: boolean;
  isSelected?: boolean;
  isSuccess?: boolean;
  isPartial?: boolean;
}

export function drawTargetArea({
  targetX,
  widthM,
  terrainPoints,
  transform,
  ctx,
  isPaused,
  isHovered = false,
  isSelected = false,
  isSuccess = false,
  isPartial = false,
}: DrawTargetAreaOptions): void {
  const leftX = targetX - widthM / 2;
  const rightX = targetX + widthM / 2;

  // Evaluate terrain contour across target area width
  const samples = 8;
  const step = (rightX - leftX) / samples;
  const polygonPoints: { x: number; y: number }[] = [];

  for (let i = 0; i <= samples; i++) {
    const wx = leftX + i * step;
    const wy = getTerrainHeightAt(wx, terrainPoints);
    polygonPoints.push({ x: wx, y: wy });
  }

  const { scale } = transform;
  const padHeightM = 0.08; // target pad thickness in meters

  ctx.save();

  // Color scheme: subtle green outline / fill
  const strokeColor = isSuccess
    ? '#22c55e'
    : isPartial
    ? '#eab308'
    : isSelected || isHovered
    ? '#4ade80'
    : 'rgba(34, 197, 94, 0.7)';

  const fillColor = isSuccess
    ? 'rgba(34, 197, 94, 0.35)'
    : isPartial
    ? 'rgba(234, 179, 8, 0.25)'
    : isSelected || isHovered
    ? 'rgba(34, 197, 94, 0.25)'
    : 'rgba(34, 197, 94, 0.15)';

  // 1. Draw Target Surface Pad
  ctx.beginPath();
  const firstTop = worldToCanvas({ x: polygonPoints[0].x, y: polygonPoints[0].y + padHeightM }, transform);
  ctx.moveTo(firstTop.x, firstTop.y);

  for (let i = 1; i < polygonPoints.length; i++) {
    const p = worldToCanvas({ x: polygonPoints[i].x, y: polygonPoints[i].y + padHeightM }, transform);
    ctx.lineTo(p.x, p.y);
  }

  for (let i = polygonPoints.length - 1; i >= 0; i--) {
    const p = worldToCanvas({ x: polygonPoints[i].x, y: polygonPoints[i].y }, transform);
    ctx.lineTo(p.x, p.y);
  }
  ctx.closePath();

  ctx.fillStyle = fillColor;
  ctx.fill();
  ctx.strokeStyle = strokeColor;
  ctx.lineWidth = isSelected ? 2.5 : 1.5;
  ctx.stroke();

  // 2. Target Boundary Markers (Left & Right vertical flags)
  const leftBottom = worldToCanvas({ x: leftX, y: getTerrainHeightAt(leftX, terrainPoints) }, transform);
  const leftFlagTop = worldToCanvas({ x: leftX, y: getTerrainHeightAt(leftX, terrainPoints) + 0.35 }, transform);
  const rightBottom = worldToCanvas({ x: rightX, y: getTerrainHeightAt(rightX, terrainPoints) }, transform);
  const rightFlagTop = worldToCanvas({ x: rightX, y: getTerrainHeightAt(rightX, terrainPoints) + 0.35 }, transform);

  // Dashed boundary lines
  ctx.setLineDash([4, 3]);
  ctx.strokeStyle = strokeColor;
  ctx.lineWidth = 1.5;

  ctx.beginPath();
  ctx.moveTo(leftBottom.x, leftBottom.y);
  ctx.lineTo(leftFlagTop.x, leftFlagTop.y);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(rightBottom.x, rightBottom.y);
  ctx.lineTo(rightFlagTop.x, rightFlagTop.y);
  ctx.stroke();

  ctx.setLineDash([]); // reset dash

  // 3. Target Bullseye / Center Mark
  const centerTerrainY = getTerrainHeightAt(targetX, terrainPoints);
  const centerCanvas = worldToCanvas({ x: targetX, y: centerTerrainY + padHeightM / 2 }, transform);

  ctx.beginPath();
  ctx.arc(centerCanvas.x, centerCanvas.y, 4, 0, Math.PI * 2);
  ctx.fillStyle = strokeColor;
  ctx.fill();

  // Target Label
  ctx.fillStyle = isSuccess ? '#86efac' : '#4ade80';
  ctx.font = 'bold 9px ui-monospace, monospace';
  ctx.textAlign = 'center';
  ctx.fillText(
    isSuccess ? 'TARGET: SUCCESS' : isPartial ? 'TARGET: PARTIAL' : 'TARGET DROP ZONE',
    centerCanvas.x,
    centerCanvas.y - 12
  );

  // Paused Drag Cue
  if (isPaused && isHovered) {
    ctx.fillStyle = '#fef08a';
    ctx.font = '9px ui-sans-serif, sans-serif';
    ctx.fillText('↔ Drag Target Area', centerCanvas.x, centerCanvas.y - 24);
  }

  ctx.restore();
}

/**
 * Hit test for target area along the terrain
 */
export function hitTestTargetArea(
  canvasPos: { x: number; y: number },
  targetX: number,
  widthM: number,
  terrainPoints: TerrainPoint[],
  transform: CanvasTransform
): boolean {
  const leftX = targetX - widthM / 2;
  const rightX = targetX + widthM / 2;

  const leftCanvas = worldToCanvas({ x: leftX, y: 0 }, transform);
  const rightCanvas = worldToCanvas({ x: rightX, y: 0 }, transform);

  if (canvasPos.x < Math.min(leftCanvas.x, rightCanvas.x) - 10 || canvasPos.x > Math.max(leftCanvas.x, rightCanvas.x) + 10) {
    return false;
  }

  const worldX = targetX;
  const terrainY = getTerrainHeightAt(worldX, terrainPoints);
  const targetCanvas = worldToCanvas({ x: worldX, y: terrainY }, transform);

  // Check vertical distance near the terrain surface
  return Math.abs(canvasPos.y - targetCanvas.y) <= 30;
}
