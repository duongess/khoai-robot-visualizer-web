import { ObjectStatus } from '../../types/simulation';
import { CanvasTransform, worldToCanvas } from './coordinate-system';

export interface DrawObjectOptions {
  position: { x: number; y: number };
  widthM: number;
  heightM: number;
  mass: number;
  status: ObjectStatus;
  transform: CanvasTransform;
  ctx: CanvasRenderingContext2D;
  isPaused: boolean;
  isHovered?: boolean;
  isSelected?: boolean;
  isGrasped?: boolean;
}

const OBJECT_STATUS_STYLES: Record<
  ObjectStatus,
  { border: string; fill: string; text: string; glow?: string; badge: string }
> = {
  idle: {
    border: '#64748b',
    fill: 'rgba(100, 116, 139, 0.3)',
    text: '#cbd5e1',
    badge: 'IDLE',
  },
  targeted: {
    border: '#38bdf8',
    fill: 'rgba(56, 189, 248, 0.35)',
    text: '#7dd3fc',
    glow: '#0284c7',
    badge: 'TARGETED',
  },
  grasping: {
    border: '#60a5fa',
    fill: 'rgba(96, 165, 250, 0.35)',
    text: '#93c5fd',
    glow: '#2563eb',
    badge: 'GRASPING',
  },
	attached: {
		border: '#38bdf8',
		fill: 'rgba(56, 189, 248, 0.4)',
		text: '#bae6fd',
		glow: '#0284c7',
		badge: 'ATTACHED',
	},
	transported: {
		border: '#0284c7',
		fill: 'rgba(2, 132, 199, 0.45)',
		text: '#e0f2fe',
		glow: '#0284c7',
		badge: 'TRANSPORTED',
	},
	released: {
		border: '#fbbf24',
		fill: 'rgba(251, 191, 36, 0.35)',
		text: '#fef08a',
		glow: '#d97706',
		badge: 'RELEASED',
	},
  grasped: {
    border: '#38bdf8',
    fill: 'rgba(56, 189, 248, 0.4)',
    text: '#bae6fd',
    glow: '#0284c7',
    badge: 'GRASPED',
  },
  lifting: {
    border: '#38bdf8',
    fill: 'rgba(56, 189, 248, 0.45)',
    text: '#e0f2fe',
    glow: '#0369a1',
    badge: 'LIFTING',
  },
  carrying: {
    border: '#0284c7',
    fill: 'rgba(2, 132, 199, 0.45)',
    text: '#e0f2fe',
    glow: '#0284c7',
    badge: 'CARRYING',
  },
  releasing: {
    border: '#fbbf24',
    fill: 'rgba(251, 191, 36, 0.35)',
    text: '#fef08a',
    glow: '#d97706',
    badge: 'RELEASING',
  },
  falling: {
    border: '#a855f7',
    fill: 'rgba(168, 85, 247, 0.35)',
    text: '#f3e8ff',
    glow: '#9333ea',
    badge: 'FALLING',
  },
  placed: {
    border: '#22c55e',
    fill: 'rgba(34, 197, 94, 0.4)',
    text: '#86efac',
    glow: '#16a34a',
    badge: 'PLACED',
  },
  slipping: {
    border: '#f97316',
    fill: 'rgba(249, 115, 22, 0.4)',
    text: '#fdba74',
    glow: '#ea580c',
    badge: 'SLIPPING',
  },
  broken: {
    border: '#ef4444',
    fill: 'rgba(239, 68, 68, 0.45)',
    text: '#fca5a5',
    glow: '#b91c1c',
    badge: 'BROKEN',
  },
};

export function drawDraggableObject({
  position,
  widthM,
  heightM,
  mass,
  status,
  transform,
  ctx,
  isPaused,
  isHovered = false,
  isSelected = false,
  isGrasped = false,
}: DrawObjectOptions): void {
  const { scale } = transform;
  const style = OBJECT_STATUS_STYLES[status] || OBJECT_STATUS_STYLES.idle;

  const centerCanvas = worldToCanvas(position, transform);
  const pixW = Math.max(28, widthM * scale);
  const pixH = Math.max(20, heightM * scale);

  const left = centerCanvas.x - pixW / 2;
  const top = centerCanvas.y - pixH / 2;

  ctx.save();

  // Selected or Hovered halo when paused
  if (isPaused) {
    if (isSelected || isHovered) {
      ctx.strokeStyle = isSelected ? '#38bdf8' : 'rgba(56, 189, 248, 0.5)';
      ctx.lineWidth = isSelected ? 3 : 2;
      ctx.strokeRect(left - 4, top - 4, pixW + 8, pixH + 8);
    }
  }

  // Glow if active status
  if (style.glow) {
    ctx.shadowColor = style.glow;
    ctx.shadowBlur = status === 'slipping' || status === 'broken' || status === 'placed' ? 14 : 8;
  }

  // Object Box
  ctx.fillStyle = style.fill;
  ctx.strokeStyle = style.border;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.roundRect(left, top, pixW, pixH, 5);
  ctx.fill();
  ctx.stroke();

  ctx.shadowBlur = 0; // reset shadow

  // Inner corner packaging stripes / industrial container ribs
  ctx.strokeStyle = style.border;
  ctx.lineWidth = 1;
  ctx.strokeRect(left + 3, top + 3, pixW - 6, pixH - 6);

  // Object Center Labels (Mass and Status Badge)
  ctx.fillStyle = '#ffffff';
  ctx.font = 'bold 10px ui-sans-serif, system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(`${mass.toFixed(1)} kg`, centerCanvas.x, centerCanvas.y - 4);

  ctx.fillStyle = style.text;
  ctx.font = 'bold 8px ui-monospace, monospace';
  ctx.fillText(style.badge, centerCanvas.x, centerCanvas.y + 7);

  // Paused Drag Cue
  if (isPaused && isHovered) {
    ctx.fillStyle = isGrasped ? '#fb923c' : '#fef08a';
    ctx.font = '9px ui-sans-serif, sans-serif';
    ctx.fillText(isGrasped ? 'Grasped by gripper' : 'Move Object', centerCanvas.x, top - 8);
  }

  ctx.restore();
}

/**
 * Hit test object bounding box
 */
export function hitTestObject(
  canvasPos: { x: number; y: number },
  objPosition: { x: number; y: number },
  widthM: number,
  heightM: number,
  transform: CanvasTransform
): boolean {
  const centerCanvas = worldToCanvas(objPosition, transform);
  const { scale } = transform;
  const pixW = Math.max(28, widthM * scale);
  const pixH = Math.max(20, heightM * scale);

  const left = centerCanvas.x - pixW / 2;
  const right = centerCanvas.x + pixW / 2;
  const top = centerCanvas.y - pixH / 2;
  const bottom = centerCanvas.y + pixH / 2;

  return (
    canvasPos.x >= left &&
    canvasPos.x <= right &&
    canvasPos.y >= top &&
    canvasPos.y <= bottom
  );
}
