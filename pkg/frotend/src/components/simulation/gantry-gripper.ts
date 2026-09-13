import { CanvasTransform, worldToCanvas } from './coordinate-system';
import { GantryState } from '../../types/simulation';

export interface DrawGantryOptions {
  gantry: GantryState;
  transform: CanvasTransform;
  ctx: CanvasRenderingContext2D;
  isPaused: boolean;
  isCarriageHovered?: boolean;
  isCarriageSelected?: boolean;
  isGripperHovered?: boolean;
  isGripperSelected?: boolean;
  breakForce?: number;
}

export function drawGantryGripper({
  gantry,
  transform,
  ctx,
  isPaused,
  isCarriageHovered = false,
  isCarriageSelected = false,
  isGripperHovered = false,
  isGripperSelected = false,
  breakForce = 12.0,
}: DrawGantryOptions): void {
  const { scale } = transform;
  const railY = gantry.rail_y;
  const carriageX = gantry.carriage_x;
  const gripperY = gantry.gripper_y;
  const jawOpening = gantry.jaw_opening; // 0 (closed) to 1 (open)
  const gripForce = gantry.grip_force;

  // World bounds for rail length
  const railMinX = transform.bounds.minX + 0.3;
  const railMaxX = transform.bounds.maxX - 0.3;

  const railStart = worldToCanvas({ x: railMinX, y: railY }, transform);
  const railEnd = worldToCanvas({ x: railMaxX, y: railY }, transform);
  const carriagePos = worldToCanvas({ x: carriageX, y: railY }, transform);
  const gripperPos = worldToCanvas({ x: carriageX, y: gripperY }, transform);

  // 1. Top Gantry Rail Beam (Structural I-beam / Box rail)
  ctx.save();
  const railHeightPx = Math.max(14, 0.12 * scale);
  const railTopY = railStart.y - railHeightPx / 2;
  const railW = railEnd.x - railStart.x;

  // Rail shadow / glow
  ctx.fillStyle = '#0f172a';
  ctx.fillRect(railStart.x - 6, railTopY - 4, railW + 12, railHeightPx + 8);

  // Main beam gradient
  const railGrad = ctx.createLinearGradient(0, railTopY, 0, railTopY + railHeightPx);
  railGrad.addColorStop(0, '#334155');
  railGrad.addColorStop(0.5, '#1e293b');
  railGrad.addColorStop(1, '#0f172a');
  ctx.fillStyle = railGrad;
  ctx.fillRect(railStart.x, railTopY, railW, railHeightPx);

  // Rail inner track groove
  ctx.strokeStyle = '#0284c7';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(railStart.x, railStart.y);
  ctx.lineTo(railEnd.x, railStart.y);
  ctx.stroke();

  // Metric tick markings on rail
  ctx.strokeStyle = 'rgba(56, 189, 248, 0.25)';
  ctx.lineWidth = 1;
  for (let wx = Math.ceil(railMinX); wx <= Math.floor(railMaxX); wx++) {
    const tick = worldToCanvas({ x: wx, y: railY }, transform);
    ctx.beginPath();
    ctx.moveTo(tick.x, railTopY);
    ctx.lineTo(tick.x, railTopY + 5);
    ctx.stroke();
  }

  // End stops
  ctx.fillStyle = '#f59e0b';
  ctx.fillRect(railStart.x - 8, railTopY - 3, 8, railHeightPx + 6);
  ctx.fillRect(railEnd.x, railTopY - 3, 8, railHeightPx + 6);

  // 2. Movable Carriage
  const carriageW = Math.max(34, 0.4 * scale);
  const carriageH = Math.max(22, 0.22 * scale);
  const carriageLeft = carriagePos.x - carriageW / 2;
  const carriageTop = railStart.y - carriageH / 2;

  // Hover/Selected halo for carriage
  if (isPaused) {
    if (isCarriageSelected || isCarriageHovered) {
      ctx.strokeStyle = isCarriageSelected ? '#38bdf8' : '#38bdf888';
      ctx.lineWidth = isCarriageSelected ? 3 : 2;
      ctx.strokeRect(carriageLeft - 4, carriageTop - 4, carriageW + 8, carriageH + 8);
    }
  }

  // Carriage body
  const carriageGrad = ctx.createLinearGradient(carriageLeft, 0, carriageLeft + carriageW, 0);
  carriageGrad.addColorStop(0, '#0369a1');
  carriageGrad.addColorStop(0.5, '#0284c7');
  carriageGrad.addColorStop(1, '#0369a1');
  ctx.fillStyle = carriageGrad;
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.roundRect(carriageLeft, carriageTop, carriageW, carriageH, 4);
  ctx.fill();
  ctx.stroke();

  // Carriage Roller Bearings (top & bottom wheels)
  ctx.fillStyle = '#cbd5e1';
  [-carriageW * 0.35, carriageW * 0.35].forEach((offset) => {
    ctx.beginPath();
    ctx.arc(carriagePos.x + offset, railTopY - 2, 4, 0, Math.PI * 2);
    ctx.arc(carriagePos.x + offset, railTopY + railHeightPx + 2, 4, 0, Math.PI * 2);
    ctx.fill();
  });

  // Carriage Label / Drag handle cue
  ctx.fillStyle = '#ffffff';
  ctx.font = 'bold 9px ui-monospace, monospace';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText('CARRIAGE', carriagePos.x, carriageTop + carriageH / 2);

  // 3. Vertical Shaft (Telescoping rod extending from carriage to gripper)
  const shaftW = Math.max(8, 0.08 * scale);
  const shaftTopY = carriageTop + carriageH;
  const shaftBottomY = gripperPos.y - 12; // connect to gripper housing
  const shaftLen = Math.max(0, shaftBottomY - shaftTopY);

  if (shaftLen > 0) {
    // Outer tube
    const shaftGrad = ctx.createLinearGradient(carriagePos.x - shaftW / 2, 0, carriagePos.x + shaftW / 2, 0);
    shaftGrad.addColorStop(0, '#475569');
    shaftGrad.addColorStop(0.4, '#94a3b8');
    shaftGrad.addColorStop(1, '#334155');
    ctx.fillStyle = shaftGrad;
    ctx.strokeStyle = '#64748b';
    ctx.lineWidth = 1;
    ctx.fillRect(carriagePos.x - shaftW / 2, shaftTopY, shaftW, shaftLen);
    ctx.strokeRect(carriagePos.x - shaftW / 2, shaftTopY, shaftW, shaftLen);

    // Segment hash lines on telescoping cylinder
    ctx.strokeStyle = 'rgba(15, 23, 42, 0.4)';
    for (let sy = shaftTopY + 12; sy < shaftBottomY; sy += 14) {
      ctx.beginPath();
      ctx.moveTo(carriagePos.x - shaftW / 2 + 1, sy);
      ctx.lineTo(carriagePos.x + shaftW / 2 - 1, sy);
      ctx.stroke();
    }
  }

  // 4. Vertical Gripper Assembly at (carriageX, gripperY)
  const gripperHousingW = Math.max(28, 0.32 * scale);
  const gripperHousingH = Math.max(16, 0.16 * scale);
  const gBoxLeft = gripperPos.x - gripperHousingW / 2;
  const gBoxTop = gripperPos.y - gripperHousingH;

  // Hover / Selection indicator for gripper
  if (isPaused && (isGripperSelected || isGripperHovered)) {
    ctx.strokeStyle = isGripperSelected ? '#38bdf8' : '#38bdf888';
    ctx.lineWidth = isGripperSelected ? 3 : 2;
    ctx.strokeRect(gBoxLeft - 6, gBoxTop - 6, gripperHousingW + 12, gripperHousingH + 34);
  }

  // Actuator housing
  const gGrad = ctx.createLinearGradient(0, gBoxTop, 0, gBoxTop + gripperHousingH);
  gGrad.addColorStop(0, '#1e293b');
  gGrad.addColorStop(1, '#0f172a');
  ctx.fillStyle = gGrad;
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.roundRect(gBoxLeft, gBoxTop, gripperHousingW, gripperHousingH, 3);
  ctx.fill();
  ctx.stroke();

  // Status LED on gripper housing
  const isSqueezing = gripForce > 0.5;
  const isBreaking = gripForce >= breakForce;
  ctx.fillStyle = isBreaking ? '#ef4444' : isSqueezing ? '#22c55e' : '#38bdf8';
  ctx.beginPath();
  ctx.arc(gripperPos.x, gBoxTop + gripperHousingH / 2, 3, 0, Math.PI * 2);
  ctx.fill();

  // 5. Two Gripper Jaws (Left & Right)
  // Jaw spread depends on jawOpening (0 = clamped ~0.32m, 1 = wide open ~0.55m)
  const baseSpreadM = 0.22 + jawOpening * 0.24; // in meters
  const halfSpreadPx = (baseSpreadM * scale) / 2;
  const jawLengthPx = Math.max(24, 0.3 * scale);
  const jawThickness = Math.max(5, 0.05 * scale);
  const padLength = Math.max(12, 0.14 * scale);

  const jawY = gripperPos.y;

  // Horizontal guide bar connecting jaws to housing
  ctx.strokeStyle = '#64748b';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(gripperPos.x - halfSpreadPx, jawY);
  ctx.lineTo(gripperPos.x + halfSpreadPx, jawY);
  ctx.stroke();

  // Left Jaw
  const leftJawX = gripperPos.x - halfSpreadPx;
  ctx.fillStyle = '#0284c7';
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 1.5;
  // Vertical arm
  ctx.fillRect(leftJawX - jawThickness, jawY, jawThickness, jawLengthPx);
  ctx.strokeRect(leftJawX - jawThickness, jawY, jawThickness, jawLengthPx);
  // Inward tip foot
  ctx.fillRect(leftJawX - jawThickness, jawY + jawLengthPx - 4, jawThickness + 6, 4);

  // Right Jaw
  const rightJawX = gripperPos.x + halfSpreadPx;
  ctx.fillRect(rightJawX, jawY, jawThickness, jawLengthPx);
  ctx.strokeRect(rightJawX, jawY, jawThickness, jawLengthPx);
  // Inward tip foot
  ctx.fillRect(rightJawX - 6, jawY + jawLengthPx - 4, jawThickness + 6, 4);

  // High-friction rubber contact pads on inner edges
  const padTopY = jawY + jawLengthPx - padLength - 2;
  ctx.fillStyle = isBreaking ? '#ef4444' : isSqueezing ? '#4ade80' : '#f59e0b';
  // Left pad (on inner right face of left jaw)
  ctx.fillRect(leftJawX, padTopY, 3, padLength);
  // Right pad (on inner left face of right jaw)
  ctx.fillRect(rightJawX - 3, padTopY, 3, padLength);

  // 6. Inward Normal Grip Force Arrows when force is active
  if (gripForce > 0.5) {
    const arrowLen = Math.min(22, 6 + (gripForce / breakForce) * 16);
    ctx.strokeStyle = isBreaking ? '#ef4444' : '#4ade80';
    ctx.fillStyle = isBreaking ? '#ef4444' : '#4ade80';
    ctx.lineWidth = 2;

    const midPadY = padTopY + padLength / 2;

    // Left inward arrow (points right)
    ctx.beginPath();
    ctx.moveTo(leftJawX - arrowLen, midPadY);
    ctx.lineTo(leftJawX + 2, midPadY);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(leftJawX - 2, midPadY - 4);
    ctx.lineTo(leftJawX + 5, midPadY);
    ctx.lineTo(leftJawX - 2, midPadY + 4);
    ctx.closePath();
    ctx.fill();

    // Right inward arrow (points left)
    ctx.beginPath();
    ctx.moveTo(rightJawX + arrowLen, midPadY);
    ctx.lineTo(rightJawX - 2, midPadY);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(rightJawX + 2, midPadY - 4);
    ctx.lineTo(rightJawX - 5, midPadY);
    ctx.lineTo(rightJawX + 2, midPadY + 4);
    ctx.closePath();
    ctx.fill();

    // Force readout text above gripper
    ctx.fillStyle = isBreaking ? '#fca5a5' : '#86efac';
    ctx.font = 'bold 10px ui-monospace, monospace';
    ctx.textAlign = 'center';
    ctx.fillText(`${gripForce.toFixed(1)} N`, gripperPos.x, gBoxTop - 6);
  }

  // Paused Drag Hints
  if (isPaused) {
    ctx.fillStyle = '#fef08a';
    ctx.font = '9px ui-sans-serif, sans-serif';
    ctx.textAlign = 'center';
    if (isCarriageHovered) {
      ctx.fillText('↔ Drag Carriage', carriagePos.x, carriageTop - 6);
    }
    if (isGripperHovered) {
      ctx.fillText('↕ Drag Gripper', gripperPos.x, jawY + jawLengthPx + 14);
    }
  }

  ctx.restore();
}
