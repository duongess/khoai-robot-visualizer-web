import React, { useEffect, useRef, useState, useCallback } from 'react';
import { useSimulationStore, SelectedEntityType } from '../../lib/simulation-store';
import {
  getCanvasTransform,
  worldToCanvas,
  canvasToWorld,
  getTerrainHeightAt,
  DEFAULT_WORLD_BOUNDS,
  CanvasTransform,
} from './coordinate-system';
import { drawGantryGripper } from './gantry-gripper';
import { drawDraggableObject, hitTestObject } from './draggable-object';
import {
  drawEditableTerrain,
  hitTestTerrainPoint,
  hitTestTerrainSurface,
} from './editable-terrain';
import { drawTargetArea, hitTestTargetArea } from './target-area';
import { GantryState, ObjectStatus, TerrainPoint } from '../../types/simulation';
import { Trash2, RotateCcw, AlertCircle, Info } from 'lucide-react';

export const RobotCanvas: React.FC = () => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const containerRef = useRef<HTMLDivElement | null>(null);

  const {
    latestTelemetry,
    runtimeStatus,
    draftConfig,
    confirmedConfig,
    selectedEntity,
    selectedTerrainPointId,
    hoveredEntity,
    setSelectedEntity,
    setHoveredEntity,
    setIsDraggingEntity,
    updateDraftObject,
    updateDraftGantry,
    updateDraftTarget,
    updateDraftTerrain,
    addTerrainPoint,
    deleteSelectedTerrainPoint,
    resetFlatTerrain,
    submitSceneUpdate,
    configError,
  } = useSimulationStore();

  const isPaused = runtimeStatus === 'paused';
  const workspaceBounds = latestTelemetry?.worker.workspace ?? confirmedConfig.workspace ?? DEFAULT_WORLD_BOUNDS;

  // Active dragging state ref
  const dragTargetRef = useRef<SelectedEntityType>(null);
  const dragPointIdRef = useRef<string | null>(null);
  const pointerDownPosRef = useRef<{ x: number; y: number }>({ x: 0, y: 0 });
  const [cursorStyle, setCursorStyle] = useState<string>('default');

  // Animation / Render state ref for 60fps smoothing
  const renderStateRef = useRef<{
    carriageX: number;
    gripperY: number;
    jawOpening: number;
    gripForce: number;
    objX: number;
    objY: number;
    status: ObjectStatus;
    targetX: number;
    targetWidth: number;
    terrainPoints: TerrainPoint[];
  }>({
    carriageX: draftConfig.gantry.carriage_x,
    gripperY: draftConfig.gantry.gripper_y,
    jawOpening: 0.8,
    gripForce: 0.0,
    objX: draftConfig.object.position_x,
    objY: draftConfig.object.position_y,
    status: 'idle',
    targetX: draftConfig.target.position_x,
    targetWidth: draftConfig.target.width,
    terrainPoints: draftConfig.terrain.points,
  });

  // Keep state synchronized with telemetry (when running) or draftConfig (when paused)
  useEffect(() => {
    if (isPaused) {
      const d = draftConfig;
      const s = renderStateRef.current;
      s.carriageX = d.gantry.carriage_x;
      s.gripperY = d.gantry.gripper_y;
      s.objX = d.object.position_x;
      s.objY = d.object.position_y;
      s.targetX = d.target.position_x;
      s.targetWidth = d.target.width;
      s.terrainPoints = d.terrain.points;
      s.gripForce = d.gantry.initial_grip_force;
      s.status = 'idle';
    } else if (latestTelemetry) {
      const w = latestTelemetry.worker;
      const s = renderStateRef.current;
      s.status = w.object.status;
      s.targetX = w.target.position_x;
      s.targetWidth = w.target.width;
      s.terrainPoints = w.terrain?.points || draftConfig.terrain.points;
    }
  }, [isPaused, draftConfig, latestTelemetry]);

  // Main 60 FPS Render Loop (Fixed camera & fixed world bounds)
  useEffect(() => {
    let animId: number;

    const render = () => {
      const canvas = canvasRef.current;
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      if (!ctx) return;

      const width = canvas.parentElement?.clientWidth || 800;
      const height = canvas.parentElement?.clientHeight || 480;
      const dpr = window.devicePixelRatio || 1;

      if (canvas.width !== width * dpr || canvas.height !== height * dpr) {
        canvas.width = width * dpr;
        canvas.height = height * dpr;
      }

      ctx.save();
      ctx.scale(dpr, dpr);

      // Compute fixed camera transform
      const transform = getCanvasTransform(width, height, workspaceBounds, 20);

      // Smooth lerp when running
      const cur = renderStateRef.current;
      if (!isPaused && latestTelemetry) {
        const w = latestTelemetry.worker;
        const lerpFactor = 0.28;
        cur.carriageX += (w.gantry.carriage_x - cur.carriageX) * lerpFactor;
        cur.gripperY += (w.gantry.gripper_y - cur.gripperY) * lerpFactor;
        cur.jawOpening += (w.gantry.jaw_opening - cur.jawOpening) * lerpFactor;
        cur.gripForce += (w.gantry.grip_force - cur.gripForce) * lerpFactor;
        cur.objX += (w.object.position.x - cur.objX) * lerpFactor;
        cur.objY += (w.object.position.y - cur.objY) * lerpFactor;
        cur.status = w.object.status;
      }

      // 1. Dark Industrial Workspace Background
      ctx.fillStyle = '#090d16';
      ctx.fillRect(0, 0, width, height);

      // Subtle engineering grid in world space
      ctx.strokeStyle = 'rgba(30, 41, 59, 0.4)';
      ctx.lineWidth = 1;
      const gridStepM = 0.5; // grid every 0.5 meters
      for (let wx = workspaceBounds.minX; wx <= workspaceBounds.maxX; wx += gridStepM) {
        const p1 = worldToCanvas({ x: wx, y: workspaceBounds.minY }, transform);
        const p2 = worldToCanvas({ x: wx, y: workspaceBounds.maxY }, transform);
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
      }
      for (let wy = workspaceBounds.minY; wy <= workspaceBounds.maxY; wy += gridStepM) {
        const p1 = worldToCanvas({ x: workspaceBounds.minX, y: wy }, transform);
        const p2 = worldToCanvas({ x: workspaceBounds.maxX, y: wy }, transform);
        ctx.beginPath();
        ctx.moveTo(p1.x, p1.y);
        ctx.lineTo(p2.x, p2.y);
        ctx.stroke();
      }

      // Workspace outer boundary frame
      const frameTL = worldToCanvas({ x: workspaceBounds.minX, y: workspaceBounds.maxY }, transform);
      const frameBR = worldToCanvas({ x: workspaceBounds.maxX, y: workspaceBounds.minY }, transform);
      ctx.strokeStyle = 'rgba(56, 189, 248, 0.2)';
      ctx.lineWidth = 1.5;
      ctx.strokeRect(frameTL.x, frameTL.y, frameBR.x - frameTL.x, frameBR.y - frameTL.y);

      // Metric boundary label
      ctx.fillStyle = 'rgba(148, 163, 184, 0.4)';
      ctx.font = '9px ui-monospace, monospace';
      ctx.textAlign = 'left';
      ctx.fillText(`WORKSPACE: ${(workspaceBounds.maxX-workspaceBounds.minX).toFixed(1)}m × ${(workspaceBounds.maxY-workspaceBounds.minY).toFixed(1)}m [BACKEND]`, frameTL.x + 8, frameTL.y + 14);

      // 2. Editable 2D Terrain
      drawEditableTerrain({
        points: isPaused ? draftConfig.terrain.points : cur.terrainPoints,
        transform,
        ctx,
        isPaused,
        selectedPointId: selectedTerrainPointId,
        hoveredPointId: hoveredEntity === 'terrain_point' ? selectedTerrainPointId : null,
      });

      // 3. Target Drop Area on terrain
      const currentTargetX = isPaused ? draftConfig.target.position_x : cur.targetX;
      const currentTargetW = isPaused ? draftConfig.target.width : cur.targetWidth;
      const isTargetSuccess = cur.status === 'placed';
      const isTargetPartial = cur.status === 'releasing';

      drawTargetArea({
        targetX: currentTargetX,
        widthM: currentTargetW,
        terrainPoints: isPaused ? draftConfig.terrain.points : cur.terrainPoints,
        transform,
        ctx,
        isPaused,
        isHovered: hoveredEntity === 'target',
        isSelected: selectedEntity === 'target',
        isSuccess: isTargetSuccess,
        isPartial: isTargetPartial,
      });

      // 4. Draggable Object
      const currentObjX = isPaused ? draftConfig.object.position_x : cur.objX;
      const currentObjY = isPaused ? draftConfig.object.position_y : cur.objY;
      const isGrasped = cur.status === 'grasped' || cur.status === 'attached' || cur.status === 'transported' || cur.status === 'lifting' || cur.status === 'carrying';

      drawDraggableObject({
        position: { x: currentObjX, y: currentObjY },
        widthM: draftConfig.object.width,
        heightM: draftConfig.object.height,
        mass: draftConfig.object.mass,
        status: cur.status,
        transform,
        ctx,
        isPaused,
        isHovered: hoveredEntity === 'object',
        isSelected: selectedEntity === 'object',
        isGrasped,
      });

      // 5. Overhead Claw Gantry Mechanism (Rail, Carriage, Shaft, Gripper)
      const gantryState: GantryState = {
        rail_y: draftConfig.gantry.rail_y,
        carriage_x: isPaused ? draftConfig.gantry.carriage_x : cur.carriageX,
        gripper_y: isPaused ? draftConfig.gantry.gripper_y : cur.gripperY,
        jaw_opening: isPaused ? 0.75 : cur.jawOpening,
        grip_force: isPaused ? 0.0 : cur.gripForce,
      };

      drawGantryGripper({
        gantry: gantryState,
        transform,
        ctx,
        isPaused,
        isCarriageHovered: hoveredEntity === 'carriage',
        isCarriageSelected: selectedEntity === 'carriage',
        isGripperHovered: hoveredEntity === 'gripper',
        isGripperSelected: selectedEntity === 'gripper',
        breakForce: draftConfig.object.break_force,
      });

      ctx.restore();
      animId = requestAnimationFrame(render);
    };

    animId = requestAnimationFrame(render);
    return () => cancelAnimationFrame(animId);
  }, [
    isPaused,
    draftConfig,
    latestTelemetry,
    selectedEntity,
    selectedTerrainPointId,
    hoveredEntity,
  ]);

  // Pointer Event Handlers for direct scene editing (paused mode only)
  const handlePointerDown = (e: React.PointerEvent<HTMLCanvasElement>) => {
    if (!isPaused) return;

    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const canvasPos = {
      x: e.clientX - rect.left,
      y: e.clientY - rect.top,
    };

    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    const transform = getCanvasTransform(width, height, workspaceBounds, 20);

    pointerDownPosRef.current = canvasPos;

    // Check hit tests in order:
    // 1. Terrain control point
    const hitPt = hitTestTerrainPoint(canvasPos, draftConfig.terrain.points, transform);
    if (hitPt) {
      dragTargetRef.current = 'terrain_point';
      dragPointIdRef.current = hitPt.id;
      setSelectedEntity('terrain_point', hitPt.id);
      setIsDraggingEntity(true);
      setCursorStyle('grabbing');
      canvas.setPointerCapture(e.pointerId);
      return;
    }

    // 2. Draggable object (unless grasped)
    const isObjectHit = hitTestObject(
      canvasPos,
      { x: draftConfig.object.position_x, y: draftConfig.object.position_y },
      draftConfig.object.width,
      draftConfig.object.height,
      transform
    );
    if (isObjectHit) {
      dragTargetRef.current = 'object';
      setSelectedEntity('object');
      setIsDraggingEntity(true);
      setCursorStyle('grabbing');
      canvas.setPointerCapture(e.pointerId);
      return;
    }

    // 3. Gripper head (vertical movement)
    const gripperCanvas = worldToCanvas(
      { x: draftConfig.gantry.carriage_x, y: draftConfig.gantry.gripper_y },
      transform
    );
    const distGripper = Math.hypot(canvasPos.x - gripperCanvas.x, canvasPos.y - gripperCanvas.y);
    if (distGripper <= 28) {
      dragTargetRef.current = 'gripper';
      setSelectedEntity('gripper');
      setIsDraggingEntity(true);
      setCursorStyle('ns-resize');
      canvas.setPointerCapture(e.pointerId);
      return;
    }

    // 4. Carriage (horizontal movement along rail)
    const railCanvas = worldToCanvas(
      { x: draftConfig.gantry.carriage_x, y: draftConfig.gantry.rail_y },
      transform
    );
    const distCarriage = Math.hypot(canvasPos.x - railCanvas.x, canvasPos.y - railCanvas.y);
    if (distCarriage <= 32) {
      dragTargetRef.current = 'carriage';
      setSelectedEntity('carriage');
      setIsDraggingEntity(true);
      setCursorStyle('ew-resize');
      canvas.setPointerCapture(e.pointerId);
      return;
    }

    // 5. Target drop area
    const isTargetHit = hitTestTargetArea(
      canvasPos,
      draftConfig.target.position_x,
      draftConfig.target.width,
      draftConfig.terrain.points,
      transform
    );
    if (isTargetHit) {
      dragTargetRef.current = 'target';
      setSelectedEntity('target');
      setIsDraggingEntity(true);
      setCursorStyle('grabbing');
      canvas.setPointerCapture(e.pointerId);
      return;
    }

    // Clicking empty space deselects
    setSelectedEntity(null);
  };

  const handlePointerMove = (e: React.PointerEvent<HTMLCanvasElement>) => {
    if (!isPaused) {
      if (cursorStyle !== 'default') setCursorStyle('default');
      return;
    }

    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const canvasPos = {
      x: e.clientX - rect.left,
      y: e.clientY - rect.top,
    };

    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    const transform = getCanvasTransform(width, height, workspaceBounds, 20);

    // If dragging an entity, update draft state immediately in world coordinates
    if (dragTargetRef.current) {
      const worldPos = canvasToWorld(canvasPos, transform);

      switch (dragTargetRef.current) {
        case 'object': {
          // Clamp object within world horizontal bounds
          const halfW = draftConfig.object.width / 2;
          const clampedX = Math.min(workspaceBounds.maxX-halfW, Math.max(workspaceBounds.minX+halfW, worldPos.x));
          // Clamp Y to be above terrain
          const terrainY = getTerrainHeightAt(clampedX, draftConfig.terrain.points);
          const minY = terrainY + draftConfig.object.height / 2;
          const clampedY = Math.min(workspaceBounds.maxY-draftConfig.object.height/2, Math.max(minY, worldPos.y));

          updateDraftObject({
            position_x: Number(clampedX.toFixed(2)),
            position_y: Number(clampedY.toFixed(2)),
          });
          break;
        }

        case 'carriage': {
          const clampedX = Math.min(
            draftConfig.gantry.max_x,
            Math.max(draftConfig.gantry.min_x, worldPos.x)
          );
          updateDraftGantry({
            carriage_x: Number(clampedX.toFixed(2)),
          });
          break;
        }

        case 'gripper': {
          // These limits originate with the backend's physical gripper model.
          const minY = draftConfig.gantry.min_y;
          const maxY = draftConfig.gantry.max_y;
          const clampedY = Math.min(maxY, Math.max(minY, worldPos.y));
          updateDraftGantry({
            gripper_y: Number(clampedY.toFixed(2)),
          });
          break;
        }

        case 'target': {
          const halfWidth = draftConfig.target.width / 2;
          const clampedX = Math.min(
            workspaceBounds.maxX - halfWidth,
            Math.max(workspaceBounds.minX + halfWidth, worldPos.x),
          );
          updateDraftTarget({
            position_x: Number(clampedX.toFixed(2)),
          });
          break;
        }

        case 'terrain_point': {
          const ptId = dragPointIdRef.current;
          if (!ptId) break;
          const pts = [...draftConfig.terrain.points].sort((a, b) => a.x - b.x);
          const idx = pts.findIndex((p) => p.id === ptId);
          if (idx === -1) break;

          const isLeftBoundary = idx === 0;
          const isRightBoundary = idx === pts.length - 1;

          // Clamping X between neighbors
          let clampedX = pts[idx].x;
          if (!isLeftBoundary && !isRightBoundary) {
            const minAllowedX = pts[idx - 1].x + 0.2;
            const maxAllowedX = pts[idx + 1].x - 0.2;
            clampedX = Math.min(maxAllowedX, Math.max(minAllowedX, worldPos.x));
          }

		  const clampedY = Math.min(workspaceBounds.maxY, Math.max(workspaceBounds.minY, worldPos.y));

          pts[idx] = {
            id: ptId,
            x: Number(clampedX.toFixed(2)),
            y: Number(clampedY.toFixed(2)),
          };

          updateDraftTerrain({ points: pts });
          break;
        }
      }
      return;
    }

    // Hover hit detection when not dragging
    const hitPt = hitTestTerrainPoint(canvasPos, draftConfig.terrain.points, transform);
    if (hitPt) {
      setHoveredEntity('terrain_point');
      setCursorStyle('grab');
      return;
    }

    const isObj = hitTestObject(
      canvasPos,
      { x: draftConfig.object.position_x, y: draftConfig.object.position_y },
      draftConfig.object.width,
      draftConfig.object.height,
      transform
    );
    if (isObj) {
      setHoveredEntity('object');
      setCursorStyle('grab');
      return;
    }

    const gripperCanvas = worldToCanvas(
      { x: draftConfig.gantry.carriage_x, y: draftConfig.gantry.gripper_y },
      transform
    );
    if (Math.hypot(canvasPos.x - gripperCanvas.x, canvasPos.y - gripperCanvas.y) <= 28) {
      setHoveredEntity('gripper');
      setCursorStyle('ns-resize');
      return;
    }

    const railCanvas = worldToCanvas(
      { x: draftConfig.gantry.carriage_x, y: draftConfig.gantry.rail_y },
      transform
    );
    if (Math.hypot(canvasPos.x - railCanvas.x, canvasPos.y - railCanvas.y) <= 32) {
      setHoveredEntity('carriage');
      setCursorStyle('ew-resize');
      return;
    }

    const isTarget = hitTestTargetArea(
      canvasPos,
      draftConfig.target.position_x,
      draftConfig.target.width,
      draftConfig.terrain.points,
      transform
    );
    if (isTarget) {
      setHoveredEntity('target');
      setCursorStyle('grab');
      return;
    }

    setHoveredEntity(null);
    setCursorStyle('default');
  };

  const handlePointerUp = async (e: React.PointerEvent<HTMLCanvasElement>) => {
    if (!isPaused) return;

    if (dragTargetRef.current) {
      const canvas = canvasRef.current;
      if (canvas && canvas.hasPointerCapture(e.pointerId)) {
        canvas.releasePointerCapture(e.pointerId);
      }

      setIsDraggingEntity(false);
      setCursorStyle('default');

      // Send single PUT /api/scene update on release (Section 12 & 13)
      const target = dragTargetRef.current;
      dragTargetRef.current = null;
      dragPointIdRef.current = null;

      if (target === 'object') {
        await submitSceneUpdate({
          object: {
            id: draftConfig.object.id,
            position: { x: draftConfig.object.position_x, y: draftConfig.object.position_y },
          },
        });
      } else if (target === 'carriage' || target === 'gripper') {
        await submitSceneUpdate({
          gantry: {
            carriage_x: draftConfig.gantry.carriage_x,
            gripper_y: draftConfig.gantry.gripper_y,
          },
        });
      } else if (target === 'target') {
        await submitSceneUpdate({
          target: {
            position: { x: draftConfig.target.position_x },
          },
        });
      } else if (target === 'terrain_point') {
        await submitSceneUpdate({
          terrain: {
            points: draftConfig.terrain.points,
          },
        });
      }
    }
  };

  // Double-click to add terrain point
  const handleDoubleClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!isPaused) return;

    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const canvasPos = {
      x: e.clientX - rect.left,
      y: e.clientY - rect.top,
    };

    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    const transform = getCanvasTransform(width, height, workspaceBounds, 20);

    const hitSurface = hitTestTerrainSurface(canvasPos, draftConfig.terrain.points, transform);
    if (hitSurface) {
      const newPt: TerrainPoint = {
        id: `t-${Date.now().toString(36)}`,
        x: hitSurface.worldX,
        y: hitSurface.worldY,
      };
      addTerrainPoint(newPt);
    }
  };

  // Delete key listener for selected terrain point
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (!isPaused) return;
      if (e.key === 'Delete' || e.key === 'Backspace') {
        if (selectedEntity === 'terrain_point' && selectedTerrainPointId) {
          deleteSelectedTerrainPoint();
        }
      }
    };
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [isPaused, selectedEntity, selectedTerrainPointId, deleteSelectedTerrainPoint]);

  return (
    <div
      id="simulation-canvas-container"
      ref={containerRef}
      className="relative w-full h-full min-h-[460px] bg-slate-950 rounded-lg overflow-hidden border border-slate-800 flex flex-col"
    >
      {/* Top Workspace Header Bar */}
      <div
        id="canvas-header-bar"
        className="px-3.5 py-2 bg-slate-900/90 border-b border-slate-800 flex items-center justify-between z-10 select-none"
      >
        <div className="flex items-center gap-2 text-xs font-mono">
          <span className="w-2 h-2 rounded-full bg-cyan-400 animate-pulse" />
          <span className="text-slate-200 font-semibold">Overhead Gantry Simulation</span>
          <span className="text-slate-500">|</span>
          <span className="text-slate-400">Claw-Machine Force Control</span>
        </div>

        {/* Status / Editing State */}
        <div className="flex items-center gap-3 text-xs">
          {isPaused ? (
            <div className="flex items-center gap-2">
              <span className="px-2.5 py-0.5 rounded-full bg-amber-500/10 border border-amber-500/30 text-amber-400 font-mono text-[11px] flex items-center gap-1.5">
                <span className="w-1.5 h-1.5 rounded-full bg-amber-400" />
                Paused: Scene Editing Enabled
              </span>

              {/* Quick Terrain Point Actions */}
              {selectedEntity === 'terrain_point' && selectedTerrainPointId && (
                <button
                  id="delete-terrain-point-btn"
                  onClick={deleteSelectedTerrainPoint}
                  className="flex items-center gap-1 px-2 py-0.5 rounded bg-red-500/10 hover:bg-red-500/20 text-red-400 border border-red-500/30 font-mono text-[11px] transition-colors"
                  title="Delete Selected Terrain Point (Del)"
                >
                  <Trash2 className="w-3 h-3" />
                  <span>Delete Point</span>
                </button>
              )}

              <button
                id="reset-flat-terrain-btn"
                onClick={resetFlatTerrain}
                className="flex items-center gap-1 px-2.5 py-0.5 rounded bg-slate-800 hover:bg-slate-700 text-slate-300 border border-slate-700 font-mono text-[11px] transition-colors"
                title="Reset terrain to flat baseline"
              >
                <RotateCcw className="w-3 h-3" />
                <span>Reset Flat Ground</span>
              </button>
            </div>
          ) : (
            <span className="px-2.5 py-0.5 rounded-full bg-emerald-500/10 border border-emerald-500/30 text-emerald-400 font-mono text-[11px]">
              Active Authoritative Simulation
            </span>
          )}
        </div>
      </div>

      {/* Main Canvas with Pointer Events */}
      <div className="relative flex-1 w-full h-full overflow-hidden">
        <canvas
          id="gantry-simulation-canvas"
          ref={canvasRef}
          onPointerDown={handlePointerDown}
          onPointerMove={handlePointerMove}
          onPointerUp={handlePointerUp}
          onPointerCancel={handlePointerUp}
          onDoubleClick={handleDoubleClick}
          className="w-full h-full block touch-none"
          style={{ cursor: cursorStyle }}
        />

        {/* Paused Editing Helper Guidance Toast */}
        {isPaused && (
          <div
            id="paused-scene-editing-hints"
            className="absolute bottom-2.5 left-3 right-3 bg-slate-900/90 border border-slate-800/90 backdrop-blur-xs rounded-md px-3 py-1.5 flex items-center justify-between text-[11px] text-slate-300 font-mono pointer-events-none"
          >
            <div className="flex items-center gap-2">
              <Info className="w-3.5 h-3.5 text-cyan-400" />
              <span>
                <strong className="text-white">Direct Editing:</strong> Drag object • Drag carriage horizontally • Drag gripper vertically • Drag terrain points • Double-click terrain to add point
              </span>
            </div>
            {selectedEntity && (
              <span className="text-cyan-400 uppercase font-semibold">
                Selected: {selectedEntity}
              </span>
            )}
          </div>
        )}

        {/* Error notification if backend rejected an update */}
        {configError && (
          <div
            id="scene-config-error-banner"
            className="absolute top-3 left-1/2 -translate-x-1/2 bg-red-950/90 border border-red-800 text-red-200 px-4 py-1.5 rounded-md text-xs font-mono flex items-center gap-2 shadow-lg"
          >
            <AlertCircle className="w-4 h-4 text-red-400" />
            <span>{configError}</span>
          </div>
        )}
      </div>
    </div>
  );
};
