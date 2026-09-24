import React, { useState } from 'react';
import { useSimulationStore } from '../../lib/simulation-store';
import {
  Sliders,
  Check,
  Undo2,
  RotateCcw,
  AlertCircle,
  Box,
  Layers,
  Mountain,
} from 'lucide-react';
import { getTerrainHeightAt } from './coordinate-system';

export const SceneEditor: React.FC = () => {
  const {
    draftConfig,
    runtimeStatus,
    isDraftDirty,
    configError,
    updateDraftObject,
    updateDraftGantry,
    updateDraftTerrain,
    applyDraftConfig,
    discardDraftConfig,
    restoreDefaultDraftConfig,
  } = useSimulationStore();

  const [activeTab, setActiveTab] = useState<'object' | 'gantry' | 'terrain'>('object');
  const [saveFeedback, setSaveFeedback] = useState<string | null>(null);
	const objectRestY = getTerrainHeightAt(draftConfig.object.position_x, draftConfig.terrain.points) + draftConfig.object.height / 2;

  if (runtimeStatus !== 'paused') {
    return (
      <div
        id="scene-editor-disabled-panel"
        className="bg-slate-900 border border-slate-800 rounded-lg p-5 flex flex-col items-center justify-center text-center space-y-3 min-h-[380px]"
      >
        <div className="w-10 h-10 rounded-full bg-slate-800 flex items-center justify-center text-slate-500">
          <Sliders className="w-5 h-5" />
        </div>
        <div>
          <h4 className="text-sm font-semibold text-slate-200">Scene Editing Locked</h4>
          <p className="text-xs text-slate-400 mt-1 max-w-[220px]">
            Scene properties can only be modified while the simulation is paused.
          </p>
        </div>
        <div className="text-[11px] font-mono text-cyan-400 bg-cyan-950/60 border border-cyan-800/60 px-3 py-1.5 rounded">
          Press [ Pause ] to unlock editor
        </div>
      </div>
    );
  }

  const handleApply = async () => {
    const res = await applyDraftConfig();
    if (res.success) {
      setSaveFeedback('Configuration applied to simulation!');
      setTimeout(() => setSaveFeedback(null), 3000);
    }
  };

  return (
    <div
      id="scene-editor-active-panel"
      className="bg-slate-900 border border-slate-700/80 rounded-lg p-4 flex flex-col justify-between space-y-3 shadow-lg"
    >
      {/* Header */}
      <div>
        <div className="flex items-center justify-between border-b border-slate-800 pb-2.5">
          <div className="flex items-center gap-2">
            <Sliders className="w-4 h-4 text-amber-400" />
            <h3 className="text-xs font-semibold text-slate-100 uppercase tracking-wider font-mono">
              Scene Configuration Draft
            </h3>
          </div>
          {isDraftDirty && (
            <span className="text-[10px] font-mono px-2 py-0.5 rounded bg-amber-500/20 text-amber-300 border border-amber-500/40">
              Unapplied Draft
            </span>
          )}
        </div>

        {/* Tab Switcher: Object, Gantry, Terrain */}
        <div className="flex items-center gap-1 mt-3 bg-slate-950 p-1 rounded-lg border border-slate-800">
          <button
            onClick={() => setActiveTab('object')}
            className={`flex-1 flex items-center justify-center gap-1 py-1.5 rounded text-xs font-mono transition-colors ${
              activeTab === 'object'
                ? 'bg-slate-800 text-cyan-400 font-semibold'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <Box className="w-3.5 h-3.5" />
            <span>Object</span>
          </button>
          <button
            onClick={() => setActiveTab('gantry')}
            className={`flex-1 flex items-center justify-center gap-1 py-1.5 rounded text-xs font-mono transition-colors ${
              activeTab === 'gantry'
                ? 'bg-slate-800 text-cyan-400 font-semibold'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <Layers className="w-3.5 h-3.5" />
            <span>Gantry</span>
          </button>
          <button
            onClick={() => setActiveTab('terrain')}
            className={`flex-1 flex items-center justify-center gap-1 py-1.5 rounded text-xs font-mono transition-colors ${
              activeTab === 'terrain'
                ? 'bg-slate-800 text-cyan-400 font-semibold'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            <Mountain className="w-3.5 h-3.5" />
            <span>Terrain</span>
          </button>
        </div>
      </div>

      {/* Inputs Form */}
      <div className="space-y-3 max-h-[300px] overflow-y-auto pr-1 text-xs">
        {activeTab === 'object' && (
          <div className="space-y-3">
            {/* Object Position X */}
            <div>
              <div className="flex justify-between text-slate-300 font-mono text-[11px] mb-1">
                <span>Position X: {draftConfig.object.position_x.toFixed(2)} m</span>
                <span className="text-slate-500">[0.5 - 5.5 m]</span>
              </div>
              <input
                type="range"
                min="0.5"
                max="5.5"
                step="0.05"
                value={draftConfig.object.position_x}
                onChange={(e) => updateDraftObject({ position_x: parseFloat(e.target.value) })}
                className="w-full accent-cyan-500 bg-slate-950 h-1.5 rounded-lg cursor-pointer"
              />
            </div>


			<div className="rounded border border-slate-800 bg-slate-950/60 px-2.5 py-2 font-mono text-[11px] text-slate-400">
				Rest height Y: <span className="text-slate-200">{objectRestY.toFixed(2)} m</span> · derived from terrain
			</div>

            {/* Mass */}
            <div>
              <div className="flex justify-between text-slate-300 font-mono text-[11px] mb-1">
                <span>Mass: {draftConfig.object.mass.toFixed(2)} kg</span>
                <span className="text-slate-500">[0.1 - 5.0 kg]</span>
              </div>
              <input
                type="range"
                min="0.1"
                max="5.0"
                step="0.1"
                value={draftConfig.object.mass}
                onChange={(e) => updateDraftObject({ mass: parseFloat(e.target.value) })}
                className="w-full accent-cyan-500 bg-slate-950 h-1.5 rounded-lg cursor-pointer"
              />
            </div>

            {/* Friction Coefficient */}
            <div>
              <div className="flex justify-between text-slate-300 font-mono text-[11px] mb-1">
                <span>Friction (μ): {draftConfig.object.friction.toFixed(2)}</span>
                <span className="text-slate-500">[0.05 - 1.0]</span>
              </div>
              <input
                type="range"
                min="0.05"
                max="1.0"
                step="0.05"
                value={draftConfig.object.friction}
                onChange={(e) => updateDraftObject({ friction: parseFloat(e.target.value) })}
                className="w-full accent-cyan-500 bg-slate-950 h-1.5 rounded-lg cursor-pointer"
              />
            </div>

            {/* Break Force */}
            <div>
              <div className="flex justify-between text-slate-300 font-mono text-[11px] mb-1">
                <span>Break Force: {draftConfig.object.break_force.toFixed(1)} N</span>
                <span className="text-slate-500">[5 - 30 N]</span>
              </div>
              <input
                type="range"
                min="5"
                max="30"
                step="0.5"
                value={draftConfig.object.break_force}
                onChange={(e) => updateDraftObject({ break_force: parseFloat(e.target.value) })}
                className="w-full accent-red-500 bg-slate-950 h-1.5 rounded-lg cursor-pointer"
              />
            </div>
          </div>
        )}

        {activeTab === 'gantry' && (
          <div className="space-y-3">
            {/* Carriage Position X */}
            <div>
              <div className="flex justify-between text-slate-300 font-mono text-[11px] mb-1">
                <span>Carriage Position X: {draftConfig.gantry.carriage_x.toFixed(2)} m</span>
                <span className="text-slate-500">[{draftConfig.gantry.min_x} - {draftConfig.gantry.max_x} m]</span>
              </div>
              <input
                type="range"
                min={draftConfig.gantry.min_x}
                max={draftConfig.gantry.max_x}
                step="0.05"
                value={draftConfig.gantry.carriage_x}
                onChange={(e) => updateDraftGantry({ carriage_x: parseFloat(e.target.value) })}
                className="w-full accent-cyan-500 bg-slate-950 h-1.5 rounded-lg cursor-pointer"
              />
            </div>

            {/* Gripper Height Y */}
            <div>
              <div className="flex justify-between text-slate-300 font-mono text-[11px] mb-1">
                <span>Gripper Height Y: {draftConfig.gantry.gripper_y.toFixed(2)} m</span>
                <span className="text-slate-500">[{draftConfig.gantry.min_y} - {draftConfig.gantry.max_y} m]</span>
              </div>
              <input
                type="range"
                min={draftConfig.gantry.min_y}
                max={draftConfig.gantry.max_y}
                step="0.05"
                value={draftConfig.gantry.gripper_y}
                onChange={(e) => updateDraftGantry({ gripper_y: parseFloat(e.target.value) })}
                className="w-full accent-cyan-500 bg-slate-950 h-1.5 rounded-lg cursor-pointer"
              />
            </div>

            {/* Max Grip Force */}
            <div>
              <div className="flex justify-between text-slate-300 font-mono text-[11px] mb-1">
                <span>Max Grip Force: {draftConfig.gantry.maximum_grip_force} N</span>
                <span className="text-slate-500">[10 - 40 N]</span>
              </div>
              <input
                type="range"
                min="10"
                max="40"
                step="1"
                value={draftConfig.gantry.maximum_grip_force}
                onChange={(e) => updateDraftGantry({ maximum_grip_force: parseFloat(e.target.value) })}
                className="w-full accent-cyan-500 bg-slate-950 h-1.5 rounded-lg cursor-pointer"
              />
            </div>
          </div>
        )}

        {activeTab === 'terrain' && (
          <div className="space-y-3">
            {/* Gravity */}
            <div>
              <div className="flex justify-between text-slate-300 font-mono text-[11px] mb-1">
                <span>Gravity: {draftConfig.terrain.gravity.toFixed(2)} m/s²</span>
                <span className="text-slate-500">[1.6 - 20 m/s²]</span>
              </div>
              <input
                type="range"
                min="1.6"
                max="20"
                step="0.2"
                value={draftConfig.terrain.gravity}
                onChange={(e) => updateDraftTerrain({ gravity: parseFloat(e.target.value) })}
                className="w-full accent-cyan-500 bg-slate-950 h-1.5 rounded-lg cursor-pointer"
              />
            </div>

            {/* Ground Friction */}
            <div>
              <div className="flex justify-between text-slate-300 font-mono text-[11px] mb-1">
                <span>Ground Friction: {draftConfig.terrain.ground_friction.toFixed(2)}</span>
                <span className="text-slate-500">[0.1 - 1.0]</span>
              </div>
              <input
                type="range"
                min="0.1"
                max="1.0"
                step="0.05"
                value={draftConfig.terrain.ground_friction}
                onChange={(e) => updateDraftTerrain({ ground_friction: parseFloat(e.target.value) })}
                className="w-full accent-cyan-500 bg-slate-950 h-1.5 rounded-lg cursor-pointer"
              />
            </div>

            <div className="bg-slate-950 p-2.5 rounded border border-slate-800 text-[11px] font-mono text-slate-400">
              <span>Control Points: {draftConfig.terrain.points.length} nodes</span>
              <p className="text-[10px] text-slate-500 mt-1">
                Drag nodes directly on the canvas or double-click the terrain to insert new points.
              </p>
            </div>
          </div>
        )}
      </div>

      {/* Validation Message / Feedback */}
      {configError && (
        <div className="flex items-center gap-1.5 p-2 bg-red-950/60 border border-red-800/80 rounded text-[11px] font-mono text-red-300">
          <AlertCircle className="w-3.5 h-3.5 shrink-0" />
          <span>{configError}</span>
        </div>
      )}

      {saveFeedback && (
        <div className="flex items-center gap-1.5 p-2 bg-emerald-950/60 border border-emerald-800/80 rounded text-[11px] font-mono text-emerald-300">
          <Check className="w-3.5 h-3.5 shrink-0" />
          <span>{saveFeedback}</span>
        </div>
      )}

      {/* Action Buttons: Apply, Discard, Restore Defaults */}
      <div className="pt-2 border-t border-slate-800 space-y-2">
        <div className="flex items-center gap-2">
          <button
            id="apply-scene-config-btn"
            onClick={handleApply}
            className="flex-1 flex items-center justify-center gap-1.5 py-2 px-3 rounded-md text-xs font-bold bg-cyan-500 hover:bg-cyan-400 text-slate-950 shadow-sm transition-colors"
          >
            <Check className="w-4 h-4" />
            <span>Apply Changes</span>
          </button>

          <button
            id="discard-scene-config-btn"
            onClick={discardDraftConfig}
            title="Discard changes and restore last accepted backend configuration"
            className="py-2 px-3 rounded-md text-xs font-mono text-slate-300 bg-slate-800 hover:bg-slate-700 border border-slate-700 transition-colors"
          >
            <Undo2 className="w-4 h-4" />
          </button>

          <button
            id="restore-default-config-btn"
            onClick={restoreDefaultDraftConfig}
            title="Reset draft to factory default parameters"
            className="py-2 px-3 rounded-md text-xs font-mono text-slate-300 bg-slate-800 hover:bg-slate-700 border border-slate-700 transition-colors"
          >
            <RotateCcw className="w-4 h-4" />
          </button>
        </div>
      </div>
    </div>
  );
};
