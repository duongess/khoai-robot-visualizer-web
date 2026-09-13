import React from 'react';
import { useSimulationStore } from '../../lib/simulation-store';
import { Play, Pause, RotateCw, RefreshCw, Layers, ShieldCheck, AlertTriangle } from 'lucide-react';
import { SimulationMode } from '../../types/simulation';

export const SimulationControls: React.FC = () => {
  const {
    runtimeStatus,
    selectedMode,
    startSimulation,
    pauseSimulation,
    resumeSimulation,
    resetSimulation,
    requestModeChange,
    selectedWorkerId,
    setSelectedWorker,
    showModeDialog,
    pendingMode,
    confirmModeChange,
    cancelModeChange,
  } = useSimulationStore();

  const isRunning = runtimeStatus === 'running';
  const isPaused = runtimeStatus === 'paused';
  const isStopped = runtimeStatus === 'stopped';
  const isResetting = runtimeStatus === 'resetting';

  const statusBadge = {
    running: { label: 'Running', bg: 'bg-emerald-500/10', border: 'border-emerald-500/30', text: 'text-emerald-400', dot: 'bg-emerald-400' },
    paused: { label: 'Paused', bg: 'bg-amber-500/10', border: 'border-amber-500/30', text: 'text-amber-400', dot: 'bg-amber-400' },
    stopped: { label: 'Stopped', bg: 'bg-slate-500/10', border: 'border-slate-500/30', text: 'text-slate-400', dot: 'bg-slate-400' },
    resetting: { label: 'Resetting', bg: 'bg-cyan-500/10', border: 'border-cyan-500/30', text: 'text-cyan-400', dot: 'bg-cyan-400' },
    error: { label: 'Error', bg: 'bg-red-500/10', border: 'border-red-500/30', text: 'text-red-400', dot: 'bg-red-400' },
  }[runtimeStatus] || { label: runtimeStatus, bg: 'bg-slate-500/10', border: 'border-slate-500/30', text: 'text-slate-400', dot: 'bg-slate-400' };

  return (
    <>
      <div
        id="simulation-control-bar"
        className="w-full bg-slate-900 border border-slate-800 rounded-lg p-3 px-4 flex flex-wrap items-center justify-between gap-4 shadow-sm"
      >
        {/* Left Section: Primary Stepping Actions */}
        <div className="flex items-center gap-2">
          {/* Start Button */}
          <button
            id="control-start-btn"
            onClick={startSimulation}
            disabled={isRunning || isResetting}
            className={`flex items-center gap-2 px-3.5 py-1.5 rounded-md text-sm font-medium border transition-colors ${
              isRunning
                ? 'bg-slate-800/40 text-slate-500 border-slate-800 cursor-not-allowed'
                : 'bg-emerald-600 hover:bg-emerald-500 text-white border-emerald-500 shadow-xs'
            }`}
          >
            <Play className="w-4 h-4 fill-current" />
            <span>Start</span>
          </button>

          {/* Pause Button */}
          <button
            id="control-pause-btn"
            onClick={pauseSimulation}
            disabled={!isRunning}
            className={`flex items-center gap-2 px-3.5 py-1.5 rounded-md text-sm font-medium border transition-colors ${
              !isRunning
                ? 'bg-slate-800/40 text-slate-500 border-slate-800 cursor-not-allowed'
                : 'bg-amber-600 hover:bg-amber-500 text-white border-amber-500 shadow-xs'
            }`}
          >
            <Pause className="w-4 h-4 fill-current" />
            <span>Pause</span>
          </button>

          {/* Resume Button */}
          <button
            id="control-resume-btn"
            onClick={resumeSimulation}
            disabled={!isPaused}
            className={`flex items-center gap-2 px-3.5 py-1.5 rounded-md text-sm font-medium border transition-colors ${
              !isPaused
                ? 'bg-slate-800/40 text-slate-500 border-slate-800 cursor-not-allowed'
                : 'bg-sky-600 hover:bg-sky-500 text-white border-sky-500 shadow-xs'
            }`}
          >
            <RotateCw className="w-4 h-4" />
            <span>Resume</span>
          </button>

          {/* Reset Button */}
          <button
            id="control-reset-btn"
            onClick={resetSimulation}
            disabled={isResetting}
            className="flex items-center gap-2 px-3.5 py-1.5 rounded-md text-sm font-medium bg-slate-800 hover:bg-slate-700 text-slate-200 border border-slate-700 transition-colors"
          >
            <RefreshCw className={`w-4 h-4 ${isResetting ? 'animate-spin' : ''}`} />
            <span>Reset</span>
          </button>

          {/* Runtime Status Pill */}
          <div
            id="runtime-status-pill"
            className={`ml-2 flex items-center gap-2 px-3 py-1 rounded-full text-xs font-mono font-medium border ${statusBadge.bg} ${statusBadge.border} ${statusBadge.text}`}
          >
            <span className={`w-2 h-2 rounded-full ${statusBadge.dot} ${isRunning ? 'animate-ping' : ''}`} />
            <span>{statusBadge.label}</span>
          </div>
        </div>

        {/* Center/Right Section: Mode Selector & Worker Switcher */}
        <div className="flex items-center gap-4">
          {/* Mode Selector */}
          <div className="flex items-center gap-1.5 bg-slate-950 border border-slate-800 p-1 rounded-lg">
            <span className="text-xs text-slate-400 px-2 font-mono flex items-center gap-1">
              <Layers className="w-3.5 h-3.5 text-cyan-400" />
              Mode:
            </span>
            <button
              id="mode-independent-btn"
              onClick={() => requestModeChange('independent')}
              className={`px-3 py-1 rounded text-xs font-medium transition-all ${
                selectedMode === 'independent'
                  ? 'bg-cyan-500 text-slate-950 font-semibold shadow-xs'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              Independent
            </button>
            <button
              id="mode-swarmdex-btn"
              onClick={() => requestModeChange('swarm')}
              className={`px-3 py-1 rounded text-xs font-medium transition-all ${
                selectedMode === 'swarm'
                  ? 'bg-cyan-500 text-slate-950 font-semibold shadow-xs'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              SwarmDex
            </button>
          </div>

          {/* Worker Selector */}
          <div className="flex items-center gap-2">
            <label htmlFor="worker-select-dropdown" className="text-xs text-slate-400 font-mono">
              Worker:
            </label>
            <select
              id="worker-select-dropdown"
              value={selectedWorkerId}
              onChange={(e) => setSelectedWorker(Number(e.target.value))}
              className="bg-slate-950 border border-slate-800 text-slate-200 text-xs font-mono rounded-md px-2.5 py-1.5 focus:outline-none focus:border-cyan-500 cursor-pointer"
            >
              {[1, 2, 3, 4, 5, 6, 7, 8].map((id) => (
                <option key={id} value={id}>
                  Worker #{id}
                </option>
              ))}
            </select>
          </div>
        </div>
      </div>

      {/* Confirmation Modal when switching mode */}
      {showModeDialog && (
        <div
          id="mode-change-confirmation-modal"
          className="fixed inset-0 z-50 flex items-center justify-center bg-black/70 backdrop-blur-xs p-4"
        >
          <div className="bg-slate-900 border border-slate-700 rounded-xl max-w-md w-full p-6 shadow-2xl space-y-4">
            <div className="flex items-start gap-3">
              <div className="p-2.5 rounded-lg bg-amber-500/10 border border-amber-500/30 text-amber-400">
                <AlertTriangle className="w-5 h-5" />
              </div>
              <div>
                <h3 className="text-base font-semibold text-white">
                  Switch Simulation Mode to {pendingMode === 'swarm' ? 'SwarmDex' : 'Independent'}?
                </h3>
                <p className="text-xs text-slate-300 mt-1 leading-relaxed">
                  Switching modes will reset current active episode trajectories and re-initialize the experience
                  replay buffers.
                </p>
              </div>
            </div>

            <div className="bg-slate-950/70 border border-slate-800 rounded-lg p-3 text-xs text-slate-300 space-y-1.5">
              <div className="flex justify-between">
                <span className="text-slate-400">Current Mode:</span>
                <span className="font-mono text-cyan-300 capitalize">{selectedMode}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Target Mode:</span>
                <span className="font-mono text-amber-300 capitalize">{pendingMode}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Episode Action:</span>
                <span className="font-mono text-slate-200">Re-index & Reset</span>
              </div>
            </div>

            <div className="flex items-center justify-end gap-3 pt-2">
              <button
                id="cancel-mode-change-btn"
                onClick={cancelModeChange}
                className="px-4 py-2 text-xs font-medium text-slate-300 bg-slate-800 hover:bg-slate-700 rounded-lg transition-colors"
              >
                Cancel
              </button>
              <button
                id="confirm-mode-change-btn"
                onClick={confirmModeChange}
                className="px-4 py-2 text-xs font-medium text-slate-950 bg-cyan-400 hover:bg-cyan-300 rounded-lg font-semibold transition-colors shadow-xs"
              >
                Confirm & Switch Mode
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
};
