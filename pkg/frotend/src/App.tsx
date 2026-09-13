import React, { useEffect } from 'react';
import { useSimulationStore } from './lib/simulation-store';
import { mockTelemetryClient } from './lib/mock-telemetry-client';
import { RobotCanvas } from './components/simulation/RobotCanvas';
import { SimulationControls } from './components/controls/SimulationControls';
import { PhysicsMetrics } from './components/metrics/PhysicsMetrics';
import { RuntimeMetrics } from './components/metrics/RuntimeMetrics';
import { WorkerInspection } from './components/workers/WorkerInspection';
import { ChartsPanel } from './components/metrics/ChartsPanel';
import { SceneEditor } from './components/simulation/SceneEditor';
import { ComparisonPanel } from './components/comparison/ComparisonPanel';
import {
  Activity,
  Layers,
  Radio,
  Sliders,
  Cpu,
  BarChart3,
  ExternalLink,
} from 'lucide-react';

export default function App() {
  const {
    runtimeStatus,
    selectedMode,
    webSocketStatus,
    setLatestTelemetry,
    activeTab,
    setActiveTab,
  } = useSimulationStore();

  // Connect mock telemetry adapter for Phase 1 visual development
  useEffect(() => {
    const unsubscribe = mockTelemetryClient.subscribe((snapshot) => {
      setLatestTelemetry(snapshot);
    });
    return () => {
      unsubscribe();
    };
  }, [setLatestTelemetry]);

  return (
    <div className="min-h-screen bg-slate-950 text-slate-100 flex flex-col font-sans selection:bg-cyan-500 selection:text-slate-950">
      {/* Top Header Bar */}
      <header
        id="app-header"
        className="h-14 bg-slate-900 border-b border-slate-800 px-4 sm:px-6 flex items-center justify-between shrink-0 shadow-md"
      >
        <div className="flex items-center gap-4">
          <div className="flex items-center gap-2.5">
            <div className="w-8 h-8 rounded-lg bg-cyan-500/10 border border-cyan-500/30 flex items-center justify-center text-cyan-400">
              <Cpu className="w-5 h-5" />
            </div>
            <div>
              <h1 className="text-sm font-bold tracking-tight text-white flex items-center gap-2">
                <span>SwarmDex Visualizer</span>
                <span className="text-[10px] font-mono px-1.5 py-0.5 rounded bg-slate-800 text-slate-400 font-normal">
                  MVP v0.1
                </span>
              </h1>
              <p className="text-[10px] font-mono text-slate-400 leading-none mt-0.5">
                Force-Control Robotics & Distributed SAC Simulator
              </p>
            </div>
          </div>

          {/* Navigation Tabs */}
          <div className="hidden sm:flex items-center gap-1 bg-slate-950 p-1 rounded-lg border border-slate-800 ml-4">
            <button
              id="nav-tab-simulation"
              onClick={() => setActiveTab('simulation')}
              className={`flex items-center gap-1.5 px-3 py-1 text-xs font-medium rounded-md transition-all ${
                activeTab === 'simulation'
                  ? 'bg-slate-800 text-cyan-400 shadow-xs'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              <Activity className="w-3.5 h-3.5" />
              <span>Simulation Dashboard</span>
            </button>
            <button
              id="nav-tab-comparison"
              onClick={() => setActiveTab('comparison')}
              className={`flex items-center gap-1.5 px-3 py-1 text-xs font-medium rounded-md transition-all ${
                activeTab === 'comparison'
                  ? 'bg-slate-800 text-cyan-400 shadow-xs'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              <BarChart3 className="w-3.5 h-3.5" />
              <span>Mode Comparison</span>
            </button>
          </div>
        </div>

        {/* Header Right Status Badges */}
        <div className="flex items-center gap-3">
          {/* Mode Indicator */}
          <div
            id="header-mode-badge"
            className="flex items-center gap-1.5 px-2.5 py-1 bg-slate-950 border border-slate-800 rounded-md text-xs font-mono"
          >
            <Layers className="w-3.5 h-3.5 text-cyan-400" />
            <span className="text-slate-400">Mode:</span>
            <span className="text-cyan-300 font-semibold uppercase">{selectedMode}</span>
          </div>

          {/* Telemetry Source / Stream Indicator */}
          <div
            id="header-telemetry-badge"
            className="hidden md:flex items-center gap-1.5 px-2.5 py-1 bg-slate-950 border border-slate-800 rounded-md text-xs font-mono"
          >
            <Radio className="w-3.5 h-3.5 text-emerald-400 animate-pulse" />
            <span className="text-slate-400">Stream:</span>
            <span className="text-emerald-400 font-medium">16Hz Live Telemetry</span>
          </div>
        </div>
      </header>

      {/* Main Container */}
      <main className="flex-1 p-3 sm:p-4 md:p-5 max-w-[1600px] w-full mx-auto flex flex-col gap-4">
        {activeTab === 'comparison' ? (
          <ComparisonPanel />
        ) : (
          <>
            {/* Top Section: Simulation Canvas & Side Panels */}
            <div className="grid grid-cols-1 lg:grid-cols-12 gap-4 items-stretch">
              {/* Left Column (Canvas): takes 8 columns on large screens */}
              <div className="lg:col-span-8 flex flex-col min-h-[460px] h-[520px]">
                <RobotCanvas />
              </div>

              {/* Right Column: Runtime Information or Scene Editor (when paused) */}
              <div className="lg:col-span-4 flex flex-col gap-3 min-h-[460px] h-[520px] overflow-y-auto pr-1">
                {runtimeStatus === 'paused' ? (
                  <>
                    <SceneEditor />
                    <PhysicsMetrics />
                  </>
                ) : (
                  <>
                    <PhysicsMetrics />
                    <WorkerInspection />
                  </>
                )}
              </div>
            </div>

            {/* Middle Section: Simulation Stepping Controls & Mode Selector */}
            <div className="w-full">
              <SimulationControls />
            </div>

            {/* Metrics Overview Grid */}
            <div className="w-full">
              <RuntimeMetrics />
            </div>

            {/* Bottom Section: Real-time Metric Charts */}
            <div className="w-full pt-1 pb-4">
              <ChartsPanel />
            </div>
          </>
        )}
      </main>

      {/* Industrial Footer */}
      <footer className="h-8 bg-slate-950 border-t border-slate-800/80 px-4 flex items-center justify-between text-[11px] font-mono text-slate-500 shrink-0">
        <div>SwarmDex Visualizer — Two-Segment Force Control Architecture</div>
        <div className="flex items-center gap-4">
          <span>Target Architecture: Go backend / Next.js Web</span>
          <span>Port 3000 Active</span>
        </div>
      </footer>
    </div>
  );
}
