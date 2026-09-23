import React from 'react';
import { useSimulationStore } from '../../lib/simulation-store';
import { Cpu, CheckCircle2, AlertCircle, ArrowUpRight } from 'lucide-react';

export const WorkerInspection: React.FC = () => {
  const { latestTelemetry, selectedWorkerId, setSelectedWorker } = useSimulationStore();
  const worker = latestTelemetry?.worker;
  const runtime = latestTelemetry?.runtime;

  const workersList = [1, 2, 3, 4, 5, 6, 7, 8];

  return (
    <div
      id="worker-inspection-panel"
      className="bg-slate-900 border border-slate-800 rounded-lg p-4 space-y-3.5 shadow-sm"
    >
      <div className="flex items-center justify-between">
        <h3 className="text-xs font-semibold text-slate-200 uppercase tracking-wider font-mono flex items-center gap-1.5">
          <Cpu className="w-3.5 h-3.5 text-sky-400" />
          <span>Worker Inspection</span>
        </h3>
        <span className="text-[11px] font-mono text-slate-400">
          Observing #{selectedWorkerId}
        </span>
      </div>

      {/* Horizontal Worker Switcher Pills */}
      <div className="flex flex-wrap gap-1.5">
        {workersList.map((id) => {
          const isSelected = selectedWorkerId === id;
          return (
            <button
              key={id}
              id={`select-worker-btn-${id}`}
              onClick={() => setSelectedWorker(id)}
              className={`px-2.5 py-1 text-xs font-mono rounded-md border transition-all ${
                isSelected
                  ? 'bg-sky-500 text-slate-950 font-bold border-sky-400 shadow-xs'
                  : 'bg-slate-950 hover:bg-slate-800 text-slate-300 border-slate-800'
              }`}
            >
              Worker {id}
            </button>
          );
        })}
      </div>

      {/* Worker Detail Attributes Grid */}
      <div className="grid grid-cols-2 sm:grid-cols-3 gap-2 text-xs font-mono">
        <div className="bg-slate-950/70 border border-slate-800/80 p-2 rounded">
          <span className="text-[10px] text-slate-400 block">Worker ID</span>
          <span className="font-bold text-slate-100">Worker #{worker?.id ?? selectedWorkerId}</span>
        </div>

        <div className="bg-slate-950/70 border border-slate-800/80 p-2 rounded">
          <span className="text-[10px] text-slate-400 block">Episode ID</span>
          <span className="font-bold text-indigo-300">#{worker?.episode_id ?? 492}</span>
        </div>

        <div className="bg-slate-950/70 border border-slate-800/80 p-2 rounded">
          <span className="text-[10px] text-slate-400 block">Episode Step</span>
          <span className="font-bold text-slate-200">{worker?.episode_step ?? 73}</span>
        </div>

        <div className="bg-slate-950/70 border border-slate-800/80 p-2 rounded">
          <span className="text-[10px] text-slate-400 block">Current State</span>
          <span className="font-bold text-sky-400 uppercase">{worker?.object.status ?? 'STABLE'}</span>
        </div>

        <div className="bg-slate-950/70 border border-slate-800/80 p-2 rounded">
          <span className="text-[10px] text-slate-400 block">{worker?.control?.residual_enabled ? 'Base + Residual → Target' : 'Raw → Filtered Force'}</span>
          <span className="font-bold text-purple-300">
            {worker?.control?.residual_enabled
              ? `${(worker.control.base_action?.grip_target ?? 0).toFixed(2)} + ${(worker.control.residual_action?.gripper ?? 0).toFixed(3)} → ${(worker.control.final_action?.grip_target ?? 0).toFixed(2)} N`
              : <>{worker?.last_action.force_rate_command?.toFixed(3) ?? worker?.last_action.normalized_grip_force ?? 0} → {worker?.last_action.filtered_gripper?.toFixed(3) ?? 0}</>}
          </span>
        </div>

        <div className="bg-slate-950/70 border border-slate-800/80 p-2 rounded">
          <span className="text-[10px] text-slate-400 block">Last Reward</span>
          <span className="font-bold text-emerald-400">
            +{worker?.last_reward ?? 1.40}
          </span>
        </div>

        <div className="bg-slate-950/70 border border-slate-800/80 p-2 rounded">
          <span className="text-[10px] text-slate-400 block">Outcome</span>
          <span className="font-bold text-slate-200 capitalize">{worker?.outcome ?? 'running'}</span>
        </div>

        <div className="bg-slate-950/70 border border-slate-800/80 p-2 rounded">
          <span className="text-[10px] text-slate-400 block">Object Status</span>
          <span className="font-bold text-emerald-300 capitalize">{worker?.object.status ?? 'stable'}</span>
        </div>

        <div className="bg-slate-950/70 border border-slate-800/80 p-2 rounded">
          <span className="text-[10px] text-slate-400 block">Episode Policy</span>
          <span className="font-bold text-cyan-300">v{worker?.episode_policy_version ?? runtime?.policy_version ?? 0}</span>
        </div>
      </div>
    </div>
  );
};
