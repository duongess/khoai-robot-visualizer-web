import React from 'react';
import { useSimulationStore } from '../../lib/simulation-store';
import {
  Activity,
  Cpu,
  Zap,
  TrendingUp,
  Database,
  Layers,
  Radio,
  Clock,
  Award,
  Hash,
} from 'lucide-react';

export const RuntimeMetrics: React.FC = () => {
  const { latestTelemetry, runtimeStatus, selectedMode, webSocketStatus, selectedWorkerId } =
    useSimulationStore();

  const r = latestTelemetry?.runtime;
  const w = latestTelemetry?.worker;

  const metrics = [
    {
      id: 'metric-runtime-status',
      label: 'Runtime Status',
      value: (r?.status ?? runtimeStatus).toUpperCase(),
      subtext: 'Stepping State',
      icon: Activity,
      color:
        runtimeStatus === 'running'
          ? 'text-emerald-400'
          : runtimeStatus === 'paused'
          ? 'text-amber-400'
          : 'text-slate-400',
    },
    {
      id: 'metric-training-mode',
      label: 'Training Mode',
      value: (r?.mode ?? selectedMode).toUpperCase(),
      subtext: selectedMode === 'swarm' ? 'Shared SAC Learner' : 'Isolated Replay',
      icon: Layers,
      color: 'text-cyan-400',
    },
    {
      id: 'metric-active-workers',
      label: 'Active Workers',
      value: r?.active_workers ?? 100,
      subtext: 'Concurrent threads',
      icon: Cpu,
      color: 'text-sky-400',
    },
    {
      id: 'metric-steps-sec',
      label: 'Env Steps / sec',
      value: (r?.steps_per_second ?? 32421).toLocaleString(),
      subtext: 'High-throughput physics',
      icon: Zap,
      color: 'text-amber-400',
    },
    {
      id: 'metric-episodes-sec',
      label: 'Episodes / sec',
      value: (r?.episodes_per_second ?? 124).toLocaleString(),
      subtext: 'Completed runs',
      icon: Clock,
      color: 'text-slate-300',
    },
    {
      id: 'metric-total-steps',
      label: 'Total Env Steps',
      value: (r?.total_steps ?? 840210).toLocaleString(),
      subtext: 'Total transitions',
      icon: Hash,
      color: 'text-slate-200',
    },
    {
      id: 'metric-current-episode',
      label: 'Current Episode',
      value: `#${w?.episode_id ?? 492}`,
      subtext: `Step ${w?.episode_step ?? 73}`,
      icon: Hash,
      color: 'text-indigo-400',
    },
    {
      id: 'metric-success-rate',
      label: 'Success Rate',
      value: `${((r?.success_rate ?? 0.927) * 100).toFixed(1)}%`,
      subtext: 'Rolling 1,000 ep',
      icon: Award,
      color: 'text-emerald-400',
    },
    {
      id: 'metric-avg-reward',
      label: 'Average Reward',
      value: (r?.average_reward ?? 14.83).toFixed(2),
      subtext: 'Cumulative discount',
      icon: TrendingUp,
      color: 'text-emerald-400',
    },
    {
      id: 'metric-replay-buffer',
      label: 'Replay Buffer',
      value: (r?.replay_buffer_size ?? 100000).toLocaleString(),
      subtext: 'Transitions queued',
      icon: Database,
      color: 'text-purple-400',
    },
    {
      id: 'metric-training-batches',
      label: 'Training Batches',
      value: (r?.training_batches ?? 3282).toLocaleString(),
      subtext: 'Gradient updates',
      icon: Zap,
      color: 'text-blue-400',
    },
    {
      id: 'metric-policy-version',
      label: 'Policy Version',
      value: `v${r?.policy_version ?? 184}`,
      subtext: 'SAC checkpoint',
      icon: Layers,
      color: 'text-cyan-300',
    },
    {
      id: 'metric-current-worker',
      label: 'Selected Worker',
      value: `Worker #${selectedWorkerId}`,
      subtext: `Step: ${w?.episode_step ?? 0}`,
      icon: Cpu,
      color: 'text-sky-300',
    },
    {
      id: 'metric-ws-status',
      label: 'Telemetry Stream',
      value: webSocketStatus === 'mock' ? 'MOCK TELEMETRY' : webSocketStatus.toUpperCase(),
      subtext: 'Live 16Hz pipe',
      icon: Radio,
      color: 'text-emerald-400',
    },
  ];

  return (
    <div id="runtime-metrics-panel" className="w-full">
      <div className="flex items-center justify-between mb-2">
        <h3 className="text-xs font-semibold text-slate-400 uppercase tracking-wider font-mono">
          Runtime Telemetry Metrics
        </h3>
        <span className="text-[11px] font-mono text-slate-500">14 Telemetry Vectors</span>
      </div>

      <div className="grid grid-cols-2 sm:grid-cols-4 md:grid-cols-7 gap-2">
        {metrics.map((m) => {
          const Icon = m.icon;
          return (
            <div
              key={m.id}
              id={m.id}
              className="bg-slate-900 border border-slate-800/90 rounded-md p-2.5 flex flex-col justify-between hover:border-slate-700 transition-colors"
            >
              <div className="flex items-center justify-between text-slate-400 text-[10px] font-mono mb-1">
                <span className="truncate">{m.label}</span>
                <Icon className="w-3 h-3 text-slate-500 shrink-0 ml-1" />
              </div>
              <div className={`text-sm font-bold font-mono truncate ${m.color}`}>
                {m.value}
              </div>
              <div className="text-[9px] font-mono text-slate-500 truncate mt-1">
                {m.subtext}
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
};
