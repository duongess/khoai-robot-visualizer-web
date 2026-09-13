import React from 'react';
import { useSimulationStore } from '../../lib/simulation-store';
import { ShieldCheck, AlertTriangle, AlertOctagon, Layers, Cpu, Zap, Activity } from 'lucide-react';
import { TaskPhase } from '../../types/simulation';

const TASK_PHASE_LABELS: Record<TaskPhase, { step: number; label: string; desc: string }> = {
  approach_horizontal: { step: 1, label: 'Move Above Object', desc: 'Carriage aligning over package' },
  lower: { step: 2, label: 'Lower Gripper', desc: 'Lowering shaft toward object' },
  open: { step: 3, label: 'Open Gripper', desc: 'Expanding jaws for capture' },
  close: { step: 4, label: 'Close Around Object', desc: 'Contracting jaws on sides' },
  grip: { step: 5, label: 'Apply Grip Force', desc: 'Reaching required normal force' },
  lift: { step: 6, label: 'Lift Object', desc: 'Vertical shaft raising payload' },
  carry: { step: 7, label: 'Move to Target', desc: 'Carriage traversing horizontally' },
  lower_target: { step: 8, label: 'Lower to Target', desc: 'Lowering payload onto target zone' },
  release: { step: 9, label: 'Release Object', desc: 'Opening jaws, dropping payload' },
  retract: { step: 10, label: 'Retract Gripper', desc: 'Raising empty claw to travel height' },
};

export const PhysicsMetrics: React.FC = () => {
  const {
    latestTelemetry,
    draftConfig,
    runtimeStatus,
    selectedMode,
    selectedWorkerId,
  } = useSimulationStore();

  const worker = latestTelemetry?.worker;
  const runtime = latestTelemetry?.runtime;
  const obj = worker?.object;
  const gantry = worker?.gantry;

  // Active or draft values
  const carriageX = gantry?.carriage_x ?? draftConfig.gantry.carriage_x;
  const gripperY = gantry?.gripper_y ?? draftConfig.gantry.gripper_y;
  const currentGripForce = gantry?.grip_force ?? draftConfig.gantry.initial_grip_force;
  const requiredGripForce = obj?.required_grip_force ?? 7.15;
  const breakForce = obj?.break_force ?? draftConfig.object.break_force;
  const safetyMargin = currentGripForce - requiredGripForce;
  const mass = obj?.mass ?? draftConfig.object.mass;
  const friction = obj?.friction ?? draftConfig.object.friction;
  const objectStatus = obj?.status ?? 'idle';
  const taskPhase: TaskPhase = worker?.task_phase ?? 'approach_horizontal';
  const phaseInfo = TASK_PHASE_LABELS[taskPhase] || { step: 1, label: taskPhase, desc: '' };

  // Runtime Metrics
  const currentReward = worker?.last_reward ?? 1.45;
  const successRate = runtime?.success_rate ?? (selectedMode === 'swarm' ? 0.942 : 0.748);
  const policyVersion = runtime?.policy_version ?? 184;
  const stepsPerSec = runtime?.steps_per_second ?? (runtimeStatus === 'running' ? (selectedMode === 'swarm' ? 32420 : 4120) : 0);

  // Safety state evaluation according to Section 5 rules
  let forceState: 'stable' | 'slipping' | 'break_risk' = 'stable';
  if (currentGripForce < requiredGripForce) {
    forceState = 'slipping';
  } else if (currentGripForce >= breakForce) {
    forceState = 'break_risk';
  } else {
    forceState = 'stable';
  }

  // Force spectrum gauge calculations
  const maxSpectrum = breakForce * 1.25;
  const gaugePercent = Math.min(100, Math.max(0, (currentGripForce / maxSpectrum) * 100));
  const reqPercent = Math.min(100, (requiredGripForce / maxSpectrum) * 100);
  const breakPercent = Math.min(100, (breakForce / maxSpectrum) * 100);

  return (
    <div
      id="side-information-panel"
      className="bg-slate-900 border border-slate-800 rounded-lg p-4 space-y-4 shadow-sm flex flex-col justify-between text-xs"
    >
      {/* 1. Task & Executive Status */}
      <div className="space-y-2.5">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-1.5">
            <Activity className="w-3.5 h-3.5 text-cyan-400" />
            <h3 className="font-semibold text-slate-200 uppercase tracking-wider font-mono text-[11px]">
              Task Telemetry
            </h3>
          </div>

          <div className="flex items-center gap-2">
            <span className="px-2 py-0.5 rounded text-[10px] font-mono bg-slate-950 border border-slate-800 text-slate-300">
              Worker #{selectedWorkerId}
            </span>
            <span className="px-2 py-0.5 rounded text-[10px] font-mono uppercase bg-cyan-500/10 border border-cyan-500/30 text-cyan-400 font-semibold">
              {selectedMode}
            </span>
          </div>
        </div>

        {/* Current Task Phase Banner */}
        <div
          id="current-task-phase-box"
          className="bg-slate-950 border border-slate-800/80 rounded-md p-2.5 space-y-1"
        >
          <div className="flex items-center justify-between text-[11px]">
            <span className="text-slate-400 font-mono">Phase {phaseInfo.step}/10:</span>
            <span className="font-mono font-semibold text-cyan-300">{phaseInfo.label}</span>
          </div>
          <p className="text-[10px] text-slate-400 leading-tight font-mono">{phaseInfo.desc}</p>
        </div>

        {/* Object & Gantry State Grid */}
        <div className="grid grid-cols-2 gap-2 font-mono text-[11px]">
          <div className="bg-slate-950/60 border border-slate-800/80 p-2 rounded">
            <div className="text-[10px] text-slate-400">Object State</div>
            <div className="font-bold text-slate-100 uppercase tracking-wide mt-0.5">
              {objectStatus}
            </div>
          </div>

          <div className="bg-slate-950/60 border border-slate-800/80 p-2 rounded">
            <div className="text-[10px] text-slate-400">Simulation Status</div>
            <div className="font-bold text-cyan-400 uppercase tracking-wide mt-0.5">
              {runtimeStatus}
            </div>
          </div>

          <div className="bg-slate-950/60 border border-slate-800/80 p-2 rounded">
            <div className="text-[10px] text-slate-400">Carriage X</div>
            <div className="font-bold text-slate-200 mt-0.5">{carriageX.toFixed(2)} m</div>
          </div>

          <div className="bg-slate-950/60 border border-slate-800/80 p-2 rounded">
            <div className="text-[10px] text-slate-400">Gripper Height Y</div>
            <div className="font-bold text-slate-200 mt-0.5">{gripperY.toFixed(2)} m</div>
          </div>
        </div>
      </div>

      {/* 2. Force Control Information & Formula */}
      <div className="space-y-2.5 pt-2 border-t border-slate-800/80">
        <div className="flex items-center justify-between">
          <h3 className="font-semibold text-slate-200 uppercase tracking-wider font-mono text-[11px] flex items-center gap-1.5">
            <span>Force-Control Physics</span>
          </h3>

          {/* Grip Safety Badge */}
          {forceState === 'stable' && (
            <span className="flex items-center gap-1 px-2 py-0.5 rounded-full text-[10px] font-mono font-medium bg-emerald-500/10 border border-emerald-500/30 text-emerald-400">
              <ShieldCheck className="w-3 h-3" />
              Stable Grip
            </span>
          )}
          {forceState === 'slipping' && (
            <span className="flex items-center gap-1 px-2 py-0.5 rounded-full text-[10px] font-mono font-medium bg-orange-500/10 border border-orange-500/30 text-orange-400">
              <AlertTriangle className="w-3 h-3" />
              Slipping
            </span>
          )}
          {forceState === 'break_risk' && (
            <span className="flex items-center gap-1 px-2 py-0.5 rounded-full text-[10px] font-mono font-medium bg-red-500/10 border border-red-500/30 text-red-400">
              <AlertOctagon className="w-3 h-3" />
              Break Risk!
            </span>
          )}
        </div>

        {/* Formula Box from Section 9 */}
        <div className="bg-slate-950 border border-slate-800 rounded-md p-2.5 font-mono text-[11px] space-y-1">
          <div className="text-cyan-400 font-semibold">
            F_required = m(g + a) / (2μ)
          </div>
          <div className="text-slate-400 text-[10px]">
            Condition: <span className="text-emerald-400 font-semibold">F_req ≤ F_grip &lt; F_break</span>
          </div>
        </div>

        {/* Force Spectrum Bar */}
        <div>
          <div className="flex justify-between text-[10px] font-mono text-slate-400 mb-1">
            <span>Grip Force Spectrum</span>
            <span className="text-slate-100 font-semibold">{currentGripForce.toFixed(2)} N</span>
          </div>
          <div className="relative h-2.5 bg-slate-950 rounded-full overflow-hidden border border-slate-800">
            {/* Slipping Zone */}
            <div
              className="absolute left-0 top-0 bottom-0 bg-orange-500/20"
              style={{ width: `${reqPercent}%` }}
              title="Slipping Zone (F < F_req)"
            />
            {/* Safe Grip Zone */}
            <div
              className="absolute top-0 bottom-0 bg-emerald-500/25 border-x border-emerald-500/40"
              style={{ left: `${reqPercent}%`, width: `${Math.max(0, breakPercent - reqPercent)}%` }}
              title="Safe Grip Zone (F_req ≤ F < F_break)"
            />
            {/* Break Zone */}
            <div
              className="absolute top-0 bottom-0 right-0 bg-red-500/30"
              style={{ left: `${breakPercent}%` }}
              title="Crush / Break Zone (F ≥ F_break)"
            />
            {/* Grip Force Needle */}
            <div
              className={`absolute top-0 bottom-0 w-1.5 rounded-xs transition-all duration-100 ${
                forceState === 'stable' ? 'bg-emerald-400' : forceState === 'slipping' ? 'bg-orange-400' : 'bg-red-500'
              }`}
              style={{ left: `calc(${gaugePercent}% - 3px)` }}
            />
          </div>
          <div className="flex justify-between text-[9px] font-mono text-slate-400 mt-1">
            <span>0N (Slip)</span>
            <span className="text-emerald-400">Req: {requiredGripForce.toFixed(1)}N</span>
            <span className="text-red-400">Break: {breakForce.toFixed(1)}N</span>
          </div>
        </div>

        {/* Force & Physical Parameters Grid */}
        <div className="grid grid-cols-2 gap-2 font-mono text-[11px]">
          <div className="bg-slate-950/60 border border-slate-800/80 p-2 rounded">
            <div className="text-[10px] text-slate-400">Current Grip</div>
            <div className="text-sm font-bold text-slate-100 mt-0.5">{currentGripForce.toFixed(2)} N</div>
          </div>

          <div className="bg-slate-950/60 border border-slate-800/80 p-2 rounded">
            <div className="text-[10px] text-slate-400">Required Grip</div>
            <div className="text-sm font-bold text-cyan-400 mt-0.5">{requiredGripForce.toFixed(2)} N</div>
          </div>

          <div className="bg-slate-950/60 border border-slate-800/80 p-2 rounded">
            <div className="text-[10px] text-slate-400">Break Force</div>
            <div className="text-sm font-bold text-red-400 mt-0.5">{breakForce.toFixed(1)} N</div>
          </div>

          <div className="bg-slate-950/60 border border-slate-800/80 p-2 rounded">
            <div className="text-[10px] text-slate-400">Safety Margin</div>
            <div
              className={`text-sm font-bold mt-0.5 ${
                safetyMargin >= 0 ? 'text-emerald-400' : 'text-orange-400'
              }`}
            >
              {safetyMargin >= 0 ? `+${safetyMargin.toFixed(2)}` : safetyMargin.toFixed(2)} N
            </div>
          </div>

          <div className="bg-slate-950/60 border border-slate-800/80 p-2 rounded">
            <div className="text-[10px] text-slate-400">Object Mass</div>
            <div className="font-semibold text-slate-200 mt-0.5">{mass.toFixed(2)} kg</div>
          </div>

          <div className="bg-slate-950/60 border border-slate-800/80 p-2 rounded">
            <div className="text-[10px] text-slate-400">Friction Coeff (μ)</div>
            <div className="font-semibold text-slate-200 mt-0.5">{friction.toFixed(2)}</div>
          </div>
        </div>
      </div>

      {/* 3. Training Performance Metrics */}
      <div className="space-y-2 pt-2 border-t border-slate-800/80">
        <h3 className="font-semibold text-slate-400 uppercase tracking-wider font-mono text-[10px] flex items-center gap-1">
          <Zap className="w-3 h-3 text-amber-400" />
          <span>Worker & Policy Learning</span>
        </h3>

        <div className="grid grid-cols-2 gap-2 font-mono text-[11px]">
          <div className="bg-slate-950/60 border border-slate-800/80 p-2 rounded">
            <div className="text-[10px] text-slate-400">Current Reward</div>
            <div className="font-bold text-emerald-400 mt-0.5">{currentReward.toFixed(2)}</div>
          </div>

          <div className="bg-slate-950/60 border border-slate-800/80 p-2 rounded">
            <div className="text-[10px] text-slate-400">Success Rate</div>
            <div className="font-bold text-cyan-400 mt-0.5">{(successRate * 100).toFixed(1)}%</div>
          </div>

          <div className="bg-slate-950/60 border border-slate-800/80 p-2 rounded">
            <div className="text-[10px] text-slate-400">Policy Version</div>
            <div className="font-semibold text-slate-200 mt-0.5">v{policyVersion}</div>
          </div>

          <div className="bg-slate-950/60 border border-slate-800/80 p-2 rounded">
            <div className="text-[10px] text-slate-400">Steps / Sec</div>
            <div className="font-semibold text-slate-200 mt-0.5">{stepsPerSec.toLocaleString()}</div>
          </div>
        </div>
      </div>
    </div>
  );
};
