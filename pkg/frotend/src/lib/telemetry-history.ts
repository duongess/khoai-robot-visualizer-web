import { ChartSample } from '../types/simulation';

export const MAX_TELEMETRY_HISTORY = 3600;

const STORAGE_KEY = 'swarmdex.telemetry-history.v1';
const STORAGE_WRITE_DELAY_MS = 1000;

let pendingHistory: ChartSample[] | null = null;
let storageWriteTimer: number | null = null;

export function appendTelemetrySample(
  history: ChartSample[],
  sample: ChartSample,
  limit = MAX_TELEMETRY_HISTORY,
): ChartSample[] {
  // total_steps is runtime-wide and remains monotonic when an episode ends.
  // A regression means the backend itself restarted, so history from the old
  // runtime must not be plotted on the new runtime's x-axis.
  const last = history[history.length - 1];
  const base = last && sample.total_steps < last.total_steps ? [] : history;
  const next = [...base, sample];
  return next.length > limit ? next.slice(next.length - limit) : next;
}

export function loadTelemetryHistory(): ChartSample[] {
  if (typeof window === 'undefined') return [];

  try {
    const stored = window.sessionStorage.getItem(STORAGE_KEY);
    if (!stored) return [];
    const parsed = JSON.parse(stored) as unknown;
    if (!Array.isArray(parsed)) return [];
    return parsed.filter(isChartSample).slice(-MAX_TELEMETRY_HISTORY);
  } catch {
    return [];
  }
}

export function saveTelemetryHistory(history: ChartSample[]): void {
  if (typeof window === 'undefined') return;

  // sessionStorage writes are synchronous. Coalesce the 10 Hz telemetry
  // stream so retaining a longer history does not stall chart rendering.
  pendingHistory = history;
  if (storageWriteTimer !== null) return;
  storageWriteTimer = window.setTimeout(flushTelemetryHistory, STORAGE_WRITE_DELAY_MS);
}

function flushTelemetryHistory(): void {
  storageWriteTimer = null;
  if (!pendingHistory || typeof window === 'undefined') return;

  try {
    window.sessionStorage.setItem(STORAGE_KEY, JSON.stringify(pendingHistory));
  } catch {
    // Telemetry must keep flowing even when storage is unavailable or full.
  } finally {
    pendingHistory = null;
  }
}

function isChartSample(value: unknown): value is ChartSample {
  if (!value || typeof value !== 'object') return false;
  const sample = value as Partial<ChartSample>;
  return [
    sample.timestamp,
    sample.episode_id,
    sample.episode_step,
    sample.total_steps,
    sample.average_reward,
    sample.success_rate,
    sample.steps_per_second,
    sample.grip_force,
    sample.required_grip_force,
    sample.break_force,
  ].every((field) => typeof field === 'number' && Number.isFinite(field));
}
