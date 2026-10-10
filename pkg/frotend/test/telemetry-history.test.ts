import { describe, test } from 'node:test';
import assert from 'node:assert';
import { appendTelemetrySample } from '../src/lib/telemetry-history';
import { ChartSample } from '../src/types/simulation';

function sample(episodeId: number, episodeStep: number, totalSteps: number): ChartSample {
  return {
    timestamp: totalSteps,
    episode_id: episodeId,
    episode_step: episodeStep,
    total_steps: totalSteps,
    average_reward: totalSteps / 10,
    success_rate: 0.5,
    steps_per_second: 100,
    grip_force: 5,
    required_grip_force: 4,
    break_force: 12,
  };
}

describe('telemetry chart history', () => {
  test('keeps prior samples when a new episode starts at step zero', () => {
    let history: ChartSample[] = [];
    history = appendTelemetrySample(history, sample(7, 98, 1098));
    history = appendTelemetrySample(history, sample(7, 99, 1099));
    history = appendTelemetrySample(history, sample(8, 0, 1100));

    assert.deepStrictEqual(history.map((item) => item.episode_id), [7, 7, 8]);
    assert.deepStrictEqual(history.map((item) => item.total_steps), [1098, 1099, 1100]);
  });

  test('starts a fresh history only when the runtime-wide step counter regresses', () => {
    const previous = [sample(7, 99, 1099), sample(8, 0, 1100)];
    const history = appendTelemetrySample(previous, sample(1, 0, 0));

    assert.deepStrictEqual(history, [sample(1, 0, 0)]);
  });

  test('keeps the configured number of most recent samples', () => {
    let history: ChartSample[] = [];
    for (let step = 0; step < 5; step += 1) {
      history = appendTelemetrySample(history, sample(1, step, step), 3);
    }

    assert.deepStrictEqual(history.map((item) => item.total_steps), [2, 3, 4]);
  });
});
