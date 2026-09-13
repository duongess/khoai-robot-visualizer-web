import { test, describe } from 'node:test';
import assert from 'node:assert';
import { DEFAULT_SCENE_CONFIG } from '../src/lib/mock-telemetry-client';

describe('Force-Control Physics Formulas and Status Rules', () => {
  // Formula: F_required = m(g + a) / (2μ)
  const calculateRequiredForce = (m: number, g: number, a: number, mu: number) => {
    return (m * (g + a)) / (2 * Math.max(0.001, mu));
  };

  const evaluateGripStatus = (currentForce: number, reqForce: number, breakForce: number) => {
    if (currentForce < reqForce) return 'slipping';
    if (currentForce >= breakForce) return 'broken';
    return 'stable';
  };

  test('calculates required grip force correctly', () => {
    const m = 0.8; // kg
    const g = 9.81; // m/s^2
    const a = 0.15; // m/s^2
    const mu = 0.35; // friction

    const fReq = calculateRequiredForce(m, g, a, mu);
    // (0.8 * 9.96) / 0.7 = 7.968 / 0.7 ≈ 11.3828
    assert.ok(Math.abs(fReq - 11.383) < 0.05, `Expected ~11.38N, got ${fReq}`);
  });

  test('classifies stable grip when F_req <= F_grip < F_break', () => {
    const reqForce = 6.73;
    const breakForce = 12.0;
    const gripForce = 8.5;

    const status = evaluateGripStatus(gripForce, reqForce, breakForce);
    assert.strictEqual(status, 'stable');
  });

  test('classifies slipping warning when F_grip < F_req', () => {
    const reqForce = 6.73;
    const breakForce = 12.0;
    const gripForce = 4.2;

    const status = evaluateGripStatus(gripForce, reqForce, breakForce);
    assert.strictEqual(status, 'slipping');
  });

  test('classifies broken warning when F_grip >= F_break', () => {
    const reqForce = 6.73;
    const breakForce = 12.0;
    const gripForce = 12.5;

    const status = evaluateGripStatus(gripForce, reqForce, breakForce);
    assert.strictEqual(status, 'broken');
  });
});

describe('Scene Configuration Defaults & Validation', () => {
  test('default scene configuration satisfies all boundary conditions', () => {
    const { object, robot, terrain } = DEFAULT_SCENE_CONFIG;

    assert.ok(object.mass > 0, 'Object mass must be positive');
    assert.ok(object.friction > 0 && object.friction <= 2.0, 'Friction must be valid');
    assert.ok(object.break_force > 0, 'Break force must be positive');
    assert.ok(robot.maximum_grip_force > robot.minimum_grip_force, 'Max grip force must exceed min');
    assert.ok(terrain.gravity > 0, 'Gravity must be positive');
  });
});
