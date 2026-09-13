import { test, describe } from 'node:test';
import assert from 'node:assert';
import {
  DEFAULT_WORLD_BOUNDS,
  getCanvasTransform,
  worldToCanvas,
  canvasToWorld,
  getTerrainHeightAt,
} from '../src/components/simulation/coordinate-system';
import { DEFAULT_SCENE_CONFIG } from '../src/lib/mock-telemetry-client';
import { TerrainPoint } from '../src/types/simulation';

describe('Coordinate Conversion and Fixed Viewport', () => {
  test('worldToCanvas and canvasToWorld roundtrip preserves coordinates', () => {
    const transform = getCanvasTransform(800, 600, DEFAULT_WORLD_BOUNDS, 20);

    const worldPoint = { x: 3.25, y: 1.65 };
    const canvasPoint = worldToCanvas(worldPoint, transform);
    const convertedBack = canvasToWorld(canvasPoint, transform);

    assert.ok(
      Math.abs(convertedBack.x - worldPoint.x) < 0.001,
      `Expected X ${worldPoint.x}, got ${convertedBack.x}`
    );
    assert.ok(
      Math.abs(convertedBack.y - worldPoint.y) < 0.001,
      `Expected Y ${worldPoint.y}, got ${convertedBack.y}`
    );
  });

  test('aspect ratio is preserved and world bounds fit within canvas', () => {
    const transform = getCanvasTransform(1000, 500, DEFAULT_WORLD_BOUNDS, 20);
    assert.ok(transform.scale > 0, 'Scale must be positive');
    assert.ok(transform.offsetX >= 20, 'Left offset should respect padding');
    assert.ok(transform.offsetY >= 20, 'Top offset should respect padding');
  });
});

describe('Editable 2D Terrain Calculations', () => {
  const samplePoints: TerrainPoint[] = [
    { id: '1', x: 0.0, y: 0.3 },
    { id: '2', x: 2.0, y: 0.5 },
    { id: '3', x: 4.0, y: 0.2 },
    { id: '4', x: 6.0, y: 0.4 },
  ];

  test('getTerrainHeightAt interpolates linearly between control points', () => {
    // At x = 1.0 (midway between x=0.0, y=0.3 and x=2.0, y=0.5), y should be 0.4
    const heightAt1 = getTerrainHeightAt(1.0, samplePoints);
    assert.ok(Math.abs(heightAt1 - 0.4) < 0.001, `Expected 0.4, got ${heightAt1}`);

    // At x = 3.0 (midway between x=2.0, y=0.5 and x=4.0, y=0.2), y should be 0.35
    const heightAt3 = getTerrainHeightAt(3.0, samplePoints);
    assert.ok(Math.abs(heightAt3 - 0.35) < 0.001, `Expected 0.35, got ${heightAt3}`);

    // At boundary x = 0.0
    const heightAt0 = getTerrainHeightAt(0.0, samplePoints);
    assert.strictEqual(heightAt0, 0.3);

    // Outside boundary x > 6.0
    const heightAt7 = getTerrainHeightAt(7.0, samplePoints);
    assert.strictEqual(heightAt7, 0.4);
  });
});

describe('Gantry Overhead System Bounds', () => {
  test('gantry carriage and gripper bounds are within world bounds', () => {
    const { gantry } = DEFAULT_SCENE_CONFIG;
    assert.ok(gantry.rail_y <= DEFAULT_WORLD_BOUNDS.maxY, 'Rail must be within world Y');
    assert.ok(gantry.min_x >= DEFAULT_WORLD_BOUNDS.minX, 'Min carriage X must be within world');
    assert.ok(gantry.max_x <= DEFAULT_WORLD_BOUNDS.maxX, 'Max carriage X must be within world');
    assert.ok(gantry.carriage_x >= gantry.min_x && gantry.carriage_x <= gantry.max_x);
    assert.ok(gantry.gripper_y >= gantry.min_y && gantry.gripper_y <= gantry.max_y);
  });
});
