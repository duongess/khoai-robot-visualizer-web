import { SceneConfig } from '../types/simulation';

export const DEFAULT_SCENE_CONFIG: SceneConfig = {
  object: { id: 'object-1', position_x: 1.5, position_y: 0.425, mass: 0.8, friction: 0.35, break_force: 18, initial_vertical_velocity: 0, width: 0.35, height: 0.25 },
  gantry: { rail_y: 3.5, carriage_x: 1.5, gripper_y: 2.8, min_x: 0, max_x: 6, min_y: 0, max_y: 3.5, initial_grip_force: 0, minimum_grip_force: 0, maximum_grip_force: 20 },
  terrain: { points: [{ id: 'terrain-1', x: 0, y: 0.3 }, { id: 'terrain-2', x: 1.5, y: 0.3 }, { id: 'terrain-3', x: 3, y: 0.5 }, { id: 'terrain-4', x: 4.5, y: 0.25 }, { id: 'terrain-5', x: 6, y: 0.25 }], ground_friction: 0.35, gravity: 9.81 },
  target: { position_x: 4.5, width: 0.8 },
};
