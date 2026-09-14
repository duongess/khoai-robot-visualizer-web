export type RuntimeStatus = 'stopped' | 'running' | 'paused' | 'resetting' | 'error';
export type SimulationMode = 'independent' | 'swarm';

export type ObjectStatus =
  | 'idle'
  | 'targeted'
  | 'grasping'
  | 'grasped'
  | 'lifting'
  | 'carrying'
  | 'releasing'
  | 'falling'
  | 'placed'
  | 'slipping'
  | 'broken';

export type TaskPhase =
  | 'approach_horizontal'
  | 'lower'
  | 'open'
  | 'close'
  | 'grip'
  | 'lift'
  | 'carry'
  | 'lower_target'
  | 'release'
  | 'retract';

export interface WorldBounds {
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
}

export interface TerrainPoint {
  id: string;
  x: number;
  y: number;
}

export interface ObjectConfig {
  id?: string;
  position_x: number;
  position_y: number;
  mass: number;
  friction: number;
  break_force: number;
  initial_vertical_velocity: number;
  width: number;
  height: number;
}

export interface GantryConfig {
  rail_y: number;
  carriage_x: number;
  gripper_y: number;
  min_x: number;
  max_x: number;
  min_y: number;
  max_y: number;
  initial_grip_force: number;
  minimum_grip_force: number;
  maximum_grip_force: number;
}

export interface TargetAreaConfig {
  position_x: number;
  width: number;
}

export interface TerrainConfig {
  points: TerrainPoint[];
  ground_friction: number;
  gravity: number;
  slope?: number;
}

export interface SceneConfig {
  object: ObjectConfig;
  gantry: GantryConfig;
  terrain: TerrainConfig;
  target: TargetAreaConfig;
  // Deprecated robot alias for legacy test compatibility
  robot?: {
    base_position_x: number;
    base_position_y: number;
    shoulder_angle: number;
    elbow_angle: number;
    initial_grip_force: number;
    minimum_grip_force: number;
    maximum_grip_force: number;
  };
}

export interface RuntimeMetrics {
  status: RuntimeStatus;
  mode: SimulationMode;
  active_workers: number;
  steps_per_second: number;
  episodes_per_second: number;
  total_steps: number;
  success_rate: number;
  average_reward: number;
  replay_buffer_size: number;
  training_batches: number;
  policy_version: number;
}

export interface GantryState {
  carriage_x: number;
  gripper_y: number;
  grip_force: number;
  jaw_opening: number; // 0 = closed, 1 = fully open
  rail_y: number;
}

// Kept for backward compatibility if any legacy component imports it
export interface RobotState {
  base_position: {
    x: number;
    y: number;
  };
  shoulder_angle: number;
  elbow_angle: number;
  gripper_position: {
    x: number;
    y: number;
  };
  grip_force: number;
}

export interface ObjectState {
  id?: string;
  position: {
    x: number;
    y: number;
  };
  velocity: {
    x: number;
    y: number;
  };
  mass: number;
  friction: number;
  break_force: number;
  required_grip_force: number;
  safety_margin: number;
  status: ObjectStatus;
}

export interface WorkerState {
  id: number;
  episode_id: number;
  episode_step: number;
  task_phase: TaskPhase;
  gantry: GantryState;
  robot?: RobotState; // compatibility shim
  object: ObjectState;
  target: {
    position_x: number;
    width: number;
  };
  terrain: {
    points: TerrainPoint[];
  };
  last_action: {
    normalized_grip_force: number;
  };
  last_reward: number;
  done: boolean;
  outcome: string;
}

export interface SimulationSnapshot {
  type: 'simulation_snapshot';
  timestamp: string;
  runtime: RuntimeMetrics;
  worker: WorkerState;
}

export interface ChartSample {
  timestamp: number;
  step: number;
  total_steps: number;
  average_reward: number;
  success_rate: number;
  steps_per_second: number;
  grip_force: number;
  required_grip_force: number;
  break_force: number;
}

export interface BenchmarkMetrics {
  success_rate: number;
  average_reward: number;
  samples_to_threshold: number;
  steps_per_second: number;
}

export interface ComparisonData {
  independent: BenchmarkMetrics | null;
  swarm: BenchmarkMetrics | null;
}

// Scene Update API Payload (PUT /api/scene)
export interface SceneUpdatePayload {
  object?: {
    id?: string;
    position: {
      x: number;
      y: number;
    };
    mass?: number;
    friction?: number;
    break_force?: number;
    width?: number;
    height?: number;
  };
  gantry?: {
    carriage_x: number;
    gripper_y: number;
  };
  target?: {
    position: {
      x: number;
      y?: number;
    };
  };
  terrain?: {
    points: TerrainPoint[];
  };
}
