export type RuntimeStatus = 'stopped' | 'running' | 'paused' | 'stalled' | 'resetting' | 'error';
export type SimulationMode = 'independent' | 'swarm';

export type ObjectStatus =
  | 'idle'
  | 'targeted'
  | 'grasping'
	| 'attached'
	| 'transported'
	| 'released'
  | 'grasped'
  | 'lifting'
  | 'carrying'
  | 'releasing'
  | 'falling'
  | 'placed'
  | 'slipping'
  | 'broken';

export type TaskPhase =
  | 'idle'
  | 'approach_object'
  | 'lower_to_object'
  | 'grip_object'
  | 'lift_object'
  | 'move_to_target'
  | 'lower_at_target'
  | 'release_object'
  | 'success'
  | 'failure'
  // Legacy mock-only values retained until the mock telemetry fixture is removed.
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
	workspace?: WorldBounds & { coordinate_system_version?: number };
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
  last_error?: string;
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
	gripper_state?: 'open' | 'closed';
	contact_state?: 'none' | 'contact';
	force_valid?: boolean;
  gantry: GantryState;
  robot?: RobotState; // compatibility shim
  object: ObjectState;
  target: {
    position_x: number;
    position_y?: number;
    width: number;
  };
  workspace?: WorldBounds & {
    safeMinX: number;
    safeMaxX: number;
    safeMinY: number;
    safeMaxY: number;
    boundaryHit: boolean;
    coordinate_system_version: number;
  };
  terrain: {
    points: TerrainPoint[];
  };
  last_action: {
    normalized_grip_force: number;
		filtered_horizontal?: number;
		filtered_vertical?: number;
		filtered_gripper?: number;
    force_rate_command?: number;
    force_rate_newtons_per_second?: number;
    force_action_mode?: 'increase' | 'hold' | 'decrease' | 'release';
  };
  last_reward: number;
  cumulative_reward?: number;
  distance_to_object?: number;
  distance_to_target?: number;
  done: boolean;
  outcome: string;
  latest_vertical_action?: number;
	episode_policy_version?: number;
	control?: {
		dt: number;
		carriage_x: number;
		gripper_y: number;
		velocity_x: number;
		velocity_y: number;
		error_x: number;
		error_y: number;
		vertical_acceleration?: number;
		invalid_contact_frames?: number;
		slip_severity?: number;
		slip_frames?: number;
	};
  velocity_y?: number;
  target_grasp_y?: number;
  vertical_error?: number;
	contact_detected?: boolean;
	object_attached?: boolean;
	object_stable?: boolean;
	slipping?: boolean;
	approach_reward?: number;
	grip_reward?: number;
	delivery_reward?: number;
	success_reward?: number;
	penalty_reward?: number;
	total_step_reward?: number;
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
