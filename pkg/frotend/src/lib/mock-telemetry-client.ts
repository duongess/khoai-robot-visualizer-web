import {
  SceneConfig,
  SimulationSnapshot,
  RuntimeStatus,
  SimulationMode,
  ObjectStatus,
  TaskPhase,
  WorkerState,
  SceneUpdatePayload,
} from '../types/simulation';
import { getTerrainHeightAt } from '../components/simulation/coordinate-system';

export const DEFAULT_SCENE_CONFIG: SceneConfig = {
  object: {
    id: 'object-1',
    position_x: 1.5,
    position_y: 0.45,
    mass: 0.8,
    friction: 0.35,
    break_force: 12.0,
    initial_vertical_velocity: 0.0,
    width: 0.35,
    height: 0.25,
  },
  gantry: {
    rail_y: 2.85,
    carriage_x: 1.5,
    gripper_y: 1.8,
    min_x: 0.5,
    max_x: 5.5,
    min_y: 0.4,
    max_y: 2.65,
    initial_grip_force: 7.82,
    minimum_grip_force: 0.0,
    maximum_grip_force: 20.0,
  },
  terrain: {
    points: [
      { id: 't-1', x: 0.0, y: 0.35 },
      { id: 't-2', x: 1.5, y: 0.35 },
      { id: 't-3', x: 3.0, y: 0.55 },
      { id: 't-4', x: 4.5, y: 0.25 },
      { id: 't-5', x: 6.0, y: 0.25 },
    ],
    ground_friction: 0.6,
    gravity: 9.81,
    slope: 0.0,
  },
  target: {
    position_x: 4.5,
    width: 0.8,
  },
  robot: {
    base_position_x: 0.0,
    base_position_y: 0.0,
    shoulder_angle: 0.42,
    elbow_angle: 1.17,
    initial_grip_force: 7.82,
    minimum_grip_force: 0.0,
    maximum_grip_force: 20.0,
  },
};

type SnapshotListener = (snapshot: SimulationSnapshot) => void;

interface WorkerSimulationData {
  episodeId: number;
  step: number;
  phase: TaskPhase;
  phaseTimer: number;
  carriageX: number;
  gripperY: number;
  jawOpening: number; // 0 = closed, 1 = open
  gripForce: number;
  objX: number;
  objY: number;
  objVx: number;
  objVy: number;
  status: ObjectStatus;
  lastReward: number;
  done: boolean;
}

class MockTelemetryClient {
  private status: RuntimeStatus = 'running';
  private mode: SimulationMode = 'swarm';
  private selectedWorkerId: number = 1;
  private totalWorkers: number = 8;
  private config: SceneConfig = JSON.parse(JSON.stringify(DEFAULT_SCENE_CONFIG));
  private listeners: Set<SnapshotListener> = new Set();
  private timer: ReturnType<typeof setInterval> | null = null;

  // Simulation counters
  private totalSteps: number = 84210;
  private trainingBatches: number = 3280;
  private policyVersion: number = 184;

  private workers: Map<number, WorkerSimulationData> = new Map();

  constructor() {
    this.initWorkers();
    this.startLoop();
  }

  private initWorkers() {
    for (let i = 1; i <= this.totalWorkers; i++) {
      const terrainHeightAtObj = getTerrainHeightAt(this.config.object.position_x, this.config.terrain.points);
      const initialObjY = terrainHeightAtObj + this.config.object.height / 2;

      this.workers.set(i, {
        episodeId: 100 + i * 15,
        step: (i * 12) % 110,
        phase: 'approach_horizontal',
        phaseTimer: 0,
        carriageX: this.config.gantry.carriage_x,
        gripperY: this.config.gantry.gripper_y,
        jawOpening: 0.8,
        gripForce: 0.0,
        objX: this.config.object.position_x,
        objY: initialObjY,
        objVx: 0,
        objVy: 0,
        status: 'idle',
        lastReward: 1.25,
        done: false,
      });
    }
  }

  private startLoop() {
    if (this.timer !== null) return;
    this.timer = setInterval(() => {
      this.step();
    }, 60); // ~16Hz telemetry updates
    if (this.timer && typeof this.timer === 'object' && 'unref' in this.timer) {
      (this.timer as any).unref();
    }
  }

  private stopLoop() {
    if (this.timer !== null) {
      clearInterval(this.timer);
      this.timer = null;
    }
  }

  private step() {
    if (this.status === 'running') {
      this.totalSteps += this.mode === 'swarm' ? 32 : 8;
      if (this.totalSteps % 100 === 0) {
        this.trainingBatches += 1;
        if (this.trainingBatches % 50 === 0) {
          this.policyVersion += 1;
        }
      }

      // Step each worker through the 11-step pick-and-place sequence
      this.workers.forEach((w) => {
        w.step += 1;
        w.phaseTimer += 0.05;

        const objX = this.config.object.position_x;
        const targetX = this.config.target.position_x;
        const targetTerrainY = getTerrainHeightAt(targetX, this.config.terrain.points);
        const objTerrainY = getTerrainHeightAt(objX, this.config.terrain.points);

        const m = this.config.object.mass;
        const g = this.config.terrain.gravity;
        const mu = this.config.object.friction;
        const accel = 0.15;
        const requiredForce = (m * (g + accel)) / (2 * Math.max(0.01, mu));
        const breakForce = this.config.object.break_force;

        const travelY = 2.3; // safe high travel clearance
        const graspY = objTerrainY + this.config.object.height + 0.12;
        const releaseY = targetTerrainY + this.config.object.height + 0.12;

        // Sequence State Machine:
        // 1. approach_horizontal: move carriage above object at travelY
        // 2. lower: lower gripper to graspY with jaws open
        // 3. open: ensure jaws open
        // 4. close: close jaws around object
        // 5. grip: apply grip force
        // 6. lift: lift gripper with object back to travelY
        // 7. carry: move carriage horizontally to targetX
        // 8. lower_target: lower gripper to releaseY
        // 9. release: open jaws & release force
        // 10. placed: object rests on target
        // 11. retract: lift empty gripper back up to travelY

        switch (w.phase) {
          case 'approach_horizontal': {
            w.status = 'targeted';
            w.gripperY = travelY;
            w.jawOpening = 0.85;
            w.gripForce = 0.0;
            const dx = objX - w.carriageX;
            if (Math.abs(dx) > 0.05) {
              w.carriageX += Math.sign(dx) * 0.08;
            } else {
              w.carriageX = objX;
              w.phase = 'lower';
              w.phaseTimer = 0;
            }
            break;
          }

          case 'lower': {
            w.status = 'grasping';
            w.carriageX = objX;
            w.jawOpening = 0.85;
            if (w.gripperY > graspY) {
              w.gripperY = Math.max(graspY, w.gripperY - 0.07);
            } else {
              w.gripperY = graspY;
              w.phase = 'open';
              w.phaseTimer = 0;
            }
            break;
          }

          case 'open': {
            w.status = 'grasping';
            w.jawOpening = 1.0;
            if (w.phaseTimer >= 0.2) {
              w.phase = 'close';
              w.phaseTimer = 0;
            }
            break;
          }

          case 'close': {
            w.status = 'grasping';
            w.jawOpening = Math.max(0.15, w.jawOpening - 0.15);
            if (w.jawOpening <= 0.18) {
              w.phase = 'grip';
              w.phaseTimer = 0;
            }
            break;
          }

          case 'grip': {
            w.status = 'grasped';
            // Policy applies force in safe zone
            const targetForce = requiredForce + 1.2;
            w.gripForce = Math.min(targetForce, w.gripForce + 1.5);
            if (w.gripForce >= requiredForce) {
              w.phase = 'lift';
              w.phaseTimer = 0;
            }
            break;
          }

          case 'lift': {
            w.status = 'lifting';
            w.gripForce = requiredForce + 1.2;
            w.gripperY = Math.min(travelY, w.gripperY + 0.06);
            // Object moves together with gripper
            w.objX = w.carriageX;
            w.objY = w.gripperY - 0.15;
            if (w.gripperY >= travelY) {
              w.phase = 'carry';
              w.phaseTimer = 0;
            }
            break;
          }

          case 'carry': {
            w.status = 'carrying';
            w.gripperY = travelY;
            w.objY = w.gripperY - 0.15;
            const dx = targetX - w.carriageX;
            if (Math.abs(dx) > 0.05) {
              w.carriageX += Math.sign(dx) * 0.07;
              w.objX = w.carriageX;
            } else {
              w.carriageX = targetX;
              w.objX = targetX;
              w.phase = 'lower_target';
              w.phaseTimer = 0;
            }
            break;
          }

          case 'lower_target': {
            w.status = 'carrying';
            w.carriageX = targetX;
            w.objX = targetX;
            if (w.gripperY > releaseY) {
              w.gripperY = Math.max(releaseY, w.gripperY - 0.06);
              w.objY = w.gripperY - 0.15;
            } else {
              w.gripperY = releaseY;
              w.objY = targetTerrainY + this.config.object.height / 2;
              w.phase = 'release';
              w.phaseTimer = 0;
            }
            break;
          }

          case 'release': {
            w.status = 'releasing';
            w.jawOpening = Math.min(0.9, w.jawOpening + 0.15);
            w.gripForce = Math.max(0, w.gripForce - 1.8);
            if (w.jawOpening >= 0.85) {
              w.status = 'placed';
              w.phase = 'retract';
              w.phaseTimer = 0;
            }
            break;
          }

          case 'retract': {
            w.gripForce = 0.0;
            w.jawOpening = 0.8;
            if (w.gripperY < travelY) {
              w.gripperY = Math.min(travelY, w.gripperY + 0.08);
            } else {
              // Completed full episode cycle, reset to beginning
              if (w.phaseTimer > 0.6) {
                w.episodeId += 1;
                w.step = 0;
                w.carriageX = this.config.gantry.carriage_x;
                w.gripperY = this.config.gantry.gripper_y;
                w.objX = this.config.object.position_x;
                w.objY = objTerrainY + this.config.object.height / 2;
                w.status = 'idle';
                w.phase = 'approach_horizontal';
                w.phaseTimer = 0;
              }
            }
            break;
          }
        }

        // Safety Force Evaluation
        if (w.status === 'grasped' || w.status === 'lifting' || w.status === 'carrying') {
          if (w.gripForce >= breakForce) {
            w.status = 'broken';
            w.phase = 'release';
          } else if (w.gripForce < requiredForce * 0.9) {
            w.status = 'slipping';
            w.objY = Math.max(objTerrainY + this.config.object.height / 2, w.objY - 0.04);
          }
        }

        w.lastReward =
          w.status === 'placed'
            ? 2.5
            : w.status === 'carrying' || w.status === 'lifting'
            ? 1.4
            : w.status === 'slipping'
            ? 0.2
            : w.status === 'broken'
            ? -1.5
            : 0.8;
      });
    }

    this.broadcastSnapshot();
  }

  private broadcastSnapshot() {
    const currentWorker = this.workers.get(this.selectedWorkerId) || this.workers.get(1)!;

    const m = this.config.object.mass;
    const g = this.config.terrain.gravity;
    const mu = this.config.object.friction;
    const accel = 0.15;
    const requiredForce = (m * (g + accel)) / (2 * Math.max(0.01, mu));
    const safetyMargin = currentWorker.gripForce - requiredForce;

    const workerState: WorkerState = {
      id: this.selectedWorkerId,
      episode_id: currentWorker.episodeId,
      episode_step: currentWorker.step,
      task_phase: currentWorker.phase,
      gantry: {
        carriage_x: Number(currentWorker.carriageX.toFixed(3)),
        gripper_y: Number(currentWorker.gripperY.toFixed(3)),
        grip_force: Number(currentWorker.gripForce.toFixed(2)),
        jaw_opening: Number(currentWorker.jawOpening.toFixed(3)),
        rail_y: this.config.gantry.rail_y,
      },
      // Backward compatibility robot shim for any legacy consumer
      robot: {
        base_position: { x: currentWorker.carriageX, y: this.config.gantry.rail_y },
        shoulder_angle: 0,
        elbow_angle: 0,
        gripper_position: { x: currentWorker.carriageX, y: currentWorker.gripperY },
        grip_force: Number(currentWorker.gripForce.toFixed(2)),
      },
      object: {
        id: this.config.object.id || 'object-1',
        position: {
          x: Number(currentWorker.objX.toFixed(3)),
          y: Number(currentWorker.objY.toFixed(3)),
        },
        velocity: {
          x: Number(currentWorker.objVx.toFixed(3)),
          y: Number(currentWorker.objVy.toFixed(3)),
        },
        mass: this.config.object.mass,
        friction: this.config.object.friction,
        break_force: this.config.object.break_force,
        required_grip_force: Number(requiredForce.toFixed(2)),
        safety_margin: Number(safetyMargin.toFixed(2)),
        status: currentWorker.status,
      },
      target: {
        position_x: this.config.target.position_x,
        width: this.config.target.width,
      },
      terrain: {
        points: this.config.terrain.points,
      },
      last_action: {
        normalized_grip_force: Number((currentWorker.gripForce / this.config.gantry.maximum_grip_force).toFixed(3)),
      },
      last_reward: Number(currentWorker.lastReward.toFixed(2)),
      done: currentWorker.done,
      outcome: this.status === 'running' ? 'running' : this.status,
    };

    const snapshot: SimulationSnapshot = {
      type: 'simulation_snapshot',
      timestamp: new Date().toISOString(),
      runtime: {
        status: this.status,
        mode: this.mode,
        active_workers: this.status === 'running' ? (this.mode === 'swarm' ? 100 : 16) : 0,
        steps_per_second: this.status === 'running' ? (this.mode === 'swarm' ? 32420 : 4120) : 0,
        episodes_per_second: this.status === 'running' ? (this.mode === 'swarm' ? 124 : 18) : 0,
        total_steps: this.totalSteps,
        success_rate: this.mode === 'swarm' ? 0.942 : 0.748,
        average_reward: this.mode === 'swarm' ? 16.24 : 9.85,
        replay_buffer_size: this.mode === 'swarm' ? 100000 : 25000,
        training_batches: this.trainingBatches,
        policy_version: this.policyVersion,
      },
      worker: workerState,
    };

    this.listeners.forEach((listener) => {
      try {
        listener(snapshot);
      } catch (err) {
        console.error('Error in telemetry listener:', err);
      }
    });
  }

  // Public API
  public subscribe(listener: SnapshotListener): () => void {
    this.listeners.add(listener);
    this.broadcastSnapshot();
    return () => {
      this.listeners.delete(listener);
    };
  }

  public start() {
    this.status = 'running';
    this.broadcastSnapshot();
  }

  public pause() {
    this.status = 'paused';
    this.broadcastSnapshot();
  }

  public resume() {
    this.status = 'running';
    this.broadcastSnapshot();
  }

  public reset() {
    this.status = 'resetting';
    this.broadcastSnapshot();
    setTimeout(() => {
      this.initWorkers();
      this.status = 'running';
      this.broadcastSnapshot();
    }, 400);
  }

  public setMode(mode: SimulationMode) {
    this.mode = mode;
    this.broadcastSnapshot();
  }

  public setSelectedWorker(id: number) {
    if (id >= 1 && id <= this.totalWorkers) {
      this.selectedWorkerId = id;
      this.broadcastSnapshot();
    }
  }

  public getConfig(): SceneConfig {
    return JSON.parse(JSON.stringify(this.config));
  }

  /**
   * Applies full scene configuration
   */
  public applyConfig(newConfig: SceneConfig): { success: boolean; config?: SceneConfig; error?: string } {
    if (this.status !== 'paused') {
      return {
        success: false,
        error: 'The simulation must be paused before updating the scene configuration.',
      };
    }
    this.config = JSON.parse(JSON.stringify(newConfig));

    const w = this.workers.get(this.selectedWorkerId);
    if (w) {
      w.carriageX = newConfig.gantry.carriage_x;
      w.gripperY = newConfig.gantry.gripper_y;
      w.objX = newConfig.object.position_x;
      w.objY = newConfig.object.position_y;
    }
    this.broadcastSnapshot();
    return { success: true, config: this.config };
  }

  /**
   * Handles PUT /api/scene update payload from direct pointer release
   */
  public applySceneUpdate(update: SceneUpdatePayload): { success: boolean; config?: SceneConfig; error?: string } {
    if (this.status !== 'paused') {
      return {
        success: false,
        error: 'Scene editing is only permitted while simulation is paused.',
      };
    }

    if (update.object) {
      this.config.object.position_x = update.object.position.x;
      this.config.object.position_y = update.object.position.y;
    }

    if (update.gantry) {
      this.config.gantry.carriage_x = update.gantry.carriage_x;
      this.config.gantry.gripper_y = update.gantry.gripper_y;
    }

    if (update.target) {
      this.config.target.position_x = update.target.position.x;
    }

    if (update.terrain && update.terrain.points) {
      this.config.terrain.points = [...update.terrain.points].sort((a, b) => a.x - b.x);
    }

    // Sync to active worker
    const w = this.workers.get(this.selectedWorkerId);
    if (w) {
      if (update.gantry) {
        w.carriageX = update.gantry.carriage_x;
        w.gripperY = update.gantry.gripper_y;
      }
      if (update.object) {
        w.objX = update.object.position.x;
        w.objY = update.object.position.y;
      }
    }

    this.broadcastSnapshot();
    return { success: true, config: this.config };
  }

  public getStatus(): RuntimeStatus {
    return this.status;
  }

  public getMode(): SimulationMode {
    return this.mode;
  }

  public destroy() {
    this.stopLoop();
    this.listeners.clear();
  }
}

export const mockTelemetryClient = new MockTelemetryClient();
