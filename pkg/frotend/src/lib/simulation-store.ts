import { create } from 'zustand';
import {
  RuntimeStatus,
  SimulationMode,
  SimulationSnapshot,
  ChartSample,
  SceneConfig,
  SceneUpdatePayload,
  TerrainPoint,
} from '../types/simulation';
import { DEFAULT_SCENE_CONFIG } from './default-scene-config';
import { runtimeClient } from './runtime-client';
import { updateSceneApi } from './simulation-api';
import { DEFAULT_FLAT_TERRAIN_POINTS } from '../components/simulation/editable-terrain';

export type SelectedEntityType = 'object' | 'carriage' | 'gripper' | 'terrain_point' | 'target' | null;

export interface SimulationStore {
  // Runtime State
  runtimeStatus: RuntimeStatus;
  selectedWorkerId: number;
  selectedMode: SimulationMode;
  webSocketStatus: 'connected' | 'connecting' | 'disconnected';
  latestTelemetry: SimulationSnapshot | null;
  telemetryHistory: ChartSample[];
  showVectors: boolean;
  activeTab: 'simulation' | 'comparison';

  // Interaction State (Section 12)
  selectedEntity: SelectedEntityType;
  selectedTerrainPointId: string | null;
  hoveredEntity: SelectedEntityType;
  isDraggingEntity: boolean;

  // Configuration Management
  confirmedConfig: SceneConfig;
  draftConfig: SceneConfig;
  isDraftDirty: boolean;
  configError: string | null;

  // Mode Change Dialog
  showModeDialog: boolean;
  pendingMode: SimulationMode | null;

  // Actions
  setLatestTelemetry: (snapshot: SimulationSnapshot) => void;
  setSelectedWorker: (id: number) => void;
  startSimulation: () => Promise<void>;
  pauseSimulation: () => Promise<void>;
  resumeSimulation: () => Promise<void>;
  resetSimulation: () => Promise<void>;
  requestModeChange: (mode: SimulationMode) => void;
  confirmModeChange: () => void;
  cancelModeChange: () => void;

  setSelectedEntity: (entity: SelectedEntityType, terrainPointId?: string | null) => void;
  setHoveredEntity: (entity: SelectedEntityType) => void;
  setIsDraggingEntity: (dragging: boolean) => void;

  updateDraftObject: (updates: Partial<SceneConfig['object']>) => void;
  updateDraftGantry: (updates: Partial<SceneConfig['gantry']>) => void;
  updateDraftRobot: (updates: Partial<NonNullable<SceneConfig['robot']>>) => void;
  updateDraftTerrain: (updates: Partial<SceneConfig['terrain']>) => void;
  updateDraftTarget: (updates: Partial<SceneConfig['target']>) => void;

  // Scene Editing Helpers (Section 7)
  addTerrainPoint: (point: TerrainPoint) => void;
  deleteSelectedTerrainPoint: () => void;
  resetFlatTerrain: () => void;

  // API Interaction (PUT /api/scene)
  submitSceneUpdate: (payload: SceneUpdatePayload) => Promise<{ success: boolean; error?: string }>;
  applyDraftConfig: () => Promise<{ success: boolean; error?: string }>;
  discardDraftConfig: () => void;
  restoreDefaultDraftConfig: () => void;
  toggleVectors: () => void;
  setActiveTab: (tab: 'simulation' | 'comparison') => void;
}

const MAX_HISTORY = 600;

export const useSimulationStore = create<SimulationStore>((set, get) => ({
  runtimeStatus: 'running',
  selectedWorkerId: 1,
  selectedMode: 'swarm',
  webSocketStatus: 'disconnected',
  latestTelemetry: null,
  telemetryHistory: [],
  showVectors: true,
  activeTab: 'simulation',

  selectedEntity: null,
  selectedTerrainPointId: null,
  hoveredEntity: null,
  isDraggingEntity: false,

  confirmedConfig: JSON.parse(JSON.stringify(DEFAULT_SCENE_CONFIG)),
  draftConfig: JSON.parse(JSON.stringify(DEFAULT_SCENE_CONFIG)),
  isDraftDirty: false,
  configError: null,

  showModeDialog: false,
  pendingMode: null,

  setLatestTelemetry: (snapshot: SimulationSnapshot) => {
    const prevHistory = get().telemetryHistory;
    const gripForce = snapshot.worker.gantry?.grip_force ?? snapshot.worker.robot?.grip_force ?? 0;

    const newSample: ChartSample = {
      timestamp: Date.now(),
      step: snapshot.worker.episode_step,
      total_steps: snapshot.runtime.total_steps,
      average_reward: snapshot.runtime.average_reward,
      success_rate: snapshot.runtime.success_rate,
      steps_per_second: snapshot.runtime.steps_per_second,
      grip_force: gripForce,
      required_grip_force: snapshot.worker.object.required_grip_force,
      break_force: snapshot.worker.object.break_force,
    };

    const updatedHistory = [...prevHistory, newSample];
    if (updatedHistory.length > MAX_HISTORY) {
      updatedHistory.splice(0, updatedHistory.length - MAX_HISTORY);
    }

    set({
      latestTelemetry: snapshot,
      runtimeStatus: snapshot.runtime.status,
      selectedMode: snapshot.runtime.mode,
      telemetryHistory: updatedHistory,
    });
  },

  setSelectedWorker: (id: number) => {
    set({ selectedWorkerId: id });
  },

  startSimulation: async () => {
    await runtimeClient.command('/api/simulation/start');
    set({ runtimeStatus: 'running', selectedEntity: null, isDraggingEntity: false });
  },

  pauseSimulation: async () => {
    await runtimeClient.command('/api/simulation/pause');
    // When paused, clone current confirmed configuration into draft
    set({
      runtimeStatus: 'paused',
      draftConfig: JSON.parse(JSON.stringify(get().confirmedConfig)),
      isDraftDirty: false,
      configError: null,
    });
  },

  resumeSimulation: async () => {
    await runtimeClient.command('/api/simulation/resume');
    set({ runtimeStatus: 'running', selectedEntity: null, isDraggingEntity: false });
  },

  resetSimulation: async () => {
    await runtimeClient.command('/api/simulation/reset');
    set({ runtimeStatus: 'resetting', selectedEntity: null, isDraggingEntity: false });
  },

  requestModeChange: (mode: SimulationMode) => {
    if (mode === get().selectedMode) return;
    set({
      showModeDialog: true,
      pendingMode: mode,
    });
  },

  confirmModeChange: async () => {
    const { pendingMode } = get();
    if (!pendingMode) return;
    await runtimeClient.command('/api/simulation/mode', { mode: pendingMode });
    await runtimeClient.command('/api/simulation/reset');
    set({
      selectedMode: pendingMode,
      showModeDialog: false,
      pendingMode: null,
      runtimeStatus: 'resetting',
    });
  },

  cancelModeChange: () => {
    set({
      showModeDialog: false,
      pendingMode: null,
    });
  },

  setSelectedEntity: (entity, terrainPointId = null) => {
    set({ selectedEntity: entity, selectedTerrainPointId: terrainPointId });
  },

  setHoveredEntity: (entity) => {
    set({ hoveredEntity: entity });
  },

  setIsDraggingEntity: (dragging) => {
    set({ isDraggingEntity: dragging });
  },

  updateDraftObject: (updates) => {
    const current = get().draftConfig;
    const nextConfig = {
      ...current,
      object: { ...current.object, ...updates },
    };
    set({ draftConfig: nextConfig, isDraftDirty: true, configError: null });
  },

  updateDraftGantry: (updates) => {
    const current = get().draftConfig;
    const nextConfig = {
      ...current,
      gantry: { ...current.gantry, ...updates },
    };
    set({ draftConfig: nextConfig, isDraftDirty: true, configError: null });
  },

  updateDraftRobot: (updates) => {
    const current = get().draftConfig;
    const nextConfig = {
      ...current,
      robot: { ...(current.robot || DEFAULT_SCENE_CONFIG.robot!), ...updates },
    };
    set({ draftConfig: nextConfig, isDraftDirty: true, configError: null });
  },

  updateDraftTerrain: (updates) => {
    const current = get().draftConfig;
    const nextConfig = {
      ...current,
      terrain: { ...current.terrain, ...updates },
    };
    set({ draftConfig: nextConfig, isDraftDirty: true, configError: null });
  },

  updateDraftTarget: (updates) => {
    const current = get().draftConfig;
    const nextConfig = {
      ...current,
      target: { ...current.target, ...updates },
    };
    set({ draftConfig: nextConfig, isDraftDirty: true, configError: null });
  },

  addTerrainPoint: (point: TerrainPoint) => {
    const current = get().draftConfig;
    const points = [...current.terrain.points, point].sort((a, b) => a.x - b.x);
    get().updateDraftTerrain({ points });
    set({ selectedTerrainPointId: point.id, selectedEntity: 'terrain_point' });
    // Trigger update to backend
    get().submitSceneUpdate({ terrain: { points } });
  },

  deleteSelectedTerrainPoint: () => {
    const { draftConfig, selectedTerrainPointId } = get();
    if (!selectedTerrainPointId) return;
    if (draftConfig.terrain.points.length <= 2) {
      set({ configError: 'Terrain must contain at least 2 control points.' });
      return;
    }

    const updatedPoints = draftConfig.terrain.points.filter((p) => p.id !== selectedTerrainPointId);
    get().updateDraftTerrain({ points: updatedPoints });
    set({ selectedTerrainPointId: null, selectedEntity: null });
    get().submitSceneUpdate({ terrain: { points: updatedPoints } });
  },

  resetFlatTerrain: () => {
    const flatPoints = JSON.parse(JSON.stringify(DEFAULT_FLAT_TERRAIN_POINTS));
    get().updateDraftTerrain({ points: flatPoints });
    set({ selectedTerrainPointId: null, selectedEntity: null });
    get().submitSceneUpdate({ terrain: { points: flatPoints } });
  },

  submitSceneUpdate: async (payload: SceneUpdatePayload) => {
    const res = await updateSceneApi(payload);
    if (res.success && res.config) {
      set({
        confirmedConfig: JSON.parse(JSON.stringify(res.config)),
        draftConfig: JSON.parse(JSON.stringify(res.config)),
        isDraftDirty: false,
        configError: null,
      });
      return { success: true };
    } else {
      // Revert draft to confirmed on error
      const confirmed = get().confirmedConfig;
      set({
        draftConfig: JSON.parse(JSON.stringify(confirmed)),
        configError: res.error || 'Backend rejected scene update.',
      });
      return { success: false, error: res.error };
    }
  },

  applyDraftConfig: async () => {
    const { draftConfig, runtimeStatus } = get();
    if (runtimeStatus !== 'paused') {
      return {
        success: false,
        error: 'The simulation must be paused before updating the scene configuration.',
      };
    }

    if (draftConfig.object.mass <= 0) {
      return { success: false, error: 'Object mass must be greater than 0 kg.' };
    }
    if (draftConfig.object.friction <= 0 || draftConfig.object.friction > 2.0) {
      return { success: false, error: 'Friction must be between 0.01 and 2.0.' };
    }
    if (draftConfig.object.break_force <= 0) {
      return { success: false, error: 'Break force must be positive.' };
    }

    const res = await updateSceneApi({
      object: {
        position: { x: draftConfig.object.position_x, y: draftConfig.object.position_y },
        mass: draftConfig.object.mass,
        friction: draftConfig.object.friction,
        break_force: draftConfig.object.break_force,
        width: draftConfig.object.width,
        height: draftConfig.object.height,
      },
      gantry: {
        carriage_x: draftConfig.gantry.carriage_x,
        gripper_y: draftConfig.gantry.gripper_y,
      },
      target: {
        position: { x: draftConfig.target.position_x, y: 0 },
      },
      terrain: {
        points: draftConfig.terrain.points,
      },
    });

    if (res.success && res.config) {
      set({
        confirmedConfig: JSON.parse(JSON.stringify(res.config)),
        draftConfig: JSON.parse(JSON.stringify(res.config)),
        isDraftDirty: false,
        configError: null,
      });
      return { success: true };
    } else {
      set({ configError: res.error || 'Failed to apply configuration' });
      return { success: false, error: res.error };
    }
  },

  discardDraftConfig: () => {
    const confirmed = get().confirmedConfig;
    set({
      draftConfig: JSON.parse(JSON.stringify(confirmed)),
      isDraftDirty: false,
      configError: null,
      selectedEntity: null,
      selectedTerrainPointId: null,
    });
  },

  restoreDefaultDraftConfig: () => {
    set({
      draftConfig: JSON.parse(JSON.stringify(DEFAULT_SCENE_CONFIG)),
      isDraftDirty: true,
      configError: null,
    });
  },

  toggleVectors: () => {
    set((state) => ({ showVectors: !state.showVectors }));
  },

  setActiveTab: (tab) => {
    set({ activeTab: tab });
  },
}));
