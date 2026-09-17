package pkg

import (
	"encoding/json"
	"errors"
	"math"
	"net/http"
	"sync"
	"time"

	"github.com/duongess/khoai-robot-control-framework/pkg/framework"
	"github.com/duongess/khoai-robot-visualizer-web/pkg/forcecontrol"
	"golang.org/x/net/websocket"
)

// APIServer exposes visualizer-specific HTTP and WebSocket endpoints.
type APIServer struct {
	runtime *framework.Runtime
	mu      sync.RWMutex
	config  forcecontrol.Config
}

func NewAPIServer(runtime *framework.Runtime, config forcecontrol.Config) (*APIServer, error) {
	if runtime == nil {
		return nil, errors.New("runtime is required")
	}
	return &APIServer{runtime: runtime, config: config}, nil
}

func (s *APIServer) APIHandler() http.Handler {
	return http.HandlerFunc(s.serveAPI)
}

func (s *APIServer) WebSocketHandler() http.Handler {
	return websocket.Server{Handshake: func(*websocket.Config, *http.Request) error { return nil }, Handler: func(connection *websocket.Conn) {
		defer connection.Close()
		ticker := time.NewTicker(100 * time.Millisecond)
		defer ticker.Stop()
		for range ticker.C {
			if err := websocket.JSON.Send(connection, s.telemetry()); err != nil {
				return
			}
		}
	}}
}

func (s *APIServer) serveAPI(w http.ResponseWriter, r *http.Request) {
	switch {
	case r.Method == http.MethodGet && r.URL.Path == "/api/health":
		s.writeJSON(w, http.StatusOK, map[string]any{"status": "ok", "runtime": s.runtime.Snapshot().Status})
	case r.Method == http.MethodGet && r.URL.Path == "/api/status":
		s.writeJSON(w, http.StatusOK, s.runtime.Snapshot())
	case r.Method == http.MethodGet && r.URL.Path == "/api/config":
		s.writeJSON(w, http.StatusOK, map[string]any{"config": s.sceneConfig()})
	case r.Method == http.MethodPut && r.URL.Path == "/api/scene":
		s.updateScene(w, r)
	case r.Method == http.MethodPost && r.URL.Path == "/api/simulation/start":
		s.lifecycle(w, func() error { return s.runtime.Start(r.Context()) })
	case r.Method == http.MethodPost && r.URL.Path == "/api/simulation/pause":
		s.lifecycle(w, s.runtime.Pause)
	case r.Method == http.MethodPost && r.URL.Path == "/api/simulation/resume":
		s.lifecycle(w, s.runtime.Resume)
	case r.Method == http.MethodPost && r.URL.Path == "/api/simulation/reset":
		s.lifecycle(w, s.runtime.Reset)
	case r.Method == http.MethodPost && r.URL.Path == "/api/evaluation/approve":
		s.approveCurriculumReview(w, r)
	case r.Method == http.MethodPost && r.URL.Path == "/api/model/save":
		s.saveModel(w, r)
	case r.Method == http.MethodPost && r.URL.Path == "/api/simulation/stop":
		s.runtime.Stop()
		s.writeJSON(w, http.StatusOK, map[string]string{"status": "ok"})
	case r.Method == http.MethodPost && r.URL.Path == "/api/simulation/mode":
		s.writeJSON(w, http.StatusOK, map[string]string{"status": "ok"})
	case r.Method == http.MethodGet && r.URL.Path == "/api/workers":
		s.writeJSON(w, http.StatusOK, map[string]any{"workers": s.runtime.Snapshot().Workers})
	default:
		s.writeError(w, http.StatusNotFound, "NOT_FOUND", "The requested API endpoint does not exist.")
	}
}

func (s *APIServer) lifecycle(w http.ResponseWriter, operation func() error) {
	if err := operation(); err != nil {
		s.writeError(w, http.StatusConflict, "SIMULATION_STATE", err.Error())
		return
	}
	s.writeJSON(w, http.StatusOK, map[string]string{"status": "ok"})
}

type curriculumReviewRequest struct {
	WorkerID int `json:"worker_id"`
}

func (s *APIServer) approveCurriculumReview(w http.ResponseWriter, r *http.Request) {
	if s.runtime.Snapshot().Status != framework.RuntimePaused {
		s.writeError(w, http.StatusConflict, "SIMULATION_NOT_PAUSED", "Pause the simulation before approving a curriculum review.")
		return
	}
	var request curriculumReviewRequest
	decoder := json.NewDecoder(r.Body)
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&request); err != nil || request.WorkerID <= 0 {
		s.writeError(w, http.StatusBadRequest, "INVALID_REVIEW", "worker_id must be a positive integer.")
		return
	}
	if err := s.runtime.ApproveCurriculumReview(request.WorkerID); err != nil {
		s.writeError(w, http.StatusConflict, "CURRICULUM_REVIEW_REJECTED", err.Error())
		return
	}
	s.writeJSON(w, http.StatusOK, map[string]any{"status": "approved", "worker_id": request.WorkerID, "note": "No synthetic reward or success transition was recorded."})
}

// saveModel forwards a named, complete SAC checkpoint request to the local
// learner. It intentionally remains available while the runtime is running:
// the learner serializes a lock-consistent snapshot without resetting workers.
func (s *APIServer) saveModel(w http.ResponseWriter, r *http.Request) {
	result, err := s.runtime.SaveCheckpoint(r.Context())
	if err != nil {
		s.writeError(w, http.StatusBadRequest, "MODEL_SAVE_REJECTED", err.Error())
		return
	}
	s.writeJSON(w, http.StatusOK, map[string]any{
		"status": "saved", "model_name": result.ModelName,
		"policy_version": result.PolicyVersion, "training_step": result.TrainingStep,
	})
}

type sceneUpdate struct {
	Object *sceneObjectUpdate `json:"object"`
	Gantry *struct {
		CarriageX float64 `json:"carriage_x"`
		GripperY  float64 `json:"gripper_y"`
	} `json:"gantry"`
	Target *struct {
		Position struct {
			X float64 `json:"x"`
			// Target Y is accepted for backwards-compatible scene payloads, but
			// placement height is derived from the authoritative terrain.
			Y *float64 `json:"y"`
		} `json:"position"`
	} `json:"target"`
	Terrain *struct {
		Points []terrainPoint `json:"points"`
	} `json:"terrain"`
}

type sceneObjectUpdate struct {
	// ID is a stable frontend selection identifier, not an environment body ID.
	// Accept it so direct canvas drags remain compatible with strict decoding.
	ID       string `json:"id"`
	Position struct {
		X float64 `json:"x"`
		Y float64 `json:"y"`
	} `json:"position"`
	Mass       *float64 `json:"mass"`
	Friction   *float64 `json:"friction"`
	BreakForce *float64 `json:"break_force"`
	Width      *float64 `json:"width"`
	Height     *float64 `json:"height"`
}

type terrainPoint struct {
	ID string  `json:"id"`
	X  float64 `json:"x"`
	Y  float64 `json:"y"`
}

func (s *APIServer) updateScene(w http.ResponseWriter, r *http.Request) {
	if s.runtime.Snapshot().Status != framework.RuntimePaused {
		s.writeError(w, http.StatusConflict, "SIMULATION_NOT_PAUSED", "The simulation must be paused before updating the scene.")
		return
	}
	var update sceneUpdate
	decoder := json.NewDecoder(r.Body)
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&update); err != nil {
		s.writeError(w, http.StatusBadRequest, "INVALID_SCENE", "The scene update must be valid JSON.")
		return
	}
	s.mu.Lock()
	next := s.config
	if update.Object != nil {
		next.InitialObjectX = update.Object.Position.X
		if update.Object.Mass != nil {
			next.InitialObjectMass = *update.Object.Mass
		}
		if update.Object.Friction != nil {
			next.ObjectFriction = *update.Object.Friction
		}
		if update.Object.BreakForce != nil {
			next.ObjectBreakForce = *update.Object.BreakForce
		}
		if update.Object.Width != nil {
			next.ObjectWidth = *update.Object.Width
		}
		if update.Object.Height != nil {
			next.ObjectHeight = *update.Object.Height
		}
	}
	if update.Gantry != nil {
		next.InitialCarriageX, next.InitialGripperY = update.Gantry.CarriageX, update.Gantry.GripperY
	}
	if update.Target != nil {
		next.TargetX = update.Target.Position.X
	}
	if update.Terrain != nil {
		if len(update.Terrain.Points) < 2 {
			s.mu.Unlock()
			s.writeError(w, http.StatusBadRequest, "INVALID_TERRAIN", "Terrain must contain at least two control points.")
			return
		}
		next.Terrain = make([]forcecontrol.TerrainPoint, len(update.Terrain.Points))
		for i, point := range update.Terrain.Points {
			if !finite(point.X) || !finite(point.Y) {
				s.mu.Unlock()
				s.writeError(w, http.StatusBadRequest, "INVALID_TERRAIN", "Terrain coordinates must be finite.")
				return
			}
			next.Terrain[i] = forcecontrol.TerrainPoint{X: point.X, Y: point.Y}
		}
	}
	if !finite(next.InitialObjectX) || !finite(next.InitialCarriageX) || !finite(next.InitialGripperY) || !finite(next.TargetX) {
		s.mu.Unlock()
		s.writeError(w, http.StatusBadRequest, "INVALID_SCENE", "Scene coordinates must be finite.")
		return
	}
	if next.InitialObjectMass <= 0 || next.ObjectFriction <= 0 || next.ObjectBreakForce <= 0 || next.ObjectWidth <= 0 || next.ObjectHeight <= 0 {
		s.mu.Unlock()
		s.writeError(w, http.StatusBadRequest, "INVALID_SCENE", "Object mass, friction, dimensions, and break force must be positive.")
		return
	}
	if err := forcecontrol.UpdateRegistration(s.runtime, next); err != nil {
		s.mu.Unlock()
		s.writeError(w, http.StatusBadRequest, "INVALID_SCENE", err.Error())
		return
	}
	s.config = next
	s.mu.Unlock()
	s.writeJSON(w, http.StatusOK, map[string]any{"status": "ok", "config": s.sceneConfig()})
}

func (s *APIServer) sceneConfig() map[string]any {
	s.mu.RLock()
	config := s.config
	s.mu.RUnlock()
	points := make([]terrainPoint, len(config.Terrain))
	for i, point := range config.Terrain {
		points[i] = terrainPoint{ID: "terrain-" + string(rune('1'+i)), X: point.X, Y: point.Y}
	}
	bounds := forcecontrol.SafeGripperBounds(config, config.InitialCarriageX)
	return map[string]any{"workspace": map[string]any{"minX": config.Workspace.MinX, "maxX": config.Workspace.MaxX, "minY": config.Workspace.MinY, "maxY": config.Workspace.MaxY, "coordinate_system_version": forcecontrol.CoordinateSystemVersion}, "object": map[string]any{"id": "object-1", "position_x": config.InitialObjectX, "position_y": terrainAt(config, config.InitialObjectX) + config.ObjectHeight/2, "mass": config.InitialObjectMass, "friction": config.ObjectFriction, "break_force": config.ObjectBreakForce, "initial_vertical_velocity": 0, "width": config.ObjectWidth, "height": config.ObjectHeight}, "gantry": map[string]any{"rail_y": config.RailY, "carriage_x": config.InitialCarriageX, "gripper_y": config.InitialGripperY, "min_x": bounds.MinX, "max_x": bounds.MaxX, "min_y": bounds.MinY, "max_y": bounds.MaxY, "initial_grip_force": 0, "minimum_grip_force": 0, "maximum_grip_force": config.MaxGripForce}, "terrain": map[string]any{"points": points, "ground_friction": config.ObjectFriction, "gravity": config.Gravity}, "target": map[string]any{"position_x": config.TargetX, "width": config.TargetWidth}, "curriculum": map[string]any{"stage": config.Curriculum.Stage, "contact_start_height_offset": config.Curriculum.ContactStartHeightOffset, "contact_stable_steps": config.Curriculum.ContactStableSteps, "contact_successes_required": config.Curriculum.ContactSuccessesRequired, "grasp_hold_seconds": config.Curriculum.GraspHoldSeconds, "randomization": config.Curriculum.Randomization, "align_episode_step_limit": config.Curriculum.AlignEpisodeStepLimit, "episode_step_limit": config.Curriculum.EpisodeStepLimit}}
}

func (s *APIServer) telemetry() map[string]any {
	snapshot := s.runtime.Snapshot()
	s.mu.RLock()
	config := s.config
	s.mu.RUnlock()
	worker := map[string]any{}
	if len(snapshot.Workers) > 0 {
		selected := snapshot.Workers[0]
		runtimeConfig := config
		terrainPoints := s.sceneConfig()["terrain"].(map[string]any)["points"]
		contactEpisodeSuccesses := float64(selected.Info["curriculum_contact_success_streak"])
		contactEpisodeSuccessesRequired := float64(selected.Info["curriculum_contact_successes_required"])
		graspHoldFrames := float64(selected.Info["curriculum_grasp_hold_frames"])
		graspHoldFramesRequired := float64(selected.Info["curriculum_grasp_hold_frames_required"])
		if value, ok := selected.Metadata["curriculum_contact_success_streak"].(int); ok {
			contactEpisodeSuccesses = float64(value)
		}
		if value, ok := selected.Metadata["curriculum_contact_successes_required"].(int); ok {
			contactEpisodeSuccessesRequired = float64(value)
		}
		if value, ok := selected.Metadata["curriculum_grasp_hold_frames"].(int); ok {
			graspHoldFrames = float64(value)
		}
		if value, ok := selected.Metadata["curriculum_grasp_hold_frames_required"].(int); ok {
			graspHoldFramesRequired = float64(value)
		}
		if points, ok := selected.Metadata["terrain_points"].([]forcecontrol.TerrainPoint); ok && len(points) >= 2 {
			runtimeConfig.Terrain = append([]forcecontrol.TerrainPoint(nil), points...)
			displayPoints := make([]terrainPoint, len(points))
			for index, point := range points {
				displayPoints[index] = terrainPoint{ID: "terrain-" + string(rune('1'+index)), X: point.X, Y: point.Y}
			}
			terrainPoints = displayPoints
		}
		values := selected.State
		get := func(index int) float64 {
			if index >= len(values) {
				return 0
			}
			return float64(values[index])
		}
		carriageX, gripperY := denormalize(get(0), config.Workspace.MinX, config.Workspace.MaxX), denormalize(get(1), config.Workspace.MinY, config.Workspace.MaxY)
		objectX, objectY := denormalize(get(4), config.Workspace.MinX, config.Workspace.MaxX), denormalize(get(5), config.Workspace.MinY, config.Workspace.MaxY)
		phase := forcecontrol.PhaseFromNormalized(float32(get(19)))
		lastAction := []float32{0, 0, 0}
		if len(selected.LastAction) == 3 {
			lastAction = selected.LastAction
		}
		forceRateCommand := lastAction[2]
		filteredForceRateCommand := selected.Info["filtered_action_gripper"]
		forceActionMode := "hold"
		switch {
		case float64(filteredForceRateCommand) <= config.ReleaseActionThreshold:
			forceActionMode = "release"
		case filteredForceRateCommand > float32(config.ActionDeadZone):
			forceActionMode = "increase"
		case filteredForceRateCommand < -float32(config.ActionDeadZone):
			forceActionMode = "decrease"
		}
		gripForce := denormalize(get(16), 0, config.MaxGripForce)
		safeBounds := forcecontrol.SafeGripperBounds(runtimeConfig, carriageX)
		targetX, targetY := denormalize(get(8), config.Workspace.MinX, config.Workspace.MaxX), denormalize(get(9), config.Workspace.MinY, config.Workspace.MaxY)
		targetGraspY := forcecontrol.GraspHeight(runtimeConfig, objectY, carriageX)
		boundaryHit := selected.Info["boundary_hit"] > 0
		gripperClosed := selected.Info["gripper_closed"] > 0
		contactDetected := selected.Info["contact_detected"] > 0
		objectAttached := selected.Info["object_attached"] > 0
		objectReleased := selected.Info["object_released"] > 0
		objectBroken := selected.Info["object_broken"] > 0
		status := objectStatus(phase, gripperClosed, contactDetected, objectAttached, objectReleased, objectBroken)
		energyEvent := energyEventLabel(selected.Info["energy_event_code"])
		worker = map[string]any{
			"id": selected.ID, "episode_id": selected.EpisodeID, "episode_step": selected.EpisodeStep, "episode_policy_version": selected.PolicyVersion,
			"action_source": selected.ActionSource, "curriculum_stage": curriculumStageLabel(selected.Info["curriculum_stage_code"]),
			"task_phase": phase.String(), "gripper_state": ternary(gripperClosed, "closed", "open"), "contact_state": ternary(contactDetected, "contact", "none"),
			"force_valid":     selected.Info["force_valid"] > 0,
			"gantry":          map[string]any{"carriage_x": carriageX, "gripper_y": gripperY, "grip_force": gripForce, "jaw_opening": 1 - (get(14)+1)/2, "rail_y": config.RailY},
			"object":          map[string]any{"id": "object-1", "position": map[string]any{"x": objectX, "y": objectY}, "velocity": map[string]any{"x": denormalize(get(6), -config.MaxHorizontalSpeed, config.MaxHorizontalSpeed), "y": denormalize(get(7), -2*config.Gravity, 2*config.Gravity)}, "mass": config.InitialObjectMass, "friction": config.ObjectFriction, "break_force": config.ObjectBreakForce, "required_grip_force": selected.Info["required_grip_force"], "safety_margin": gripForce - float64(selected.Info["required_grip_force"]), "status": status},
			"target":          map[string]any{"position_x": targetX, "position_y": targetY, "width": config.TargetWidth},
			"workspace":       map[string]any{"minX": config.Workspace.MinX, "maxX": config.Workspace.MaxX, "minY": config.Workspace.MinY, "maxY": config.Workspace.MaxY, "safeMinX": safeBounds.MinX, "safeMaxX": safeBounds.MaxX, "safeMinY": safeBounds.MinY, "safeMaxY": safeBounds.MaxY, "boundaryHit": boundaryHit, "coordinate_system_version": forcecontrol.CoordinateSystemVersion},
			"workspace_min_x": config.Workspace.MinX, "workspace_max_x": config.Workspace.MaxX, "workspace_min_y": config.Workspace.MinY, "workspace_max_y": config.Workspace.MaxY,
			"terrain":                map[string]any{"points": terrainPoints},
			"last_action":            map[string]any{"horizontal": lastAction[0], "vertical": lastAction[1], "gripper": lastAction[2], "filtered_horizontal": selected.Info["filtered_action_horizontal"], "filtered_vertical": selected.Info["filtered_action_vertical"], "filtered_gripper": filteredForceRateCommand, "dead_zone_removed_horizontal": selected.Info["dead_zone_removed_horizontal"] > 0, "dead_zone_removed_vertical": selected.Info["dead_zone_removed_vertical"] > 0, "dead_zone_removed_gripper": selected.Info["dead_zone_removed_gripper"] > 0, "force_rate_command": forceRateCommand, "force_rate_newtons_per_second": float64(filteredForceRateCommand) * config.MaxGripForceRate, "force_action_mode": forceActionMode, "normalized_grip_force": lastAction[2]},
			"control":                map[string]any{"dt": selected.Info["control_timestep"], "phase_numeric": selected.Info["phase_numeric"], "raw_horizontal_action": selected.Info["raw_action_horizontal"], "raw_vertical_action": selected.Info["raw_action_vertical"], "raw_gripper_action": selected.Info["raw_action_gripper"], "filtered_horizontal_action": selected.Info["filtered_action_horizontal"], "filtered_vertical_action": selected.Info["filtered_action_vertical"], "filtered_gripper_action": selected.Info["filtered_action_gripper"], "carriage_x": selected.Info["carriage_x"], "gripper_y": selected.Info["gripper_y"], "velocity_x": selected.Info["carriage_velocity_x"], "velocity_y": selected.Info["gripper_velocity_y"], "error_x": selected.Info["gripper_to_object_error_x"], "error_y": selected.Info["gripper_to_object_error_y"], "vertical_acceleration": selected.Info["vertical_acceleration"], "boundary_hit": selected.Info["boundary_hit"] > 0, "invalid_contact_frames": selected.Info["invalid_contact_frames"], "slip_severity": selected.Info["slip_severity"], "slip_frames": selected.Info["slip_frames"]},
			"latest_vertical_action": lastAction[1], "velocity_y": denormalize(get(3), -config.MaxVerticalSpeed, config.MaxVerticalSpeed), "target_grasp_y": targetGraspY, "vertical_error": targetGraspY - gripperY,
			"last_reward": selected.LastReward, "cumulative_reward": selected.EpisodeReward, "distance_to_object": selected.Info["gripper_to_object_distance"], "distance_to_target": selected.Info["object_to_target_distance"],
			"contact_detected": contactDetected, "object_attached": objectAttached, "object_stable": selected.Info["object_stable"] > 0, "slipping": selected.Info["slipping"] > 0,
			"contact_episode_successes": contactEpisodeSuccesses, "contact_episode_successes_required": contactEpisodeSuccessesRequired,
			"grasp_hold_frames": graspHoldFrames, "grasp_hold_frames_required": graspHoldFramesRequired,
			"homeostasis":     map[string]any{"enabled": selected.Info["homeostasis_enabled"] > 0, "energy": selected.Info["energy"], "delta": selected.Info["energy_delta"], "decay": selected.Info["energy_decay"], "food_gain": selected.Info["energy_food_gain"], "reward": selected.Info["homeostasis_reward"], "event": energyEvent},
			"approach_reward": selected.Info["approach_reward"], "contact_reward": selected.Info["contact_reward"], "grip_reward": selected.Info["grip_reward"], "lift_reward": selected.Info["lift_reward"], "delivery_reward": selected.Info["delivery_reward"], "success_reward": selected.Info["success_reward"], "detached_force_penalty": selected.Info["detached_force_penalty"], "penalty_reward": selected.Info["penalty_reward"], "total_step_reward": selected.Info["total_step_reward"], "done": selected.Outcome != framework.OutcomeRunning, "outcome": selected.Outcome,
		}
	}
	return map[string]any{"type": "simulation_snapshot", "timestamp": time.Now().UTC().Format(time.RFC3339Nano), "runtime": map[string]any{"status": snapshot.Status, "mode": "swarm", "active_workers": snapshot.ActiveWorkers, "steps_per_second": snapshot.StepsPerSecond, "episodes_per_second": snapshot.EpisodesPerSecond, "total_steps": snapshot.TotalSteps, "total_episodes": snapshot.TotalEpisodes, "success_rate": snapshot.SuccessRate, "contact_rate": snapshot.ContactRate, "attachment_rate": snapshot.AttachmentRate, "average_reward": snapshot.AverageReward, "replay_buffer_size": snapshot.ReplayBufferSize, "training_batches": snapshot.TrainingBatches, "policy_version": snapshot.PolicyVersion, "training_step": snapshot.TrainingStep, "actor_loss": snapshot.ActorLoss, "critic_loss": snapshot.CriticLoss, "alpha_loss": snapshot.AlphaLoss, "entropy": snapshot.Entropy, "critic_one_q": snapshot.CriticOneQ, "critic_two_q": snapshot.CriticTwoQ, "alpha": snapshot.Alpha, "actor_log_std": []float32{snapshot.ActorLogStdHorizontal, snapshot.ActorLogStdVertical, snapshot.ActorLogStdGripper}, "action_statistics": snapshot.ActionStatistics, "phase_occupancy": snapshot.PhaseOccupancy, "failure_reasons": snapshot.FailureReasons, "last_error": snapshot.LastError}, "worker": worker}
}

func energyEventLabel(code float32) string {
	switch int(code) {
	case 1:
		return "Secure grasp: energy restored"
	case 2:
		return "Lift milestone: energy restored"
	case 3:
		return "Target reached: energy restored"
	case 4:
		return "Successful placement: full meal"
	case 5:
		return "Unsafe drop: energy lost"
	case 6:
		return "Object break: energy lost"
	default:
		return ""
	}
}

// curriculumStageLabel converts the environment's compact Info value into the
// active lesson name. In automatic mode this is deliberately not the static
// launch setting ("auto"), so the dashboard shows the real current lesson.
func curriculumStageLabel(code float32) string {
	switch int(code) {
	case 1:
		return "align-and-contact"
	case 2:
		return "grasp"
	case 3:
		return "lift"
	case 4:
		return "transport-and-release"
	case 5:
		return "full-pick-and-place"
	default:
		return ""
	}
}

func objectStatus(phase forcecontrol.Phase, gripperClosed, contactDetected, attached, released, broken bool) string {
	switch phase {
	case forcecontrol.PhaseSuccess:
		return "placed"
	}
	if broken {
		return "broken"
	}
	if attached {
		if phase == forcecontrol.PhaseMoveToTarget || phase == forcecontrol.PhaseLowerAtTarget || phase == forcecontrol.PhaseLiftObject {
			return "transported"
		}
		return "attached"
	}
	if contactDetected && gripperClosed {
		return "grasping"
	}
	if released {
		return "released"
	}
	return "idle"
}

func ternary(condition bool, whenTrue, whenFalse string) string {
	if condition {
		return whenTrue
	}
	return whenFalse
}

func (s *APIServer) writeJSON(w http.ResponseWriter, status int, value any) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	_ = json.NewEncoder(w).Encode(value)
}
func (s *APIServer) writeError(w http.ResponseWriter, status int, code, message string) {
	s.writeJSON(w, status, map[string]any{"error": map[string]string{"code": code, "message": message}})
}
func finite(value float64) bool { return !math.IsNaN(value) && !math.IsInf(value, 0) }
func denormalize(value, minimum, maximum float64) float64 {
	return minimum + (value+1)*(maximum-minimum)/2
}
func terrainAt(config forcecontrol.Config, x float64) float64 {
	if len(config.Terrain) == 0 {
		return config.Workspace.MinY
	}
	if x <= config.Terrain[0].X {
		return config.Terrain[0].Y
	}
	for i := 1; i < len(config.Terrain); i++ {
		if x <= config.Terrain[i].X {
			left, right := config.Terrain[i-1], config.Terrain[i]
			return left.Y + (x-left.X)*(right.Y-left.Y)/(right.X-left.X)
		}
	}
	return config.Terrain[len(config.Terrain)-1].Y
}
func requiredForce(config forcecontrol.Config) float64 {
	return config.InitialObjectMass * config.Gravity / (2 * config.ObjectFriction)
}
