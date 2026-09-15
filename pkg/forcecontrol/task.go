package forcecontrol

import (
	"errors"

	"github.com/duongess/khoai-robot-control-framework/pkg/framework"
)

// Outcome is the framework outcome type used by this task.
type Outcome = framework.Outcome

// Outcome values describe the terminal or ongoing task result.
const (
	OutcomeRunning framework.Outcome = "running"
	OutcomeSuccess framework.Outcome = "success"
	OutcomeFailure framework.Outcome = "failure"
)

// Task adapts the physical environment to the control framework contract.
type Task struct {
	environment *Environment
	config      Config
}

var _ framework.Task = (*Task)(nil)

// NewTask creates an independent force-control task instance.
func NewTask(seed int64, config Config) *Task {
	return &Task{environment: newEnvironment(seed, config), config: config}
}

func (t *Task) Reset() (framework.State, error) {
	if t == nil || t.environment == nil {
		return nil, errors.New("force-control task is not initialized")
	}
	t.environment.reset()
	if err := t.environment.ValidateState(); err != nil {
		return nil, err
	}
	return t.environment.observation(), nil
}

func (t *Task) Step(action framework.Action) (framework.StepResult, error) {
	if t == nil || t.environment == nil {
		return framework.StepResult{}, errors.New("force-control task is not initialized")
	}
	_, reward, outcome, done, err := t.environment.step(action)
	if err != nil {
		return framework.StepResult{}, err
	}
	state, breakdown := t.environment.state, t.environment.lastReward
	info := map[string]float32{
		"raw_action_horizontal":      actionValue(action, 0),
		"raw_action_vertical":        actionValue(action, 1),
		"raw_action_gripper":         actionValue(action, 2),
		"filtered_action_horizontal": float32(t.environment.filteredAction[0]),
		"filtered_action_vertical":   float32(t.environment.filteredAction[1]),
		"filtered_action_gripper":    float32(t.environment.filteredAction[2]),
		"control_timestep":           float32(t.config.TimeStep),
		"carriage_x":                 float32(state.CarriageX),
		"gripper_y":                  float32(state.GripperY),
		"carriage_velocity_x":        float32(state.CarriageVelocityX),
		"gripper_velocity_y":         float32(state.GripperVelocityY),
		"gripper_to_object_error_x":  float32(state.ObjectX - state.CarriageX),
		"gripper_to_object_error_y":  float32(t.environment.objectGripHeightFor(state) - state.GripperY),
		"vertical_acceleration":      float32(t.environment.verticalAcceleration),
		"required_grip_force":        float32(t.environment.requiredForce()),
		"invalid_contact_frames":     float32(t.environment.invalidContactFrames),
		"slip_frames":                float32(t.environment.slipFrames),
		"slip_severity":              float32(t.environment.slipSeverity()),
		"slipping":                   float32(boolToFloat(state.Grip.Slipping)),
		"boundary_hit":               float32(boolToFloat(state.BoundaryHit)),
		"coordinate_system_version":  CoordinateSystemVersion,
		"gripper_closed":             float32(boolToFloat(state.Grip.GripperClosed)),
		"contact_detected":           float32(boolToFloat(state.Grip.ContactDetected)),
		"force_valid":                float32(boolToFloat(state.Grip.ForceValid)),
		"object_attached":            float32(boolToFloat(state.Grip.ObjectAttached)),
		"object_released":            float32(boolToFloat(t.environment.wasEverGrasped && !state.Grip.ObjectAttached)),
		"object_broken":              float32(boolToFloat(state.ObjectBroken)),
		"object_stable":              float32(boolToFloat(t.environment.objectStable())),
		"gripper_to_object_distance": float32(gripperObjectDistance(state)),
		"object_to_target_distance":  float32(targetDistance(state)),
		"approach_reward":            float32(breakdown.Approach),
		"grip_reward":                float32(breakdown.Grip),
		"lift_reward":                float32(breakdown.Lift),
		"delivery_reward":            float32(breakdown.Delivery),
		"success_reward":             float32(breakdown.Success),
		"penalty_reward":             float32(breakdown.Penalty),
		"total_step_reward":          float32(breakdown.Total),
	}
	return framework.StepResult{State: t.environment.observation(), Reward: float32(reward), Outcome: outcome, Done: done, Info: info}, nil
}

func actionValue(action framework.Action, index int) float32 {
	if index >= len(action) {
		return 0
	}
	return action[index]
}

func boolToFloat(value bool) float64 {
	if value {
		return 1
	}
	return 0
}
