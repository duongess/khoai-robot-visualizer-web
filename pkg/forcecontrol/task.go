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
	return framework.StepResult{State: t.environment.observation(), Reward: float32(reward), Outcome: outcome, Done: done}, nil
}
