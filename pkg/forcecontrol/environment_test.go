package forcecontrol

import (
	"math"
	"testing"

	"github.com/duongess/khoai-robot-control-framework/pkg/framework"
)

func TestTaskResetIsDeterministic(t *testing.T) {
	first := NewTask(42, DefaultConfig())
	second := NewTask(42, DefaultConfig())
	firstState, err := first.Reset()
	if err != nil {
		t.Fatal(err)
	}
	secondState, err := second.Reset()
	if err != nil {
		t.Fatal(err)
	}
	if len(firstState) != 19 || !equalStates(firstState, secondState) {
		t.Fatalf("reset states differ: %#v and %#v", firstState, secondState)
	}
}

func TestTaskValidatesAndClampsActions(t *testing.T) {
	task := NewTask(1, DefaultConfig())
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}
	if _, err := task.Step(framework.Action{0, 1}); err == nil {
		t.Fatal("expected action dimension error")
	}
	if _, err := task.Step(framework.Action{float32(math.NaN()), 0, 0}); err == nil {
		t.Fatal("expected non-finite action error")
	}
	if _, err := task.Step(framework.Action{2, -2, 2}); err != nil {
		t.Fatalf("clamped action returned error: %v", err)
	}
}

func TestTaskKeepsObservationsNormalized(t *testing.T) {
	task := NewTask(3, DefaultConfig())
	state, err := task.Reset()
	if err != nil {
		t.Fatal(err)
	}
	for step := 0; step < 10; step++ {
		for i, value := range state {
			if value < -1 || value > 1 || math.IsNaN(float64(value)) || math.IsInf(float64(value), 0) {
				t.Fatalf("state[%d] = %v is not normalized", i, value)
			}
		}
		result, stepErr := task.Step(framework.Action{1, 1, -1})
		if stepErr != nil {
			t.Fatal(stepErr)
		}
		state = result.State
	}
}

func TestHorizontalAndVerticalControlsMoveWithinBounds(t *testing.T) {
	config := DefaultConfig()
	task := NewTask(1, config)
	initial, err := task.Reset()
	if err != nil {
		t.Fatal(err)
	}
	left, err := task.Step(framework.Action{-1, -1, -1})
	if err != nil {
		t.Fatal(err)
	}
	if left.State[0] >= initial[0] || left.State[1] >= initial[1] {
		t.Fatalf("negative controls did not move left/down: initial=%v next=%v", initial, left.State)
	}
	right, err := task.Step(framework.Action{1, 1, 1})
	if err != nil {
		t.Fatal(err)
	}
	if right.State[0] <= left.State[0] || right.State[1] <= left.State[1] {
		t.Fatalf("positive controls did not move right/up: left=%v next=%v", left.State, right.State)
	}
}

func TestGripControlSupportsSafeGraspAndBreakage(t *testing.T) {
	config := DefaultConfig()
	config.ObjectBreakForce = 18
	config.InitialGripperY = config.Terrain[1].Y + config.ObjectHeight
	task := NewTask(1, config)
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}
	result, err := task.Step(framework.Action{0, 0, 0.4})
	if err != nil {
		t.Fatal(err)
	}
	if result.Outcome != OutcomeRunning || !task.environment.state.ObjectGrasped {
		t.Fatalf("safe grip did not grasp object: result=%#v state=%#v", result, task.environment.state)
	}

	brokenTask := NewTask(1, config)
	if _, err := brokenTask.Reset(); err != nil {
		t.Fatal(err)
	}
	result, err = brokenTask.Step(framework.Action{0, 0, 1})
	if err != nil {
		t.Fatal(err)
	}
	if result.Outcome != OutcomeFailure || !result.Done || !brokenTask.environment.state.ObjectBroken {
		t.Fatalf("excessive grip did not break object: result=%#v state=%#v", result, brokenTask.environment.state)
	}
}

func TestTaskOutcomes(t *testing.T) {
	config := DefaultConfig()
	config.MaxEpisodeSteps = 1
	timeoutTask := NewTask(1, config)
	if _, err := timeoutTask.Reset(); err != nil {
		t.Fatal(err)
	}
	timeout, err := timeoutTask.Step(framework.Action{0, 0, -1})
	if err != nil || timeout.Outcome != OutcomeFailure || !timeout.Done {
		t.Fatalf("timeout outcome = %#v, error = %v", timeout, err)
	}

	placementConfig := DefaultConfig()
	placementTask := NewTask(1, placementConfig)
	if _, err := placementTask.Reset(); err != nil {
		t.Fatal(err)
	}
	placementTask.environment.state.ObjectX = placementConfig.TargetX
	placementTask.environment.state.ObjectY = placementTask.environment.terrainHeight(placementConfig.TargetX) + placementConfig.ObjectHeight/2 + 0.01
	placementTask.environment.state.CarriageX = placementConfig.TargetX
	placementTask.environment.state.GripperY = placementTask.environment.state.ObjectY + placementConfig.ObjectHeight/2
	placementTask.environment.state.ObjectGrasped = true
	placement, err := placementTask.Step(framework.Action{0, 0, -1})
	if err != nil || placement.Outcome != OutcomeSuccess || !placement.Done {
		t.Fatalf("placement outcome = %#v, error = %v", placement, err)
	}
}

func TestTasksAreIsolated(t *testing.T) {
	config := DefaultConfig()
	first := NewTask(1, config)
	second := NewTask(1, config)
	if _, err := first.Reset(); err != nil {
		t.Fatal(err)
	}
	if _, err := second.Reset(); err != nil {
		t.Fatal(err)
	}
	if _, err := first.Step(framework.Action{1, 1, -1}); err != nil {
		t.Fatal(err)
	}
	secondAfter, err := second.Step(framework.Action{0, 0, -1})
	if err != nil {
		t.Fatal(err)
	}
	fresh := NewTask(1, config)
	if _, err := fresh.Reset(); err != nil {
		t.Fatal(err)
	}
	freshAfter, err := fresh.Step(framework.Action{0, 0, -1})
	if err != nil {
		t.Fatal(err)
	}
	if !equalStates(secondAfter.State, freshAfter.State) {
		t.Fatal("stepping one task affected another task")
	}
}

func TestRegisterRejectsDuplicateTask(t *testing.T) {
	runtime := framework.NewRuntime()
	config := DefaultConfig()
	if err := Register(runtime, config); err != nil {
		t.Fatal(err)
	}
	if err := Register(runtime, config); err == nil {
		t.Fatal("expected duplicate registration error")
	}
}

func equalStates(first, second framework.State) bool {
	if len(first) != len(second) {
		return false
	}
	for i := range first {
		if first[i] != second[i] {
			return false
		}
	}
	return true
}
