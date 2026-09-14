package forcecontrol

import (
	"math"
	"testing"

	"github.com/duongess/khoai-robot-control-framework/pkg/framework"
)

func TestResetStartsActiveEpisodeAndGoalConditionedObservation(t *testing.T) {
	task := NewTask(42, DefaultConfig())
	observation, err := task.Reset()
	if err != nil {
		t.Fatal(err)
	}
	if task.environment.state.Phase != PhaseApproachObject {
		t.Fatalf("reset phase = %s, want %s", task.environment.state.Phase, PhaseApproachObject)
	}
	if len(observation) != ObservationDimension {
		t.Fatalf("observation dimension = %d, want %d", len(observation), ObservationDimension)
	}
	if observation[observationTargetX] == observation[observationObjectX] || observation[observationTargetFromObjectX] == 0 {
		t.Fatalf("observation is missing distinct object/target information: %v", observation)
	}
	if PhaseFromNormalized(observation[observationPhase]) != PhaseApproachObject {
		t.Fatalf("observation phase = %v", observation[observationPhase])
	}
	for index, value := range observation {
		if value < -1 || value > 1 || math.IsNaN(float64(value)) || math.IsInf(float64(value), 0) {
			t.Fatalf("observation[%d]=%v is not finite and normalized", index, value)
		}
	}
}

func TestKnownActionsMoveGripperAndAreClamped(t *testing.T) {
	task := NewTask(1, DefaultConfig())
	initial, err := task.Reset()
	if err != nil {
		t.Fatal(err)
	}
	left, err := task.Step(framework.Action{-1, -1, -1})
	if err != nil {
		t.Fatal(err)
	}
	if left.State[observationGripperX] >= initial[observationGripperX] || left.State[observationGripperY] >= initial[observationGripperY] {
		t.Fatal("known negative action did not move left/down")
	}
	if _, err := task.Step(framework.Action{2, -2, 2}); err != nil {
		t.Fatalf("clamped action returned %v", err)
	}
	if _, err := task.Step(framework.Action{float32(math.NaN()), 0, 0}); err == nil {
		t.Fatal("expected invalid action rejection")
	}
}

func TestApproachProgressRewardsTowardMovement(t *testing.T) {
	config := DefaultConfig()
	config.InitialCarriageX = 0.5
	toward := NewTask(1, config)
	away := NewTask(1, config)
	_, _ = toward.Reset()
	_, _ = away.Reset()
	towardResult, err := toward.Step(framework.Action{1, 0, -1})
	if err != nil {
		t.Fatal(err)
	}
	awayResult, err := away.Step(framework.Action{-1, 0, -1})
	if err != nil {
		t.Fatal(err)
	}
	if towardResult.Reward <= awayResult.Reward {
		t.Fatalf("toward reward %v <= away reward %v", towardResult.Reward, awayResult.Reward)
	}
}

func TestGripBonusIsOneTimeAndExcessiveForceFails(t *testing.T) {
	config := DefaultConfig()
	config.InitialGripperY = GraspHeight(config, config.Terrain[1].Y+config.ObjectHeight/2, config.InitialCarriageX)
	task := NewTask(1, config)
	_, _ = task.Reset()
	grip, err := task.Step(framework.Action{0, 0, 0.5})
	if err != nil || !task.environment.state.ObjectGrasped {
		t.Fatalf("safe grip result=%#v error=%v", grip, err)
	}
	again, err := task.Step(framework.Action{0, 0, 0.5})
	if err != nil {
		t.Fatal(err)
	}
	if grip.Reward-again.Reward < float32(config.Reward.SuccessfulGripReward)-0.01 {
		t.Fatalf("grip bonus was repeated: first=%v second=%v", grip.Reward, again.Reward)
	}

	broken := NewTask(1, config)
	_, _ = broken.Reset()
	result, err := broken.Step(framework.Action{0, 0, 1})
	if err != nil || !result.Done || result.Outcome != OutcomeFailure {
		t.Fatalf("break result=%#v error=%v", result, err)
	}
}

func TestDeliveryProgressAndReleaseOutcomes(t *testing.T) {
	task := NewTask(1, DefaultConfig())
	_, _ = task.Reset()
	advanceToPhase(t, task, PhaseMoveToTarget)
	progress, err := task.Step(framework.Action{1, 0, 0.5})
	if err != nil || progress.Reward <= 0 {
		t.Fatalf("delivery progress=%#v error=%v", progress, err)
	}

	outside := NewTask(1, DefaultConfig())
	_, _ = outside.Reset()
	outside.environment.state.ObjectGrasped = true
	outside.environment.state.Phase = PhaseReleaseObject
	outside.environment.state.CarriageX = 1
	outside.environment.state.ObjectX = 1
	outside.environment.state.GripperY = outside.environment.targetRestHeight()
	outside.environment.state.ObjectY = outside.environment.targetRestHeight() - outside.environment.config.ObjectHeight/2
	result, err := outside.Step(framework.Action{0, 0, -1})
	if err != nil {
		t.Fatal(err)
	}
	if result.Outcome == OutcomeSuccess || result.Done && result.Outcome == OutcomeSuccess {
		t.Fatalf("release outside target reported success: %#v", result)
	}
}

func TestTimeoutAndResetClearEpisodeState(t *testing.T) {
	config := DefaultConfig()
	config.MaxEpisodeSteps = 1
	task := NewTask(1, config)
	_, _ = task.Reset()
	result, err := task.Step(framework.Action{0, 0, -1})
	if err != nil || !result.Done || result.Outcome != OutcomeFailure {
		t.Fatalf("timeout=%#v error=%v", result, err)
	}
	task.environment.gripBonusAwarded, task.environment.wasEverGrasped, task.environment.failureReason = true, true, "timeout"
	_, _ = task.Reset()
	if task.environment.gripBonusAwarded || task.environment.wasEverGrasped || task.environment.failureReason != "" || task.environment.state.EpisodeStep != 0 || task.environment.state.Phase != PhaseApproachObject {
		t.Fatalf("reset retained episode state: %#v", task.environment)
	}
}

func TestVerticalDirectionAndPhysicalWorkspaceClipping(t *testing.T) {
	config := DefaultConfig()
	config.InitialGripperY = SafeGripperBounds(config, config.InitialCarriageX).MaxY
	task := NewTask(1, config)
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}
	top := task.environment.state.GripperY
	up, err := task.Step(framework.Action{0, 1, -1})
	if err != nil {
		t.Fatal(err)
	}
	if task.environment.state.GripperY != top || task.environment.state.GripperVelocityY != 0 || !task.environment.state.BoundaryHit || up.Info["boundary_hit"] != 1 {
		t.Fatalf("positive Y should clip at top: state=%#v info=%v", task.environment.state, up.Info)
	}
	down, err := task.Step(framework.Action{0, -1, -1})
	if err != nil || task.environment.state.GripperY >= top || task.environment.state.GripperVelocityY >= 0 || down.Info["boundary_hit"] != 0 {
		t.Fatalf("negative Y should move down: state=%#v result=%#v err=%v", task.environment.state, down, err)
	}

	for step := 0; step < 100; step++ {
		if _, err := task.Step(framework.Action{0, -1, -1}); err != nil {
			t.Fatal(err)
		}
	}
	bounds := SafeGripperBounds(config, task.environment.state.CarriageX)
	if task.environment.state.GripperY != bounds.MinY || task.environment.state.GripperVelocityY != 0 || !task.environment.state.BoundaryHit {
		t.Fatalf("negative Y escaped lower bound: state=%#v bounds=%#v", task.environment.state, bounds)
	}
}

func TestHorizontalMovementCannotLeavePhysicalWorkspace(t *testing.T) {
	config := DefaultConfig()
	config.InitialCarriageX = SafeGripperBounds(config, config.InitialCarriageX).MaxX
	task := NewTask(1, config)
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}
	right, err := task.Step(framework.Action{1, 0, -1})
	if err != nil {
		t.Fatal(err)
	}
	bounds := SafeGripperBounds(config, task.environment.state.CarriageX)
	if task.environment.state.CarriageX != bounds.MaxX || task.environment.state.CarriageVelocityX != 0 || right.Info["boundary_hit"] != 1 {
		t.Fatalf("positive X escaped upper bound: state=%#v bounds=%#v", task.environment.state, bounds)
	}
}

func TestLoweringTowardReachableGraspHeightRewardsProgress(t *testing.T) {
	config := DefaultConfig()
	config.InitialCarriageX = config.InitialObjectX
	config.InitialGripperY = 1.2
	toward, away := NewTask(1, config), NewTask(1, config)
	_, _ = toward.Reset()
	_, _ = away.Reset()
	toward.environment.state.Phase = PhaseLowerToObject
	away.environment.state.Phase = PhaseLowerToObject
	down, err := toward.Step(framework.Action{0, -1, -1})
	if err != nil {
		t.Fatal(err)
	}
	up, err := away.Step(framework.Action{0, 1, -1})
	if err != nil {
		t.Fatal(err)
	}
	if down.Reward <= up.Reward {
		t.Fatalf("lowering reward %v <= upward reward %v", down.Reward, up.Reward)
	}
}

func TestInvalidInitialStateIsRejected(t *testing.T) {
	config := DefaultConfig()
	config.InitialGripperY = config.Workspace.MaxY
	if err := Register(framework.NewRuntime(), config); err == nil {
		t.Fatal("expected initial gripper outside physical bounds to be rejected")
	}
}

func TestCheckpointSchemaRejectsPriorCoordinateSemantics(t *testing.T) {
	if err := ValidateCheckpointSchema(CoordinateSystemVersion - 1); err == nil {
		t.Fatal("expected incompatible checkpoint schema to be rejected")
	}
	if err := ValidateCheckpointSchema(CoordinateSystemVersion); err != nil {
		t.Fatalf("current checkpoint schema rejected: %v", err)
	}
}

func TestScriptedPickAndPlaceSucceeds(t *testing.T) {
	task := NewTask(1, DefaultConfig())
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}
	advanceToPhase(t, task, PhaseReleaseObject)
	result, err := task.Step(framework.Action{0, 0, -1})
	if err != nil {
		t.Fatal(err)
	}
	if !result.Done || result.Outcome != OutcomeSuccess || task.environment.state.Phase != PhaseSuccess {
		t.Fatalf("scripted episode did not succeed: result=%#v environment=%s", result, task.environment)
	}
}

func TestRegisterStillWorks(t *testing.T) {
	runtime := framework.NewRuntime()
	if err := Register(runtime, DefaultConfig()); err != nil {
		t.Fatal(err)
	}
	if err := Register(runtime, DefaultConfig()); err == nil {
		t.Fatal("expected duplicate registration error")
	}
}

func advanceToPhase(t *testing.T, task *Task, wanted Phase) {
	t.Helper()
	for step := 0; step < 200 && task.environment.state.Phase != wanted; step++ {
		phase := task.environment.state.Phase
		action := framework.Action{0, 0, 0.5}
		switch phase {
		case PhaseApproachObject:
			action = framework.Action{0, 0, -1}
		case PhaseLowerToObject:
			action = framework.Action{0, -1, -1}
		case PhaseGripObject:
			action = framework.Action{0, 0, 0.5}
		case PhaseLiftObject:
			action = framework.Action{0, 1, 0.5}
		case PhaseMoveToTarget:
			action = framework.Action{1, 0, 0.5}
		case PhaseLowerAtTarget:
			action = framework.Action{0, -1, 0.5}
		default:
			t.Fatalf("cannot advance from phase %s", phase)
		}
		result, err := task.Step(action)
		if err != nil {
			t.Fatal(err)
		}
		if result.Done {
			t.Fatalf("episode ended before %s: %#v", wanted, result)
		}
	}
	if task.environment.state.Phase != wanted {
		t.Fatalf("phase=%s, want %s", task.environment.state.Phase, wanted)
	}
}
