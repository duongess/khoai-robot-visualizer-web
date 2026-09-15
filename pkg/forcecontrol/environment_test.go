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

func TestHomeostasisEnergyDecaysOnceResetsAndStaysObservable(t *testing.T) {
	config := DefaultConfig()
	config.Homeostasis.InitialEnergy = 0.60
	config.Homeostasis.EnergyDecayPerStep = 0.01
	task := NewTask(42, config)
	observation, err := task.Reset()
	if err != nil {
		t.Fatal(err)
	}
	if task.environment.state.Energy != 0.60 || observation[observationEnergy] != float32(normalize01(0.60)) {
		t.Fatalf("reset energy was not authoritative/observable: state=%v observation=%v", task.environment.state.Energy, observation[observationEnergy])
	}
	result, err := task.Step(framework.Action{0, 0, -1})
	if err != nil {
		t.Fatal(err)
	}
	if got, want := task.environment.state.Energy, 0.59; math.Abs(got-want) > 1e-9 || math.Abs(float64(result.Info["energy_delta"])+0.01) > 1e-6 || math.Abs(float64(result.Info["energy_decay"])+0.01) > 1e-6 || result.Info["energy_food_gain"] != 0 || math.Abs(float64(result.Info["homeostasis_reward"])+0.01) > 1e-6 {
		t.Fatalf("energy did not decay exactly once: state=%v info=%#v", got, result.Info)
	}
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}
	if task.environment.state.Energy != config.Homeostasis.InitialEnergy || task.environment.gripFoodAwarded || task.environment.liftFoodAwarded || task.environment.deliveryFoodAwarded || task.environment.successFoodAwarded {
		t.Fatalf("reset did not clear homeostasis state: %#v", task.environment)
	}
}

func TestHomeostasisMilestonesAreVerifiedOneTimeAndClamped(t *testing.T) {
	task := attachedTask(t, DefaultConfig())
	if !task.environment.gripFoodAwarded {
		t.Fatal("physical secure grasp did not restore energy")
	}
	energyAfterGrip := task.environment.state.Energy
	stillAttached, err := task.Step(framework.Action{0, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	if stillAttached.Info["energy_food_gain"] != 0 || task.environment.state.Energy >= energyAfterGrip {
		t.Fatalf("secure grasp food was farmed: %#v", stillAttached.Info)
	}

	task.environment.state.Phase = PhaseLiftObject
	for step := 0; step < 30 && !task.environment.liftFoodAwarded; step++ {
		if _, err := task.Step(framework.Action{0, 1, 0}); err != nil {
			t.Fatal(err)
		}
	}
	if !task.environment.liftFoodAwarded {
		t.Fatal("verified lift did not restore energy")
	}
	task.environment.state.Phase = PhaseMoveToTarget
	for step := 0; step < 40 && !task.environment.deliveryFoodAwarded; step++ {
		if _, err := task.Step(framework.Action{1, 0, 0}); err != nil {
			t.Fatal(err)
		}
	}
	if !task.environment.deliveryFoodAwarded {
		t.Fatal("verified attached delivery did not restore energy")
	}

	// Clamp at the upper and lower physical energy limits, even when a test
	// supplies an otherwise valid milestone/failure state.
	task.environment.state.Energy = 0.99
	task.environment.gripFoodAwarded = false
	task.environment.deliveryFoodAwarded = true
	task.environment.liftFoodAwarded = true
	task.environment.state.Grip.ObjectAttached = true
	task.environment.state.Grip.Slipping = false
	task.environment.state.ObjectX = task.environment.state.CarriageX
	task.environment.state.ObjectY = task.environment.state.GripperY - task.config.ObjectHeight/2
	task.environment.updateHomeostasis("")
	if task.environment.state.Energy != 1 {
		t.Fatalf("energy upper clamp failed: %v", task.environment.state.Energy)
	}
	task.environment.state.Energy = 0.05
	task.environment.updateHomeostasis("object_break")
	if task.environment.state.Energy != 0 {
		t.Fatalf("energy lower clamp failed: %v", task.environment.state.Energy)
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
	var grip framework.StepResult
	var err error
	for step := 0; step < 30 && !task.environment.state.ObjectGrasped; step++ {
		grip, err = task.Step(framework.Action{0, 0, 0.5})
		if err != nil {
			t.Fatal(err)
		}
	}
	if !task.environment.state.ObjectGrasped {
		t.Fatalf("safe grip did not reach attachment force: %#v", grip)
	}
	again, err := task.Step(framework.Action{0, 0, 0.5})
	if err != nil {
		t.Fatal(err)
	}
	if grip.Reward-again.Reward < float32(config.Reward.SuccessfulGripReward)-0.01 {
		t.Fatalf("grip bonus was repeated: first=%v second=%v", grip.Reward, again.Reward)
	}

	broken := attachedTask(t, config)
	broken.environment.state.GripForce = broken.environment.state.ObjectBreakForce
	previousEnergy := broken.environment.state.Energy
	result, err := broken.Step(framework.Action{0, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	if !result.Done || result.Outcome != OutcomeFailure {
		t.Fatalf("break result=%#v error=%v", result, err)
	}
	if got, want := broken.environment.state.Energy, previousEnergy-config.Homeostasis.EnergyDecayPerStep-config.Homeostasis.BreakEnergyLoss; math.Abs(got-want) > 1e-9 || result.Info["energy_event_code"] != float32(energyEventBreak) {
		t.Fatalf("break energy loss mismatch: got=%v want=%v info=%#v", got, want, result.Info)
	}
}

func TestDeliveryProgressAndReleaseOutcomes(t *testing.T) {
	task := NewTask(1, DefaultConfig())
	_, _ = task.Reset()
	advanceToPhase(t, task, PhaseMoveToTarget)
	// The previous lift command decelerates under the low-level controller;
	// settle it before asserting horizontal delivery progress.
	for step := 0; step < 10 && task.environment.state.GripperVelocityY > 0; step++ {
		if _, err := task.Step(framework.Action{0, -1, 0}); err != nil {
			t.Fatal(err)
		}
	}
	progress, err := task.Step(framework.Action{1, 0, 0.5})
	if err != nil || progress.Reward <= 0 {
		t.Fatalf("delivery progress=%#v error=%v", progress, err)
	}

	outside := NewTask(1, DefaultConfig())
	_, _ = outside.Reset()
	outside.environment.state.Grip = GripState{GripperClosed: true, ContactDetected: true, ForceValid: true, ObjectAttached: true}
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
	task.environment.gripBonusAwarded, task.environment.wasEverGrasped, task.environment.invalidGripPenaltyAwarded, task.environment.insufficientGripPenaltyAwarded, task.environment.emptyTargetPenaltyAwarded, task.environment.successRewardAwarded, task.environment.failureReason = true, true, true, true, true, true, "timeout"
	_, _ = task.Reset()
	if task.environment.gripBonusAwarded || task.environment.wasEverGrasped || task.environment.invalidGripPenaltyAwarded || task.environment.insufficientGripPenaltyAwarded || task.environment.emptyTargetPenaltyAwarded || task.environment.successRewardAwarded || task.environment.failureReason != "" || task.environment.state.EpisodeStep != 0 || task.environment.state.Phase != PhaseApproachObject || task.environment.state.Grip.ObjectAttached {
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

func TestFirstNegativeVerticalActionDescendsInWorldCoordinates(t *testing.T) {
	task := NewTask(1, DefaultConfig())
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}
	initialY := task.environment.state.GripperY
	result, err := task.Step(framework.Action{0, -1, -1})
	if err != nil {
		t.Fatal(err)
	}
	if got, want := task.environment.state.GripperVelocityY, -0.3; math.Abs(got-want) > 1e-9 {
		t.Fatalf("first negative vertical velocity = %v, want %v", got, want)
	}
	if got, want := task.environment.state.GripperY, initialY-0.03; math.Abs(got-want) > 1e-9 {
		t.Fatalf("first negative vertical position = %v, want %v", got, want)
	}
	if result.Info["raw_action_vertical"] != -1 || math.Abs(float64(result.Info["filtered_action_vertical"])-(-0.3)) > 1e-6 || result.Info["phase_numeric"] != float32(PhaseLowerToObject) {
		t.Fatalf("vertical action or phase telemetry is incorrect: %#v", result.Info)
	}
	for step := 0; step < 20; step++ {
		if _, err := task.Step(framework.Action{0, 0, -1}); err != nil {
			t.Fatal(err)
		}
	}
	if math.Abs(task.environment.state.GripperVelocityY) > 1e-6 {
		t.Fatalf("zero vertical actions did not settle velocity: %v", task.environment.state.GripperVelocityY)
	}
}

func TestVerticalSignReversalDoesNotKeepAStaleUpwardCommand(t *testing.T) {
	task := NewTask(1, DefaultConfig())
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}
	if _, err := task.Step(framework.Action{0, 1, -1}); err != nil {
		t.Fatal(err)
	}
	before := task.environment.state.GripperY
	reversed, err := task.Step(framework.Action{0, -1, -1})
	if err != nil {
		t.Fatal(err)
	}
	if reversed.Info["filtered_action_vertical"] >= 0 || task.environment.state.GripperVelocityY > 0 || task.environment.state.GripperY > before {
		t.Fatalf("negative vertical reversal retained upward motion: state=%#v info=%#v", task.environment.state, reversed.Info)
	}
}

func TestIdleLoweringAndEmptyClosedGripperReceiveStepPenalties(t *testing.T) {
	idle := NewTask(1, DefaultConfig())
	if _, err := idle.Reset(); err != nil {
		t.Fatal(err)
	}
	idle.environment.state.Phase = PhaseLowerToObject
	still, err := idle.Step(framework.Action{0, 0, -1})
	if err != nil {
		t.Fatal(err)
	}
	if still.Info["penalty_reward"] >= float32(idle.config.Reward.TimePenalty) {
		t.Fatalf("idle lowering was not penalized: %#v", still.Info)
	}

	empty := NewTask(1, DefaultConfig())
	if _, err := empty.Reset(); err != nil {
		t.Fatal(err)
	}
	if _, err := empty.Step(framework.Action{0, 0, 0.5}); err != nil {
		t.Fatal(err)
	}
	continued, err := empty.Step(framework.Action{0, 0, 0.5})
	if err != nil {
		t.Fatal(err)
	}
	if continued.Info["penalty_reward"] >= float32(empty.config.Reward.TimePenalty+empty.config.Reward.EmptyGripStepPenalty) {
		t.Fatalf("continued empty closed grip was not penalized: %#v", continued.Info)
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

func TestRegistrationRejectsImpossibleSafeGripBand(t *testing.T) {
	config := DefaultConfig()
	config.ObjectBreakForce = config.MaxGripForce + 1
	if err := Register(framework.NewRuntime(), config); err == nil {
		t.Fatal("expected an impossible safe grip-force interval to be rejected")
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
	var result framework.StepResult
	var err error
	for step := 0; step < task.config.StablePlacementSteps+4; step++ {
		result, err = task.Step(framework.Action{0, 0, -1})
		if err != nil {
			t.Fatal(err)
		}
		if result.Done {
			break
		}
	}
	if !result.Done || result.Outcome != OutcomeSuccess || task.environment.state.Phase != PhaseSuccess {
		t.Fatalf("scripted episode did not succeed: result=%#v environment=%s", result, task.environment)
	}
	if !task.environment.gripFoodAwarded || !task.environment.liftFoodAwarded || !task.environment.deliveryFoodAwarded || !task.environment.successFoodAwarded || result.Info["energy_event_code"] != float32(energyEventSuccess) || result.Info["energy_food_gain"] <= 0 {
		t.Fatalf("verified completion did not award each homeostasis milestone once: flags=%t/%t/%t/%t info=%#v", task.environment.gripFoodAwarded, task.environment.liftFoodAwarded, task.environment.deliveryFoodAwarded, task.environment.successFoodAwarded, result.Info)
	}
}

func TestReleasedObjectMustStabilizeBeforeSuccess(t *testing.T) {
	task := NewTask(1, DefaultConfig())
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}
	advanceToPhase(t, task, PhaseReleaseObject)
	first, err := task.Step(framework.Action{0, 0, -1})
	if err != nil {
		t.Fatal(err)
	}
	if first.Done || task.environment.state.Phase == PhaseSuccess {
		t.Fatalf("release succeeded before the object stabilized: %#v", first)
	}
}

func TestEmptyGripperAtTargetCannotAttachEarnDeliveryOrSucceed(t *testing.T) {
	config := DefaultConfig()
	task := NewTask(1, config)
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}
	// Regression for the dashboard screenshot: travel right, close with a
	// numerically valid force, but never contact the left-side object.
	for step := 0; step < 30 && task.environment.state.CarriageX < config.TargetX; step++ {
		if _, err := task.Step(framework.Action{1, 0, -1}); err != nil {
			t.Fatal(err)
		}
	}
	result, err := task.Step(framework.Action{0, 0, 0.5})
	if err != nil {
		t.Fatal(err)
	}
	grip := task.environment.state.Grip
	if grip.ContactDetected || grip.ObjectAttached || task.environment.state.ObjectGrasped {
		t.Fatalf("empty target close created a grasp: %#v", grip)
	}
	if result.Info["delivery_reward"] != 0 || result.Info["grip_reward"] != 0 || result.Info["energy_food_gain"] != 0 || result.Outcome == OutcomeSuccess {
		t.Fatalf("empty gripper earned target reward or success: %#v", result)
	}
	if result.Info["penalty_reward"] >= float32(config.Reward.TimePenalty) {
		t.Fatalf("empty target did not receive its one-time penalty: %#v", result.Info)
	}
}

func TestInsufficientForceAndContactlessForceDoNotAttach(t *testing.T) {
	config := DefaultConfig()
	config.InitialGripperY = GraspHeight(config, config.Terrain[1].Y+config.ObjectHeight/2, config.InitialCarriageX)
	near := NewTask(1, config)
	_, _ = near.Reset()
	result, err := near.Step(framework.Action{0, 0, -0.2})
	if err != nil {
		t.Fatal(err)
	}
	if !near.environment.state.Grip.ContactDetected || near.environment.state.Grip.ObjectAttached || result.Info["grip_reward"] != 0 {
		t.Fatalf("insufficient force attached object: state=%#v result=%#v", near.environment.state.Grip, result)
	}

	far := NewTask(1, DefaultConfig())
	_, _ = far.Reset()
	result, err = far.Step(framework.Action{0, 0, 0.5})
	if err != nil {
		t.Fatal(err)
	}
	if far.environment.state.Grip.ContactDetected || far.environment.state.Grip.ObjectAttached || result.Info["grip_reward"] != 0 {
		t.Fatalf("force without contact attached object: state=%#v result=%#v", far.environment.state.Grip, result)
	}
}

func TestSustainedInsufficientContactForceIsNotAFreePolicy(t *testing.T) {
	config := DefaultConfig()
	config.InitialCarriageX = config.InitialObjectX
	config.InitialGripperY = GraspHeight(config, terrainHeightForConfig(config, config.InitialObjectX)+config.ObjectHeight/2, config.InitialCarriageX)
	task := NewTask(1, config)
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}
	task.environment.state.Phase = PhaseGripObject
	if _, err := task.Step(framework.Action{0, 0, -0.2}); err != nil {
		t.Fatal(err)
	}
	second, err := task.Step(framework.Action{0, 0, -0.2})
	if err != nil {
		t.Fatal(err)
	}
	if second.Info["penalty_reward"] >= float32(config.Reward.TimePenalty) || second.Reward >= 0 {
		t.Fatalf("sustained insufficient force should receive a step penalty: %#v", second)
	}
}

func TestGripForceHasBoundedSlewAndImmediateRelease(t *testing.T) {
	config := DefaultConfig()
	config.InitialCarriageX = config.InitialObjectX
	config.InitialGripperY = GraspHeight(config, terrainHeightForConfig(config, config.InitialObjectX)+config.ObjectHeight/2, config.InitialCarriageX)
	task := NewTask(1, config)
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}
	if _, err := task.Step(framework.Action{0, 0, 0.5}); err != nil {
		t.Fatal(err)
	}
	if got, limit := task.environment.state.GripForce, 0.5*config.MaxGripForceRate*config.TimeStep; math.Abs(got-limit) > 1e-9 {
		t.Fatalf("first grip-force increment = %v, want rate-limited %v", got, limit)
	}
	firstForce := task.environment.state.GripForce
	if _, err := task.Step(framework.Action{0, 0, -0.5}); err != nil {
		t.Fatal(err)
	}
	if task.environment.state.GripForce >= firstForce || !task.environment.state.Grip.GripperClosed {
		t.Fatalf("negative rate command did not reduce force while keeping the gripper closed: %#v", task.environment.state.Grip)
	}
	if _, err := task.Step(framework.Action{0, 0, -1}); err != nil {
		t.Fatal(err)
	}
	if task.environment.state.GripForce != 0 || task.environment.state.Grip.GripperClosed {
		t.Fatalf("release command did not immediately open: %#v", task.environment.state.Grip)
	}
}

func TestFixedGripRangeDoesNotChangeDuringLift(t *testing.T) {
	task := attachedTask(t, DefaultConfig())
	fixedRequiredForce := task.environment.requiredForce()
	for step := 0; step < 4; step++ {
		result, err := task.Step(framework.Action{0, 1, 0})
		if err != nil || result.Done {
			t.Fatalf("lift failed at step %d: result=%#v err=%v", step, result, err)
		}
		if task.environment.requiredForce() != fixedRequiredForce || result.Info["required_grip_force"] != float32(fixedRequiredForce) {
			t.Fatalf("required force changed during lift: got=%v want=%v", task.environment.requiredForce(), fixedRequiredForce)
		}
	}
	if task.environment.state.Grip.Slipping || !task.environment.state.Grip.ObjectAttached {
		t.Fatalf("fixed safe force should keep the object attached: %#v", task.environment.state)
	}
}

func TestExcessForceAndAbruptForceChangesArePenalized(t *testing.T) {
	low := attachedTask(t, DefaultConfig())
	high := attachedTask(t, DefaultConfig())
	high.environment.state.GripForce = high.environment.requiredForce() + 4
	lowResult, err := low.Step(framework.Action{0, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	highResult, err := high.Step(framework.Action{0, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	if highResult.Reward >= lowResult.Reward {
		t.Fatalf("excess force was not less rewarding: low=%v high=%v", lowResult.Reward, highResult.Reward)
	}
	changed, err := low.Step(framework.Action{0, 0, 1})
	if err != nil {
		t.Fatal(err)
	}
	if changed.Info["penalty_reward"] >= lowResult.Info["penalty_reward"] {
		t.Fatalf("abrupt force change was not penalized: steady=%#v changed=%#v", lowResult.Info, changed.Info)
	}
}

func TestMassAndFrictionChangePolicyForceDemandWithoutLeakingRequirement(t *testing.T) {
	lightConfig := DefaultConfig()
	heavyConfig := DefaultConfig()
	heavyConfig.InitialObjectMass = 0.95
	heavyConfig.ObjectFriction = 0.32
	for _, config := range []*Config{&lightConfig, &heavyConfig} {
		config.InitialCarriageX = config.InitialObjectX
		config.InitialGripperY = GraspHeight(*config, terrainHeightForConfig(*config, config.InitialObjectX)+config.ObjectHeight/2, config.InitialCarriageX)
	}
	light := NewTask(1, lightConfig)
	heavy := NewTask(1, heavyConfig)
	for _, task := range []*Task{light, heavy} {
		if _, err := task.Reset(); err != nil {
			t.Fatal(err)
		}
	}
	lightSteps, heavySteps := forceStepsToAttach(t, light), forceStepsToAttach(t, heavy)
	if heavySteps <= lightSteps {
		t.Fatalf("heavier/lower-friction object did not require more policy force steps: light=%d heavy=%d", lightSteps, heavySteps)
	}
	if light.environment.observation()[observationRequiredGripForceBaseline] != 0 || heavy.environment.observation()[observationRequiredGripForceBaseline] != 0 {
		t.Fatal("default policy observation leaked analytic required force")
	}
}

func TestContinuousGripDetachesOnlyForReleaseOrSustainedLoss(t *testing.T) {
	released := attachedTask(t, DefaultConfig())
	if _, err := released.Step(framework.Action{0, 0, -1}); err != nil {
		t.Fatal(err)
	}
	if released.environment.state.Grip.ObjectAttached || released.environment.state.GripForce != 0 {
		t.Fatalf("explicit release did not detach: %#v", released.environment.state)
	}

	insufficient := attachedTask(t, DefaultConfig())
	for step := 0; step < insufficient.config.SlipDetachFrames; step++ {
		if _, err := insufficient.Step(framework.Action{0, 1, -0.5}); err != nil {
			t.Fatal(err)
		}
	}
	if insufficient.environment.state.Grip.ObjectAttached {
		t.Fatal("persistent insufficient force did not detach")
	}

	contactLost := attachedTask(t, DefaultConfig())
	for frame := 0; frame < contactLost.config.GripDetachInvalidFrames; frame++ {
		contactLost.environment.state.ObjectX = contactLost.environment.state.CarriageX + 1
		if _, err := contactLost.Step(framework.Action{0, 0, 0}); err != nil {
			t.Fatal(err)
		}
		if frame+1 < contactLost.config.GripDetachInvalidFrames && !contactLost.environment.state.Grip.ObjectAttached {
			t.Fatalf("contact hysteresis detached after %d invalid frames", frame+1)
		}
	}
	if contactLost.environment.state.Grip.ObjectAttached {
		t.Fatal("sustained lost contact did not detach")
	}
}

func forceStepsToAttach(t *testing.T, task *Task) int {
	t.Helper()
	for step := 1; step <= 40; step++ {
		if _, err := task.Step(framework.Action{0, 0, 1}); err != nil {
			t.Fatal(err)
		}
		if task.environment.state.Grip.ObjectAttached {
			return step
		}
	}
	t.Fatalf("policy force did not attach object: %#v", task.environment.state)
	return 0
}

func TestMotionFilteringDeadZoneAndAccelerationBounds(t *testing.T) {
	config := DefaultConfig()
	config.ActionDeadZone = 0.03 // verify configurability independently of the lower default.
	config.InitialCarriageX = 3
	config.InitialGripperY = 2
	task := NewTask(1, config)
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}

	still, err := task.Step(framework.Action{0.02, -0.02, 0.02})
	if err != nil {
		t.Fatal(err)
	}
	if task.environment.state.CarriageVelocityX != 0 || task.environment.state.GripperVelocityY != 0 || still.Info["filtered_action_horizontal"] != 0 {
		t.Fatalf("dead-zone noise moved the gripper: state=%#v info=%#v", task.environment.state, still.Info)
	}

	forward, err := task.Step(framework.Action{1, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	maxDelta := config.MaxHorizontalAcceleration * config.TimeStep
	if got := task.environment.state.CarriageVelocityX; got <= 0 || got > maxDelta+1e-9 {
		t.Fatalf("forward velocity %v exceeds acceleration bound %v", got, maxDelta)
	}
	if got := forward.Info["filtered_action_horizontal"]; math.Abs(float64(got)-config.ActionSmoothingAlpha) > 1e-6 {
		t.Fatalf("filtered action = %v, want %v", got, config.ActionSmoothingAlpha)
	}
	previousVelocity := task.environment.state.CarriageVelocityX
	reverse, err := task.Step(framework.Action{-1, 0, 0})
	if err != nil {
		t.Fatal(err)
	}
	if delta := math.Abs(task.environment.state.CarriageVelocityX - previousVelocity); delta > maxDelta+1e-9 {
		t.Fatalf("reversal changed velocity by %v, limit %v", delta, maxDelta)
	}
	if task.environment.state.CarriageVelocityX < 0 {
		t.Fatalf("velocity reversed in one step near target: previous=%v current=%v info=%#v", previousVelocity, task.environment.state.CarriageVelocityX, reverse.Info)
	}
	if got := reverse.Info["control_timestep"]; math.Abs(float64(got)-config.TimeStep) > 1e-6 {
		t.Fatalf("telemetry timestep=%v, want fixed %v", got, config.TimeStep)
	}
}

func TestDeadZoneReportsSuppressedCommandsAndKeepsCommandsAboveThreshold(t *testing.T) {
	config := DefaultConfig()
	config.InitialCarriageX, config.InitialGripperY = 3, 2
	config.ActionDeadZone = 0.005
	config.ActionSmoothingAlpha = 1
	task := NewTask(1, config)
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}
	below, err := task.Step(framework.Action{0, -0.004, 0})
	if err != nil {
		t.Fatal(err)
	}
	if below.Info["filtered_action_vertical"] != 0 || below.Info["dead_zone_removed_vertical"] != 1 || task.environment.state.GripperVelocityY != 0 {
		t.Fatalf("below-dead-zone command was not explicitly removed: %#v", below.Info)
	}
	above, err := task.Step(framework.Action{0, -0.006, 0})
	if err != nil {
		t.Fatal(err)
	}
	if above.Info["filtered_action_vertical"] >= 0 || above.Info["dead_zone_removed_vertical"] != 0 || task.environment.state.GripperY >= config.InitialGripperY {
		t.Fatalf("above-dead-zone descent did not reach physics: state=%#v info=%#v", task.environment.state, above.Info)
	}
}

func TestCurriculumResetsUseRealStateAndTerminalCriteria(t *testing.T) {
	lowerConfig := DefaultConfig()
	lowerConfig.Curriculum.Stage = CurriculumLowerAndContact
	lowerConfig.Curriculum.ContactStableSteps = 1
	lower := NewTask(1, lowerConfig)
	if _, err := lower.Reset(); err != nil {
		t.Fatal(err)
	}
	if lower.environment.state.Phase != PhaseLowerToObject || lower.environment.state.CarriageX != lower.environment.state.ObjectX {
		t.Fatalf("lower curriculum did not initialize above object: %#v", lower.environment.state)
	}
	// The policy, rather than the curriculum, must command the remaining
	// physical descent before valid stable contact can finish the stage.
	for step := 0; step < 20 && lower.environment.state.Phase != PhaseSuccess; step++ {
		if _, err := lower.Step(framework.Action{0, -1, 0}); err != nil {
			t.Fatal(err)
		}
	}
	if lower.environment.state.Phase != PhaseSuccess {
		t.Fatalf("lower curriculum did not accept stable physical contact: %#v", lower.environment.state)
	}

	liftConfig := DefaultConfig()
	liftConfig.Curriculum.Stage = CurriculumGraspAndLift
	lift := NewTask(1, liftConfig)
	if _, err := lift.Reset(); err != nil {
		t.Fatal(err)
	}
	if lift.environment.state.Phase != PhaseGripObject || lift.environment.state.Grip.ObjectAttached {
		t.Fatalf("grasp curriculum must still require a learned attachment: %#v", lift.environment.state)
	}

	transportConfig := DefaultConfig()
	transportConfig.Curriculum.Stage = CurriculumTransportAndRelease
	transport := NewTask(1, transportConfig)
	if _, err := transport.Reset(); err != nil {
		t.Fatal(err)
	}
	if transport.environment.state.Phase != PhaseMoveToTarget || !transport.environment.state.Grip.ObjectAttached || !transport.environment.state.Grip.ForceValid {
		t.Fatalf("transport curriculum did not initialize a physical carried object: %#v", transport.environment.state)
	}
}

func TestAlternatingCommandsRemainVelocityBounded(t *testing.T) {
	config := DefaultConfig()
	config.InitialCarriageX = 3
	config.InitialGripperY = 2
	task := NewTask(1, config)
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}
	previousVelocity := task.environment.state.CarriageVelocityX
	for step := 0; step < 20; step++ {
		action := float32(1)
		if step%2 == 1 {
			action = -1
		}
		if _, err := task.Step(framework.Action{action, 0, 0}); err != nil {
			t.Fatal(err)
		}
		velocity := task.environment.state.CarriageVelocityX
		if math.Abs(velocity) > config.MaxHorizontalSpeed+1e-9 {
			t.Fatalf("step %d velocity %v exceeds maximum %v", step, velocity, config.MaxHorizontalSpeed)
		}
		if delta := math.Abs(velocity - previousVelocity); delta > config.MaxHorizontalAcceleration*config.TimeStep+1e-9 {
			t.Fatalf("step %d oscillation changed velocity by %v", step, delta)
		}
		previousVelocity = velocity
	}
}

func TestDeliveryRewardRequiresAttachedObject(t *testing.T) {
	config := DefaultConfig()
	empty := NewTask(1, config)
	_, _ = empty.Reset()
	empty.environment.state.Phase = PhaseMoveToTarget
	empty.environment.state.CarriageX = 3
	empty.environment.state.GripperY = 2
	result, err := empty.Step(framework.Action{1, 0, -1})
	if err != nil {
		t.Fatal(err)
	}
	if result.Info["delivery_reward"] != 0 {
		t.Fatalf("empty gripper received delivery reward: %#v", result.Info)
	}

	attached := attachedTask(t, config)
	attached.environment.state.Phase = PhaseMoveToTarget
	result, err = attached.Step(framework.Action{1, 0, 0.5})
	if err != nil {
		t.Fatal(err)
	}
	if result.Info["delivery_reward"] <= 0 {
		t.Fatalf("attached object did not receive object-to-target progress: %#v", result.Info)
	}
}

func TestDropDuringTransportFailsAndPenalizes(t *testing.T) {
	task := attachedTask(t, DefaultConfig())
	task.environment.state.Phase = PhaseMoveToTarget
	previousEnergy := task.environment.state.Energy
	result, err := task.Step(framework.Action{0, 0, -1})
	if err != nil {
		t.Fatal(err)
	}
	if !result.Done || result.Outcome != OutcomeFailure || result.Reward > float32(task.config.Reward.DroppedObjectPenalty) {
		t.Fatalf("transport drop was not penalized: %#v", result)
	}
	if got, want := task.environment.state.Energy, previousEnergy-task.config.Homeostasis.EnergyDecayPerStep-task.config.Homeostasis.UnsafeDropEnergyLoss; math.Abs(got-want) > 1e-9 || result.Info["energy_event_code"] != float32(energyEventUnsafeDrop) {
		t.Fatalf("unsafe drop energy loss mismatch: got=%v want=%v info=%#v", got, want, result.Info)
	}
}

func TestPhaseCannotSkipSecureAttachment(t *testing.T) {
	task := NewTask(1, DefaultConfig())
	_, _ = task.Reset()
	task.environment.state.Phase = PhaseGripObject
	if _, err := task.Step(framework.Action{0, 0, 0.5}); err != nil {
		t.Fatal(err)
	}
	if task.environment.state.Phase == PhaseLiftObject || task.environment.state.Grip.ObjectAttached {
		t.Fatalf("phase skipped secure attachment: %#v", task.environment.state)
	}
}

func attachedTask(t *testing.T, config Config) *Task {
	t.Helper()
	config.InitialCarriageX = config.InitialObjectX
	config.InitialGripperY = GraspHeight(config, terrainHeightForConfig(config, config.InitialObjectX)+config.ObjectHeight/2, config.InitialCarriageX)
	task := NewTask(1, config)
	if _, err := task.Reset(); err != nil {
		t.Fatal(err)
	}
	for step := 0; step < 30 && !task.environment.state.Grip.ObjectAttached; step++ {
		if _, err := task.Step(framework.Action{0, 0, 0.5}); err != nil {
			t.Fatalf("could not establish valid attachment: state=%#v err=%v", task.environment.state.Grip, err)
		}
	}
	if !task.environment.state.Grip.ObjectAttached {
		t.Fatalf("could not establish valid attachment: state=%#v", task.environment.state.Grip)
	}
	return task
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
		action := framework.Action{0, 0, 0}
		switch phase {
		case PhaseApproachObject:
			action = framework.Action{0, 0, -1}
		case PhaseLowerToObject:
			action = framework.Action{0, -1, -1}
		case PhaseGripObject:
			action = framework.Action{0, 0, testPolicyGripRate(task)}
		case PhaseLiftObject:
			action = framework.Action{0, 1, testPolicyGripRate(task)}
		case PhaseMoveToTarget:
			action = framework.Action{1, 0, testPolicyGripRate(task)}
		case PhaseLowerAtTarget:
			action = framework.Action{0, -1, testPolicyGripRate(task)}
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

// testPolicyGripRate is a privileged test-only reference policy used to drive
// the scripted lifecycle; production SAC receives slip/contact feedback rather
// than this analytic force value.
func testPolicyGripRate(task *Task) float32 {
	desired := task.environment.requiredForce() + 0.3
	force := task.environment.state.GripForce
	switch {
	case force < desired-0.2:
		return 1
	case force > desired+0.5:
		return -0.5
	default:
		return 0
	}
}
