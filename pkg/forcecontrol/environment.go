package forcecontrol

import (
	"errors"
	"fmt"
	"math"
	"math/rand"
)

// ObservationDimension is the fixed goal-conditioned policy input size.
const ObservationDimension = 20

// CoordinateSystemVersion changes whenever action meaning or observation
// normalization changes. Checkpoints for earlier schemas must not be reused.
const CoordinateSystemVersion = 2

const (
	observationGripperX = iota
	observationGripperY
	observationGripperVelocityX
	observationGripperVelocityY
	observationObjectX
	observationObjectY
	observationObjectVelocityX
	observationObjectVelocityY
	observationTargetX
	observationTargetY
	observationObjectFromGripperX
	observationObjectFromGripperY
	observationTargetFromObjectX
	observationTargetFromObjectY
	observationGripperClosed
	observationObjectGripped
	observationGripForce
	observationRequiredGripForce
	observationBreakForce
	observationPhase
)

// Phase describes progress through one policy-controlled pick-and-place episode.
type Phase int

const (
	PhaseIdle Phase = iota
	PhaseApproachObject
	PhaseLowerToObject
	PhaseGripObject
	PhaseLiftObject
	PhaseMoveToTarget
	PhaseLowerAtTarget
	PhaseReleaseObject
	PhaseSuccess
	PhaseFailure
)

func (p Phase) String() string {
	switch p {
	case PhaseApproachObject:
		return "approach_object"
	case PhaseLowerToObject:
		return "lower_to_object"
	case PhaseGripObject:
		return "grip_object"
	case PhaseLiftObject:
		return "lift_object"
	case PhaseMoveToTarget:
		return "move_to_target"
	case PhaseLowerAtTarget:
		return "lower_at_target"
	case PhaseReleaseObject:
		return "release_object"
	case PhaseSuccess:
		return "success"
	case PhaseFailure:
		return "failure"
	default:
		return "idle"
	}
}

// PhaseFromNormalized decodes the phase value exposed in an observation.
func PhaseFromNormalized(value float32) Phase {
	phase := int(math.Round((float64(value) + 1) * float64(PhaseFailure) / 2))
	if phase < int(PhaseIdle) || phase > int(PhaseFailure) {
		return PhaseIdle
	}
	return Phase(phase)
}

// State is the authoritative physical and task state for one environment.
type State struct {
	CarriageX         float64
	GripperY          float64
	CarriageVelocityX float64
	GripperVelocityY  float64
	GripperOpening    float64
	GripForce         float64
	ObjectX           float64
	ObjectY           float64
	ObjectVelocityX   float64
	ObjectVelocityY   float64
	ObjectMass        float64
	ObjectFriction    float64
	ObjectBreakForce  float64
	TargetX           float64
	TargetY           float64
	ObjectGrasped     bool
	ObjectBroken      bool
	ObjectPlaced      bool
	BoundaryHit       bool
	Phase             Phase
	EpisodeStep       int
}

// Environment owns all mutable state for one independent task instance.
type Environment struct {
	config Config
	seed   int64
	random *rand.Rand
	state  State
	ready  bool

	gripBonusAwarded bool
	wasEverGrasped   bool
	failureReason    string
}

func newEnvironment(seed int64, config Config) *Environment {
	return &Environment{config: config, seed: seed, random: rand.New(rand.NewSource(seed))}
}

func (e *Environment) reset() State {
	e.random = rand.New(rand.NewSource(e.seed))
	terrainY := e.terrainHeight(e.config.InitialObjectX)
	e.state = State{
		CarriageX:        e.config.InitialCarriageX,
		GripperY:         e.config.InitialGripperY,
		GripperOpening:   1,
		ObjectX:          e.config.InitialObjectX,
		ObjectY:          terrainY + e.config.ObjectHeight/2,
		ObjectMass:       e.config.InitialObjectMass,
		ObjectFriction:   e.config.ObjectFriction,
		ObjectBreakForce: e.config.ObjectBreakForce,
		TargetX:          e.config.TargetX,
		TargetY:          e.terrainHeight(e.config.TargetX),
		Phase:            PhaseApproachObject,
	}
	e.gripBonusAwarded, e.wasEverGrasped, e.failureReason, e.ready = false, false, "", true
	return e.state
}

func (e *Environment) step(action []float32) (State, float64, Outcome, bool, error) {
	if !e.ready {
		return State{}, 0, OutcomeRunning, false, errors.New("force-control environment must be reset before stepping")
	}
	values, err := e.validatedAction(action)
	if err != nil {
		return State{}, 0, OutcomeRunning, false, err
	}
	if e.state.Phase == PhaseSuccess || e.state.Phase == PhaseFailure {
		return e.state, 0, OutcomeFailure, true, errors.New("force-control episode is terminal; reset before stepping")
	}

	previous := e.state
	e.state.BoundaryHit = false
	e.applyHorizontalControl(values[0])
	e.applyVerticalControl(values[1])
	e.applyGripControl(values[2])
	e.updateGraspState()
	if e.state.ObjectGrasped {
		e.wasEverGrasped = true
	}
	e.updateObjectPhysics()
	e.resolveTerrainCollision()
	e.state.EpisodeStep++
	if err := e.ValidateState(); err != nil {
		e.failureReason, e.state.Phase = "invalid_state", PhaseFailure
		return e.state, e.config.Reward.WorkspacePenalty, OutcomeFailure, true, nil
	}

	terminalReason := e.detectFailure(previous)
	if terminalReason != "" {
		e.failureReason, e.state.Phase = terminalReason, PhaseFailure
		reward := e.reward(previous) + e.failurePenalty(terminalReason)
		return e.state, reward, OutcomeFailure, true, nil
	}
	e.updatePhase(previous)
	if e.state.Phase == PhaseSuccess {
		return e.state, e.reward(previous) + e.config.Reward.SuccessfulPlacement, OutcomeSuccess, true, nil
	}
	return e.state, e.reward(previous), OutcomeRunning, false, nil
}

func (e *Environment) validatedAction(action []float32) ([3]float64, error) {
	if len(action) != 3 {
		return [3]float64{}, errors.New("force-control action must contain exactly three values")
	}
	values := [3]float64{}
	for index, value := range action {
		if math.IsNaN(float64(value)) || math.IsInf(float64(value), 0) {
			return [3]float64{}, errors.New("force-control action must contain only finite values")
		}
		values[index] = clamp(float64(value), -1, 1)
		if math.Abs(values[index]) < e.config.ActionDeadZone {
			values[index] = 0
		}
	}
	return values, nil
}

func (e *Environment) applyHorizontalControl(value float64) {
	e.state.CarriageVelocityX = value * e.config.MaxHorizontalSpeed
	next := e.state.CarriageX + e.state.CarriageVelocityX*e.config.TimeStep
	safeMinX, safeMaxX, _, _ := e.safeBounds()
	e.state.CarriageX = clamp(next, safeMinX, safeMaxX)
	if e.state.CarriageX != next {
		e.state.CarriageVelocityX = 0
		e.state.BoundaryHit = true
	}
}

func (e *Environment) applyVerticalControl(value float64) {
	e.state.GripperVelocityY = value * e.config.MaxVerticalSpeed
	next := e.state.GripperY + e.state.GripperVelocityY*e.config.TimeStep
	_, _, safeMinY, safeMaxY := e.safeBounds()
	e.state.GripperY = clamp(next, safeMinY, safeMaxY)
	if e.state.GripperY != next {
		e.state.GripperVelocityY = 0
		e.state.BoundaryHit = true
	}
}

func (e *Environment) applyGripControl(value float64) {
	e.state.GripperOpening = clamp((1-value)/2, 0, 1)
	e.state.GripForce = clamp((value+1)/2*e.config.MaxGripForce, 0, e.config.MaxGripForce)
}

func (e *Environment) updateGraspState() {
	required := e.requiredForce()
	horizontalOverlap := math.Abs(e.state.CarriageX-e.state.ObjectX) <= e.config.HorizontalTolerance
	verticalClose := math.Abs(e.state.GripperY-e.objectGripHeight()) <= e.config.VerticalTolerance
	canGrasp := horizontalOverlap && verticalClose && e.state.GripperOpening <= 0.35 && e.state.GripForce >= required
	if e.state.GripForce > e.state.ObjectBreakForce {
		e.state.ObjectBroken, e.state.ObjectGrasped = true, false
		return
	}
	if e.state.ObjectGrasped && e.state.GripForce < required {
		e.state.ObjectGrasped = false
		return
	}
	if !e.state.ObjectGrasped && canGrasp {
		e.state.ObjectGrasped = true
	}
}

func (e *Environment) updateObjectPhysics() {
	if e.state.ObjectGrasped {
		e.state.ObjectVelocityX, e.state.ObjectVelocityY = e.state.CarriageVelocityX, e.state.GripperVelocityY
		e.state.ObjectX = e.state.CarriageX
		e.state.ObjectY = e.state.GripperY - e.config.ObjectHeight/2
		return
	}
	e.state.ObjectVelocityY -= e.config.Gravity * e.config.TimeStep
	e.state.ObjectX += e.state.ObjectVelocityX * e.config.TimeStep
	e.state.ObjectY += e.state.ObjectVelocityY * e.config.TimeStep
}

func (e *Environment) resolveTerrainCollision() {
	ground := e.terrainHeight(e.state.ObjectX)
	minimumY := ground + e.config.ObjectHeight/2
	if e.state.ObjectY <= minimumY {
		e.state.ObjectY, e.state.ObjectVelocityY = minimumY, 0
		e.state.ObjectVelocityX *= 0.8
		if !e.state.ObjectGrasped && math.Abs(e.state.ObjectX-e.state.TargetX) <= e.config.TargetWidth/2 {
			e.state.ObjectPlaced = true
		}
	}
}

func (e *Environment) detectFailure(previous State) string {
	switch {
	case e.state.ObjectBroken:
		return "object_break"
	case e.objectOutOfBounds():
		return "workspace_violation"
	case previous.ObjectGrasped && !e.state.ObjectGrasped && previous.Phase != PhaseReleaseObject:
		return "unsafe_drop"
	case e.state.EpisodeStep >= e.config.MaxEpisodeSteps:
		return "timeout"
	}
	return ""
}

func (e *Environment) updatePhase(previous State) {
	if e.state.ObjectPlaced && previous.Phase == PhaseReleaseObject {
		e.state.Phase = PhaseSuccess
		return
	}
	switch previous.Phase {
	case PhaseApproachObject:
		if math.Abs(e.state.CarriageX-e.state.ObjectX) <= e.config.HorizontalTolerance {
			e.state.Phase = PhaseLowerToObject
		}
	case PhaseLowerToObject:
		if math.Abs(e.state.GripperY-e.objectGripHeight()) <= e.config.VerticalTolerance {
			e.state.Phase = PhaseGripObject
		}
	case PhaseGripObject:
		if e.state.ObjectGrasped {
			e.state.Phase = PhaseLiftObject
		}
	case PhaseLiftObject:
		if e.state.ObjectGrasped && e.state.ObjectY >= e.requiredCarryHeight() {
			e.state.Phase = PhaseMoveToTarget
		}
	case PhaseMoveToTarget:
		if e.state.ObjectGrasped && math.Abs(e.state.ObjectX-e.state.TargetX) <= e.config.TargetWidth/2 {
			e.state.Phase = PhaseLowerAtTarget
		}
	case PhaseLowerAtTarget:
		if e.state.ObjectGrasped && e.state.GripperY <= e.targetReleaseGuideHeight()+e.config.ReleaseTolerance {
			e.state.Phase = PhaseReleaseObject
		}
	}
}

func (e *Environment) reward(previous State) float64 {
	reward := e.config.Reward.TimePenalty
	if previous.Phase == PhaseApproachObject && !e.state.ObjectGrasped {
		reward += e.config.Reward.ApproachProgressScale * (gripperObjectDistance(previous) - gripperObjectDistance(e.state))
	}
	if previous.Phase == PhaseLowerToObject && !e.state.ObjectGrasped {
		previousError := math.Abs(previous.GripperY - e.objectGripHeightFor(previous))
		currentError := math.Abs(e.state.GripperY - e.objectGripHeight())
		reward += e.config.Reward.LowerProgressScale * (previousError - currentError)
	}
	if e.state.ObjectGrasped && !e.gripBonusAwarded {
		reward += e.config.Reward.SuccessfulGripReward
		e.gripBonusAwarded = true
	}
	if previous.Phase == PhaseLiftObject && e.state.ObjectGrasped {
		reward += e.config.Reward.LiftProgressScale * (e.state.ObjectY - previous.ObjectY)
	}
	if previous.Phase == PhaseMoveToTarget && previous.ObjectGrasped && e.state.ObjectGrasped {
		reward += e.config.Reward.DeliveryProgressScale * (targetDistance(previous) - targetDistance(e.state))
	}
	if previous.Phase == PhaseGripObject && !e.state.ObjectGrasped && e.state.GripForce < e.requiredForce() {
		reward += e.config.Reward.InsufficientGripPenalty
	}
	if e.state.BoundaryHit {
		reward += e.config.Reward.BoundaryCollisionPenalty
	}
	return reward
}

func (e *Environment) failurePenalty(reason string) float64 {
	switch reason {
	case "object_break":
		return e.config.Reward.BreakPenalty
	case "workspace_violation":
		return e.config.Reward.WorkspacePenalty
	default:
		return e.config.Reward.UnsafeDropPenalty
	}
}

func (e *Environment) observation() []float32 {
	state := e.state
	values := []float64{
		normalize(state.CarriageX, e.config.Workspace.MinX, e.config.Workspace.MaxX),
		normalize(state.GripperY, e.config.Workspace.MinY, e.config.Workspace.MaxY),
		normalize(state.CarriageVelocityX, -e.config.MaxHorizontalSpeed, e.config.MaxHorizontalSpeed),
		normalize(state.GripperVelocityY, -e.config.MaxVerticalSpeed, e.config.MaxVerticalSpeed),
		normalize(state.ObjectX, e.config.Workspace.MinX, e.config.Workspace.MaxX),
		normalize(state.ObjectY, e.config.Workspace.MinY, e.config.Workspace.MaxY),
		normalize(state.ObjectVelocityX, -e.config.MaxHorizontalSpeed, e.config.MaxHorizontalSpeed),
		normalize(state.ObjectVelocityY, -2*e.config.Gravity, 2*e.config.Gravity),
		normalize(state.TargetX, e.config.Workspace.MinX, e.config.Workspace.MaxX),
		normalize(state.TargetY, e.config.Workspace.MinY, e.config.Workspace.MaxY),
		normalize(state.ObjectX-state.CarriageX, -e.config.Workspace.MaxX, e.config.Workspace.MaxX),
		normalize(state.ObjectY-state.GripperY, -e.config.Workspace.MaxY, e.config.Workspace.MaxY),
		normalize(state.TargetX-state.ObjectX, -e.config.Workspace.MaxX, e.config.Workspace.MaxX),
		normalize(state.TargetY-state.ObjectY, -e.config.Workspace.MaxY, e.config.Workspace.MaxY),
		normalize01(1 - state.GripperOpening),
		boolValue(state.ObjectGrasped),
		normalize(state.GripForce, 0, e.config.MaxGripForce),
		normalize(e.requiredForce(), 0, e.config.MaxGripForce),
		normalize(state.ObjectBreakForce, 0, e.config.MaxGripForce),
		normalize(float64(state.Phase), float64(PhaseIdle), float64(PhaseFailure)),
	}
	result := make([]float32, len(values))
	for index, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			result[index] = 0
		} else {
			result[index] = float32(clamp(value, -1, 1))
		}
	}
	return result
}

func (e *Environment) objectGripHeight() float64 {
	return GraspHeight(e.config, e.state.ObjectY, e.state.CarriageX)
}
func (e *Environment) objectGripHeightFor(state State) float64 {
	return GraspHeight(e.config, state.ObjectY, state.CarriageX)
}
func (e *Environment) targetRestHeight() float64 { return e.state.TargetY + e.config.ObjectHeight/2 }
func (e *Environment) targetReleaseGuideHeight() float64 {
	return GraspHeight(e.config, e.targetRestHeight(), e.state.CarriageX)
}
func (e *Environment) requiredCarryHeight() float64 {
	return math.Max(e.terrainHeight(e.state.ObjectX), e.state.TargetY) + e.config.ObjectHeight/2 + e.config.LiftClearance
}

// safeBounds applies physical gripper geometry to the configured workspace.
// GripperY is the jaw-guide reference point: fingers extend downward and the
// housing extends upward, so raw workspace edges are never valid centers.
func (e *Environment) safeBounds() (safeMinX, safeMaxX, safeMinY, safeMaxY float64) {
	bounds := SafeGripperBounds(e.config, e.state.CarriageX)
	return bounds.MinX, bounds.MaxX, bounds.MinY, bounds.MaxY
}

// ValidateState rejects non-physical coordinates before they reach the policy,
// telemetry, or renderer. It is deliberately based on world coordinates only.
func (e *Environment) ValidateState() error {
	workspace := e.config.Workspace
	if !finite(workspace.MinX) || !finite(workspace.MaxX) || !finite(workspace.MinY) || !finite(workspace.MaxY) || workspace.MaxX <= workspace.MinX || workspace.MaxY <= workspace.MinY {
		return errors.New("invalid workspace bounds")
	}
	if e.config.GripperWidth <= 0 || e.config.GripperBodyHeight <= 0 || e.config.GripperFingerLength < 0 || e.config.GripperClearance < 0 {
		return errors.New("invalid gripper geometry")
	}
	safeMinX, safeMaxX, safeMinY, safeMaxY := e.safeBounds()
	if safeMinX > safeMaxX || safeMinY > safeMaxY {
		return errors.New("gripper geometry does not fit inside workspace")
	}
	state := e.state
	for name, value := range map[string]float64{
		"carriage_x": state.CarriageX, "gripper_y": state.GripperY, "object_x": state.ObjectX, "object_y": state.ObjectY,
		"target_x": state.TargetX, "target_y": state.TargetY, "velocity_x": state.CarriageVelocityX, "velocity_y": state.GripperVelocityY,
	} {
		if !finite(value) {
			return fmt.Errorf("%s is non-finite", name)
		}
	}
	if state.CarriageX < safeMinX || state.CarriageX > safeMaxX || state.GripperY < safeMinY || state.GripperY > safeMaxY {
		return fmt.Errorf("gripper outside safe bounds x=[%g,%g] y=[%g,%g]", safeMinX, safeMaxX, safeMinY, safeMaxY)
	}
	if state.ObjectX < workspace.MinX || state.ObjectX > workspace.MaxX || state.ObjectY < workspace.MinY || state.ObjectY > workspace.MaxY || state.TargetX < workspace.MinX || state.TargetX > workspace.MaxX || state.TargetY < workspace.MinY || state.TargetY > workspace.MaxY {
		return errors.New("object or target outside workspace")
	}
	return nil
}

func (e *Environment) requiredForce() float64 {
	return e.state.ObjectMass * e.config.Gravity / (2 * e.state.ObjectFriction)
}
func (e *Environment) objectOutOfBounds() bool {
	return e.state.ObjectX < e.config.Workspace.MinX || e.state.ObjectX > e.config.Workspace.MaxX || e.state.ObjectY < e.config.Workspace.MinY-1 || e.state.ObjectY > e.config.Workspace.MaxY
}
func (e *Environment) terrainHeight(x float64) float64 {
	points := e.config.Terrain
	if len(points) == 0 {
		return e.config.Workspace.MinY
	}
	if x <= points[0].X {
		return points[0].Y
	}
	for index := 1; index < len(points); index++ {
		if x <= points[index].X {
			left, right := points[index-1], points[index]
			ratio := (x - left.X) / (right.X - left.X)
			return left.Y + ratio*(right.Y-left.Y)
		}
	}
	return points[len(points)-1].Y
}

func gripperObjectDistance(state State) float64 {
	return math.Hypot(state.CarriageX-state.ObjectX, state.GripperY-state.ObjectY)
}
func targetDistance(state State) float64 {
	return math.Hypot(state.TargetX-state.ObjectX, state.TargetY-state.ObjectY)
}
func normalize(value, minimum, maximum float64) float64 {
	if maximum <= minimum {
		return 0
	}
	return 2*(value-minimum)/(maximum-minimum) - 1
}
func normalize01(value float64) float64 { return 2*clamp(value, 0, 1) - 1 }
func boolValue(value bool) float64 {
	if value {
		return 1
	}
	return -1
}
func clamp(value, minimum, maximum float64) float64 {
	return math.Min(math.Max(value, minimum), maximum)
}

func finite(value float64) bool { return !math.IsNaN(value) && !math.IsInf(value, 0) }

func (e *Environment) String() string {
	return fmt.Sprintf("phase=%s step=%d", e.state.Phase, e.state.EpisodeStep)
}
