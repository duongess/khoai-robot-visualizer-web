package forcecontrol

import (
	"errors"
	"fmt"
	"math"
	"math/rand"
)

// ObservationDimension is the fixed goal-conditioned policy input size.
const ObservationDimension = 30

// CoordinateSystemVersion changes whenever policy-facing semantics, reward
// semantics, or reset distributions change. Checkpoints for earlier schemas
// must not be reused for a scientific comparison.
const CoordinateSystemVersion = 33

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
	observationGripperFromObjectX
	observationGripperFromObjectY
	observationTargetFromObjectX
	observationTargetFromObjectY
	observationGripperClosed
	observationObjectGripped
	observationGripForce
	observationSlipSeverity
	observationGripperVerticalAcceleration
	observationPhase
	observationContactDetected
	observationRequiredGripForceBaseline
	observationEnergy
	observationAlignedX
	observationInGraspZone
	observationLastActionHorizontal
	observationLastActionVertical
	observationLastActionGripper
	observationAlignmentGateAwarded
	observationContactBonusAwarded
)

// GripState distinguishes a command to close the gripper from physical object
// contact and a secure attachment. Force without contact is never a grasp.
type GripState struct {
	GripperClosed   bool
	ContactDetected bool
	ForceValid      bool
	ObjectAttached  bool
	Slipping        bool
}

// RewardBreakdown keeps every reward term observable so training behaviour can
// be diagnosed without reverse engineering a scalar reward from telemetry.
type RewardBreakdown struct {
	Approach      float64
	Contact       float64
	Grip          float64
	Lift          float64
	Delivery      float64
	Success       float64
	Homeostasis   float64
	DetachedForce float64
	BreakRisk     float64
	UnderGrip     float64
	LiftStall     float64
	Stability     float64
	Descent       float64
	Hover         float64
	Smoothness    float64
	Retraction    float64
	Penalty       float64
	Total         float64
}

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
	CarriageX            float64
	GripperY             float64
	CarriageVelocityX    float64
	GripperVelocityY     float64
	GripperOpening       float64
	GripForce            float64
	ObjectX              float64
	ObjectY              float64
	ObjectVelocityX      float64
	ObjectVelocityY      float64
	ObjectMass           float64
	ObjectFriction       float64
	ObjectBreakForce     float64
	TargetX              float64
	TargetY              float64
	ObjectGrasped        bool
	ObjectBroken         bool
	ObjectPlaced         bool
	PlacementStableSteps int
	BoundaryHit          bool
	Grip                 GripState
	// ContactBonusAwarded is episode state. It prevents re-contact reward
	// pumping and is included in the observation so reward semantics remain
	// Markovian from the policy's perspective.
	ContactBonusAwarded bool
	Phase               Phase
	EpisodeStep         int
	// Energy is an authoritative, normalized homeostatic reserve in [0, 1].
	Energy float64
}

// Environment owns all mutable state for one independent task instance.
type Environment struct {
	config      Config
	baseTerrain []TerrainPoint
	seed        int64
	random      *rand.Rand
	state       State
	ready       bool

	gripBonusAwarded       bool
	secureGripBonusAwarded bool
	gripFoodAwarded        bool
	liftFoodAwarded        bool
	deliveryFoodAwarded    bool
	successFoodAwarded     bool
	wasEverGrasped         bool
	// objectWasLifted latches a verified carry-height event for this episode.
	// It is deliberately distinct from a momentary attachment: after it becomes
	// true, losing the object outside the target is a terminal mid-air drop.
	objectWasLifted                bool
	invalidGripPenaltyAwarded      bool
	insufficientGripPenaltyAwarded bool
	emptyTargetPenaltyAwarded      bool
	successRewardAwarded           bool
	failureReason                  string
	lastReward                     RewardBreakdown
	lastAppliedAction              [3]float64
	// The three action snapshots are emitted by the Python dual-loop actor.
	// Every channel is normalized: X velocity, Y velocity, and grip force-rate.
	// Go selects one vector by ControlMode, then applies only physical guards.
	baseAction                [3]float64
	residualAction            [3]float64
	finalAction               [3]float64
	appliedAction             [3]float64
	alignmentGateAwarded      bool
	hoverPenaltyAccumulated   float64
	boundaryHitFrames         int
	deadZoneRemoved           [3]bool
	filterDeadZoneRemoved     [3]bool
	filteredAction            [3]float64
	verticalAcceleration      float64
	verticalRetractionBlocked bool
	invalidContactFrames      int
	slipFrames                int
	releaseCommanded          bool
	contactBeforeMotion       bool
	lastEnergyDelta           float64
	lastEnergyDecay           float64
	lastEnergyFoodGain        float64
	lastEnergyEvent           energyEvent
	contactStableFrames       int
	secureGripHoldFrames      int
	liftHoldFrames            int
	contactWithoutGripFrames  int
	contactSuccessStreak      int
	activeCurriculumStage     CurriculumStage
	advanceCurriculumOnReset  bool
	resetCount                uint64
}

// energyEvent is compactly encoded for StepResult.Info while the visualizer
// turns it into a human-readable label.
type energyEvent uint8

const (
	energyEventNone energyEvent = iota
	energyEventSecureGrasp
	energyEventLift
	energyEventDelivery
	energyEventSuccess
	energyEventUnsafeDrop
	energyEventBreak
)

func newEnvironment(seed int64, config Config) *Environment {
	stage := config.Curriculum.Stage.canonical()
	if stage == CurriculumAutomatic {
		stage = CurriculumAlignAndContact
	}
	baseTerrain := append([]TerrainPoint(nil), config.Terrain...)
	return &Environment{config: config, baseTerrain: baseTerrain, seed: seed, random: rand.New(rand.NewSource(seed)), activeCurriculumStage: stage}
}

func (e *Environment) reset() State {
	// A user/runtime reset of an unfinished first-lesson episode is not a
	// verified contact success. Clear the streak so it cannot be advanced by
	// abandoning unsuccessful attempts between valid contacts.
	if e.ready && e.config.Curriculum.Stage == CurriculumAutomatic && e.currentCurriculumStage() == CurriculumAlignAndContact && e.state.Phase != PhaseSuccess {
		e.contactSuccessStreak = 0
	}
	// Keep terminal telemetry attributable to the stage that was actually
	// completed. The next stage begins only with the following episode.
	if e.advanceCurriculumOnReset {
		e.advanceCurriculum()
		e.advanceCurriculumOnReset = false
	}
	e.resetTerrain()
	objectX, targetX, objectMass, objectFriction := e.sampleEpisodeParameters()
	terrainY := e.terrainHeight(objectX)
	e.state = State{
		CarriageX:        e.config.InitialCarriageX,
		GripperY:         e.config.InitialGripperY,
		GripperOpening:   1,
		ObjectX:          objectX,
		ObjectY:          terrainY + e.config.ObjectHeight/2,
		ObjectMass:       objectMass,
		ObjectFriction:   objectFriction,
		ObjectBreakForce: e.config.ObjectBreakForce,
		TargetX:          targetX,
		TargetY:          e.terrainHeight(targetX),
		Phase:            PhaseApproachObject,
		Energy:           clamp(e.config.Homeostasis.InitialEnergy, 0, 1),
	}
	if e.config.Curriculum.Randomization.Enabled && e.resetCount > 0 {
		_, _, minimumY, maximumY := e.safeBounds()
		e.state.GripperY = clamp(e.state.GripperY+e.symmetricJitter(e.config.Curriculum.Randomization.ContactStartHeightJitter), minimumY, maximumY)
	}
	e.gripBonusAwarded = false
	e.secureGripBonusAwarded = false
	e.state.ContactBonusAwarded = false
	e.gripFoodAwarded = false
	e.liftFoodAwarded = false
	e.deliveryFoodAwarded = false
	e.successFoodAwarded = false
	e.wasEverGrasped = false
	e.objectWasLifted = false
	e.invalidGripPenaltyAwarded = false
	e.insufficientGripPenaltyAwarded = false
	e.emptyTargetPenaltyAwarded = false
	e.successRewardAwarded = false
	e.failureReason = ""
	e.lastReward = RewardBreakdown{}
	e.lastAppliedAction = [3]float64{}
	e.baseAction = [3]float64{}
	e.residualAction = [3]float64{}
	e.finalAction = [3]float64{}
	e.appliedAction = [3]float64{}
	e.alignmentGateAwarded = false
	e.hoverPenaltyAccumulated = 0
	e.boundaryHitFrames = 0
	e.deadZoneRemoved = [3]bool{}
	e.filterDeadZoneRemoved = [3]bool{}
	e.filteredAction = [3]float64{}
	e.verticalAcceleration = 0
	e.verticalRetractionBlocked = false
	e.invalidContactFrames = 0
	e.slipFrames = 0
	e.releaseCommanded = false
	e.contactBeforeMotion = false
	e.lastEnergyDelta = 0
	e.lastEnergyDecay = 0
	e.lastEnergyFoodGain = 0
	e.lastEnergyEvent = energyEventNone
	e.contactStableFrames = 0
	e.secureGripHoldFrames = 0
	e.liftHoldFrames = 0
	e.contactWithoutGripFrames = 0
	e.applyCurriculumReset()
	e.resetCount++
	e.ready = true
	return e.state
}

// applyCurriculumReset keeps every lesson physically fresh. The reverse grasp
// curriculum intentionally starts in contact, but never attaches the object or
// selects grip force for the policy.
func (e *Environment) applyCurriculumReset() {
	e.state.Phase = PhaseApproachObject
	switch e.currentCurriculumStage() {
	case CurriculumAlignAndContact:
		e.placeNearObjectForAlignLesson()
	case CurriculumGrasp:
		e.placeNearObjectForGraspLesson()
	}
}

// placeNearObjectForAlignLesson samples a symmetric, detached relative pose
// for lesson 1. It chooses left/right with equal probability, then samples a
// safe offset on that side. The policy must still learn the signed X approach;
// reset never aligns, contacts, or attaches the object on its behalf.
func (e *Environment) placeNearObjectForAlignLesson() {
	safeMinX, safeMaxX, _, _ := e.safeBounds()
	minimumDistance := math.Max(e.config.AlignmentExitTolerance, e.config.GraspHorizontalTolerance) + 0.05
	maximumDistance := math.Max(minimumDistance, e.config.Curriculum.AlignStartDistance+e.symmetricJitter(e.config.Curriculum.AlignStartDistanceJitter))
	// A single coin flip is intentionally retained even near workspace edges:
	// the distance range is shortened on that side rather than biasing all
	// starts toward the roomier side of the rail.
	direction := 1.0
	if e.random.Intn(2) == 0 {
		direction = -1
	}
	availableDistance := safeMaxX - e.state.ObjectX
	if direction < 0 {
		availableDistance = e.state.ObjectX - safeMinX
	}
	maximumDistance = math.Min(maximumDistance, availableDistance)
	// Alignment object-spawn bounds guarantee this under the default geometry.
	// If a custom workspace is too narrow, use the largest safe detached pose
	// instead of emitting an invalid state.
	if maximumDistance < minimumDistance {
		maximumDistance = math.Max(0, availableDistance)
	}
	distance := maximumDistance
	if maximumDistance > minimumDistance {
		distance = minimumDistance + e.random.Float64()*(maximumDistance-minimumDistance)
	}
	e.state.CarriageX = clamp(e.state.ObjectX+direction*distance, safeMinX, safeMaxX)
	e.state.CarriageVelocityX = 0
	_, _, minimumY, maximumY := e.safeBounds()
	graspY := e.objectGripHeight()
	// Lesson 1 begins from a visibly high, random gantry pose. The old
	// grasp-height band was only 0.15--0.30m above a floor object, which made a
	// reset look as though the robot had already descended. InitialGripperY is
	// the operator-configured nominal start height; randomize nearby while also
	// retaining a physical clearance above the grasp pose.
	nominalStartY := clamp(e.config.InitialGripperY, minimumY, maximumY)
	minimumStartY := math.Max(graspY+e.config.Curriculum.ContactStartHeightOffset, nominalStartY-0.40)
	minimumStartY = clamp(minimumStartY, minimumY, maximumY)
	maximumStartY := clamp(nominalStartY+0.20, minimumStartY, maximumY)
	e.state.GripperY = minimumStartY + e.random.Float64()*(maximumStartY-minimumStartY)
	e.state.GripperVelocityY = 0
}

// placeNearObjectForGraspLesson is a reverse-curriculum reset: geometry is
// already a valid contact pose, so the actor's remaining task is to learn its
// force-rate command. The object remains detached and the gripper open; no
// attachment or force is fabricated on reset.
func (e *Environment) placeNearObjectForGraspLesson() {
	e.state.CarriageX = e.state.ObjectX
	e.state.CarriageVelocityX = 0
	e.state.GripperY = e.objectGripHeightFor(e.state)
	e.state.GripperVelocityY = 0
	e.state.Grip = GripState{ContactDetected: true}
	e.state.ObjectGrasped = false
	e.state.Phase = PhaseGripObject
}

func (e *Environment) nearObjectCarriageX(distance, jitter, minimumDistance float64) float64 {
	distance = math.Max(minimumDistance, distance+e.symmetricJitter(jitter))
	minimumX := e.config.Workspace.MinX + e.config.GripperWidth/2
	maximumX := e.config.Workspace.MaxX - e.config.GripperWidth/2
	direction := 1.0
	if e.random.Intn(2) == 0 {
		direction = -1
	}
	candidate := e.state.ObjectX + direction*distance
	if candidate < minimumX || candidate > maximumX {
		candidate = e.state.ObjectX - direction*distance
	}
	candidate = clamp(candidate, minimumX, maximumX)
	if math.Abs(candidate-e.state.ObjectX) < minimumDistance {
		candidate = clamp(e.state.ObjectX-direction*minimumDistance, minimumX, maximumX)
	}
	return candidate
}

// sampleEpisodeParameters produces a deterministic sequence for a given task
// seed while varying every automatic-curriculum reset. Manual scene edits set
// the centre of each distribution rather than being overwritten by a second
// frontend-only object position.
func (e *Environment) sampleEpisodeParameters() (objectX, targetX, mass, friction float64) {
	objectX, targetX = e.config.InitialObjectX, e.config.TargetX
	mass, friction = e.config.InitialObjectMass, e.config.ObjectFriction
	randomization := e.config.Curriculum.Randomization
	stage := e.currentCurriculumStage()
	if stage == CurriculumAlignAndContact {
		// Lesson 1 must learn a directional approach rather than memorize the
		// historical 1.5m object location. On the default 6m rail this is the
		// requested uniform [0.8, 5.2] distribution; custom workspaces fall back
		// to their geometry-safe interior.
		minimumObjectX, maximumObjectX := e.alignmentObjectSpawnBounds()
		objectX = minimumObjectX + e.random.Float64()*(maximumObjectX-minimumObjectX)
	}
	// The first reset after creating/replacing a task is exact. This makes a
	// paused manual scene edit observable as the episode's authoritative state;
	// subsequent resets vary around that manually chosen baseline.
	if randomization.Enabled && e.resetCount > 0 {
		if stage != CurriculumAlignAndContact {
			objectX = clamp(objectX+e.symmetricJitter(randomization.ObjectXJitter), e.config.Workspace.MinX+e.config.ObjectWidth/2, e.config.Workspace.MaxX-e.config.ObjectWidth/2)
		}
		targetX = clamp(targetX+e.symmetricJitter(randomization.TargetXJitter), e.config.Workspace.MinX+e.config.TargetWidth/2, e.config.Workspace.MaxX-e.config.TargetWidth/2)
		mass = math.Max(0.01, mass+e.symmetricJitter(randomization.ObjectMassJitter))
		friction = clamp(friction+e.symmetricJitter(randomization.ObjectFrictionJitter), 0.05, 2)
	}
	// An object that starts inside its target has already satisfied the
	// transport geometry before an action is taken.  That is particularly
	// harmful in full pick-and-place training: depending on the random seed,
	// SAC can learn that the target vector is irrelevant and collapse to a rail
	// boundary.  Keep the two task landmarks distinct for *every* curriculum
	// stage, including a manually centred first reset.  This is reset
	// distribution hygiene, not a controller hint.
	targetX = e.separateTargetFromObject(objectX, targetX)
	return
}

func (e *Environment) alignmentObjectSpawnBounds() (minimumX, maximumX float64) {
	safeMinimum := e.config.Workspace.MinX + e.config.ObjectWidth/2
	safeMaximum := e.config.Workspace.MaxX - e.config.ObjectWidth/2
	minimumX = math.Max(0.8, safeMinimum)
	maximumX = math.Min(5.2, safeMaximum)
	if minimumX > maximumX {
		return safeMinimum, safeMaximum
	}
	return minimumX, maximumX
}

// separateTargetFromObject prevents align-and-contact episodes from starting
// as an already-completed placement problem. It preserves the configured
// target side when possible and otherwise uses the other valid side.
func (e *Environment) separateTargetFromObject(objectX, targetX float64) float64 {
	minimumSeparation := (e.config.ObjectWidth+e.config.TargetWidth)/2 + e.config.HorizontalTolerance
	minimumTargetX := e.config.Workspace.MinX + e.config.TargetWidth/2
	maximumTargetX := e.config.Workspace.MaxX - e.config.TargetWidth/2
	if math.Abs(targetX-objectX) >= minimumSeparation {
		return clamp(targetX, minimumTargetX, maximumTargetX)
	}
	direction := 1.0
	if targetX < objectX {
		direction = -1
	}
	candidate := objectX + direction*minimumSeparation
	if candidate < minimumTargetX || candidate > maximumTargetX {
		candidate = objectX - direction*minimumSeparation
	}
	return clamp(candidate, minimumTargetX, maximumTargetX)
}

// resetTerrain restores the manually configured terrain and then, for
// automatic randomized episodes, perturbs only its heights. Object and target
// resting heights are calculated after this call, so neither can be embedded
// in the terrain merely because the ground changed.
func (e *Environment) resetTerrain() {
	// Allocate a new slice rather than reusing e.config.Terrain's backing array:
	// Config is owned by the caller and must not be mutated by per-episode
	// terrain randomization.
	e.config.Terrain = append([]TerrainPoint(nil), e.baseTerrain...)
	randomization := e.config.Curriculum.Randomization
	if !randomization.Enabled || e.resetCount == 0 || randomization.TerrainHeightJitter == 0 {
		return
	}
	maximumHeight := e.config.Workspace.MaxY - e.config.GripperBodyHeight - e.config.GripperFingerLength - e.config.GripperClearance
	for index := range e.config.Terrain {
		e.config.Terrain[index].Y = clamp(e.config.Terrain[index].Y+e.symmetricJitter(randomization.TerrainHeightJitter), e.config.Workspace.MinY, maximumHeight)
	}
}

func (e *Environment) symmetricJitter(magnitude float64) float64 {
	if magnitude == 0 {
		return 0
	}
	return (2*e.random.Float64() - 1) * magnitude
}

func (e *Environment) currentCurriculumStage() CurriculumStage {
	if e.config.Curriculum.Stage == CurriculumAutomatic {
		return e.activeCurriculumStage
	}
	return e.config.Curriculum.Stage.canonical()
}

func (e *Environment) advanceCurriculum() {
	if e.config.Curriculum.Stage != CurriculumAutomatic {
		return
	}
	switch e.activeCurriculumStage {
	case CurriculumAlignAndContact:
		e.activeCurriculumStage = CurriculumGrasp
	case CurriculumGrasp:
		e.activeCurriculumStage = CurriculumLift
	case CurriculumLift:
		e.activeCurriculumStage = CurriculumTransportAndRelease
	case CurriculumTransportAndRelease:
		e.activeCurriculumStage = CurriculumFullPickAndPlace
	}
	e.contactSuccessStreak = 0
}

// approveCurriculumReview is an explicit human override for an automatic
// curriculum. It changes only the next reset distribution; it never marks an
// episode successful or manufactures a reward/replay transition.
func (e *Environment) approveCurriculumReview() error {
	if e.config.Curriculum.Stage != CurriculumAutomatic {
		return errors.New("manual curriculum review requires FORCE_CONTROL_CURRICULUM=auto")
	}
	if e.activeCurriculumStage == CurriculumFullPickAndPlace {
		return errors.New("full-pick-and-place is the final curriculum stage")
	}
	e.advanceCurriculum()
	return nil
}

func (e *Environment) step(action []float32) (State, float64, Outcome, bool, error) {
	return e.stepDecomposed(action, nil, nil)
}

func (e *Environment) stepDecomposed(action, flyBase, residual []float32) (State, float64, Outcome, bool, error) {
	if !e.ready {
		return State{}, 0, OutcomeRunning, false, errors.New("force-control environment must be reset before stepping")
	}
	finalValues, err := e.actionVector(action)
	if err != nil {
		return State{}, 0, OutcomeRunning, false, err
	}
	if e.state.Phase == PhaseSuccess || e.state.Phase == PhaseFailure {
		return e.state, 0, OutcomeFailure, true, errors.New("force-control episode is terminal; reset before stepping")
	}

	baseValues := [3]float64{}
	if len(flyBase) > 0 {
		baseValues, err = e.actionVector(flyBase)
		if err != nil {
			return State{}, 0, OutcomeRunning, false, fmt.Errorf("fly base action: %w", err)
		}
	}
	residualValues := finalValues
	if len(residual) > 0 {
		residualValues, err = e.actionVector(residual)
		if err != nil {
			return State{}, 0, OutcomeRunning, false, fmt.Errorf("SAC residual action: %w", err)
		}
	}
	selected := finalValues
	// A caller that supplies only the legacy/final action has no branches to
	// select. Execute that action in every mode instead of treating a missing
	// base or residual vector as an all-zero command.
	decompositionAvailable := len(flyBase) > 0 && len(residual) > 0
	if decompositionAvailable {
		switch e.config.EffectiveControlMode() {
		case ModeBaseOnly:
			selected = baseValues
		case ModePureRL:
			selected = residualValues
		case ModeResidual:
			selected = finalValues
		}
	}
	values, err := e.validatedAction(float32Action(selected))
	if err != nil {
		return State{}, 0, OutcomeRunning, false, err
	}
	filteredValues := e.filterAction(values)
	previous := e.state
	previousAction := e.lastAppliedAction
	e.baseAction = baseValues
	e.residualAction = residualValues
	e.finalAction = finalValues
	e.appliedAction = filteredValues
	e.contactBeforeMotion = e.contactDetected()
	e.state.BoundaryHit = false
	if e.boundaryHitFrames > 0 {
		e.boundaryHitFrames--
	}
	e.verticalRetractionBlocked = false
	mode := e.config.EffectiveControlMode()
	// A force already at/above the material threshold is a physical fracture.
	// The command clamp below prevents a new policy command from reaching this
	// state; it must not retroactively repair an externally corrupted state.
	if e.state.GripForce >= e.state.ObjectBreakForce {
		e.state.ObjectBroken = true
	}
	e.applyHorizontalControl(filteredValues[0])
	e.applyVerticalControl(filteredValues[1])
	if e.state.BoundaryHit {
		e.boundaryHitFrames++
	} else {
		e.boundaryHitFrames = 0
	}
	// Only upward motion requires extra carrying force. A lower-bound collision
	// can stop a downward carriage abruptly; treating that braking impulse as a
	// lift demand would create a fictitious force requirement and drop a valid
	// object at the target.
	e.verticalAcceleration = 0
	if e.state.GripperVelocityY > 0 {
		e.verticalAcceleration = math.Max(0, (e.state.GripperVelocityY-previous.GripperVelocityY)/e.config.TimeStep)
	}
	e.releaseCommanded = false
	e.applyGripControl(filteredValues[2])
	e.lastAppliedAction = filteredValues
	e.updateGripState()
	e.updateContactWithoutGripCounter()
	if e.state.Grip.ObjectAttached {
		e.wasEverGrasped = true
	}
	e.updateObjectPhysics()
	e.resolveTerrainCollision()
	if e.state.Grip.ObjectAttached && !e.state.Grip.Slipping && e.state.ObjectY >= e.requiredCarryHeight() {
		e.objectWasLifted = true
	}
	e.updatePlacementStability()
	e.state.EpisodeStep++
	if err := e.ValidateState(); err != nil {
		e.resetContactSuccessStreakOnFailure()
		e.failureReason, e.state.Phase = "invalid_state", PhaseFailure
		e.updateHomeostasis("workspace_violation")
		e.lastReward = RewardBreakdown{Homeostasis: e.lastEnergyDelta, Penalty: e.config.Reward.WorkspacePenalty, Total: e.lastEnergyDelta + e.config.Reward.WorkspacePenalty}
		return e.state, e.lastReward.Total, OutcomeFailure, true, nil
	}

	terminalReason := e.detectFailure(previous)
	if terminalReason != "" {
		e.resetContactSuccessStreakOnFailure()
		e.failureReason, e.state.Phase = terminalReason, PhaseFailure
		e.updateHomeostasis(terminalReason)
		rewardAction := filteredValues
		if mode == ModeResidual {
			rewardAction = residualValues
		}
		e.lastReward = e.rewardWithActions(previous, rewardAction, previousAction)
		e.lastReward.Penalty += e.failurePenalty(terminalReason)
		e.lastReward.Total += e.failurePenalty(terminalReason)
		return e.state, e.lastReward.Total, OutcomeFailure, true, nil
	}
	e.updatePhase(previous)
	e.updateHomeostasis("")
	if e.state.Phase == PhaseSuccess {
		rewardAction := filteredValues
		if mode == ModeResidual {
			rewardAction = residualValues
		}
		e.lastReward = e.rewardWithActions(previous, rewardAction, previousAction)
		if !e.successRewardAwarded {
			switch e.currentCurriculumStage() {
			case CurriculumAlignAndContact:
				e.lastReward.Contact = e.config.Reward.SuccessfulContactReward
				e.lastReward.Total += e.lastReward.Contact
			case CurriculumGrasp:
				// Legacy direct-force training already receives its attachment
				// event reward. Residual mode instead keeps that feedback small
				// and awards this clean terminal grasp milestone after the
				// configured verified hold.
				if e.config.EffectiveControlMode() == ModeResidual {
					e.lastReward.Success = e.config.Reward.SuccessfulGripReward
					e.lastReward.Total += e.lastReward.Success
				}
			case CurriculumLift:
				e.lastReward.Success = e.config.Reward.SuccessfulLiftReward
				e.lastReward.Total += e.lastReward.Success
			case CurriculumTransportAndRelease, CurriculumFullPickAndPlace:
				e.lastReward.Success = e.config.Reward.SuccessfulPlacement
				e.lastReward.Total += e.lastReward.Success
			}
			e.successRewardAwarded = true
		}
		if e.config.Curriculum.Stage == CurriculumAutomatic {
			if e.currentCurriculumStage() == CurriculumAlignAndContact {
				e.contactSuccessStreak++
				e.advanceCurriculumOnReset = e.contactSuccessStreak >= e.config.Curriculum.ContactSuccessesRequired
			} else {
				e.advanceCurriculumOnReset = true
			}
		}
		return e.state, e.lastReward.Total, OutcomeSuccess, true, nil
	}
	rewardAction := filteredValues
	if mode == ModeResidual {
		rewardAction = residualValues
	}
	e.lastReward = e.rewardWithActions(previous, rewardAction, previousAction)
	return e.state, e.lastReward.Total, OutcomeRunning, false, nil
}

func (e *Environment) actionVector(action []float32) ([3]float64, error) {
	if len(action) != 3 {
		return [3]float64{}, errors.New("force-control action must contain exactly three values")
	}
	values := [3]float64{}
	for index, value := range action {
		if math.IsNaN(float64(value)) || math.IsInf(float64(value), 0) {
			return [3]float64{}, errors.New("force-control action must contain only finite values")
		}
		values[index] = clamp(float64(value), -1, 1)
	}
	return values, nil
}

func float32Action(values [3]float64) []float32 {
	return []float32{float32(values[0]), float32(values[1]), float32(values[2])}
}

// resetContactSuccessStreakOnFailure makes the first automatic lesson require
// its configured number of consecutive successful episodes, rather than
// contacts accumulated across timeouts or unsafe episodes.
func (e *Environment) resetContactSuccessStreakOnFailure() {
	if e.config.Curriculum.Stage == CurriculumAutomatic && e.currentCurriculumStage() == CurriculumAlignAndContact {
		e.contactSuccessStreak = 0
	}
}

func (e *Environment) validatedAction(action []float32) ([3]float64, error) {
	if len(action) != 3 {
		return [3]float64{}, errors.New("force-control action must contain exactly three values")
	}
	values := [3]float64{}
	e.deadZoneRemoved = [3]bool{}
	e.filterDeadZoneRemoved = [3]bool{}
	for index, value := range action {
		if math.IsNaN(float64(value)) || math.IsInf(float64(value), 0) {
			return [3]float64{}, errors.New("force-control action must contain only finite values")
		}
		values[index] = clamp(float64(value), -1, 1)
		if values[index] != 0 && math.Abs(values[index]) < e.config.ActionDeadZone {
			e.deadZoneRemoved[index] = true
			values[index] = 0
		}
	}
	return values, nil
}

// filterAction rejects tiny policy noise and applies a first-order command
// filter. The explicit emergency release threshold bypasses filtering so it
// cannot be delayed by a prior closing command.
func (e *Environment) filterAction(raw [3]float64) [3]float64 {
	filtered := e.filteredAction
	for index, value := range raw {
		if index == 2 && value <= e.config.ReleaseActionThreshold {
			filtered[index] = value
			continue
		}
		// Grip is deliberately not smoothed: it is the actor's continuous force
		// rate command on every step. Smooth force behaviour is learned through
		// the transition dynamics and reward, not held by a hidden controller.
		if index == 2 {
			filtered[index] = value
			continue
		}
		// Do not allow smoothing to send a stale command in the opposite
		// direction from a new policy command. This is especially important for
		// vertical movement near the upper workspace limit: SAC exploration may
		// alternate signs, but a new negative command must never remain positive
		// merely because the previous filtered command was upward.
		if value != 0 && filtered[index]*value < 0 {
			// Brake through zero before accepting the opposite command.  Applying
			// alpha*value here still leaves a non-zero velocity in the old
			// direction and lets SAC exploration chatter around the grasp pose.
			// A one-step neutral command is a bounded, deterministic reversal and
			// prevents the controller from farming tiny back-and-forth progress.
			filtered[index] = 0
		} else {
			filtered[index] += e.config.ActionSmoothingAlpha * (value - filtered[index])
		}
		if math.Abs(filtered[index]) < e.config.ActionDeadZone {
			if filtered[index] != 0 {
				e.filterDeadZoneRemoved[index] = true
			}
			filtered[index] = 0
		}
	}
	e.filteredAction = filtered
	return filtered
}

func (e *Environment) applyHorizontalControl(value float64) {
	// During PhaseLowerToObject, a zero or tiny lateral command is still valid:
	// the policy is allowed to keep trimming horizontal error while descending,
	// and only an actual workspace boundary should clamp it to zero.
	targetVelocity := value * e.config.MaxHorizontalSpeed
	e.state.CarriageVelocityX = slew(e.state.CarriageVelocityX, targetVelocity, e.config.MaxHorizontalAcceleration*e.config.TimeStep)
	next := e.state.CarriageX + e.state.CarriageVelocityX*e.config.TimeStep
	safeMinX, safeMaxX, _, _ := e.safeBounds()
	e.state.CarriageX = clamp(next, safeMinX, safeMaxX)
	if e.state.CarriageX != next {
		e.state.CarriageVelocityX = 0
		e.filteredAction[0] = 0
		e.state.BoundaryHit = true
	}
}

func (e *Environment) applyVerticalControl(value float64) {
	targetVelocity := value * e.config.MaxVerticalSpeed
	e.state.GripperVelocityY = slew(e.state.GripperVelocityY, targetVelocity, e.config.MaxVerticalAcceleration*e.config.TimeStep)
	next := e.state.GripperY + e.state.GripperVelocityY*e.config.TimeStep
	_, _, safeMinY, safeMaxY := e.safeBounds()
	e.state.GripperY = clamp(next, safeMinY, safeMaxY)
	if e.state.GripperY != next {
		e.state.GripperVelocityY = 0
		e.filteredAction[1] = 0
		e.state.BoundaryHit = true
	}
}

func slew(current, target, maximumDelta float64) float64 {
	return current + clamp(target-current, -maximumDelta, maximumDelta)
}

func (e *Environment) applyGripControl(value float64) {
	// action[2] is a signed force-rate request. Only the explicit release band
	// opens the jaws, and it is rejected while the gripper is still airborne
	// above the release guide or before the explicit release phase.
	// A small negative command means "back off a little", not "drop the object".
	if value <= e.config.ReleaseActionThreshold {
		if e.state.GripperY > e.targetReleaseGuideHeight()+e.config.ReleaseTolerance || math.Abs(e.state.ObjectX-e.state.TargetX) > e.config.TargetWidth/2+e.config.ReleaseTolerance || e.state.Phase != PhaseReleaseObject || !e.objectHorizontallyInsideTarget() {
			e.state.GripperOpening = 0
			e.state.GripForce = clamp(e.state.GripForce, 0, e.maximumSafeGripForce())
			e.releaseCommanded = false
			return
		}
		e.state.GripperOpening = 1
		e.state.GripForce = 0
		e.releaseCommanded = true
		return
	}
	e.state.GripperOpening = 0
	// No target or hold force is selected here. The actor continuously controls
	// the signed actuator rate; this layer only enforces hardware/material caps.
	deltaForce := value * e.config.MaxGripForceRate * e.config.TimeStep
	// The safety layer does not choose a force setpoint. It only enforces the
	// absolute hardware/material envelope requested by the task specification.
	e.state.GripForce = clamp(e.state.GripForce+deltaForce, 0, e.maximumSafeGripForce())
}

func (e *Environment) updateGripState() {
	state := &e.state
	contact := e.contactDetected()
	// The kinematic attachment is updated after this state check. Use the
	// contact sampled before the carriage moved, otherwise a valid fast lift
	// would appear to lose contact merely because the object has not yet been
	// advanced to the new grasp point for this same fixed step.
	if state.Grip.ObjectAttached {
		contact = e.contactBeforeMotion
	}
	grip := GripState{
		GripperClosed:   state.GripperOpening <= e.config.ClosedOpeningThreshold,
		ContactDetected: contact,
	}
	grip.ForceValid = state.GripForce >= e.requiredForce() && state.GripForce < state.ObjectBreakForce

	// A force above the break threshold can damage an object only while the
	// gripper is in contact with it or is already carrying it. Squeezing empty
	// space must not mutate object state.
	if (state.Grip.ObjectAttached || grip.ContactDetected) && state.GripForce >= state.ObjectBreakForce {
		state.ObjectBroken = true
		grip.ObjectAttached = false
	} else if state.Grip.ObjectAttached {
		switch {
		case e.releaseCommanded:
			e.invalidContactFrames = 0
			e.slipFrames = 0
			grip.ObjectAttached = false
		case !grip.GripperClosed:
			e.invalidContactFrames = 0
			e.slipFrames = 0
			grip.ObjectAttached = false
		case !grip.ContactDetected:
			e.invalidContactFrames++
			e.slipFrames = 0
			grip.ObjectAttached = e.invalidContactFrames < e.config.GripDetachInvalidFrames
		case !grip.ForceValid:
			e.invalidContactFrames = 0
			e.slipFrames++
			grip.Slipping = true
			grip.ObjectAttached = e.slipFrames < e.config.SlipDetachFrames
		default:
			e.invalidContactFrames = 0
			e.slipFrames = 0
			grip.ObjectAttached = true
		}
	} else {
		e.invalidContactFrames = 0
		e.slipFrames = 0
		grip.ObjectAttached = grip.GripperClosed && grip.ContactDetected && grip.ForceValid
	}
	state.Grip = grip
	// Retained for callers compiled against the prior state struct. It is always
	// identical to the physically meaningful ObjectAttached bit.
	state.ObjectGrasped = grip.ObjectAttached
}

// updateContactWithoutGripCounter applies only to the grasp curriculum. The
// align-and-contact lesson is explicitly allowed to make stable zero-force
// contact, whereas grasp must teach the actor to begin a force ramp.
func (e *Environment) updateContactWithoutGripCounter() {
	if e.currentCurriculumStage() == CurriculumGrasp &&
		e.state.Grip.ContactDetected &&
		e.state.GripForce < e.config.Reward.ContactWithoutGripForceThreshold {
		e.contactWithoutGripFrames++
		return
	}
	e.contactWithoutGripFrames = 0
}

func (e *Environment) contactDetected() bool {
	horizontalError := math.Abs(e.state.CarriageX - e.state.ObjectX)
	verticalError := math.Abs(e.state.GripperY - e.objectGripHeight())
	return horizontalError <= e.config.GraspHorizontalTolerance && verticalError <= e.config.GraspVerticalTolerance
}

func (e *Environment) updateObjectPhysics() {
	if e.state.Grip.ObjectAttached {
		if e.state.Grip.Slipping {
			// Partial frictional support: insufficient force lets the object lag
			// and fall, giving the policy observable slip feedback and time to
			// increase its own force command before detachment.
			support := 1 - e.slipSeverity()
			e.state.ObjectVelocityX = e.state.CarriageVelocityX * support
			e.state.ObjectVelocityY = e.state.GripperVelocityY*support - e.config.Gravity*(1-support)*e.config.TimeStep
			e.state.ObjectX += e.state.ObjectVelocityX * e.config.TimeStep
			e.state.ObjectY += e.state.ObjectVelocityY * e.config.TimeStep
			return
		}
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
		if !e.state.Grip.ObjectAttached && e.objectInsideTarget() {
			e.state.ObjectPlaced = true
		}
	}
}

func (e *Environment) updatePlacementStability() {
	if e.state.ObjectPlaced && !e.state.Grip.ObjectAttached && math.Abs(e.state.ObjectVelocityX) <= e.config.StableVelocityThreshold && math.Abs(e.state.ObjectVelocityY) <= e.config.StableVelocityThreshold {
		e.state.PlacementStableSteps++
		return
	}
	e.state.PlacementStableSteps = 0
}

func (e *Environment) detectFailure(previous State) string {
	switch {
	case e.state.ObjectBroken:
		return "object_break"
	case e.boundaryHitFrames >= 5:
		return "boundary_collision"
	case e.objectOutOfBounds():
		return "workspace_violation"
	case e.objectWasLifted && !e.state.Grip.ObjectAttached && !e.objectInsideTarget():
		// A verified lift turns any later detach outside the target into a
		// material drop failure, even if the actor first tried to hide it by
		// changing phase or opening the gripper.
		return "mid_air_drop"
	case previous.Grip.ObjectAttached && !e.state.Grip.ObjectAttached && previous.Phase != PhaseReleaseObject:
		return "unsafe_drop"
	case previous.Phase == PhaseReleaseObject && !e.state.Grip.ObjectAttached && !e.objectInsideTarget():
		return "release_outside_target"
	case e.currentCurriculumStage() == CurriculumGrasp &&
		e.contactWithoutGripFrames >= e.config.Reward.ContactWithoutGripTimeoutFrames:
		return "idle_contact_timeout"
	case e.state.EpisodeStep >= e.maxEpisodeSteps():
		return "timeout"
	}
	return ""
}

func (e *Environment) maxEpisodeSteps() int {
	stage := e.currentCurriculumStage()
	if stage == CurriculumAlignAndContact && e.config.Curriculum.AlignEpisodeStepLimit > 0 {
		return e.config.Curriculum.AlignEpisodeStepLimit
	}
	if stage != CurriculumFullPickAndPlace && e.config.Curriculum.EpisodeStepLimit > 0 {
		return e.config.Curriculum.EpisodeStepLimit
	}
	return e.config.MaxEpisodeSteps
}

func (e *Environment) updatePhase(previous State) {
	stage := e.currentCurriculumStage()
	if stage == CurriculumGrasp {
		// A transient attachment is not a learned grasp skill. Count only an
		// uninterrupted secure hold; any detach or slip restarts verification.
		// Before attachment, retain the ordinary approach/lower/grip phase
		// transitions so the policy is still taught how to reach the object.
		if e.state.Grip.ObjectAttached && !e.state.Grip.Slipping {
			e.state.Phase = PhaseGripObject
			e.secureGripHoldFrames++
		} else {
			e.secureGripHoldFrames = 0
			e.updateStandardPhase(previous)
		}
		if e.secureGripHoldFrames >= e.requiredGraspHoldFrames() {
			e.state.Phase = PhaseSuccess
		}
		return
	}
	if stage == CurriculumLift {
		// A single frame above carry height is not a lift skill: it can be
		// immediately followed by a release. Require a continuous, secure hold
		// at height before issuing the terminal reward.
		if e.state.Grip.ObjectAttached && !e.state.Grip.Slipping && e.state.ObjectY >= e.requiredCarryHeight() {
			e.liftHoldFrames++
			e.state.Phase = PhaseLiftObject
			if e.liftHoldFrames >= e.config.Curriculum.LiftHoldFrames {
				e.state.Phase = PhaseSuccess
			}
		} else {
			e.liftHoldFrames = 0
			e.updateStandardPhase(previous)
		}
		return
	}
	if stage == CurriculumAlignAndContact {
		// Lesson 1 counts a real stable contact even if the gripper has already
		// reached an attached hold. The curriculum's first task is not “grasp
		// success”; it is “position and stabilize the end-effector against the object
		// long enough”. Counting both detached contact and a stable attached hold
		// prevents the policy from oscillating between contact and release while
		// still rewarding genuine 2s contact stability.
		e.updateStandardPhase(previous)
		stableContact := e.state.Grip.ContactDetected &&
			math.Abs(e.state.GripperVelocityY) <= e.config.StableVelocityThreshold &&
			math.Abs(e.state.CarriageVelocityX) <= e.config.StableVelocityThreshold &&
			math.Abs(e.state.CarriageX-e.state.ObjectX) <= e.config.HorizontalTolerance &&
			math.Abs(e.state.GripperY-e.objectGripHeight()) <= e.config.VerticalTolerance
		stableAttachedHold := e.state.Grip.ObjectAttached && !e.state.Grip.Slipping &&
			math.Abs(e.state.CarriageVelocityX) <= e.config.StableVelocityThreshold &&
			math.Abs(e.state.GripperVelocityY) <= e.config.StableVelocityThreshold
		if stableContact || stableAttachedHold {
			e.contactStableFrames++
		} else {
			e.contactStableFrames = 0
		}
		if e.contactStableFrames >= e.config.Curriculum.ContactStableSteps {
			e.state.Phase = PhaseSuccess
		}
		return
	}
	e.updateStandardPhase(previous)
}

func (e *Environment) requiredGraspHoldFrames() int {
	return int(math.Ceil(e.config.Curriculum.GraspHoldSeconds / e.config.TimeStep))
}

func (e *Environment) updateStandardPhase(previous State) {
	xError := math.Abs(e.state.CarriageX - e.state.ObjectX)
	switch previous.Phase {
	case PhaseApproachObject:
		if xError <= e.config.GraspHorizontalTolerance && math.Abs(e.state.CarriageVelocityX) <= e.config.StableVelocityThreshold {
			e.state.Phase = PhaseLowerToObject
		}
	case PhaseLowerToObject:
		if xError > e.config.AlignmentExitTolerance {
			e.state.Phase = PhaseApproachObject
		} else if math.Abs(e.state.GripperY-e.objectGripHeight()) <= e.config.VerticalTolerance {
			e.state.Phase = PhaseGripObject
		}
	case PhaseGripObject:
		if xError > e.config.AlignmentExitTolerance {
			e.state.Phase = PhaseApproachObject
		} else if math.Abs(e.state.GripperY-e.objectGripHeight()) > e.config.VerticalTolerance {
			e.state.Phase = PhaseLowerToObject
		} else if e.state.Grip.ObjectAttached && !e.state.Grip.Slipping {
			e.state.Phase = PhaseLiftObject
		}
	case PhaseLiftObject:
		if e.state.Grip.ObjectAttached && !e.state.Grip.Slipping && e.state.ObjectY >= e.requiredCarryHeight() {
			e.state.Phase = PhaseMoveToTarget
		}
	case PhaseMoveToTarget:
		if e.state.Grip.ObjectAttached && !e.state.Grip.Slipping && e.objectHorizontallyInsideTarget() {
			e.state.Phase = PhaseLowerAtTarget
		}
	case PhaseLowerAtTarget:
		if e.state.Grip.ObjectAttached && !e.state.Grip.Slipping && e.objectHorizontallyInsideTarget() && e.state.GripperY <= e.targetReleaseGuideHeight()+e.config.ReleaseTolerance {
			e.state.Phase = PhaseReleaseObject
		}
	case PhaseReleaseObject:
		if e.releaseCommanded && !e.state.Grip.ObjectAttached && e.objectInsideTarget() && e.objectStable() {
			e.state.Phase = PhaseSuccess
		}
	}
}

// reward is retained for focused legacy tests. Production stepping uses
// rewardWithActions so the full three-dimensional action and its predecessor
// are available for Markovian smoothness regularization.
func (e *Environment) reward(previous State, horizontalAction, gripRateAction, previousGripRateAction float64) RewardBreakdown {
	return e.rewardWithActions(previous, [3]float64{horizontalAction, 0, gripRateAction}, [3]float64{0, 0, previousGripRateAction})
}

func (e *Environment) rewardWithActions(previous State, action, previousAction [3]float64) RewardBreakdown {
	breakdown := RewardBreakdown{Homeostasis: e.lastEnergyDelta}
	// Time cost is always present. Homeostasis is an additional motivation
	// signal, not a replacement that can make hovering cost-free.
	breakdown.Penalty = e.config.Reward.TimePenalty
	attached := e.state.Grip.ObjectAttached
	stage := e.currentCurriculumStage()
	if e.state.Grip.ContactDetected && !e.state.ContactBonusAwarded {
		// This latch deliberately never resets until reset(). Re-touching an
		// object after backing off must not become an episode reward pump.
		breakdown.Contact += e.config.Reward.FirstContactBonus
		e.state.ContactBonusAwarded = true
	}
	// Contact has already paid its one sparse bonus. Freeze geometric progress
	// shaping from that point onward, otherwise a lower/retract/re-contact loop
	// can earn more than committing to grip force.
	if !attached && !e.state.ContactBonusAwarded {
		currentDX := math.Abs(e.state.CarriageX - e.state.ObjectX)
		previousDX := math.Abs(previous.CarriageX - previous.ObjectX)
		currentDY := e.graspGuideErrorFor(e.state)
		previousDY := e.graspGuideErrorFor(previous)
		switch {
		case currentDX > e.config.Reward.AlignmentEpsilonX:
			// Phase 1: reward the signed reduction in horizontal error over this
			// transition. Unlike a tanh(distance) potential, this remains useful
			// across the full rail: toward is positive, retreat is negative.
			deltaDX := previousDX - currentDX
			breakdown.Approach += e.config.Reward.ApproachProgressScale * deltaDX
			// Penalize only real lowering toward the grasp guide/clearance band,
			// not normal travel near the ceiling. The latter would train a
			// self-reinforcing “hover at the top rail” policy instead of an
			// approach-to-contact motion.
			safeTravelClearance := e.objectGripHeightFor(e.state) + e.config.GripperClearance + e.config.Reward.GraspZoneToleranceY
			prematureDescent := math.Max(0, previous.GripperY-e.state.GripperY)
			if previous.GripperY > e.state.GripperY && e.state.GripperY < safeTravelClearance {
				breakdown.Penalty -= e.config.Reward.PrematureLoweringPenaltyScale * prematureDescent
			}
		case currentDY > e.config.Reward.GraspZoneToleranceY:
			// Phase 2: the one-time event bit is exposed in the observation, so
			// this augmented state remains Markovian.
			if previousDX > e.config.Reward.AlignmentEpsilonX && !e.alignmentGateAwarded {
				breakdown.Contact += e.config.Reward.AlignmentGateBonus
				e.alignmentGateAwarded = true
			}
			breakdown.Descent = e.config.Reward.DescentProgressScale * (previousDY - currentDY)
			if math.Abs(e.state.GripperVelocityY) < e.config.Reward.HoverVelocityThreshold {
				breakdown.Hover = -e.config.Reward.HoverPenalty
				breakdown.Penalty += breakdown.Hover
				e.hoverPenaltyAccumulated -= breakdown.Hover
			}
			// Phase 3 has no approach reward: force/contact feedback below decides
			// whether a real grasp was made.
		}
	}
	regularizedAction, regularizedPrevious := action, previousAction
	if e.state.Grip.ContactDetected && action[2] > 0 {
		// Positive contact force is the residual loop's means of probing friction
		// and extinguishing slip. Do not make that exploration pay generic energy
		// or jerk costs; the near-break barrier still limits unsafe force.
		regularizedAction[2], regularizedPrevious[2] = 0, 0
	}
	breakdown.Smoothness = -e.config.Reward.ActionMagnitudePenaltyScale*actionSquared(regularizedAction) -
		e.config.Reward.ActionDeltaPenaltyScale*actionSquaredDifference(regularizedAction, regularizedPrevious)
	jerkY := regularizedAction[1] - regularizedPrevious[1]
	breakdown.Smoothness -= e.config.Reward.JerkYPenaltyScale * jerkY * jerkY
	for _, index := range [...]int{1, 2} { // vertical and force-rate actions
		if regularizedAction[index]*regularizedPrevious[index] < 0 {
			breakdown.Smoothness -= e.config.Reward.ActionFlipPenalty * math.Abs(regularizedAction[index]-regularizedPrevious[index])
		}
	}
	breakdown.Penalty += breakdown.Smoothness
	alignedX := math.Abs(e.state.CarriageX-e.state.ObjectX) <= e.config.Reward.AlignmentEpsilonX
	aboveGraspPose := e.state.GripperY > e.objectGripHeight()+e.config.Reward.GraspZoneToleranceY
	if previous.Phase == PhaseLowerToObject && alignedX && !e.state.Grip.ContactDetected && aboveGraspPose {
		// Lowering is the only phase in which moving upward is task-regressive.
		// Use both commanded action and measured velocity so inertia after an
		// action reversal cannot evade the penalty.
		isMovingUpwards := action[1] > e.config.ActionDeadZone || e.state.GripperVelocityY > e.config.Reward.HoverVelocityThreshold
		if isMovingUpwards {
			breakdown.Retraction = -e.config.Reward.UpwardRetractionPenalty
		}
		// Potential shaping already gives a negative descent reward for moving
		// away. Add a larger measured-distance term so a down/up/down sequence
		// has strictly negative return even if inertia obscures action sign.
		retractionDistance := math.Max(0, e.graspGuideErrorFor(e.state)-e.graspGuideErrorFor(previous))
		breakdown.Retraction -= e.config.Reward.RetractionDistancePenaltyScale * retractionDistance
		breakdown.Penalty += breakdown.Retraction
		// A second time cost makes an aligned lower trajectory prefer the shortest
		// route to the grasp pose over lingering for tiny depth rewards.
		breakdown.Penalty -= e.config.Reward.LoweringStepPenalty
	}
	if e.state.ContactBonusAwarded && !e.wasEverGrasped && previous.Grip.ContactDetected && !e.state.Grip.ContactDetected && !attached {
		// Penalize only the departure event. A continuing per-frame charge would
		// turn this latch into an unrelated terminal cost rather than scoring the
		// actual abandon-contact action.
		breakdown.Penalty -= e.config.Reward.LossOfContactPenalty
	}
	if stage != CurriculumAlignAndContact && !previous.Grip.ObjectAttached && attached && !e.gripBonusAwarded {
		breakdown.Grip = e.config.Reward.SuccessfulGripReward
		e.gripBonusAwarded = true
	}
	if !previous.Grip.ForceValid && e.state.Grip.ForceValid && attached && !e.secureGripBonusAwarded {
		// This transition is observable without exposing mass/friction or the
		// analytic required force: tactile slip disappeared while attachment held.
		breakdown.Grip += e.config.Residual.SecureGraspBonus
		e.secureGripBonusAwarded = true
	}
	if previous.Grip.ObjectAttached && attached && !e.state.Grip.Slipping && !e.state.ObjectBroken {
		// This intentionally remains smaller than TimePenalty. It makes keeping
		// a real grasp denser and safer than release, but cannot become a
		// stationary-hold reward pump during lift or transport.
		breakdown.Grip += e.config.Reward.AttachedHoldRewardPerSecond * e.config.TimeStep
	}
	if stage == CurriculumGrasp && previous.Grip.ObjectAttached && attached && !e.state.Grip.Slipping && !e.state.ObjectBroken {
		// Only the isolated grasp lesson rewards elapsed holding time. In the full
		// task, an attached object must earn its value through lift and delivery
		// progress rather than waiting safely in place.
		breakdown.Grip += e.config.Reward.GraspHoldRewardPerSecond * e.config.TimeStep
	}
	if previous.Phase == PhaseLiftObject && previous.Grip.ObjectAttached && attached && !e.state.Grip.Slipping {
		breakdown.Lift = e.config.Reward.LiftProgressScale * (e.state.ObjectY - previous.ObjectY)
	}
	if stage != CurriculumGrasp && previous.Phase == PhaseLiftObject && !e.objectWasLifted &&
		e.state.ObjectY < e.requiredCarryHeight() &&
		(!attached || e.state.Grip.Slipping || e.state.ObjectY <= previous.ObjectY) {
		// A lift phase with no upward object progress is not neutral. This is
		// deliberately independent of force-control semantics: the policy still
		// chooses force, but weak/no-force actions now receive an immediate cost
		// instead of waiting for a later drop or timeout.
		breakdown.LiftStall = -e.config.Reward.LiftStallPenalty
		breakdown.Penalty += breakdown.LiftStall
	}
	// Delivery reward is intentionally impossible without a secure attachment
	// and a verified lift. It uses the horizontal *object* distance to target,
	// not gripper motion; previous is s_t, so this is a Markov transition reward
	// rather than a mutable record that can be pumped by oscillation.
	if e.objectWasLifted && previous.Grip.ObjectAttached && attached && !e.state.Grip.Slipping {
		previousDistance := math.Abs(previous.ObjectX - previous.TargetX)
		currentDistance := math.Abs(e.state.ObjectX - e.state.TargetX)
		deliveryDelta := previousDistance - currentDistance
		breakdown.Delivery = e.config.Reward.TransportProgressScale * deliveryDelta
	}
	if previous.Grip.ObjectAttached && attached && previous.Phase != PhaseReleaseObject {
		if !(e.state.Grip.ContactDetected && action[2] > 0) {
			breakdown.Penalty -= e.config.Reward.GripActionChangePenalty * math.Abs(action[2]-previousAction[2])
			breakdown.Penalty -= e.config.Reward.GripForceChangePenaltyScale * math.Abs(e.state.GripForce-previous.GripForce)
		}
		if e.state.Grip.Slipping {
			// A continuous sensor-derived cost gives SAC a gradient toward more
			// force without revealing mass, friction, or requiredGripForce.
			breakdown.Penalty -= e.config.Residual.SlipSeverityPenaltyScale * e.slipSeverity()
		} else if e.state.Grip.ForceValid {
			// A per-frame stability payment is useful only in the isolated
			// grasp lesson. Paying it during full pick-and-place creates a
			// profitable "hold still forever" local optimum instead of making
			// the object reach the target.
			if stage == CurriculumGrasp {
				breakdown.Grip += e.config.Reward.AttachedForceStabilityReward
			}
			breakdown.Penalty -= e.config.Reward.ExcessGripForcePenaltyScale * math.Max(0, e.state.GripForce-e.requiredForce())
			barrierStart := e.config.Reward.NearBreakForceBarrierStartFraction * e.state.ObjectBreakForce
			usableBand := math.Max(e.state.ObjectBreakForce-barrierStart, 1e-9)
			risk := clamp((e.state.GripForce-barrierStart)/usableBand, 0, 1)
			breakdown.BreakRisk = -e.config.Reward.NearBreakForcePenaltyScale * risk * risk
			breakdown.Penalty += breakdown.BreakRisk
		}
	}
	if !previous.Grip.GripperClosed && e.state.Grip.GripperClosed && !e.state.Grip.ContactDetected {
		breakdown.Penalty += e.config.Reward.InvalidGripPenalty
	}
	// Closing in empty space remains costly on every step. The one-time event
	// penalty above marks the mistake; this small continuing cost prevents an
	// idle closed gripper from becoming a cheap equilibrium.
	if !attached && e.state.Grip.GripperClosed && !e.state.Grip.ContactDetected {
		breakdown.Penalty += e.config.Reward.EmptyGripStepPenalty
	}
	if !attached && !e.state.Grip.ContactDetected && e.state.GripForce > e.requiredForce() {
		// Scale quadratically through the usable force band: a slight excess is
		// recoverable, while approaching break force in empty space is clearly
		// worse than waiting for real contact. The reward never turns force into
		// an autonomous command; it only scores the actor's chosen rate action.
		usableBand := math.Max(e.state.ObjectBreakForce-e.requiredForce(), 1e-9)
		ratio := clamp((e.state.GripForce-e.requiredForce())/usableBand, 0, 1)
		breakdown.DetachedForce = e.config.Reward.DetachedExcessForcePenalty * ratio * ratio
		breakdown.Penalty += breakdown.DetachedForce
	}
	if stage != CurriculumAlignAndContact && e.state.Grip.ContactDetected && e.state.GripForce < e.requiredForce() {
		// Give the actor signed, step-local feedback while it ramps from zero to
		// the attachment threshold. A force decrease receives the same magnitude
		// in the negative direction, so cycling force cannot farm this reward.
		// This must operate beyond the isolated grasp lesson: a force deficit
		// during lift/transport is still physically unsafe and otherwise leaves
		// SAC with only a delayed slip/detach signal.
		deltaForce := e.state.GripForce - previous.GripForce
		breakdown.Grip += e.config.Reward.ForceProgressScale * deltaForce
	}
	if stage != CurriculumAlignAndContact && (e.state.Grip.ContactDetected || attached) && e.state.GripForce < e.requiredForce() {
		deficit := clamp((e.requiredForce()-e.state.GripForce)/math.Max(e.requiredForce(), 1e-9), 0, 1)
		breakdown.UnderGrip = -e.config.Reward.UnderGripPenaltyScale * deficit
		breakdown.Penalty += breakdown.UnderGrip
	}
	if previous.Phase == PhaseGripObject && e.state.Grip.ContactDetected && !attached {
		// The small per-step cost prevents a contact with insufficient force from
		// becoming a cheap equilibrium while the dense force-progress reward
		// above makes increasing force preferable to idling.
		if !e.state.Grip.ForceValid && !previous.Grip.GripperClosed {
			breakdown.Penalty += e.config.Reward.InsufficientGripPenalty
		}
		breakdown.Penalty += e.config.Reward.InsufficientGripStepPenalty
	}
	if !attached && !e.wasEverGrasped && e.gripperInsideTarget() && !e.emptyTargetPenaltyAwarded {
		breakdown.Penalty += e.config.Reward.EmptyTargetPenalty
		e.emptyTargetPenaltyAwarded = true
	}
	if e.state.BoundaryHit {
		breakdown.Penalty += e.config.Reward.BoundaryCollisionPenalty
	}
	if e.config.EffectiveControlMode() == ModeResidual && e.state.Phase == PhaseMoveToTarget && attached {
		// Penalize only residual corrections that amplify existing transport
		// velocity. Corrections opposing motion are legitimate inertia damping.
		amplifyingMotion := math.Max(0, e.state.CarriageVelocityX*action[0])
		breakdown.Stability = -e.config.Residual.DampingPenaltyScale * amplifyingMotion
		breakdown.Penalty += breakdown.Stability
	}
	breakdown.Total = breakdown.Approach + breakdown.Contact + breakdown.Grip + breakdown.Descent + breakdown.Lift + breakdown.Delivery + breakdown.Success + breakdown.Homeostasis + breakdown.Penalty
	return breakdown
}

// updateHomeostasis applies exactly one energy decay plus a possible one-time
// verified milestone gain or terminal safety loss for this valid environment
// step. It never changes an action or task phase.
func (e *Environment) updateHomeostasis(failureReason string) {
	e.lastEnergyDelta, e.lastEnergyDecay, e.lastEnergyFoodGain = 0, 0, 0
	e.lastEnergyEvent = energyEventNone
	if !e.config.Homeostasis.Enabled {
		return
	}

	delta := -e.config.Homeostasis.EnergyDecayPerStep
	e.lastEnergyDecay = -e.config.Homeostasis.EnergyDecayPerStep
	attached := e.state.Grip.ObjectAttached && !e.state.Grip.Slipping
	stage := e.currentCurriculumStage()
	switch {
	case failureReason == "object_break":
		delta -= e.config.Homeostasis.BreakEnergyLoss
		e.lastEnergyEvent = energyEventBreak
	case failureReason == "unsafe_drop" || failureReason == "mid_air_drop" || failureReason == "release_outside_target" || failureReason == "workspace_violation":
		delta -= e.config.Homeostasis.UnsafeDropEnergyLoss
		e.lastEnergyEvent = energyEventUnsafeDrop
	case e.state.Phase == PhaseSuccess && !e.successFoodAwarded:
		delta += e.config.Homeostasis.SuccessfulPlacementEnergyGain
		e.lastEnergyFoodGain = e.config.Homeostasis.SuccessfulPlacementEnergyGain
		e.successFoodAwarded = true
		e.lastEnergyEvent = energyEventSuccess
	case attached && e.objectHorizontallyInsideTarget() && !e.deliveryFoodAwarded:
		delta += e.config.Homeostasis.DeliveryEnergyGain
		e.lastEnergyFoodGain = e.config.Homeostasis.DeliveryEnergyGain
		e.deliveryFoodAwarded = true
		e.lastEnergyEvent = energyEventDelivery
	case attached && e.state.ObjectY >= e.requiredCarryHeight() &&
		(stage != CurriculumLift || e.liftHoldFrames >= e.config.Curriculum.LiftHoldFrames) && !e.liftFoodAwarded:
		delta += e.config.Homeostasis.LiftEnergyGain
		e.lastEnergyFoodGain = e.config.Homeostasis.LiftEnergyGain
		e.liftFoodAwarded = true
		e.lastEnergyEvent = energyEventLift
	case attached && !e.gripFoodAwarded:
		delta += e.config.Homeostasis.SecureGripEnergyGain
		e.lastEnergyFoodGain = e.config.Homeostasis.SecureGripEnergyGain
		e.gripFoodAwarded = true
		e.lastEnergyEvent = energyEventSecureGrasp
	}

	previousEnergy := e.state.Energy
	e.state.Energy = clamp(previousEnergy+delta, 0, 1)
	e.lastEnergyDelta = e.state.Energy - previousEnergy
	if e.lastEnergyDelta >= 0 && e.lastEnergyFoodGain > e.lastEnergyDelta {
		e.lastEnergyFoodGain = e.lastEnergyDelta
	}
}

func (e *Environment) failurePenalty(reason string) float64 {
	switch reason {
	case "object_break":
		if e.currentCurriculumStage() == CurriculumGrasp {
			return e.config.Reward.GraspBreakPenalty
		}
		return e.config.Reward.BreakPenalty
	case "idle_contact_timeout":
		return e.config.Reward.IdleContactTimeoutPenalty
	case "workspace_violation":
		return e.config.Reward.WorkspacePenalty
	case "mid_air_drop":
		return e.config.Reward.MidAirDropPenalty
	case "unsafe_drop", "release_outside_target":
		return e.config.Reward.DroppedObjectPenalty
	default:
		return e.config.Reward.UnsafeDropPenalty
	}
}

// failureReasonCode keeps the framework's numeric Info transport compact while
// allowing the API/UI to aggregate real terminal causes.
func (e *Environment) failureReasonCode() int {
	switch e.failureReason {
	case "timeout":
		return 1
	case "unsafe_drop":
		return 2
	case "release_outside_target":
		return 3
	case "object_break":
		return 4
	case "workspace_violation":
		return 5
	case "invalid_state":
		return 6
	case "idle_contact_timeout":
		return 7
	case "mid_air_drop":
		return 8
	default:
		return 0
	}
}

func (e *Environment) curriculumStageCode() int {
	switch e.currentCurriculumStage() {
	case CurriculumAlignAndContact:
		return 1
	case CurriculumGrasp:
		return 2
	case CurriculumLift:
		return 3
	case CurriculumTransportAndRelease:
		return 4
	case CurriculumFullPickAndPlace:
		return 5
	default:
		return 0
	}
}

func (e *Environment) observation() []float32 {
	state := e.state
	workspaceLengthX := e.config.Workspace.MaxX - e.config.Workspace.MinX
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
		normalize(state.CarriageX-state.ObjectX, -workspaceLengthX, workspaceLengthX),
		normalize(state.GripperY-e.objectGripHeightFor(state), -e.config.Workspace.MaxY, e.config.Workspace.MaxY),
		normalize(state.TargetX-state.ObjectX, -workspaceLengthX, workspaceLengthX),
		normalize(state.TargetY-state.ObjectY, -e.config.Workspace.MaxY, e.config.Workspace.MaxY),
		normalize01(1 - state.GripperOpening),
		boolValue(state.Grip.ObjectAttached),
		normalize(state.GripForce, 0, e.config.MaxGripForce),
		normalize01(e.slipSeverity()),
		normalize(e.verticalAcceleration, -e.config.MaxVerticalAcceleration, e.config.MaxVerticalAcceleration),
		normalize(float64(state.Phase), float64(PhaseIdle), float64(PhaseFailure)),
		boolValue(state.Grip.ContactDetected),
		e.requiredGripForceBaseline(),
		normalize01(state.Energy),
		boolValue(math.Abs(state.CarriageX-state.ObjectX) <= e.config.Reward.AlignmentEpsilonX),
		boolValue(e.graspGuideErrorFor(state) <= e.config.Reward.GraspZoneToleranceY),
		e.lastAppliedAction[0],
		e.lastAppliedAction[1],
		e.lastAppliedAction[2],
		boolValue(e.alignmentGateAwarded),
		boolValue(state.ContactBonusAwarded),
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

func (e *Environment) requiredGripForceBaseline() float64 {
	if !e.config.ExposeRequiredGripForceBaseline {
		return 0
	}
	return normalize(e.requiredForce(), 0, e.config.MaxGripForce)
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
func (e *Environment) graspPoseDistanceFor(state State) float64 {
	return math.Hypot(state.CarriageX-state.ObjectX, state.GripperY-e.objectGripHeightFor(state))
}

func (e *Environment) loweringErrorFor(state State) float64 {
	return math.Abs(state.GripperY - e.objectGripHeightFor(state))
}

// graspGuideErrorFor is the geometry-aware vertical error used by reward and
// observations. It compares the gripper guide with the object's grasp guide,
// rather than raw object-centre Y coordinates with incompatible origins.
func (e *Environment) graspGuideErrorFor(state State) float64 {
	return math.Abs(state.GripperY - e.objectGripHeightFor(state))
}

func (e *Environment) rewardPhaseID(state State) int {
	if math.Abs(state.CarriageX-state.ObjectX) > e.config.Reward.AlignmentEpsilonX {
		return 1
	}
	if e.graspGuideErrorFor(state) > e.config.Reward.GraspZoneToleranceY {
		return 2
	}
	return 3
}

func actionSquared(action [3]float64) float64 {
	return action[0]*action[0] + action[1]*action[1] + action[2]*action[2]
}

func actionSquaredDifference(action, previous [3]float64) float64 {
	return actionSquared([3]float64{action[0] - previous[0], action[1] - previous[1], action[2] - previous[2]})
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
		"target_x": state.TargetX, "target_y": state.TargetY, "velocity_x": state.CarriageVelocityX, "velocity_y": state.GripperVelocityY, "energy": state.Energy,
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
	if state.Energy < 0 || state.Energy > 1 {
		return fmt.Errorf("energy %g is outside [0, 1]", state.Energy)
	}
	return nil
}

func (e *Environment) requiredForce() float64 {
	// The clamp band is fixed for an episode: only the configured object mass,
	// gravity, and friction determine its lower edge. Vertical acceleration is
	// still exposed as telemetry, but must not make the displayed requirement or
	// safe force range jump between simulation steps.
	return e.state.ObjectMass * e.config.Gravity / (2 * e.state.ObjectFriction)
}

func (e *Environment) slipSeverity() float64 {
	if !e.state.Grip.ObjectAttached || e.requiredForce() <= 0 {
		return 0
	}
	return clamp((e.requiredForce()-e.state.GripForce)/e.requiredForce(), 0, 1)
}
func (e *Environment) objectInsideTarget() bool {
	return e.objectHorizontallyInsideTarget() && math.Abs(e.state.ObjectY-e.targetRestHeight()) <= e.config.ReleaseTolerance
}
func (e *Environment) objectHorizontallyInsideTarget() bool {
	return math.Abs(e.state.ObjectX-e.state.TargetX) <= e.config.TargetWidth/2
}
func (e *Environment) gripperInsideTarget() bool {
	return math.Abs(e.state.CarriageX-e.state.TargetX) <= e.config.TargetWidth/2
}
func (e *Environment) objectStable() bool {
	return e.state.PlacementStableSteps >= e.config.StablePlacementSteps
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
