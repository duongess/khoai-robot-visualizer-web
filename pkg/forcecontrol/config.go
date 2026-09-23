package forcecontrol

import "math"

// HomeostasisConfig defines the environment-owned energy reserve. Energy is a
// small motivation signal, not a replacement for the physical task rewards.
type HomeostasisConfig struct {
	Enabled                       bool
	InitialEnergy                 float64
	EnergyDecayPerStep            float64
	SecureGripEnergyGain          float64
	LiftEnergyGain                float64
	DeliveryEnergyGain            float64
	SuccessfulPlacementEnergyGain float64
	UnsafeDropEnergyLoss          float64
	BreakEnergyLoss               float64
}

// CurriculumStage selects a physically real, progressively harder task
// distribution. It never injects actions or supplies a hidden controller.
type CurriculumStage string

const (
	// CurriculumAutomatic progresses a worker through the stages after verified
	// successes without restarting the learner or replacing policy weights.
	CurriculumAutomatic           CurriculumStage = "auto"
	CurriculumAlignAndContact     CurriculumStage = "align-and-contact"
	CurriculumGrasp               CurriculumStage = "grasp"
	CurriculumLift                CurriculumStage = "lift"
	CurriculumTransportAndRelease CurriculumStage = "transport-and-release"
	CurriculumFullPickAndPlace    CurriculumStage = "full-pick-and-place"
	// Legacy values remain accepted so an existing local launch configuration
	// does not fail at registration; new runs should use the finer stages.
	CurriculumLowerAndContact CurriculumStage = "lower-and-contact"
	CurriculumGraspAndLift    CurriculumStage = "grasp-and-lift"
)

func (stage CurriculumStage) Valid() bool {
	switch stage {
	case CurriculumAutomatic, CurriculumAlignAndContact, CurriculumGrasp, CurriculumLift, CurriculumTransportAndRelease, CurriculumFullPickAndPlace, CurriculumLowerAndContact, CurriculumGraspAndLift:
		return true
	default:
		return false
	}
}

func (stage CurriculumStage) canonical() CurriculumStage {
	switch stage {
	case CurriculumLowerAndContact:
		return CurriculumAlignAndContact
	case CurriculumGraspAndLift:
		return CurriculumGrasp
	default:
		return stage
	}
}

// CurriculumRandomization defines reproducible variation around the manually
// configured scene. It is reset-distribution data, never a hidden controller.
type CurriculumRandomization struct {
	Enabled              bool
	ObjectXJitter        float64
	TargetXJitter        float64
	ObjectMassJitter     float64
	ObjectFrictionJitter float64
	// ContactStartHeightJitter is retained for compatibility with existing
	// configs; it now jitters the detached gripper's reset height rather than
	// positioning the carriage above the object.
	ContactStartHeightJitter float64
	// TerrainHeightJitter independently varies each terrain control-point
	// height on every randomized reset. X coordinates remain fixed so the
	// piecewise-linear ground never becomes self-intersecting.
	TerrainHeightJitter float64
}

// CurriculumConfig contains only task-distribution and verification settings.
type CurriculumConfig struct {
	Stage CurriculumStage
	// ContactStartHeightOffset is retained for existing launch configs. It never
	// creates contact; align-and-contact may start nearby but remains detached.
	ContactStartHeightOffset float64
	ContactStableSteps       int
	// ContactSuccessesRequired is the number of consecutive completed
	// align-and-contact episodes required before automatic curriculum advances.
	// It is intentionally distinct from ContactStableSteps, which measures
	// physical frames within one episode.
	ContactSuccessesRequired int
	// GraspHoldSeconds is the uninterrupted secure-hold duration required to
	// complete the grasp lesson. It is converted to physics frames using the
	// configured fixed TimeStep, so changing simulation speed does not silently
	// make the lesson easier or harder.
	GraspHoldSeconds float64
	// LiftHoldFrames is the number of consecutive fixed-physics frames for
	// which a securely attached object must remain above carry height before
	// the lift lesson can succeed. It prevents a one-frame lift/release exploit.
	LiftHoldFrames int
	// AlignStartDistance is the maximum initial carriage-to-object offset for
	// lesson 1. The reset chooses a side with equal probability and then samples
	// a detached offset up to this limit.
	AlignStartDistance       float64
	AlignStartDistanceJitter float64
	// GraspStartDistance and GraspStartHeightOffset define a detached, nearby
	// reset distribution for the grasp lesson. They are not a scripted grasp:
	// the policy still has to align, lower, close, and build force itself.
	GraspStartDistance       float64
	GraspStartDistanceJitter float64
	GraspStartHeightOffset   float64
	GraspStartHeightJitter   float64
	Randomization            CurriculumRandomization
	// AlignEpisodeStepLimit is the short exploration horizon for the first
	// lesson. Later lessons use EpisodeStepLimit so they have time to carry out
	// approach, grasp, lift, and transport prerequisites.
	AlignEpisodeStepLimit int
	// EpisodeStepLimit applies only to a non-full curriculum stage when
	// positive. Shorter stages must reset frequently enough to sample their
	// narrow initial distribution instead of spending a full task horizon away
	// from the learning objective.
	EpisodeStepLimit int
}

// RewardConfig contains the reward-shaping constants for one episode.
type RewardConfig struct {
	// TimePenalty is applied on every simulation step, regardless of optional
	// homeostatic motivation, so execution time always has a visible cost.
	TimePenalty float64
	// AlignmentEpsilonX and GraspZoneToleranceY are the reward gates. They are
	// intentionally stricter than collision tolerances: the policy must center
	// before descent, then enter the physical grasp slice.
	AlignmentEpsilonX   float64
	GraspZoneToleranceY float64
	// These fields remain readable for existing launch configurations. The
	// horizontal reward is now transition-based through ApproachProgressScale,
	// not a saturated distance potential.
	AlignmentPotentialScale       float64
	AlignmentPotentialSigma       float64
	PrematureLoweringPenaltyScale float64
	AlignmentGateBonus            float64
	DescentProgressScale          float64
	HoverVelocityThreshold        float64
	HoverPenalty                  float64
	ActionMagnitudePenaltyScale   float64
	ActionDeltaPenaltyScale       float64
	// ActionFlipPenalty applies only to a non-zero sign reversal of the vertical
	// or force-rate action. It supplements the general action-delta cost and
	// specifically discourages up/down and squeeze/release flutter.
	ActionFlipPenalty float64
	// JerkYPenaltyScale is a quadratic cost on changes to the vertical action.
	// It makes rapid lower/retract ratcheting expensive even before a sign flip.
	JerkYPenaltyScale float64
	// ApproachProgressScale multiplies previousDX-currentDX, so every genuine
	// approach transition is rewarded and every retreat is penalized.
	ApproachProgressScale     float64
	AlignDistancePenaltyScale float64
	// SuccessfulContactReward is a one-time terminal reward for a verified
	// physical contact in the align-and-contact lesson. It is intentionally not
	// the pick-and-place placement reward.
	SuccessfulContactReward float64
	// FirstContactBonus is paid exactly once per episode on the first physical
	// contact. Re-contacting cannot pump this reward.
	FirstContactBonus    float64
	LossOfContactPenalty float64
	// ContactClosureReward remains accepted for source/config compatibility.
	// New code uses FirstContactBonus.
	ContactClosureReward  float64
	SuccessfulGripReward  float64
	LiftProgressScale     float64
	DeliveryProgressScale float64
	// TransportProgressScale rewards signed horizontal object-to-target
	// progress only after the object has genuinely been lifted.
	TransportProgressScale float64
	SuccessfulLiftReward   float64
	SuccessfulPlacement    float64
	UnsafeDropPenalty      float64
	BreakPenalty           float64
	// GraspBreakPenalty overrides BreakPenalty only for CurriculumGrasp. It is
	// intentionally softer during early force exploration; later transport and
	// placement lessons retain the full material-damage consequence.
	GraspBreakPenalty           float64
	WorkspacePenalty            float64
	InsufficientGripPenalty     float64
	InsufficientGripStepPenalty float64
	// ForceProgressScale rewards signed dF while touching the object below the
	// required force. It gives the actor a dense gradient while it ramps force
	// instead of making attachment the first non-zero signal.
	ForceProgressScale float64
	// UnderGripPenaltyScale penalizes the normalized shortfall below required
	// force whenever the jaws have real contact (or are still attached). It
	// creates a dense incentive to continue squeezing before slip/detach.
	UnderGripPenaltyScale float64
	// LiftStallPenalty applies while a previously secured object is below carry
	// height but fails to gain vertical height. It makes a weak grip that cannot
	// lift immediately worse than increasing the policy's force-rate command.
	LiftStallPenalty float64
	// ContactWithoutGripForceThreshold and ContactWithoutGripTimeoutFrames
	// define the grasp-only failure for resting in physical contact without
	// attempting a force ramp. IdleContactTimeoutPenalty is terminal.
	ContactWithoutGripForceThreshold float64
	ContactWithoutGripTimeoutFrames  int
	IdleContactTimeoutPenalty        float64
	// GripForceProgressScale remains accepted for source/config compatibility.
	// New code uses ForceProgressScale.
	GripForceProgressScale      float64
	GripActionChangePenalty     float64
	GripForceChangePenaltyScale float64
	SlipPenalty                 float64
	ExcessGripForcePenaltyScale float64
	// NearBreakForcePenaltyScale is a quadratic force barrier within the safe
	// band. It makes approaching break force costly before the terminal damage
	// event, rather than relying on a delayed discounted failure signal.
	NearBreakForcePenaltyScale float64
	// NearBreakForceBarrierStartFraction activates the barrier only above this
	// fraction of F_break. It must be strictly in (0, 1).
	NearBreakForceBarrierStartFraction float64
	AttachedForceStabilityReward       float64
	InvalidGripPenalty                 float64
	EmptyGripStepPenalty               float64
	DetachedExcessForcePenalty         float64
	InactivityPenalty                  float64
	EmptyTargetPenalty                 float64
	DroppedObjectPenalty               float64
	// MidAirDropPenalty is used after a verified lift when attachment is lost
	// outside the target zone. It is deliberately separate from an ordinary
	// failed release because the object is no longer safely supported.
	MidAirDropPenalty        float64
	BoundaryCollisionPenalty float64
	LowerProgressScale       float64
	AlignedPositionReward    float64
	StableAlignmentReward    float64
	AlignmentVelocityPenalty float64
	ActionNearTargetPenalty  float64
	// LowerStallPenalty applies only while the carriage is horizontally
	// aligned in the lowering phase but fails to reduce its vertical grasp
	// error. It prevents X-axis dithering from being a cheap alternative to
	// attempting the learned descent.
	LowerStallPenalty float64
	// UpwardRetractionPenalty applies when the policy moves upward while
	// horizontally aligned, detached, and still above the grasp pose.
	UpwardRetractionPenalty float64
	// RetractionDistancePenaltyScale makes every measured increase in grasp
	// height error costlier than an equal descent gain. It closes the
	// down/up/down ratcheting reward exploit even while residual velocity is
	// braking after an upward command has been rejected.
	RetractionDistancePenaltyScale float64
	// LoweringStepPenalty is an additional positive cost subtracted per step
	// during a detached, aligned LowerToObject phase.
	LoweringStepPenalty float64
	// ApproachStallPenalty applies while the object remains horizontally out of
	// reach and the policy fails to reduce that X error. Descending at the wrong
	// X coordinate therefore cannot replace a real approach.
	ApproachStallPenalty float64
	// GraspHoldRewardPerSecond is dense positive feedback for sustaining a
	// secure physical attachment during the grasp lesson. It is time-scaled in
	// the environment so it remains stable if TimeStep changes.
	GraspHoldRewardPerSecond float64
	// AttachedHoldRewardPerSecond applies to every non-slipping attachment in
	// every lesson. It is deliberately below the per-second time cost, so it
	// makes holding preferable to dropping without making stationary holding a
	// profitable way to consume a whole full-task episode.
	AttachedHoldRewardPerSecond float64
}

// WorkspaceBounds is the authoritative physical coordinate system. World Y is
// measured upward from the floor; screen-space inversion happens only in React.
type WorkspaceBounds struct {
	MinX float64
	MaxX float64
	MinY float64
	MaxY float64
}

// GripperBounds are geometry-aware valid bounds for the gripper reference point.
type GripperBounds struct {
	MinX float64
	MaxX float64
	MinY float64
	MaxY float64
}

// Config defines the physical simulation and task limits.
type Config struct {
	Seed      int64
	Workspace WorkspaceBounds
	RailY     float64
	// GripperWidth is the maximum jaw-to-jaw envelope, rather than only the
	// narrower actuator housing, so an open gripper remains fully in bounds.
	GripperWidth        float64
	GripperBodyHeight   float64
	GripperFingerLength float64
	GripperClearance    float64
	MaxHorizontalSpeed  float64
	MaxVerticalSpeed    float64
	// MaxHorizontalAcceleration and MaxVerticalAcceleration bound velocity
	// changes per fixed physics step, including reversals near a target.
	MaxHorizontalAcceleration float64
	MaxVerticalAcceleration   float64
	// ActionSmoothingAlpha is the fraction of a non-safety action accepted at
	// each physics step. It must be in (0, 1]; a release remains immediate.
	ActionSmoothingAlpha float64
	MaxGripForce         float64
	// GripDetachInvalidFrames and SlipDetachFrames model physical persistence;
	// neither field selects or maintains a force for the policy.
	GripDetachInvalidFrames int
	SlipDetachFrames        int
	// MaxGripForceRate is the magnitude of the differential force command in
	// N/s at action[2] = +/-1; it is not an absolute target force.
	MaxGripForceRate float64
	// ReleaseActionThreshold is the explicit low action that immediately opens
	// the jaws and releases all grip force. Other negative actions reduce force.
	ReleaseActionThreshold   float64
	TimeStep                 float64
	Gravity                  float64
	ObjectWidth              float64
	ObjectHeight             float64
	InitialObjectX           float64
	InitialObjectMass        float64
	ObjectFriction           float64
	ObjectMass               float64
	ObjectBreakForce         float64
	InitialCarriageX         float64
	InitialGripperY          float64
	TargetX                  float64
	TargetWidth              float64
	HorizontalTolerance      float64
	VerticalTolerance        float64
	GraspHorizontalTolerance float64
	GraspVerticalTolerance   float64
	AlignmentEnterTolerance  float64
	AlignmentExitTolerance   float64
	ClosedOpeningThreshold   float64
	StableVelocityThreshold  float64
	StablePlacementSteps     int
	LiftClearance            float64
	ReleaseTolerance         float64
	ActionDeadZone           float64
	// ExposeRequiredGripForceBaseline enables privileged analytic force only for
	// an explicit baseline experiment. Learned controllers keep this false.
	ExposeRequiredGripForceBaseline bool
	MaxEpisodeSteps                 int
	Homeostasis                     HomeostasisConfig
	Curriculum                      CurriculumConfig
	Reward                          RewardConfig
	Terrain                         []TerrainPoint
}

// DefaultConfig returns a deterministic configuration for the MVP task.
func DefaultConfig() Config {
	return Config{
		Seed:                      42,
		Workspace:                 WorkspaceBounds{MinX: 0, MaxX: 6, MinY: 0, MaxY: 3.2},
		RailY:                     3.1,
		GripperWidth:              0.55,
		GripperBodyHeight:         0.16,
		GripperFingerLength:       0.30,
		GripperClearance:          0.05,
		MaxHorizontalSpeed:        1.5,
		MaxVerticalSpeed:          1.5,
		MaxHorizontalAcceleration: 3.0,
		MaxVerticalAcceleration:   3.0,
		ActionSmoothingAlpha:      0.30,
		MaxGripForce:              20,
		GripDetachInvalidFrames:   3,
		SlipDetachFrames:          4,
		MaxGripForceRate:          12,
		ReleaseActionThreshold:    -0.85,
		TimeStep:                  0.1,
		Gravity:                   9.81,
		ObjectWidth:               0.35,
		ObjectHeight:              0.25,
		InitialObjectX:            1.5,
		InitialObjectMass:         0.8,
		ObjectFriction:            0.35,
		ObjectBreakForce:          18,
		// Deliberately independent from InitialObjectX. The controller observes
		// the object-relative delta and must learn the horizontal approach; reset
		// never moves the carriage onto the object on its behalf.
		InitialCarriageX:         3.0,
		InitialGripperY:          2.8,
		TargetX:                  4.5,
		TargetWidth:              0.8,
		MaxEpisodeSteps:          1000,
		HorizontalTolerance:      0.15,
		VerticalTolerance:        0.10,
		GraspHorizontalTolerance: 0.15,
		GraspVerticalTolerance:   0.10,
		AlignmentEnterTolerance:  0.15,
		AlignmentExitTolerance:   0.25,
		ClosedOpeningThreshold:   0.35,
		StableVelocityThreshold:  0.05,
		StablePlacementSteps:     3,
		LiftClearance:            0.60,
		ReleaseTolerance:         0.08,
		// A normalized SAC action of 0.03 was large enough to erase legitimate
		// early descent commands (for example -0.004). Hardware still clamps all
		// commands; this intentionally small, configurable dead zone only removes
		// numerical noise rather than exploration.
		ActionDeadZone: 0.001,
		Homeostasis: HomeostasisConfig{
			// Keep reward shaping auditable while validating the curriculum. This
			// can be enabled later as a separate motivation experiment.
			Enabled:                       true,
			InitialEnergy:                 0.60,
			EnergyDecayPerStep:            0.001,
			SecureGripEnergyGain:          0.06,
			LiftEnergyGain:                0.08,
			DeliveryEnergyGain:            0.10,
			SuccessfulPlacementEnergyGain: 0.30,
			UnsafeDropEnergyLoss:          0.10,
			BreakEnergyLoss:               0.20,
		},
		Curriculum: CurriculumConfig{
			Stage:                    CurriculumFullPickAndPlace,
			ContactStartHeightOffset: 0.30,
			// Lesson 1 should require a genuinely stable contact, not a single lucky
			// frame. At 0.1s per physics step, 20 frames is approximately 2s of
			// sustained alignment and slow motion on both the X and Y axes; the
			// automatic curriculum still requires 10 consecutive successful episodes
			// before advancing to the grasp lesson.
			ContactStableSteps:       5,
			ContactSuccessesRequired: 10,
			// A short secure hold verifies a real attachment before lift, without
			// making the early curriculum excessively sparse.
			GraspHoldSeconds: 10,
			LiftHoldFrames:   15,
			// Start with a narrow, symmetric +/-1m approach distribution. This
			// maximum can be increased in a later curriculum experiment without
			// ever spawning contact or attachment.
			AlignStartDistance:       1.0,
			AlignStartDistanceJitter: 0,
			// Grasp begins close to, but deliberately outside, the physical
			// contact tolerances. This keeps the lesson focused on the grasp
			// rather than repeatedly relearning the already mastered approach.
			GraspStartDistance:       0.35,
			GraspStartDistanceJitter: 0.10,
			GraspStartHeightOffset:   0.35,
			GraspStartHeightJitter:   0.10,
			// Disabled for the deterministic default scene. The demo turns this on
			// for FORCE_CONTROL_CURRICULUM=auto, where every lesson benefits from
			// varied but reproducible reset conditions.
			Randomization: CurriculumRandomization{
				ObjectXJitter:            1.20,
				TargetXJitter:            1.00,
				ObjectMassJitter:         0.15,
				ObjectFrictionJitter:     0.06,
				ContactStartHeightJitter: 0.15,
				TerrainHeightJitter:      0.25,
			},
			// The first lesson resets quickly for efficient SAC exploration. Later
			// lessons retain enough horizon to perform prerequisite skills without
			// a scripted grasp or attachment at reset.
			AlignEpisodeStepLimit: 200,
			EpisodeStepLimit:      250,
		},
		Reward: RewardConfig{
			TimePenalty:         -0.01,
			AlignmentEpsilonX:   0.03,
			GraspZoneToleranceY: 0.02,
			// Horizontal shaping is transition-based through
			// ApproachProgressScale; retain the legacy potential disabled.
			AlignmentPotentialScale:       0,
			AlignmentPotentialSigma:       0.25,
			PrematureLoweringPenaltyScale: 0.15,
			AlignmentGateBonus:            2.0,
			DescentProgressScale:          3.0,
			HoverVelocityThreshold:        0.03,
			HoverPenalty:                  0.05,
			ActionMagnitudePenaltyScale:   0.002,
			ActionDeltaPenaltyScale:       0.01,
			ActionFlipPenalty:             0.15,
			JerkYPenaltyScale:             0.15,
			// Match vertical descent shaping so every metre of genuine approach
			// earns an immediate, signed transition reward.
			ApproachProgressScale:     3.0,
			AlignDistancePenaltyScale: 0.02,
			SuccessfulContactReward:   5.0,
			FirstContactBonus:         2.0,
			LossOfContactPenalty:      2.5,
			ContactClosureReward:      2.0,
			// The attachment event is helpful exploration feedback, but it must
			// never outweigh the consequence of immediately dropping the object.
			SuccessfulGripReward:        8.0,
			LiftProgressScale:           2.0,
			DeliveryProgressScale:       3.0,
			TransportProgressScale:      3.0,
			SuccessfulLiftReward:        20.0,
			SuccessfulPlacement:         50.0,
			UnsafeDropPenalty:           -10.0,
			BreakPenalty:                -20.0,
			GraspBreakPenalty:           -5.0,
			WorkspacePenalty:            -20.0,
			InsufficientGripPenalty:     -0.5,
			InsufficientGripStepPenalty: -0.01,
			// A +1 force-rate command increases 1.2N/step by default. Ramping
			// from 0 to about 11.2N therefore returns approximately +5.4 reward
			// before attachment, giving SAC a dense, low-risk exploration signal.
			ForceProgressScale:                 0.5,
			UnderGripPenaltyScale:              1.00,
			LiftStallPenalty:                   0.30,
			ContactWithoutGripForceThreshold:   2.0,
			ContactWithoutGripTimeoutFrames:    20,
			IdleContactTimeoutPenalty:          -6.0,
			GripForceProgressScale:             1.0,
			GripActionChangePenalty:            0.02,
			GripForceChangePenaltyScale:        0.02,
			SlipPenalty:                        -1.0,
			ExcessGripForcePenaltyScale:        0.05,
			NearBreakForcePenaltyScale:         0.30,
			NearBreakForceBarrierStartFraction: 0.85,
			AttachedForceStabilityReward:       0.02,
			InvalidGripPenalty:                 -1.0,
			EmptyGripStepPenalty:               -0.01,
			// Empty-space force is not a grasp. A bounded ongoing cost makes
			// building force to the material limit before contact unattractive.
			DetachedExcessForcePenalty: -0.50,
			InactivityPenalty:          -0.005,
			EmptyTargetPenalty:         -2.0,
			DroppedObjectPenalty:       -20.0,
			MidAirDropPenalty:          -25.0,
			BoundaryCollisionPenalty:   -0.25,
			// Descending toward the physical grasp guide needs a denser signal
			// than a distant terminal placement reward. This is still signed
			// progress, so upward/away motion is penalized symmetrically.
			LowerProgressScale:       3.0,
			AlignedPositionReward:    0.002,
			StableAlignmentReward:    0.005,
			AlignmentVelocityPenalty: 0.05,
			ActionNearTargetPenalty:  0.02,
			// Once aligned over the object, lateral motion without vertical
			// progress must not be safer than attempting a grasp.
			LowerStallPenalty:              -0.02,
			UpwardRetractionPenalty:        1.00,
			RetractionDistancePenaltyScale: 6.00,
			LoweringStepPenalty:            0.04,
			// While still horizontally out of reach, lowering alone is not useful
			// progress and must not be a cheap way to wait out an episode.
			ApproachStallPenalty: -0.02,
			// The grasp lesson's secure hold earns dense feedback after a valid
			// attachment; transport relies on object-to-target progress instead.
			GraspHoldRewardPerSecond: 0.50,
			// +0.005/0.1s step is smaller than the -0.01 base time cost. It
			// supplies a dense force-maintenance signal in lift/transport without
			// paying the agent to wait motionless forever.
			AttachedHoldRewardPerSecond: 0.05,
		},
		Terrain: []TerrainPoint{{X: 0, Y: 0.3}, {X: 1.5, Y: 0.3}, {X: 3, Y: 0.5}, {X: 4.5, Y: 0.25}, {X: 6, Y: 0.25}},
	}
}

// TerrainPoint is a control point for the piecewise-linear terrain surface.
type TerrainPoint struct {
	X float64
	Y float64
}

// SafeGripperBounds derives valid reference-point limits from the one workspace
// configuration and physical gripper geometry.
func SafeGripperBounds(config Config, carriageX float64) GripperBounds {
	workspace := config.Workspace
	return GripperBounds{
		MinX: workspace.MinX + config.GripperWidth/2,
		MaxX: workspace.MaxX - config.GripperWidth/2,
		MinY: terrainHeightForConfig(config, carriageX) + config.GripperFingerLength + config.GripperClearance,
		MaxY: workspace.MaxY - config.GripperBodyHeight,
	}
}

// GraspHeight returns a reachable jaw-guide height for an object.  The object
// top alone may be below the physical lower gripper limit because the fingers
// extend downward from the guide, so the geometry-aware safe minimum wins.
func GraspHeight(config Config, objectY, carriageX float64) float64 {
	return math.Max(
		objectY+config.ObjectHeight/2+config.GripperClearance,
		SafeGripperBounds(config, carriageX).MinY,
	)
}

func terrainHeightForConfig(config Config, x float64) float64 {
	points := config.Terrain
	if len(points) == 0 {
		return config.Workspace.MinY
	}
	if x <= points[0].X {
		return points[0].Y
	}
	for index := 1; index < len(points); index++ {
		if x <= points[index].X {
			left, right := points[index-1], points[index]
			return left.Y + (x-left.X)*(right.Y-left.Y)/(right.X-left.X)
		}
	}
	return points[len(points)-1].Y
}
