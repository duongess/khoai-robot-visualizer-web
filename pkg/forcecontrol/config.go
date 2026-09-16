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
	// ContactStartHeightOffset is retained for existing launch configs. Lessons
	// no longer pre-position the gripper near the object.
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
	TimePenalty                  float64
	ApproachProgressScale        float64
	SuccessfulGripReward         float64
	LiftProgressScale            float64
	DeliveryProgressScale        float64
	SuccessfulPlacement          float64
	UnsafeDropPenalty            float64
	BreakPenalty                 float64
	WorkspacePenalty             float64
	InsufficientGripPenalty      float64
	InsufficientGripStepPenalty  float64
	GripForceProgressScale       float64
	GripActionChangePenalty      float64
	GripForceChangePenaltyScale  float64
	SlipPenalty                  float64
	ExcessGripForcePenaltyScale  float64
	// NearBreakForcePenaltyScale is a quadratic force barrier within the safe
	// band. It makes approaching break force costly before the terminal damage
	// event, rather than relying on a delayed discounted failure signal.
	NearBreakForcePenaltyScale   float64
	AttachedForceStabilityReward float64
	InvalidGripPenalty           float64
	EmptyGripStepPenalty         float64
	DetachedExcessForcePenalty   float64
	InactivityPenalty            float64
	EmptyTargetPenalty           float64
	DroppedObjectPenalty         float64
	BoundaryCollisionPenalty     float64
	LowerProgressScale           float64
	// LowerStallPenalty applies only while the carriage is horizontally
	// aligned in the lowering phase but fails to reduce its vertical grasp
	// error. It prevents X-axis dithering from being a cheap alternative to
	// attempting the learned descent.
	LowerStallPenalty            float64
	// ApproachStallPenalty applies while the object remains horizontally out of
	// reach and the policy fails to reduce that X error. Descending at the wrong
	// X coordinate therefore cannot replace a real approach.
	ApproachStallPenalty         float64
	// GraspHoldRewardPerSecond is dense positive feedback for sustaining a
	// secure physical attachment during the grasp lesson. It is time-scaled in
	// the environment so it remains stable if TimeStep changes.
	GraspHoldRewardPerSecond     float64
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
	ObjectBreakForce         float64
	InitialCarriageX         float64
	InitialGripperY          float64
	TargetX                  float64
	TargetWidth              float64
	HorizontalTolerance      float64
	VerticalTolerance        float64
	GraspHorizontalTolerance float64
	GraspVerticalTolerance   float64
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
			// A real contact needs only one settled physics frame within an
			// episode, but automatic curriculum requires consecutive contact
			// episodes before proceeding to grasp.
			ContactStableSteps:       1,
			ContactSuccessesRequired: 10,
			// The grasp lesson verifies that the learned policy can sustain a
			// real, non-slipping attachment for half a minute before lift begins.
			GraspHoldSeconds: 30,
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
			AlignEpisodeStepLimit: 250,
			EpisodeStepLimit:      1000,
		},
		Reward: RewardConfig{
			TimePenalty: -0.001,
			// The first learned subgoal is horizontal object approach. Signed
			// distance progress dominates the small time cost, while movement away
			// receives the equal negative term.
			ApproachProgressScale:        3.0,
			SuccessfulGripReward:         5.0,
			LiftProgressScale:            2.0,
			DeliveryProgressScale:        3.0,
			SuccessfulPlacement:          50.0,
			UnsafeDropPenalty:            -10.0,
			BreakPenalty:                 -20.0,
			WorkspacePenalty:             -20.0,
			InsufficientGripPenalty:      -1.0,
			InsufficientGripStepPenalty:  -0.05,
			GripForceProgressScale:       0.5,
			GripActionChangePenalty:      0.02,
			GripForceChangePenaltyScale:  0.02,
			SlipPenalty:                  -1.0,
			ExcessGripForcePenaltyScale:  0.05,
			NearBreakForcePenaltyScale:   1.50,
			AttachedForceStabilityReward: 0.02,
			InvalidGripPenalty:           -1.0,
			EmptyGripStepPenalty:         -0.01,
			// Empty-space force is not a grasp. A bounded ongoing cost makes
			// building force to the material limit before contact unattractive.
			DetachedExcessForcePenalty: -0.50,
			InactivityPenalty:          -0.005,
			EmptyTargetPenalty:         -2.0,
			DroppedObjectPenalty:       -10.0,
			BoundaryCollisionPenalty:   -0.25,
			// Descending toward the physical grasp guide needs a denser signal
			// than a distant terminal placement reward. This is still signed
			// progress, so upward/away motion is penalized symmetrically.
			LowerProgressScale: 3.0,
			// Once aligned over the object, lateral motion without vertical
			// progress must not be safer than attempting a grasp.
			LowerStallPenalty: -0.02,
			// While still horizontally out of reach, lowering alone is not useful
			// progress and must not be a cheap way to wait out an episode.
			ApproachStallPenalty: -0.02,
			// Thirty seconds of a secure hold earns 4.5 reward in addition to the
			// one-time grip and final lesson-success rewards.
			GraspHoldRewardPerSecond: 0.15,
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
