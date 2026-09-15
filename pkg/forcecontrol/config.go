package forcecontrol

import "math"

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
	AttachedForceStabilityReward float64
	InvalidGripPenalty           float64
	EmptyTargetPenalty           float64
	DroppedObjectPenalty         float64
	BoundaryCollisionPenalty     float64
	LowerProgressScale           float64
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
	MaxGripForce        float64
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
	MaxEpisodeSteps          int
	Reward                   RewardConfig
	Terrain                  []TerrainPoint
}

// DefaultConfig returns a deterministic configuration for the MVP task.
func DefaultConfig() Config {
	return Config{
		Seed:                     42,
		Workspace:                WorkspaceBounds{MinX: 0, MaxX: 6, MinY: 0, MaxY: 3.2},
		RailY:                    3.1,
		GripperWidth:             0.55,
		GripperBodyHeight:        0.16,
		GripperFingerLength:      0.30,
		GripperClearance:         0.05,
		MaxHorizontalSpeed:       1.5,
		MaxVerticalSpeed:         1.5,
		MaxGripForce:             20,
		MaxGripForceRate:         12,
		ReleaseActionThreshold:   -0.85,
		TimeStep:                 0.1,
		Gravity:                  9.81,
		ObjectWidth:              0.35,
		ObjectHeight:             0.25,
		InitialObjectX:           1.5,
		InitialObjectMass:        0.8,
		ObjectFriction:           0.35,
		ObjectBreakForce:         18,
		InitialCarriageX:         1.5,
		InitialGripperY:          2.8,
		TargetX:                  4.5,
		TargetWidth:              0.8,
		MaxEpisodeSteps:          300,
		HorizontalTolerance:      0.15,
		VerticalTolerance:        0.10,
		GraspHorizontalTolerance: 0.15,
		GraspVerticalTolerance:   0.10,
		ClosedOpeningThreshold:   0.35,
		StableVelocityThreshold:  0.05,
		StablePlacementSteps:     3,
		LiftClearance:            0.60,
		ReleaseTolerance:         0.08,
		ActionDeadZone:           0.03,
		Reward: RewardConfig{
			TimePenalty:                  -0.001,
			ApproachProgressScale:        1.0,
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
			AttachedForceStabilityReward: 0.02,
			InvalidGripPenalty:           -1.0,
			EmptyTargetPenalty:           -2.0,
			DroppedObjectPenalty:         -10.0,
			BoundaryCollisionPenalty:     -0.25,
			LowerProgressScale:           1.0,
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
