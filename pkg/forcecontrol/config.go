package forcecontrol

// RewardConfig contains the reward-shaping constants for one episode.
type RewardConfig struct {
	TimePenalty             float64
	ApproachProgressScale   float64
	SuccessfulGripReward    float64
	LiftProgressScale       float64
	DeliveryProgressScale   float64
	SuccessfulPlacement     float64
	UnsafeDropPenalty       float64
	BreakPenalty            float64
	WorkspacePenalty        float64
	InsufficientGripPenalty float64
}

// Config defines the physical simulation and task limits.
type Config struct {
	Seed                int64
	WorldMinX           float64
	WorldMaxX           float64
	WorldMinY           float64
	WorldMaxY           float64
	MaxHorizontalSpeed  float64
	MaxVerticalSpeed    float64
	MaxGripForce        float64
	TimeStep            float64
	Gravity             float64
	ObjectWidth         float64
	ObjectHeight        float64
	InitialObjectX      float64
	InitialObjectMass   float64
	ObjectFriction      float64
	ObjectBreakForce    float64
	InitialCarriageX    float64
	InitialGripperY     float64
	TargetX             float64
	TargetWidth         float64
	HorizontalTolerance float64
	VerticalTolerance   float64
	LiftClearance       float64
	ReleaseTolerance    float64
	ActionDeadZone      float64
	MaxEpisodeSteps     int
	Reward              RewardConfig
	Terrain             []TerrainPoint
}

// DefaultConfig returns a deterministic configuration for the MVP task.
func DefaultConfig() Config {
	return Config{
		Seed:                42,
		WorldMinX:           0,
		WorldMaxX:           6,
		WorldMinY:           0,
		WorldMaxY:           3.5,
		MaxHorizontalSpeed:  1.5,
		MaxVerticalSpeed:    1.5,
		MaxGripForce:        20,
		TimeStep:            0.1,
		Gravity:             9.81,
		ObjectWidth:         0.35,
		ObjectHeight:        0.25,
		InitialObjectX:      1.5,
		InitialObjectMass:   0.8,
		ObjectFriction:      0.35,
		ObjectBreakForce:    18,
		InitialCarriageX:    1.5,
		InitialGripperY:     2.8,
		TargetX:             4.5,
		TargetWidth:         0.8,
		MaxEpisodeSteps:     300,
		HorizontalTolerance: 0.15,
		VerticalTolerance:   0.10,
		LiftClearance:       0.60,
		ReleaseTolerance:    0.08,
		ActionDeadZone:      0.03,
		Reward: RewardConfig{
			TimePenalty:             -0.001,
			ApproachProgressScale:   1.0,
			SuccessfulGripReward:    5.0,
			LiftProgressScale:       2.0,
			DeliveryProgressScale:   3.0,
			SuccessfulPlacement:     50.0,
			UnsafeDropPenalty:       -10.0,
			BreakPenalty:            -20.0,
			WorkspacePenalty:        -20.0,
			InsufficientGripPenalty: -1.0,
		},
		Terrain: []TerrainPoint{{X: 0, Y: 0.3}, {X: 1.5, Y: 0.3}, {X: 3, Y: 0.5}, {X: 4.5, Y: 0.25}, {X: 6, Y: 0.25}},
	}
}

// TerrainPoint is a control point for the piecewise-linear terrain surface.
type TerrainPoint struct {
	X float64
	Y float64
}
