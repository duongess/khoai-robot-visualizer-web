package forcecontrol

// RewardConfig contains the reward-shaping constants for one episode.
type RewardConfig struct {
	StepPenalty         float64
	ApproachScale       float64
	SafeGraspReward     float64
	LiftScale           float64
	TargetProgressScale float64
	PlacementReward     float64
	SlipPenalty         float64
	DropPenalty         float64
	BreakPenalty        float64
}

// Config defines the physical simulation and task limits.
type Config struct {
	Seed               int64
	WorldMinX          float64
	WorldMaxX          float64
	WorldMinY          float64
	WorldMaxY          float64
	MaxHorizontalSpeed float64
	MaxVerticalSpeed   float64
	MaxGripForce       float64
	TimeStep           float64
	Gravity            float64
	ObjectWidth        float64
	ObjectHeight       float64
	InitialObjectX     float64
	InitialObjectMass  float64
	ObjectFriction     float64
	ObjectBreakForce   float64
	InitialCarriageX   float64
	InitialGripperY    float64
	TargetX            float64
	TargetWidth        float64
	MaxEpisodeSteps    int
	Reward             RewardConfig
	Terrain            []TerrainPoint
}

// DefaultConfig returns a deterministic configuration for the MVP task.
func DefaultConfig() Config {
	return Config{
		Seed:               42,
		WorldMinX:          0,
		WorldMaxX:          6,
		WorldMinY:          0,
		WorldMaxY:          3.5,
		MaxHorizontalSpeed: 1.5,
		MaxVerticalSpeed:   1.5,
		MaxGripForce:       20,
		TimeStep:           0.1,
		Gravity:            9.81,
		ObjectWidth:        0.35,
		ObjectHeight:       0.25,
		InitialObjectX:     1.5,
		InitialObjectMass:  0.8,
		ObjectFriction:     0.35,
		ObjectBreakForce:   18,
		InitialCarriageX:   1.5,
		InitialGripperY:    2.8,
		TargetX:            4.5,
		TargetWidth:        0.8,
		MaxEpisodeSteps:    300,
		Reward: RewardConfig{
			StepPenalty:         -0.01,
			ApproachScale:       0.2,
			SafeGraspReward:     1,
			LiftScale:           0.4,
			TargetProgressScale: 0.5,
			PlacementReward:     10,
			SlipPenalty:         -0.2,
			DropPenalty:         -5,
			BreakPenalty:        -10,
		},
		Terrain: []TerrainPoint{{X: 0, Y: 0.3}, {X: 1.5, Y: 0.3}, {X: 3, Y: 0.5}, {X: 4.5, Y: 0.25}, {X: 6, Y: 0.25}},
	}
}

// TerrainPoint is a control point for the piecewise-linear terrain surface.
type TerrainPoint struct {
	X float64
	Y float64
}
