package forcecontrol

import (
	"errors"
	"math"
	"math/rand"
)

const observationDimension = 19

// State is the physical state of the pick-and-place environment.
type State struct {
	CarriageX        float64
	GripperY         float64
	GripperOpening   float64
	GripForce        float64
	ObjectX          float64
	ObjectY          float64
	ObjectVelocityX  float64
	ObjectVelocityY  float64
	ObjectMass       float64
	ObjectFriction   float64
	ObjectBreakForce float64
	TargetX          float64
	TargetY          float64
	ObjectGrasped    bool
	ObjectBroken     bool
	ObjectPlaced     bool
	EpisodeStep      int
}

// Environment owns all mutable state for one independent task instance.
type Environment struct {
	config Config
	seed   int64
	random *rand.Rand
	state  State
	ready  bool
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
	}
	e.ready = true
	return e.state
}

func (e *Environment) step(action []float32) (State, float64, Outcome, bool, error) {
	if !e.ready {
		return State{}, 0, OutcomeRunning, false, errors.New("force-control environment must be reset before stepping")
	}
	if len(action) != 3 {
		return State{}, 0, OutcomeRunning, false, errors.New("force-control action must contain exactly three values")
	}
	values := [3]float64{}
	for i, value := range action {
		if math.IsNaN(float64(value)) || math.IsInf(float64(value), 0) {
			return State{}, 0, OutcomeRunning, false, errors.New("force-control action must contain only finite values")
		}
		values[i] = clamp(float64(value), -1, 1)
	}

	previous := e.state
	e.applyHorizontalControl(values[0])
	e.applyVerticalControl(values[1])
	e.applyGripControl(values[2])
	e.updateGraspState()
	e.updateObjectPhysics()
	e.resolveTerrainCollision()
	e.clampPositions()
	e.state.EpisodeStep++

	outcome := OutcomeRunning
	done := false
	reward := e.reward(previous)
	if e.state.ObjectBroken {
		outcome, done, reward = OutcomeFailure, true, reward+e.config.Reward.BreakPenalty
	} else if e.state.ObjectPlaced {
		outcome, done, reward = OutcomeSuccess, true, reward+e.config.Reward.PlacementReward
	} else if e.objectOutOfBounds() {
		outcome, done, reward = OutcomeFailure, true, reward+e.config.Reward.DropPenalty
	} else if e.state.EpisodeStep >= e.config.MaxEpisodeSteps {
		outcome, done, reward = OutcomeFailure, true, reward+e.config.Reward.DropPenalty
	}
	return e.state, reward, outcome, done, nil
}

func (e *Environment) applyHorizontalControl(value float64) {
	e.state.CarriageX += value * e.config.MaxHorizontalSpeed * e.config.TimeStep
}

func (e *Environment) applyVerticalControl(value float64) {
	e.state.GripperY += value * e.config.MaxVerticalSpeed * e.config.TimeStep
}

func (e *Environment) applyGripControl(value float64) {
	e.state.GripperOpening = clamp((1-value)/2, 0, 1)
	e.state.GripForce = clamp((value+1)/2*e.config.MaxGripForce, 0, e.config.MaxGripForce)
}

func (e *Environment) updateGraspState() {
	required := e.requiredForce()
	horizontalOverlap := math.Abs(e.state.CarriageX-e.state.ObjectX) <= e.config.ObjectWidth
	verticalClose := math.Abs(e.state.GripperY-(e.state.ObjectY+e.config.ObjectHeight/2)) <= e.config.ObjectHeight
	canGrasp := horizontalOverlap && verticalClose && e.state.GripperOpening <= 0.35 && e.state.GripForce >= required
	if e.state.GripForce >= e.state.ObjectBreakForce {
		e.state.ObjectBroken = true
		e.state.ObjectGrasped = false
		return
	}
	if e.state.ObjectGrasped && e.state.GripForce < required {
		e.state.ObjectGrasped = false
	} else if !e.state.ObjectGrasped && canGrasp {
		e.state.ObjectGrasped = true
	}
}

func (e *Environment) updateObjectPhysics() {
	if e.state.ObjectGrasped {
		e.state.ObjectVelocityX = e.state.CarriageX - e.state.ObjectX
		e.state.ObjectVelocityY = e.state.GripperY - (e.state.ObjectY + e.config.ObjectHeight/2)
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
		e.state.ObjectY = minimumY
		e.state.ObjectVelocityY = 0
		e.state.ObjectVelocityX *= 0.8
		if !e.state.ObjectGrasped && math.Abs(e.state.ObjectX-e.state.TargetX) <= e.config.TargetWidth/2 {
			e.state.ObjectPlaced = true
		}
	}
}

func (e *Environment) clampPositions() {
	e.state.CarriageX = clamp(e.state.CarriageX, e.config.WorldMinX, e.config.WorldMaxX)
	e.state.GripperY = clamp(e.state.GripperY, e.config.WorldMinY, e.config.WorldMaxY)
}

func (e *Environment) terrainHeight(x float64) float64 {
	points := e.config.Terrain
	if len(points) == 0 {
		return e.config.WorldMinY
	}
	if x <= points[0].X {
		return points[0].Y
	}
	for i := 1; i < len(points); i++ {
		if x <= points[i].X {
			left, right := points[i-1], points[i]
			ratio := (x - left.X) / (right.X - left.X)
			return left.Y + ratio*(right.Y-left.Y)
		}
	}
	return points[len(points)-1].Y
}

func (e *Environment) requiredForce() float64 {
	return e.state.ObjectMass * e.config.Gravity / (2 * e.state.ObjectFriction)
}

func (e *Environment) objectOutOfBounds() bool {
	return e.state.ObjectX < e.config.WorldMinX || e.state.ObjectX > e.config.WorldMaxX || e.state.ObjectY < e.config.WorldMinY-1
}

func (e *Environment) reward(previous State) float64 {
	previousObjectDistance := math.Hypot(previous.CarriageX-previous.ObjectX, previous.GripperY-previous.ObjectY)
	nextObjectDistance := math.Hypot(e.state.CarriageX-e.state.ObjectX, e.state.GripperY-e.state.ObjectY)
	reward := e.config.Reward.StepPenalty + e.config.Reward.ApproachScale*(previousObjectDistance-nextObjectDistance)
	if e.state.ObjectGrasped && !previous.ObjectGrasped {
		reward += e.config.Reward.SafeGraspReward
	}
	if e.state.ObjectGrasped {
		reward += e.config.Reward.LiftScale * (e.state.ObjectY - previous.ObjectY)
		previousTargetDistance := math.Abs(previous.ObjectX - previous.TargetX)
		nextTargetDistance := math.Abs(e.state.ObjectX - e.state.TargetX)
		reward += e.config.Reward.TargetProgressScale * (previousTargetDistance - nextTargetDistance)
	}
	if e.state.GripForce >= e.state.ObjectBreakForce*0.9 {
		reward += e.config.Reward.SlipPenalty
	}
	if e.state.ObjectGrasped && e.state.GripForce < e.requiredForce() {
		reward += e.config.Reward.SlipPenalty
	}
	return reward
}

func (e *Environment) observation() []float32 {
	state := e.state
	values := []float64{
		normalize(state.CarriageX, e.config.WorldMinX, e.config.WorldMaxX),
		normalize(state.GripperY, e.config.WorldMinY, e.config.WorldMaxY),
		normalize01(state.GripperOpening),
		normalize(state.GripForce, 0, e.config.MaxGripForce),
		normalize(state.ObjectX, e.config.WorldMinX, e.config.WorldMaxX),
		normalize(state.ObjectY, e.config.WorldMinY, e.config.WorldMaxY),
		normalize(state.ObjectVelocityX, -e.config.MaxHorizontalSpeed, e.config.MaxHorizontalSpeed),
		normalize(state.ObjectVelocityY, -e.config.MaxVerticalSpeed*2, e.config.MaxVerticalSpeed*2),
		normalize(state.CarriageX-state.ObjectX, -e.config.WorldMaxX, e.config.WorldMaxX),
		normalize(state.GripperY-state.ObjectY, -e.config.WorldMaxY, e.config.WorldMaxY),
		boolValue(state.ObjectGrasped),
		normalize(state.TargetX, e.config.WorldMinX, e.config.WorldMaxX),
		normalize(state.TargetY, e.config.WorldMinY, e.config.WorldMaxY),
		normalize(state.ObjectX-state.TargetX, -e.config.WorldMaxX, e.config.WorldMaxX),
		normalize(state.ObjectY-state.TargetY, -e.config.WorldMaxY, e.config.WorldMaxY),
		normalize(state.ObjectMass, 0, 10),
		normalize(state.ObjectFriction, 0, 1),
		normalize(state.ObjectBreakForce, 0, e.config.MaxGripForce),
		normalize(e.terrainHeight(state.ObjectX), e.config.WorldMinY, e.config.WorldMaxY),
	}
	result := make([]float32, len(values))
	for i, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			result[i] = 0
		} else {
			result[i] = float32(clamp(value, -1, 1))
		}
	}
	return result
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
	if value < minimum {
		return minimum
	}
	if value > maximum {
		return maximum
	}
	return value
}
