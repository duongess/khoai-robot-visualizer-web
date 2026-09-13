// Package forcecontrol provides a deterministic grip-force simulation.
package forcecontrol

import (
	"errors"
	"math/rand"
)

const gravity = 9.81

const maxGripForce = 40.0

type Outcome string

const (
	OutcomeSlip   Outcome = "slip"
	OutcomeStable Outcome = "stable"
	OutcomeBreak  Outcome = "break"
)

type State struct {
	Mass                 float64
	FrictionCoefficient  float64
	VerticalAcceleration float64
	CurrentGripForce     float64
	BreakForce           float64
	SlipVelocity         float64
}

type Action struct {
	NormalizedGripForce float64
}

type StepResult struct {
	State         State
	RequiredForce float64
	Outcome       Outcome
	Done          bool
}

type Environment struct {
	random *rand.Rand
	state  State
}

func New(seed int64) *Environment {
	return &Environment{random: rand.New(rand.NewSource(seed))}
}

func (e *Environment) Reset() State {
	e.state = State{
		Mass:                 0.5 + 4.5*e.random.Float64(),
		FrictionCoefficient:  0.3 + 0.7*e.random.Float64(),
		VerticalAcceleration: -2 + 4*e.random.Float64(),
		BreakForce:           120 + 60*e.random.Float64(),
	}
	return e.state
}

func (e *Environment) Step(action Action) (StepResult, error) {
	if action.NormalizedGripForce < -1 || action.NormalizedGripForce > 1 {
		return StepResult{}, errors.New("normalized grip force must be between -1 and 1")
	}
	if err := validateState(e.state); err != nil {
		return StepResult{}, err
	}

	e.state.CurrentGripForce = normalizedToForce(action.NormalizedGripForce)
	requiredForce := RequiredForce(e.state)
	result := StepResult{State: e.state, RequiredForce: requiredForce, Outcome: OutcomeStable}

	if e.state.CurrentGripForce >= e.state.BreakForce {
		e.state.SlipVelocity = 0
		result.State = e.state
		result.Outcome = OutcomeBreak
		result.Done = true
	} else if e.state.CurrentGripForce < requiredForce {
		e.state.SlipVelocity = (requiredForce - e.state.CurrentGripForce) / e.state.Mass
		result.State = e.state
		result.Outcome = OutcomeSlip
	} else {
		e.state.SlipVelocity = 0
		result.State = e.state
	}

	return result, nil
}

func (e *Environment) SetState(state State) error {
	if err := validateState(state); err != nil {
		return err
	}
	e.state = state
	return nil
}

func RequiredForce(state State) float64 {
	return state.Mass * (gravity + state.VerticalAcceleration) / (2 * state.FrictionCoefficient)
}

func normalizedToForce(normalizedForce float64) float64 {
	return (normalizedForce + 1) * maxGripForce / 2
}

func validateState(state State) error {
	if state.Mass <= 0 {
		return errors.New("mass must be greater than zero")
	}
	if state.FrictionCoefficient <= 0 {
		return errors.New("friction coefficient must be greater than zero")
	}
	if state.BreakForce <= 0 {
		return errors.New("break force must be greater than zero")
	}
	return nil
}
