package forcecontrol

import "testing"

func TestResetIsDeterministicForSameSeed(t *testing.T) {
	first := New(42).Reset()
	second := New(42).Reset()

	if first != second {
		t.Fatalf("Reset() states differ: %#v and %#v", first, second)
	}
}

func TestStepReturnsSlipWhenGripForceIsBelowRequiredForce(t *testing.T) {
	environment := environmentWithState(t)

	result, err := environment.Step(Action{NormalizedGripForce: -1})
	if err != nil {
		t.Fatalf("Step() error = %v", err)
	}
	if result.Outcome != OutcomeSlip || result.Done || result.State.SlipVelocity <= 0 {
		t.Fatalf("Step() result = %#v", result)
	}
}

func TestStepReturnsStableWhenGripForceMeetsRequiredForce(t *testing.T) {
	environment := environmentWithState(t)

	result, err := environment.Step(Action{NormalizedGripForce: 0})
	if err != nil {
		t.Fatalf("Step() error = %v", err)
	}
	if result.Outcome != OutcomeStable || result.Done || result.State.SlipVelocity != 0 {
		t.Fatalf("Step() result = %#v", result)
	}
}

func TestStepReturnsBreakWhenGripForceMeetsBreakForce(t *testing.T) {
	environment := environmentWithState(t)

	result, err := environment.Step(Action{NormalizedGripForce: 1})
	if err != nil {
		t.Fatalf("Step() error = %v", err)
	}
	if result.Outcome != OutcomeBreak || !result.Done {
		t.Fatalf("Step() result = %#v", result)
	}
}

func environmentWithState(t *testing.T) *Environment {
	t.Helper()

	environment := New(1)
	if err := environment.SetState(State{
		Mass:                 2,
		FrictionCoefficient:  0.5,
		VerticalAcceleration: 0,
		BreakForce:           30,
	}); err != nil {
		t.Fatalf("SetState() error = %v", err)
	}
	return environment
}
