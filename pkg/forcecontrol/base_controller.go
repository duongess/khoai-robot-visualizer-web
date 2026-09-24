package forcecontrol

import "math"

// maximumSafeGripForce is a policy-agnostic material guard. It does not infer
// a required force or select a setpoint; it only caps the neural actor's
// continuous force-rate integration below the known fracture boundary.
func (e *Environment) maximumSafeGripForce() float64 {
	return math.Max(0, math.Min(
		e.config.MaxGripForce,
		e.state.ObjectBreakForce-e.config.Residual.ForceSafetyMargin,
	))
}
