package forcecontrol

import "math"

// controlCommand is the physical command consumed by the low-level plant.
// Horizontal and vertical values are normalized velocity requests; gripTarget
// is an absolute force setpoint in Newtons. It is intentionally private: SAC
// only ever sees and emits a bounded residual action.
type controlCommand struct {
	horizontal float64
	vertical   float64
	gripTarget float64
	release    bool
}

// baseCommand is a deterministic FSM/PD controller. It owns the nominal
// pick-and-place sequence and deliberately has no random state. The residual
// policy is applied afterwards, so a zero residual is a reproducible baseline
// rather than an idle robot.
func (e *Environment) baseCommand() controlCommand {
	config := e.config.Residual
	state := e.state
	command := controlCommand{}
	normalizeVelocity := func(velocity, maximum float64) float64 {
		return clamp(velocity/math.Max(maximum, 1e-9), -1, 1)
	}
	trackHeight := func(target float64) float64 {
		return normalizeVelocity(config.HeightGain*(target-state.GripperY), e.config.MaxVerticalSpeed)
	}
	nominalGrip := e.safeBaseGripForce()

	switch state.Phase {
	case PhaseApproachObject:
		command.horizontal = normalizeVelocity(config.ApproachGain*(state.ObjectX-state.CarriageX), e.config.MaxHorizontalSpeed)
	case PhaseLowerToObject:
		command.vertical = normalizeVelocity(-config.NominalDescentSpeed, e.config.MaxVerticalSpeed)
	case PhaseGripObject:
		command.gripTarget = nominalGrip
	case PhaseLiftObject:
		command.gripTarget = nominalGrip
		// ObjectY follows GripperY - ObjectHeight/2 while securely attached.
		// Use the PD target close to carry height but retain a minimum lift rate
		// so the controller cannot hover indefinitely below that milestone.
		carryGuide := e.requiredCarryHeight() + e.config.ObjectHeight/2
		if state.GripperY < carryGuide-e.config.VerticalTolerance {
			command.vertical = math.Max(
				normalizeVelocity(config.NominalLiftSpeed, e.config.MaxVerticalSpeed),
				trackHeight(carryGuide),
			)
		} else {
			command.vertical = trackHeight(carryGuide)
		}
	case PhaseMoveToTarget:
		command.gripTarget = nominalGrip
		command.horizontal = normalizeVelocity(config.TransportGain*(state.TargetX-state.CarriageX), e.config.MaxHorizontalSpeed)
		command.vertical = trackHeight(e.requiredCarryHeight() + e.config.ObjectHeight/2)
	case PhaseLowerAtTarget:
		command.gripTarget = nominalGrip
		command.vertical = normalizeVelocity(-config.NominalReleaseSpeed, e.config.MaxVerticalSpeed)
	case PhaseReleaseObject:
		command.release = true
	}
	return command
}

// composeResidualCommand makes the control authority auditable.  A policy
// action of zero is exactly the base FSM command. The force clamp is applied
// to the composed setpoint before it reaches the plant, so exploration cannot
// cross the configured material reserve.
func (e *Environment) composeResidualCommand(residual [3]float64) controlCommand {
	base := e.baseCommand()
	config := e.config.Residual
	command := base
	command.horizontal = clamp(base.horizontal+config.AlphaX*residual[0], -1, 1)
	command.vertical = clamp(base.vertical+config.AlphaY*residual[1], -1, 1)
	if base.release {
		command.gripTarget = 0
		return command
	}
	command.gripTarget = clamp(base.gripTarget+config.AlphaGrip*residual[2], 0, e.maximumResidualGripForce())
	return command
}

func (e *Environment) safeBaseGripForce() float64 {
	return clamp(
		e.config.Residual.NominalGripMultiplier*e.requiredForce(),
		0,
		e.maximumResidualGripForce(),
	)
}

func (e *Environment) maximumResidualGripForce() float64 {
	return math.Max(0, math.Min(e.config.MaxGripForce, e.state.ObjectBreakForce-e.config.Residual.ForceSafetyMargin))
}
