package forcecontrol

import (
	"errors"
	"fmt"

	"github.com/duongess/khoai-robot-control-framework/pkg/framework"
)

// Register adds the force-control task to a caller-owned framework runtime.
func Register(runtime *framework.Runtime, config Config) error {
	if err := validateRegistration(runtime, config); err != nil {
		return err
	}
	return runtime.RegisterTask(registration(config))
}

// UpdateRegistration replaces the task factory after an accepted scene update.
func UpdateRegistration(runtime *framework.Runtime, config Config) error {
	if err := validateRegistration(runtime, config); err != nil {
		return err
	}
	return runtime.ReplaceTask(registration(config))
}

func registration(config Config) framework.TaskRegistration {
	return framework.TaskRegistration{
		Descriptor: framework.TaskDescriptor{
			Name:            "force-control",
			StateDimension:  ObservationDimension,
			ActionDimension: 3,
			ActionMin:       -1,
			ActionMax:       1,
		},
		Factory: taskFactory{seed: config.Seed, config: config},
	}
}

func validateRegistration(runtime *framework.Runtime, config Config) error {
	if runtime == nil {
		return errors.New("runtime is required")
	}
	if config.MaxGripForce <= 0 || config.MaxHorizontalSpeed <= 0 || config.MaxVerticalSpeed <= 0 || config.TimeStep <= 0 || config.MaxEpisodeSteps <= 0 {
		return errors.New("force-control configuration must define positive speeds, grip force, time step, and episode length")
	}
	if config.ObjectWidth <= 0 || config.ObjectHeight <= 0 || config.TargetWidth <= 0 || config.ObjectFriction <= 0 || config.ObjectBreakForce <= 0 || config.HorizontalTolerance <= 0 || config.VerticalTolerance <= 0 || config.LiftClearance < 0 || config.ReleaseTolerance <= 0 {
		return errors.New("force-control configuration contains invalid object, target, or phase tolerances")
	}
	if config.Workspace.MaxX <= config.Workspace.MinX || config.Workspace.MaxY <= config.Workspace.MinY || config.GripperWidth <= 0 || config.GripperBodyHeight <= 0 || config.GripperFingerLength < 0 || config.GripperClearance < 0 || config.RailY < config.Workspace.MinY || config.RailY > config.Workspace.MaxY {
		return errors.New("force-control configuration contains invalid workspace or gripper geometry")
	}
	for _, point := range config.Terrain {
		if point.X < config.Workspace.MinX || point.X > config.Workspace.MaxX || point.Y < config.Workspace.MinY || point.Y+config.GripperFingerLength+config.GripperClearance > config.Workspace.MaxY {
			return errors.New("force-control terrain lies outside the usable workspace")
		}
	}
	probe := newEnvironment(config.Seed, config)
	probe.reset()
	if err := probe.ValidateState(); err != nil {
		return fmt.Errorf("force-control initial state is invalid: %w", err)
	}
	return nil
}

type taskFactory struct {
	seed   int64
	config Config
}

func (f taskFactory) Create() (framework.Task, error) {
	return NewTask(f.seed, f.config), nil
}
