package forcecontrol

import (
	"errors"
	"fmt"
	"math"
	"sync"

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
		Factory: &taskFactory{seed: config.Seed, config: config},
	}
}

func validateRegistration(runtime *framework.Runtime, config Config) error {
	if runtime == nil {
		return errors.New("runtime is required")
	}
	if config.MaxGripForce <= 0 || config.MaxGripForceRate <= 0 || config.MaxHorizontalSpeed <= 0 || config.MaxVerticalSpeed <= 0 || config.MaxHorizontalAcceleration <= 0 || config.MaxVerticalAcceleration <= 0 || config.ActionSmoothingAlpha <= 0 || config.ActionSmoothingAlpha > 1 || config.TimeStep <= 0 || config.MaxEpisodeSteps <= 0 {
		return errors.New("force-control configuration must define positive bounded motion, grip force, fixed time step, and episode length")
	}
	if config.ObjectWidth <= 0 || config.ObjectHeight <= 0 || config.TargetWidth <= 0 || config.ObjectFriction <= 0 || config.ObjectBreakForce <= 0 || config.GripDetachInvalidFrames <= 0 || config.SlipDetachFrames <= 0 || config.HorizontalTolerance <= 0 || config.VerticalTolerance <= 0 || config.GraspHorizontalTolerance <= 0 || config.GraspVerticalTolerance <= 0 || config.ClosedOpeningThreshold < 0 || config.ClosedOpeningThreshold > 1 || config.ReleaseActionThreshold < -1 || config.ReleaseActionThreshold >= 0 || config.StableVelocityThreshold < 0 || config.StablePlacementSteps <= 0 || config.LiftClearance < 0 || config.ReleaseTolerance <= 0 || config.ActionDeadZone < 0 || config.ActionDeadZone >= 1 {
		return errors.New("force-control configuration contains invalid object, target, or phase tolerances")
	}
	if !config.Curriculum.Stage.Valid() || config.Curriculum.ContactStartHeightOffset < 0 || config.Curriculum.ContactStableSteps <= 0 || config.Curriculum.ContactSuccessesRequired <= 0 || math.IsNaN(config.Curriculum.GraspHoldSeconds) || math.IsInf(config.Curriculum.GraspHoldSeconds, 0) || config.Curriculum.GraspHoldSeconds <= 0 || config.Curriculum.AlignEpisodeStepLimit < 0 || config.Curriculum.EpisodeStepLimit < 0 {
		return errors.New("force-control curriculum must select a known stage with positive contact and grasp-hold requirements, and non-negative episode limits")
	}
	randomization := config.Curriculum.Randomization
	for _, value := range []float64{randomization.ObjectXJitter, randomization.TargetXJitter, randomization.ObjectMassJitter, randomization.ObjectFrictionJitter, randomization.ContactStartHeightJitter, randomization.TerrainHeightJitter} {
		if math.IsNaN(value) || math.IsInf(value, 0) || value < 0 {
			return errors.New("force-control curriculum randomization ranges must be finite and non-negative")
		}
	}
	if config.ObjectBreakForce > config.MaxGripForce {
		return errors.New("force-control configuration has no safe closed-grip force interval")
	}
	for _, value := range []float64{config.Homeostasis.InitialEnergy, config.Homeostasis.EnergyDecayPerStep, config.Homeostasis.SecureGripEnergyGain, config.Homeostasis.LiftEnergyGain, config.Homeostasis.DeliveryEnergyGain, config.Homeostasis.SuccessfulPlacementEnergyGain, config.Homeostasis.UnsafeDropEnergyLoss, config.Homeostasis.BreakEnergyLoss} {
		if math.IsNaN(value) || math.IsInf(value, 0) || value < 0 {
			return errors.New("force-control homeostasis configuration must use finite non-negative gains/losses and initial energy in [0, 1]")
		}
	}
	if config.Homeostasis.InitialEnergy > 1 {
		return errors.New("force-control homeostasis configuration must use finite non-negative gains/losses and initial energy in [0, 1]")
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
	mu     sync.Mutex
	next   uint64
}

func (f *taskFactory) Create() (framework.Task, error) {
	f.mu.Lock()
	seed := f.seed + int64(f.next)*104729
	f.next++
	f.mu.Unlock()
	return NewTask(seed, f.config), nil
}
