package forcecontrol

import (
	"errors"

	"github.com/duongess/khoai-robot-control-framework/pkg/framework"
)

// Register adds the force-control task to a caller-owned framework runtime.
func Register(runtime *framework.Runtime, config Config) error {
	if runtime == nil {
		return errors.New("runtime is required")
	}
	if config.MaxGripForce <= 0 || config.MaxHorizontalSpeed <= 0 || config.MaxVerticalSpeed <= 0 || config.TimeStep <= 0 {
		return errors.New("force-control configuration must define positive speeds, grip force, and time step")
	}
	return runtime.RegisterTask(framework.TaskRegistration{
		Descriptor: framework.TaskDescriptor{
			Name:            "force-control",
			StateDimension:  observationDimension,
			ActionDimension: 3,
			ActionMin:       -1,
			ActionMax:       1,
		},
		Factory: taskFactory{seed: config.Seed, config: config},
	})
}

type taskFactory struct {
	seed   int64
	config Config
}

func (f taskFactory) Create() (framework.Task, error) {
	return NewTask(f.seed, f.config), nil
}
