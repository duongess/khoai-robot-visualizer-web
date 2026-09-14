package forcecontrol

import "fmt"

// ValidateCheckpointSchema protects a future checkpoint loader from silently
// applying a policy trained with different action or observation semantics.
// The current learner has no checkpoint-loading path, so callers must invoke
// this before loading any persisted policy into this environment.
func ValidateCheckpointSchema(checkpointVersion int) error {
	if checkpointVersion != CoordinateSystemVersion {
		return fmt.Errorf("checkpoint action schema is incompatible: checkpoint=v%d, environment=v%d; start a new training run", checkpointVersion, CoordinateSystemVersion)
	}
	return nil
}
