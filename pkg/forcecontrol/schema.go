package forcecontrol

import "fmt"

// ValidateCheckpointSchema prevents a persisted policy from silently being
// used with different environment observation, action, reward, or reset
// semantics. The Python learner validates its own architecture on restore;
// callers must also invoke this environment-level guard before reusing a
// checkpoint across a force-control schema change.
func ValidateCheckpointSchema(checkpointVersion int) error {
	if checkpointVersion != CoordinateSystemVersion {
		return fmt.Errorf("checkpoint action schema is incompatible: checkpoint=v%d, environment=v%d; start a new training run", checkpointVersion, CoordinateSystemVersion)
	}
	return nil
}
