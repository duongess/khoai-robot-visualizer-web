package main

import (
	"context"
	"fmt"
	"log"

	framework "github.com/duongess/khoai-robot-control-framework/pkg/framework"
	"github.com/duongess/khoai-robot-visualizer-web/pkg/forcecontrol"
)

const stepCount = 100

func main() {
	ctx := context.Background()
	learner, err := framework.NewLearnerClient(ctx)
	if err != nil {
		log.Fatalf("connect to learner: %v", err)
	}
	defer learner.Close()

	ready, err := learner.HealthCheck(ctx)
	if err != nil {
		log.Fatalf("check learner health: %v", err)
	}
	if !ready {
		log.Fatal("learner is not ready")
	}

	environment := forcecontrol.New(42)
	state := environment.Reset()

	for step := 1; step <= stepCount; step++ {
		actions, err := learner.PredictBatch(ctx, []framework.State{stateToFramework(state)})
		if err != nil {
			log.Fatalf("predict action at step %d: %v", step, err)
		}
		if len(actions) != 1 || len(actions[0]) != 1 {
			log.Fatalf("predict action at step %d: expected one grip action", step)
		}

		result, err := environment.Step(forcecontrol.Action{NormalizedGripForce: float64(actions[0][0])})
		if err != nil {
			log.Fatalf("apply action at step %d: %v", step, err)
		}

		reward := rewardFor(result.Outcome)
		_, err = learner.TrainBatch(ctx, []framework.Transition{{
			Observation:     stateToFramework(state),
			Action:          actions[0],
			Reward:          reward,
			NextObservation: stateToFramework(result.State),
			Done:            result.Done,
		}})
		if err != nil {
			log.Fatalf("train transition at step %d: %v", step, err)
		}

		fmt.Printf(
			"step=%d grip_force=%.2f required_force=%.2f reward=%.2f outcome=%s\n",
			step,
			result.State.CurrentGripForce,
			result.RequiredForce,
			reward,
			result.Outcome,
		)

		state = result.State
		if result.Done {
			state = environment.Reset()
		}
	}
}

func stateToFramework(state forcecontrol.State) framework.State {
	return framework.State{
		float32(state.Mass),
		float32(state.FrictionCoefficient),
		float32(state.VerticalAcceleration),
		float32(state.CurrentGripForce),
		float32(state.BreakForce),
		float32(state.SlipVelocity),
	}
}

func rewardFor(outcome forcecontrol.Outcome) float32 {
	switch outcome {
	case forcecontrol.OutcomeStable:
		return 1
	case forcecontrol.OutcomeSlip:
		return -1
	case forcecontrol.OutcomeBreak:
		return -10
	default:
		return 0
	}
}
