// benchmark-bridge exposes the existing force-control Task to an offline Python
// benchmark over newline-delimited JSON. It does not change task physics.
package main

import (
	"bufio"
	"encoding/json"
	"fmt"
	"os"

	"github.com/duongess/khoai-robot-control-framework/pkg/framework"
	"github.com/duongess/khoai-robot-visualizer-web/pkg/forcecontrol"
)

type request struct {
	Command        string    `json:"command"`
	Seed           int64     `json:"seed"`
	Friction       float64   `json:"friction"`
	PositionOffset float64   `json:"position_offset"`
	Action         []float32 `json:"action"`
}

type response struct {
	State        []float32          `json:"state,omitempty"`
	Reward       float32            `json:"reward,omitempty"`
	Done         bool               `json:"done,omitempty"`
	Success      bool               `json:"success,omitempty"`
	Info         map[string]float32 `json:"info,omitempty"`
	TimeStep     float64            `json:"time_step,omitempty"`
	MaxGripForce float64            `json:"max_grip_force,omitempty"`
	Error        string             `json:"error,omitempty"`
}

func main() {
	scanner := bufio.NewScanner(os.Stdin)
	scanner.Buffer(make([]byte, 4096), 1024*1024)
	writer := bufio.NewWriter(os.Stdout)
	defer writer.Flush()
	var task *forcecontrol.Task
	for scanner.Scan() {
		var input request
		var output response
		if err := json.Unmarshal(scanner.Bytes(), &input); err != nil {
			output.Error = err.Error()
		} else {
			switch input.Command {
			case "reset":
				config := forcecontrol.DefaultConfig()
				config.ControlMode = forcecontrol.ModeParametricSAC
				config.Curriculum.Stage = forcecontrol.CurriculumFullPickAndPlace
				config.Curriculum.Randomization.Enabled = false
				config.ObjectFriction = input.Friction
				config.InitialObjectX += input.PositionOffset
				task = forcecontrol.NewTask(input.Seed, config)
				state, err := task.Reset()
				if err != nil {
					output.Error = err.Error()
				} else {
					output.State = state
					output.TimeStep = config.TimeStep
					output.MaxGripForce = config.MaxGripForce
				}
			case "step":
				if task == nil {
					output.Error = "reset before step"
				} else {
					result, err := task.Step(framework.Action(input.Action))
					if err != nil {
						output.Error = err.Error()
					} else {
						output.State = result.State
						output.Reward = result.Reward
						output.Done = result.Done
						output.Success = result.Outcome == forcecontrol.OutcomeSuccess
						output.Info = result.Info
					}
				}
			default:
				output.Error = fmt.Sprintf("unknown command %q", input.Command)
			}
		}
		encoded, _ := json.Marshal(output)
		fmt.Fprintln(writer, string(encoded))
		writer.Flush()
	}
	if err := scanner.Err(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}
