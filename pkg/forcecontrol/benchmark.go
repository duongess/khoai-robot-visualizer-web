package forcecontrol

import (
	"context"
	"encoding/csv"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strings"

	"github.com/duongess/khoai-robot-control-framework/pkg/framework"
)

// ModeBenchmarkResult captures the outcome of one headless benchmark for a
// single control mode.
type ModeBenchmarkResult struct {
	Mode          ControlMode
	Episodes      int
	Successes     int
	SuccessRate   float64
	PeakGripForce float64
	AvgCycleTime  float64
	SlipRate      float64
	AvgReward     float64
}

// BenchmarkModeComparison runs a deterministic, no-sleep headless benchmark
// against the configured learner and returns one result per requested mode.
func BenchmarkModeComparison(ctx context.Context, learner framework.Learner, modes []ControlMode, episodes int) []ModeBenchmarkResult {
	if learner == nil {
		return nil
	}
	if episodes <= 0 {
		episodes = 1
	}
	if len(modes) == 0 {
		modes = []ControlMode{ModePureRL, ModeParametricSAC}
	}
	results := make([]ModeBenchmarkResult, 0, len(modes))
	for _, mode := range modes {
		results = append(results, benchmarkSingleMode(ctx, learner, mode, episodes))
	}
	return results
}

func benchmarkSingleMode(ctx context.Context, learner framework.Learner, mode ControlMode, episodes int) ModeBenchmarkResult {
	cfg := DefaultConfig()
	cfg.ControlMode = mode
	cfg.Curriculum.Stage = CurriculumFullPickAndPlace
	cfg.Curriculum.Randomization.Enabled = false

	result := ModeBenchmarkResult{Mode: mode}
	var totalSteps, totalSlipFrames uint64
	var totalCycleTime, totalReward float64
	var peakGripForce float64
	var successes int
	for episode := 0; episode < episodes; episode++ {
		if ctx.Err() != nil {
			break
		}
		task := NewTask(int64(episode+1), cfg)
		state, err := task.Reset()
		if err != nil {
			continue
		}
		episodeSteps := uint64(0)
		episodeSlipFrames := uint64(0)
		episodePeakGripForce := 0.0
		episodeReward := 0.0
		succeeded := false
		for {
			if ctx.Err() != nil {
				break
			}
			prediction, err := learner.PredictBatch(ctx, []framework.State{state}, 0)
			if err != nil || len(prediction.Actions) == 0 || len(prediction.Actions[0]) == 0 {
				break
			}
			action := prediction.Actions[0]
			var stepResult framework.StepResult
			if len(prediction.FlyBaseActions) > 0 && len(prediction.ResidualActions) > 0 {
				stepResult, err = task.StepDecomposed(action, prediction.FlyBaseActions[0], prediction.ResidualActions[0])
			} else {
				stepResult, err = task.Step(action)
			}
			if err != nil {
				break
			}
			state = stepResult.State
			episodeSteps++
			episodeReward += float64(stepResult.Reward)
			gripForce := task.environment.state.GripForce
			if gripForce > episodePeakGripForce {
				episodePeakGripForce = gripForce
			}
			if task.environment.state.Grip.Slipping {
				episodeSlipFrames++
			}
			if stepResult.Done {
				succeeded = stepResult.Outcome == framework.OutcomeSuccess
				if succeeded {
					successes++
				}
				break
			}
		}
		if episodePeakGripForce > peakGripForce {
			peakGripForce = episodePeakGripForce
		}
		totalCycleTime += float64(episodeSteps) * cfg.TimeStep
		totalSteps += episodeSteps
		totalSlipFrames += episodeSlipFrames
		totalReward += episodeReward
		result.Episodes++
	}
	if result.Episodes == 0 {
		return result
	}
	result.Successes = successes
	result.SuccessRate = float64(successes) / float64(result.Episodes)
	result.PeakGripForce = peakGripForce
	result.AvgCycleTime = totalCycleTime / float64(result.Episodes)
	result.SlipRate = float64(totalSlipFrames) / maxFloat64(float64(totalSteps), 1)
	result.AvgReward = totalReward / float64(result.Episodes)
	return result
}

func maxFloat64(a, b float64) float64 {
	if a > b {
		return a
	}
	return b
}

// ExportBenchmarkResultsCSV writes a benchmark table suitable for charting or
// downstream analysis.
func ExportBenchmarkResultsCSV(path string, results []ModeBenchmarkResult) error {
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return err
	}
	file, err := os.Create(path)
	if err != nil {
		return err
	}
	defer file.Close()
	writer := csv.NewWriter(file)
	if err := writer.Write([]string{"mode", "episodes", "successes", "success_rate", "peak_grip_force", "avg_cycle_time_seconds", "slip_rate", "avg_reward"}); err != nil {
		return err
	}
	for _, result := range results {
		if err := writer.Write([]string{
			string(result.Mode),
			fmt.Sprintf("%d", result.Episodes),
			fmt.Sprintf("%d", result.Successes),
			fmt.Sprintf("%.6f", result.SuccessRate),
			fmt.Sprintf("%.6f", result.PeakGripForce),
			fmt.Sprintf("%.6f", result.AvgCycleTime),
			fmt.Sprintf("%.6f", result.SlipRate),
			fmt.Sprintf("%.6f", result.AvgReward),
		}); err != nil {
			return err
		}
	}
	writer.Flush()
	return writer.Error()
}

// FormatBenchmarkMarkdown renders the benchmark results in a Markdown table.
func FormatBenchmarkMarkdown(results []ModeBenchmarkResult) string {
	if len(results) == 0 {
		return "| Mode | Episodes | Success rate | Peak grip force | Avg cycle time (s) | Slip rate |\n| --- | ---: | ---: | ---: | ---: | ---: |\n| none | 0 | 0.00% | 0.00 | 0.00 | 0.00 |"
	}
	lines := []string{"| Mode | Episodes | Success rate | Peak grip force | Avg cycle time (s) | Slip rate |", "| --- | ---: | ---: | ---: | ---: | ---: |"}
	for _, result := range results {
		lines = append(lines, fmt.Sprintf("| %s | %d | %.2f%% | %.3f | %.3f | %.3f |", result.Mode, result.Episodes, result.SuccessRate*100, result.PeakGripForce, result.AvgCycleTime, result.SlipRate))
	}
	return strings.Join(lines, "\n")
}

// WriteBenchmarkChartSVG exports a simple upright bar chart comparing success
// rates across modes. The output remains dependency-free and works in a headless
// CI job without an external plotting library.
func WriteBenchmarkChartSVG(path string, results []ModeBenchmarkResult) error {
	if len(results) == 0 {
		return os.MkdirAll(filepath.Dir(path), 0o755)
	}
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		return err
	}
	sorted := append([]ModeBenchmarkResult(nil), results...)
	sort.Slice(sorted, func(i, j int) bool { return sorted[i].SuccessRate > sorted[j].SuccessRate })
	maxValue := 1.0
	for _, result := range sorted {
		if result.SuccessRate > maxValue {
			maxValue = result.SuccessRate
		}
	}
	if maxValue < 0.01 {
		maxValue = 0.01
	}
	const width = 760
	const height = 420
	const marginLeft = 70
	const marginRight = 30
	const marginTop = 30
	const plotHeight = 260
	const barWidth = 120
	const gap = 90
	var b strings.Builder
	b.WriteString(fmt.Sprintf("<svg xmlns='http://www.w3.org/2000/svg' width='%d' height='%d' viewBox='0 0 %d %d'>", width, height, width, height))
	b.WriteString("<rect width='100%' height='100%' fill='white'/>")
	b.WriteString("<line x1='70' y1='290' x2='690' y2='290' stroke='black' stroke-width='2'/>")
	b.WriteString("<line x1='70' y1='30' x2='70' y2='290' stroke='black' stroke-width='2'/>")
	for tick := 0; tick <= 4; tick++ {
		value := maxValue * float64(tick) / 4.0
		y := 290 - (value/maxValue)*float64(plotHeight)
		b.WriteString(fmt.Sprintf("<line x1='70' y1='%g' x2='690' y2='%g' stroke='#d0d0d0' stroke-width='1'/>", y, y))
		b.WriteString(fmt.Sprintf("<text x='10' y='%g' fill='black' font-size='12'>%.0f%%</text>", y+4, value*100))
	}
	for index, result := range sorted {
		barHeight := (result.SuccessRate / maxValue) * float64(plotHeight)
		x := marginLeft + index*(barWidth+gap)
		y := 290 - barHeight
		yRounded := int(math.Round(y))
		b.WriteString(fmt.Sprintf("<rect x='%d' y='%d' width='%d' height='%d' fill='%s' rx='6'/>", x, yRounded, barWidth, int(math.Round(barHeight)), modeColor(result.Mode)))
		b.WriteString(fmt.Sprintf("<text x='%d' y='310' fill='black' font-size='12' text-anchor='middle'>%s</text>", x+barWidth/2, string(result.Mode)))
		textY := y - 8
		if textY < 40 {
			textY = 38
		}
		textYRounded := int(math.Round(textY))
		b.WriteString(fmt.Sprintf("<text x='%d' y='%d' fill='black' font-size='12' text-anchor='middle'>%.1f%%</text>", x+barWidth/2, textYRounded, result.SuccessRate*100))
	}
	b.WriteString("</svg>")
	return os.WriteFile(path, []byte(b.String()), 0o644)
}

func modeColor(mode ControlMode) string {
	switch mode {
	case ModePureRL:
		return "#4c78a8"
	case ModeParametricSAC:
		return "#f58518"
	case ModeResidual:
		return "#54a24b"
	default:
		return "#b279a2"
	}
}
