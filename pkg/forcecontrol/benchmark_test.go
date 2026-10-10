package forcecontrol

import (
	"context"
	"os"
	"path/filepath"
	"testing"

	"github.com/duongess/khoai-robot-control-framework/pkg/framework"
)

type benchmarkStubLearner struct{}

func (benchmarkStubLearner) HealthCheck(context.Context) (framework.HealthStatus, error) {
	return framework.HealthStatus{Ready: true}, nil
}

func (benchmarkStubLearner) PredictBatch(_ context.Context, states []framework.State, _ uint64) (framework.PredictionResult, error) {
	if len(states) == 0 {
		return framework.PredictionResult{}, nil
	}
	result := make([]framework.Action, len(states))
	for i := range states {
		result[i] = framework.Action{0.1, 0.2, 0.0}
	}
	return framework.PredictionResult{Actions: result, FlyBaseActions: result, ResidualActions: result, PolicyVersion: 1}, nil
}

func (benchmarkStubLearner) TrainBatch(context.Context, []framework.Transition) (framework.TrainingResult, error) {
	return framework.TrainingResult{}, nil
}

func (benchmarkStubLearner) Close() error { return nil }

func TestBenchmarkModeComparisonProducesMetricsAndChart(t *testing.T) {
	results := BenchmarkModeComparison(context.Background(), benchmarkStubLearner{}, []ControlMode{ModePureRL, ModeParametricSAC}, 2)
	if len(results) != 2 {
		t.Fatalf("expected 2 results, got %d", len(results))
	}
	if results[0].Mode != ModePureRL || results[1].Mode != ModeParametricSAC {
		t.Fatalf("unexpected benchmark ordering: %#v", results)
	}
	if results[0].Episodes == 0 || results[1].Episodes == 0 {
		t.Fatal("benchmark should run non-zero episodes")
	}
	outputDir := t.TempDir()
	chartPath := filepath.Join(outputDir, "comparison.svg")
	if err := WriteBenchmarkChartSVG(chartPath, results); err != nil {
		t.Fatalf("write chart: %v", err)
	}
	if _, err := os.Stat(chartPath); err != nil {
		t.Fatalf("chart missing: %v", err)
	}
}
