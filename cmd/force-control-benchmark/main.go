package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"log"
	"os"
	"path/filepath"
	"strconv"
	"strings"

	"github.com/duongess/khoai-robot-control-framework/pkg/framework"
	"github.com/duongess/khoai-robot-visualizer-web/pkg/forcecontrol"
)

func main() {
	ctx := context.Background()
	var (
		learnerAddress = flag.String("learner-address", "127.0.0.1:50051", "gRPC address of the Python learner")
		episodes       = flag.Int("episodes", 60, "episodes to run per mode")
		outputDir      = flag.String("output-dir", filepath.Join(".", "benchmarks"), "directory that receives the CSV and SVG exports")
		modes          = flag.String("modes", "pure_rl,parametric_sac", "comma-separated control modes to benchmark")
	)
	flag.Parse()

	learnerConfig := framework.DefaultConfig()
	learnerConfig.Address = *learnerAddress
	learner, err := framework.NewLearnerClientWithConfig(ctx, learnerConfig)
	if err != nil {
		log.Fatalf("connect to learner at %s: %v", *learnerAddress, err)
	}
	defer learner.Close()
	health, err := learner.HealthCheck(ctx)
	if err != nil {
		log.Fatalf("check learner health: %v", err)
	}
	if !health.Ready {
		log.Fatal("learner is not ready; start the Python evaluation server first")
	}

	modeList, err := parseModeList(*modes)
	if err != nil {
		log.Fatalf("parse benchmark modes: %v", err)
	}
	results := forcecontrol.BenchmarkModeComparison(ctx, learner, modeList, *episodes)
	if len(results) == 0 {
		log.Fatal("no benchmark results were produced")
	}

	if err := os.MkdirAll(*outputDir, 0o755); err != nil {
		log.Fatalf("create benchmark output directory: %v", err)
	}
	csvPath := filepath.Join(*outputDir, "benchmark_results.csv")
	if err := forcecontrol.ExportBenchmarkResultsCSV(csvPath, results); err != nil {
		log.Fatalf("write CSV export: %v", err)
	}
	chartPath := filepath.Join(*outputDir, "comparison_chart.svg")
	if err := forcecontrol.WriteBenchmarkChartSVG(chartPath, results); err != nil {
		log.Fatalf("write chart export: %v", err)
	}

	fmt.Println(forcecontrol.FormatBenchmarkMarkdown(results))
	fmt.Printf("\nCSV: %s\nSVG: %s\n", csvPath, chartPath)
}

func parseModeList(raw string) ([]forcecontrol.ControlMode, error) {
	if raw == "" {
		return nil, errors.New("at least one mode is required")
	}
	values := strings.Split(raw, ",")
	modes := make([]forcecontrol.ControlMode, 0, len(values))
	seen := map[forcecontrol.ControlMode]struct{}{}
	for _, value := range values {
		trimmed := forcecontrol.ControlMode(strings.TrimSpace(value))
		if trimmed == "" {
			continue
		}
		if !trimmed.Valid() {
			return nil, fmt.Errorf("unsupported mode %q", value)
		}
		if _, exists := seen[trimmed]; exists {
			continue
		}
		seen[trimmed] = struct{}{}
		modes = append(modes, trimmed)
	}
	if len(modes) == 0 {
		return nil, errors.New("no valid benchmark modes were provided")
	}
	return modes, nil
}

func mustInt(value string) int {
	parsed, err := strconv.Atoi(value)
	if err != nil {
		panic(err)
	}
	return parsed
}
