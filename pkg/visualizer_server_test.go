package pkg

import (
	"context"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"

	"github.com/duongess/khoai-robot-control-framework/pkg/framework"
	"github.com/duongess/khoai-robot-visualizer-web/pkg/forcecontrol"
	"golang.org/x/net/websocket"
)

type apiTestLearner struct{}

func (apiTestLearner) HealthCheck(context.Context) (framework.HealthStatus, error) {
	return framework.HealthStatus{Ready: true}, nil
}
func (apiTestLearner) PredictBatch(_ context.Context, states []framework.State) (framework.PredictionResult, error) {
	actions := make([]framework.Action, len(states))
	for i := range actions {
		actions[i] = framework.Action{0, 0, -1}
	}
	return framework.PredictionResult{Actions: actions}, nil
}
func (apiTestLearner) TrainBatch(context.Context, []framework.Transition) (framework.TrainingResult, error) {
	return framework.TrainingResult{}, nil
}
func (apiTestLearner) Close() error { return nil }

func TestAPIServerRejectsRunningSceneUpdatesAndPublishesTelemetry(t *testing.T) {
	runtime := framework.NewRuntime()
	config := forcecontrol.DefaultConfig()
	if err := forcecontrol.Register(runtime, config); err != nil {
		t.Fatal(err)
	}
	runtimeConfig := framework.DefaultRuntimeConfig()
	runtimeConfig.WorkerCount, runtimeConfig.TickInterval, runtimeConfig.WarmupTransitions = 1, time.Millisecond, 100
	if err := runtime.Configure(runtimeConfig, apiTestLearner{}); err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	if err := runtime.Start(ctx); err != nil {
		t.Fatal(err)
	}
	defer runtime.Stop()
	server, err := NewAPIServer(runtime, config)
	if err != nil {
		t.Fatal(err)
	}
	time.Sleep(300 * time.Millisecond)
	live := server.telemetry()
	runtimeTelemetry := live["runtime"].(map[string]any)
	if runtimeTelemetry["steps_per_second"].(float64) <= 0 {
		t.Fatalf("live steps_per_second = %#v", runtimeTelemetry["steps_per_second"])
	}
	workerTelemetry := live["worker"].(map[string]any)
	if workerTelemetry["task_phase"] == "idle" {
		t.Fatalf("reset task phase remained idle: %#v", workerTelemetry)
	}
	workspace := workerTelemetry["workspace"].(map[string]any)
	if workspace["maxY"] != config.Workspace.MaxY || workspace["minY"] != config.Workspace.MinY {
		t.Fatalf("telemetry did not publish authoritative workspace: %#v", workspace)
	}
	gantry := workerTelemetry["gantry"].(map[string]any)
	if gantry["gripper_y"].(float64) > workspace["maxY"].(float64) || gantry["gripper_y"].(float64) < workspace["minY"].(float64) {
		t.Fatalf("telemetry exposed gripper outside workspace: gantry=%#v workspace=%#v", gantry, workspace)
	}
	if workerTelemetry["latest_vertical_action"] == nil || workerTelemetry["target_grasp_y"] == nil || workerTelemetry["vertical_error"] == nil {
		t.Fatalf("telemetry lacks vertical diagnostics: %#v", workerTelemetry)
	}
	recorder := httptest.NewRecorder()
	request := httptest.NewRequest(http.MethodPut, "/api/scene", nil)
	server.APIHandler().ServeHTTP(recorder, request)
	if recorder.Code != http.StatusConflict {
		t.Fatalf("scene status = %d, want %d", recorder.Code, http.StatusConflict)
	}
	if err := runtime.Pause(); err != nil {
		t.Fatal(err)
	}

	httpServer := httptest.NewServer(server.WebSocketHandler())
	defer httpServer.Close()
	connection, err := websocket.Dial("ws"+httpServer.URL[len("http"):], "", httpServer.URL)
	if err != nil {
		t.Fatal(err)
	}
	defer connection.Close()
	_ = connection.SetDeadline(time.Now().Add(time.Second))
	var snapshot map[string]any
	if err := websocket.JSON.Receive(connection, &snapshot); err != nil {
		t.Fatal(err)
	}
	if snapshot["type"] != "simulation_snapshot" {
		t.Fatalf("telemetry = %#v", snapshot)
	}
}
