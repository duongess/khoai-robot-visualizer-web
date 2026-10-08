package pkg

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"

	"github.com/duongess/khoai-robot-control-framework/pkg/framework"
	"github.com/duongess/khoai-robot-visualizer-web/pkg/forcecontrol"
	"golang.org/x/net/websocket"
)

type apiFailOnceTask struct {
	failOnce   bool
	resetCount int
}

func (task *apiFailOnceTask) Reset() (framework.State, error) {
	task.resetCount++
	return framework.State{0}, nil
}

func (task *apiFailOnceTask) Step(framework.Action) (framework.StepResult, error) {
	if task.failOnce {
		task.failOnce = false
		return framework.StepResult{}, errors.New("worker 2 terminal step failed")
	}
	return framework.StepResult{State: framework.State{0}, Outcome: framework.OutcomeRunning}, nil
}

type apiFailOnceFactory struct{ tasks []*apiFailOnceTask }

func (factory *apiFailOnceFactory) Create() (framework.Task, error) {
	task := &apiFailOnceTask{failOnce: len(factory.tasks) == 1}
	factory.tasks = append(factory.tasks, task)
	return task, nil
}

func TestAPIResetRecoversWorkerErrorWithoutPausingFirst(t *testing.T) {
	runtime := framework.NewRuntime()
	factory := &apiFailOnceFactory{}
	if err := runtime.RegisterTask(framework.TaskRegistration{
		Descriptor: framework.TaskDescriptor{Name: "api-reset", StateDimension: 1, ActionDimension: 3, ActionMin: -1, ActionMax: 1},
		Factory:    factory,
	}); err != nil {
		t.Fatal(err)
	}
	config := framework.DefaultRuntimeConfig()
	config.WorkerCount, config.TickInterval, config.RandomActionWarmupTransitions, config.WarmupTransitions = 2, time.Millisecond, 0, 100000
	if err := runtime.Configure(config, apiTestLearner{}); err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	if err := runtime.Start(ctx); err != nil {
		t.Fatal(err)
	}
	defer runtime.Stop()
	deadline := time.Now().Add(time.Second)
	for runtime.Snapshot().Status != framework.RuntimeError && time.Now().Before(deadline) {
		time.Sleep(time.Millisecond)
	}
	if runtime.Snapshot().Status != framework.RuntimeError {
		t.Fatal("worker 2 did not enter runtime error state")
	}
	server, err := NewAPIServer(runtime, forcecontrol.DefaultConfig())
	if err != nil {
		t.Fatal(err)
	}
	recorder := httptest.NewRecorder()
	server.APIHandler().ServeHTTP(recorder, httptest.NewRequest(http.MethodPost, "/api/simulation/reset", nil))
	if recorder.Code != http.StatusOK {
		t.Fatalf("reset response = %d: %s", recorder.Code, recorder.Body.String())
	}
	var resetResponse struct {
		RuntimeStatus framework.RuntimeStatus `json:"runtime_status"`
	}
	if err := json.Unmarshal(recorder.Body.Bytes(), &resetResponse); err != nil || resetResponse.RuntimeStatus != framework.RuntimeRunning {
		t.Fatalf("reset did not report running status: body=%s err=%v", recorder.Body.String(), err)
	}
	if snapshot := runtime.Snapshot(); snapshot.Status != framework.RuntimeRunning || snapshot.LastError != "" {
		t.Fatalf("reset did not recover worker error: %#v", snapshot)
	}
	runtime.Stop()
	for index, task := range factory.tasks {
		if task.resetCount < 2 {
			t.Fatalf("worker %d was not reset after error", index+1)
		}
	}
}

func TestAPIServerPublishesLearnerModelAndRejectsControlModeChanges(t *testing.T) {
	runtime := framework.NewRuntime()
	server, err := NewAPIServer(runtime, forcecontrol.DefaultConfig(), framework.HealthStatus{
		ControllerType:  "parametric_mlp",
		ActiveModelName: "Parametric MLP (Dense)",
		ModelName:       "parametric_mlp",
	})
	if err != nil {
		t.Fatal(err)
	}
	healthRecorder := httptest.NewRecorder()
	server.APIHandler().ServeHTTP(healthRecorder, httptest.NewRequest(http.MethodGet, "/api/health", nil))
	if healthRecorder.Code != http.StatusOK || !bytes.Contains(healthRecorder.Body.Bytes(), []byte("Parametric MLP (Dense)")) {
		t.Fatalf("health response does not identify active learner: status=%d body=%s", healthRecorder.Code, healthRecorder.Body.String())
	}
	telemetry := server.telemetry()
	if telemetry["runtime"].(map[string]any)["active_model_name"] != "Parametric MLP (Dense)" {
		t.Fatalf("telemetry did not publish active learner identity: %#v", telemetry)
	}
	modeRecorder := httptest.NewRecorder()
	server.APIHandler().ServeHTTP(modeRecorder, httptest.NewRequest(http.MethodPost, "/api/control-mode", bytes.NewBufferString(`{"control_mode":"fly_connectome"}`)))
	if modeRecorder.Code != http.StatusMethodNotAllowed || !bytes.Contains(modeRecorder.Body.Bytes(), []byte("MODEL_MANAGED_BY_LEARNER")) {
		t.Fatalf("control-mode endpoint must be disabled: status=%d body=%s", modeRecorder.Code, modeRecorder.Body.String())
	}
}

type apiTestLearner struct{}

func (apiTestLearner) HealthCheck(context.Context) (framework.HealthStatus, error) {
	return framework.HealthStatus{Ready: true}, nil
}
func (apiTestLearner) PredictBatch(_ context.Context, states []framework.State, policyVersion uint64) (framework.PredictionResult, error) {
	actions := make([]framework.Action, len(states))
	values := make([]float32, len(states))
	for i := range actions {
		actions[i] = framework.Action{0, 0, -1}
		values[i] = -0.75
	}
	if policyVersion == 0 {
		policyVersion = 1
	}
	return framework.PredictionResult{
		Actions: actions,
		ReflexParameters: []framework.ReflexParameter{{
			Name: "channel_1.a", Values: values, MinValue: -1, MaxValue: 1, DefaultValue: -0.5,
		}},
		PolicyVersion: policyVersion,
	}, nil
}
func (apiTestLearner) TrainBatch(context.Context, []framework.Transition) (framework.TrainingResult, error) {
	return framework.TrainingResult{}, nil
}
func (apiTestLearner) SaveCheckpoint(_ context.Context) (framework.CheckpointResult, error) {
	return framework.CheckpointResult{ModelName: "grasp-v1", PolicyVersion: 7, TrainingStep: 8}, nil
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
	if workerTelemetry["contact_detected"] == nil || workerTelemetry["object_attached"] == nil || workerTelemetry["delivery_reward"] == nil {
		t.Fatalf("telemetry lacks secure-grasp diagnostics: %#v", workerTelemetry)
	}
	parameters := workerTelemetry["reflex_parameters"].([]framework.ReflexParameterValue)
	if len(parameters) != 1 || parameters[0].Name != "channel_1.a" || parameters[0].Value != -0.75 {
		t.Fatalf("telemetry lacks learner reflex parameters: %#v", workerTelemetry["reflex_parameters"])
	}
	control := workerTelemetry["control"].(map[string]any)
	if control["command_source"] != "python_evaluated_reflex" || control["model_managed"] != true {
		t.Fatalf("telemetry does not identify the learner-authoritative command: %#v", control)
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

func TestObjectStatusRequiresPhysicalContact(t *testing.T) {
	if status := objectStatus(forcecontrol.PhaseGripObject, true, false, false, false, false); status != "idle" {
		t.Fatalf("closed gripper away from object reported %q, want idle", status)
	}
	if status := objectStatus(forcecontrol.PhaseGripObject, true, true, false, false, false); status != "grasping" {
		t.Fatalf("contacting gripper reported %q, want grasping", status)
	}
	if status := objectStatus(forcecontrol.PhaseMoveToTarget, true, true, true, false, false); status != "transported" {
		t.Fatalf("attached moving object reported %q, want transported", status)
	}
}

func TestAPIServerAcceptsCanvasObjectIDAndLegacyTargetY(t *testing.T) {
	runtime := framework.NewRuntime()
	config := forcecontrol.DefaultConfig()
	if err := forcecontrol.Register(runtime, config); err != nil {
		t.Fatal(err)
	}
	if err := runtime.Configure(framework.DefaultRuntimeConfig(), apiTestLearner{}); err != nil {
		t.Fatal(err)
	}
	server, err := NewAPIServer(runtime, config)
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	if err := runtime.Start(ctx); err != nil {
		t.Fatal(err)
	}
	defer runtime.Stop()
	if err := runtime.Pause(); err != nil {
		t.Fatal(err)
	}
	payload := map[string]any{
		"object": map[string]any{"id": "object-1", "position": map[string]float64{"x": 2.1, "y": 0.7}},
		"target": map[string]any{"position": map[string]float64{"x": 4.4, "y": 0}},
	}
	body, err := json.Marshal(payload)
	if err != nil {
		t.Fatal(err)
	}
	recorder := httptest.NewRecorder()
	request := httptest.NewRequest(http.MethodPut, "/api/scene", bytes.NewReader(body))
	server.APIHandler().ServeHTTP(recorder, request)
	if recorder.Code != http.StatusOK {
		t.Fatalf("canvas scene update status=%d body=%s", recorder.Code, recorder.Body.String())
	}
	if got := server.config.InitialObjectX; got != 2.1 {
		t.Fatalf("object x = %v, want 2.1", got)
	}
	if got := server.config.TargetX; got != 4.4 {
		t.Fatalf("target x = %v, want 4.4", got)
	}
}

func TestAPIServerSavesNamedModelWithoutPausingSimulation(t *testing.T) {
	runtime := framework.NewRuntime()
	config := forcecontrol.DefaultConfig()
	if err := forcecontrol.Register(runtime, config); err != nil {
		t.Fatal(err)
	}
	if err := runtime.Configure(framework.DefaultRuntimeConfig(), apiTestLearner{}); err != nil {
		t.Fatal(err)
	}
	server, err := NewAPIServer(runtime, config)
	if err != nil {
		t.Fatal(err)
	}
	body := bytes.NewBufferString(`{"model_name":"grasp-v1"}`)
	recorder := httptest.NewRecorder()
	server.APIHandler().ServeHTTP(recorder, httptest.NewRequest(http.MethodPost, "/api/model/save", body))
	if recorder.Code != http.StatusOK {
		t.Fatalf("save model status=%d body=%s", recorder.Code, recorder.Body.String())
	}
	var response map[string]any
	if err := json.Unmarshal(recorder.Body.Bytes(), &response); err != nil {
		t.Fatal(err)
	}
	if response["model_name"] != "grasp-v1" {
		t.Fatalf("save model response=%#v", response)
	}
}
