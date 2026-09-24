package main

import (
	"context"
	"errors"
	"log"
	"net"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"syscall"
	"time"

	"github.com/duongess/khoai-robot-control-framework/pkg/framework"
	"github.com/duongess/khoai-robot-visualizer-web/pkg"
	"github.com/duongess/khoai-robot-visualizer-web/pkg/forcecontrol"
)

func main() {
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	learner, err := framework.NewLearnerClient(ctx)
	if err != nil {
		log.Fatalf("connect to learner: %v", err)
	}
	defer learner.Close()
	health, err := learner.HealthCheck(ctx)
	if err != nil {
		log.Fatalf("check learner health: %v", err)
	}
	if !health.Ready {
		log.Fatal("learner is not ready")
	}

	runtime := framework.NewRuntime()
	config := forcecontrol.DefaultConfig()
	// The demo defaults to residual control. The runtime API can switch among
	// base-only, residual, and pure-RL without restarting this process.
	config.Residual.Enabled = true
	config.ControlMode = forcecontrol.ModeResidual
	if value := os.Getenv("FORCE_CONTROL_CONTROL_MODE"); value != "" {
		config.ControlMode = forcecontrol.ControlMode(value)
		if !config.ControlMode.Valid() {
			log.Fatalf("parse FORCE_CONTROL_CONTROL_MODE: want base_only, residual, or pure_rl; got %q", value)
		}
	}
	if value := os.Getenv("FORCE_CONTROL_RESIDUAL_CONTROL"); value != "" {
		enabled, parseErr := strconv.ParseBool(value)
		if parseErr != nil {
			log.Fatalf("parse FORCE_CONTROL_RESIDUAL_CONTROL: %v", parseErr)
		}
		config.Residual.Enabled = enabled
		if os.Getenv("FORCE_CONTROL_CONTROL_MODE") == "" {
			if enabled {
				config.ControlMode = forcecontrol.ModeResidual
			} else {
				config.ControlMode = forcecontrol.ModePureRL
			}
		}
	}
	if stage := os.Getenv("FORCE_CONTROL_CURRICULUM"); stage != "" {
		config.Curriculum.Stage = forcecontrol.CurriculumStage(stage)
		// Any explicitly selected lesson is a training distribution rather than
		// a fixed demonstration scene. Keep full pick-and-place deterministic
		// unless it is reached through auto, whose flag remains enabled.
		if config.Curriculum.Stage != forcecontrol.CurriculumFullPickAndPlace {
			config.Curriculum.Randomization.Enabled = true
		}
	}
	if err := forcecontrol.Register(runtime, config); err != nil {
		log.Fatalf("register force-control task: %v", err)
	}
	runtimeConfig := framework.DefaultRuntimeConfig()
	runtimeConfig.WorkerCount = 2
	// Dual-loop modes require the learner's fly-base decomposition from the
	// first step. Exploration still comes from the stochastic SAC residual head;
	// framework-generated random actions have no corresponding fly-base vector.
	runtimeConfig.RandomActionWarmupTransitions = 0

	if err := runtime.Configure(runtimeConfig, learner); err != nil {
		log.Fatalf("configure runtime: %v", err)
	}
	api, err := pkg.NewAPIServer(runtime, config)
	if err != nil {
		log.Fatalf("initialize API server: %v", err)
	}
	uiHandler, err := pkg.NewUIHandler()
	if err != nil {
		log.Fatalf("initialize embedded frontend: %v", err)
	}

	mux := http.NewServeMux()
	mux.Handle("/api/", api.APIHandler())
	mux.Handle("/ws", api.WebSocketHandler())
	mux.Handle("/", uiHandler)
	address := os.Getenv("FORCE_CONTROL_ADDR")
	if address == "" {
		address = "127.0.0.1:8080"
	}
	listener, err := net.Listen("tcp", address)
	if err != nil {
		log.Fatalf("start HTTP server on %s: %v", address, err)
	}
	server := &http.Server{Handler: mux}
	serverErrors := make(chan error, 1)
	go func() { serverErrors <- server.Serve(listener) }()
	if err := runtime.Start(ctx); err != nil {
		_ = server.Close()
		log.Fatalf("start force-control runtime: %v", err)
	}
	log.Printf("SwarmDex visualizer is running at http://%s", address)

	select {
	case <-ctx.Done():
	case err := <-serverErrors:
		if !errors.Is(err, http.ErrServerClosed) {
			log.Printf("HTTP server failed: %v", err)
		}
	}
	runtime.Stop()
	shutdownContext, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	if err := server.Shutdown(shutdownContext); err != nil {
		log.Printf("shut down HTTP server: %v", err)
	}
}
