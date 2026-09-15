package main

import (
	"context"
	"errors"
	"log"
	"net"
	"net/http"
	"os"
	"os/signal"
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
	if stage := os.Getenv("FORCE_CONTROL_CURRICULUM"); stage != "" {
		config.Curriculum.Stage = forcecontrol.CurriculumStage(stage)
	}
	if err := forcecontrol.Register(runtime, config); err != nil {
		log.Fatalf("register force-control task: %v", err)
	}
	if err := runtime.Configure(framework.DefaultRuntimeConfig(), learner); err != nil {
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
