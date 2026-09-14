package main

import (
	"context"
	"encoding/json"
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

	runtime := framework.NewRuntime()
	if err := forcecontrol.Register(runtime, forcecontrol.DefaultConfig()); err != nil {
		log.Fatalf("register force-control task: %v", err)
	}

	uiHandler, err := pkg.NewUIHandler()
	if err != nil {
		log.Fatalf("initialize embedded frontend: %v", err)
	}

	mux := http.NewServeMux()
	mux.Handle("/api/", apiHandler())
	// Reserve the WebSocket route so it cannot be handled by the SPA fallback.
	mux.HandleFunc("/ws", func(w http.ResponseWriter, r *http.Request) {
		http.Error(w, "WebSocket telemetry is not configured", http.StatusNotImplemented)
	})
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
	log.Printf("SwarmDex visualizer is running at http://%s", address)

	select {
	case <-ctx.Done():
		shutdownContext, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		defer cancel()
		if err := server.Shutdown(shutdownContext); err != nil {
			log.Printf("shut down HTTP server: %v", err)
		}
	case err := <-serverErrors:
		if !errors.Is(err, http.ErrServerClosed) {
			log.Fatalf("HTTP server failed: %v", err)
		}
	}
}

func apiHandler() http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path == "/api/status" {
			writeJSON(w, http.StatusOK, map[string]string{"status": "running"})
			return
		}
		if r.URL.Path == "/api/scene" && r.Method == http.MethodPut {
			writeJSON(w, http.StatusOK, map[string]any{"success": true})
			return
		}
		http.NotFound(w, r)
	})
}

func writeJSON(w http.ResponseWriter, status int, value any) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(status)
	_ = json.NewEncoder(w).Encode(value)
}
