package pkg

import (
	"io/fs"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
)

func TestUIHandlerServesEmbeddedFrontend(t *testing.T) {
	handler, err := NewUIHandler()
	if err != nil {
		t.Fatalf("NewUIHandler() error = %v", err)
	}

	entries, err := fs.ReadDir(frontendFiles, "frotend/dist/assets")
	if err != nil {
		t.Fatalf("read embedded assets: %v", err)
	}
	if len(entries) == 0 {
		t.Fatal("embedded assets directory is empty")
	}
	assetPath := "/assets/" + entries[0].Name()

	tests := []struct {
		name       string
		path       string
		status     int
		content    string
		cacheValue string
	}{
		{name: "root", path: "/", status: http.StatusOK, content: "<html", cacheValue: "no-cache"},
		{name: "asset", path: assetPath, status: http.StatusOK, cacheValue: "public, max-age=31536000, immutable"},
		{name: "spa fallback", path: "/comparison", status: http.StatusOK, content: "<html", cacheValue: "no-cache"},
		{name: "missing asset", path: "/assets/missing.js", status: http.StatusNotFound},
	}

	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			recorder := httptest.NewRecorder()
			req := httptest.NewRequest(http.MethodGet, test.path, nil)
			handler.ServeHTTP(recorder, req)

			if recorder.Code != test.status {
				t.Fatalf("status = %d, want %d", recorder.Code, test.status)
			}
			if test.content != "" && !strings.Contains(strings.ToLower(recorder.Body.String()), test.content) {
				t.Fatalf("body does not contain %q", test.content)
			}
			if test.cacheValue != "" && recorder.Header().Get("Cache-Control") != test.cacheValue {
				t.Fatalf("Cache-Control = %q, want %q", recorder.Header().Get("Cache-Control"), test.cacheValue)
			}
		})
	}
}

func TestUIHandlerDoesNotInterceptAPI(t *testing.T) {
	uiHandler, err := NewUIHandler()
	if err != nil {
		t.Fatalf("NewUIHandler() error = %v", err)
	}

	mux := http.NewServeMux()
	mux.HandleFunc("/api/", func(w http.ResponseWriter, r *http.Request) {
		w.WriteHeader(http.StatusTeapot)
	})
	mux.Handle("/", uiHandler)

	recorder := httptest.NewRecorder()
	mux.ServeHTTP(recorder, httptest.NewRequest(http.MethodGet, "/api/test", nil))
	if recorder.Code != http.StatusTeapot {
		t.Fatalf("status = %d, want %d", recorder.Code, http.StatusTeapot)
	}
}
