package pkg

import (
	"embed"
	"io/fs"
	"net/http"
	"path"
	"strings"
)

// frontendFiles contains the complete Vite production build.
//
//go:embed frotend/dist
var frontendFiles embed.FS

type uiHandler struct {
	files       fs.FS
	indexHTML   []byte
	fileHandler http.Handler
}

// NewUIHandler returns a handler for the embedded React application.
func NewUIHandler() (http.Handler, error) {
	files, err := fs.Sub(frontendFiles, "frotend/dist")
	if err != nil {
		return nil, err
	}

	indexHTML, err := fs.ReadFile(files, "index.html")
	if err != nil {
		return nil, err
	}

	return &uiHandler{
		files:       files,
		indexHTML:   indexHTML,
		fileHandler: http.FileServer(http.FS(files)),
	}, nil
}

func (h *uiHandler) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	if isBackendPath(r.URL.Path) {
		http.NotFound(w, r)
		return
	}

	relativePath, ok := embeddedPath(r.URL.Path)
	if !ok {
		http.NotFound(w, r)
		return
	}

	if relativePath != "." {
		if _, err := fs.Stat(h.files, relativePath); err == nil {
			if relativePath == "index.html" {
				w.Header().Set("Cache-Control", "no-cache")
			} else if strings.HasPrefix(relativePath, "assets/") {
				w.Header().Set("Cache-Control", "public, max-age=31536000, immutable")
			}
			h.fileHandler.ServeHTTP(w, r)
			return
		}
		if strings.HasPrefix(r.URL.Path, "/assets/") {
			http.NotFound(w, r)
			return
		}
	}

	w.Header().Set("Cache-Control", "no-cache")
	w.Header().Set("Content-Type", "text/html; charset=utf-8")
	w.WriteHeader(http.StatusOK)
	_, _ = w.Write(h.indexHTML)
}

func embeddedPath(urlPath string) (string, bool) {
	cleanPath := path.Clean("/" + urlPath)
	if cleanPath == "/.." || strings.HasPrefix(cleanPath, "/../") {
		return "", false
	}
	return strings.TrimPrefix(cleanPath, "/"), true
}

func isBackendPath(urlPath string) bool {
	return urlPath == "/api" || strings.HasPrefix(urlPath, "/api/") ||
		urlPath == "/ws" || strings.HasPrefix(urlPath, "/ws/")
}
