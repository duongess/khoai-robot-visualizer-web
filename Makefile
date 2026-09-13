FRONTEND_DIR := pkg/frotend
FRONTEND_MANAGER := pnpm
BINARY := bin/force-control-demo

.PHONY: frontend-install frontend-build go-build build run test

frontend-install:
	cd $(FRONTEND_DIR) && $(FRONTEND_MANAGER) install --frozen-lockfile

frontend-build:
	cd $(FRONTEND_DIR) && $(FRONTEND_MANAGER) run build

go-build:
	@test -f $(FRONTEND_DIR)/dist/index.html || (echo "frontend production build is missing: $(FRONTEND_DIR)/dist/index.html" >&2; exit 1)
	@mkdir -p bin
	go build -o $(BINARY) ./cmd/force-control-demo

build:
	$(MAKE) frontend-install
	$(MAKE) frontend-build
	$(MAKE) go-build

run:
	go run ./cmd/force-control-demo

test:
	go test ./...
	cd $(FRONTEND_DIR) && $(FRONTEND_MANAGER) test
	cd $(FRONTEND_DIR) && $(FRONTEND_MANAGER) run lint
