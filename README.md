# SwarmDex visualizer

The visualizer is a React/Vite dashboard served by one Go binary. It calls the sibling control framework through direct Go package imports; only the framework communicates with the local Python SAC learner over gRPC.

## Local development

From the common parent directory, use the included Go workspace so both sibling modules resolve locally:

```bash
cd khoai-robot
go work sync
```

Start the learner in one terminal:

```bash
cd khoai-robot-control-framework
poetry run python -m ai
```

Start the dashboard backend in another terminal:

```bash
cd khoai-robot-visualizer-web
go run ./cmd/force-control-demo
```

Open `http://127.0.0.1:8080`. The browser connects only to the Go process through `/api/*` and `/ws`; it never connects to Python directly.

For frontend hot reload, run Vite separately:

```bash
cd khoai-robot-visualizer-web
make frontend-install
cd pkg/frotend && pnpm run dev
```

Vite proxies `/api` and `/ws` to the Go process during development.

## Production build

```bash
make build
./bin/force-control-demo
```

`make build` installs frontend dependencies, builds `pkg/frotend/dist`, verifies the embedded entrypoint, and compiles one Go executable. Node.js is required only to build or develop the frontend, not to run the binary. Set `FORCE_CONTROL_ADDR` to change the default `127.0.0.1:8080` bind address.

## Verification

```bash
go test -race ./...
cd pkg/frotend && pnpm test && pnpm run lint
```

Run the bounded local vertical-slice smoke test from this repository after the shared Go workspace is available:

```bash
bash scripts/e2e-smoke.sh
```
