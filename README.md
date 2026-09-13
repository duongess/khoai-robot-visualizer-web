# SwarmDex visualizer

This repository contains a React/Vite dashboard and a Go force-control backend. In production, Go serves the Vite build from an embedded filesystem, so the application is distributed as one binary.

## Development

Install frontend dependencies and start Vite:

```bash
make frontend-install
cd pkg/frotend && pnpm run dev
```

In another terminal, start the Go backend:

```bash
go run ./cmd/force-control-demo
```

Vite proxies `/api` and `/ws` to the Go server during development. The default dashboard URL is `http://127.0.0.1:8080` when the Go server serves the frontend directly.

## Production

Build the frontend:

```bash
make frontend-build
```

Build the single executable, including the complete `pkg/frotend/dist` directory:

```bash
make build
```

Run the application:

```bash
./bin/force-control-demo
```

The dashboard is available at `http://127.0.0.1:8080`. Set `FORCE_CONTROL_ADDR` to change the bind address. Frontend files are embedded at compile time; Node.js is not required to run the compiled binary, and is only required to build or develop the React/Vite frontend.
