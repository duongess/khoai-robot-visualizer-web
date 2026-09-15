# SwarmDex visualizer

The visualizer is a React/Vite dashboard served by one Go binary. It calls the sibling control framework through direct Go package imports; only the framework communicates with the local Python SAC learner over gRPC.

## Coordinate and safety model

The Go environment is the authoritative source of physical coordinates. Its
default workspace is `[0, 6] m × [0, 3.2] m`; world X increases rightward and
world Y increases upward from the floor. A positive vertical policy action
moves the gripper up, and a negative action moves it down. Canvas Y increases
downward, so the React renderer performs the one world-to-canvas Y inversion.

The gripper reference point is constrained using its configured width, body
height, finger length, and clearance. The default safe limits are X `[0.275,
5.725]` and Y `[0.65, 3.04]` at the initial carriage position (the lower Y
limit follows terrain as the carriage moves). Boundary impacts clamp position,
zero the outward velocity, and add a configurable negative reward. Runtime
telemetry publishes the workspace, safe bounds, boundary flag, vertical
action, velocity, reachable grasp height, and vertical error; the canvas uses
those backend bounds rather than a separate viewport height.

## Secure grasp and reward model

Closing the gripper or applying a sufficient force does not itself grasp an
object. Attachment requires a closed gripper, physical contact within the
configured horizontal and vertical grasp tolerances, and a force in the safe
range. Before attachment, only object-approach and grasp-pose progress are
rewarded. Delivery progress is strictly `previous object-to-target distance −
current object-to-target distance` and is zero without attachment. Success
requires a released object to settle inside the target zone for the configured
number of steps. Telemetry exposes contact, attachment, object state, and each
reward component for diagnosis.

The coordinate/action/reward schema is version 9. The force action is a
continuous policy-controlled actuator rate on every step; after attachment the
environment reports slip feedback but never selects a grip-force target. The
analytic required force is withheld from the default 22-value policy
observation and is available only through an explicit privileged baseline flag.
This learner currently has no
checkpoint-loading path. Any future loader must call
`forcecontrol.ValidateCheckpointSchema`; earlier policies and replay data must
be discarded because they may contain exploitable reward transitions.

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

The dashboard is a training collector and therefore samples SAC actions for
exploration. For a stable evaluation-only run, set
`LEARNER_DETERMINISTIC_INFERENCE=true` before starting the learner; it uses the
actor's deterministic mean action. Every episode pins the learner policy
version returned at its first inference, so later training updates cannot alter
that episode.

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
