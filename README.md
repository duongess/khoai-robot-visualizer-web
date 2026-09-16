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

The coordinate/action/reward/reset schema is version 11. The force action is a
continuous policy-controlled actuator rate on every step; after attachment the
environment reports slip feedback but never selects a grip-force target. The
analytic required force is withheld from the default 23-value policy
observation and is available only through an explicit privileged baseline flag.
SAC checkpoints are schema-aware at the learner level: a named checkpoint
stores its observation/action dimensions and controller configuration, and is
rejected if its saved SAC architecture does not match. Earlier policies and
replay data from a different force-control schema must still be discarded
because they may contain exploitable reward transitions or the old
pre-aligned reset distribution.

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

To resume a named checkpoint, pass its name. If it does not exist yet, this
starts a new named run; **Save Model** then creates it. If it already exists,
the learner restores the actor, critics, target critics, optimizers, entropy
temperature, RNG state, and learner counters before accepting Go workers.

```bash
cd khoai-robot-control-framework
poetry run python -m ai grasp-v1
```

The **Save Model** control can be pressed during training. Enter a name to
create or overwrite it; when the learner was started with a name, leaving the
field empty overwrites that active named model. Files are atomically written
to ignored `data/models/<name>.pt` by default (override with
`LEARNER_CHECKPOINT_DIR`). The Go replay buffer is intentionally not saved, so
after resuming start the dashboard afresh and do not mix stale transitions from
a changed environment/reward schema.

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

For a fresh curriculum run, start a new learner process (the replay buffer is
in-memory). Select `auto` to have every worker progress through
`align-and-contact` → `grasp` → `lift` → `transport-and-release` →
`full-pick-and-place` after real success. The learner process, policy weights,
and replay buffer stay alive across those lessons:

```bash
FORCE_CONTROL_CURRICULUM=auto go run ./cmd/force-control-demo
```

You can still pin a single stage (`align-and-contact`, `grasp`, `lift`,
`transport-and-release`, or `full-pick-and-place`) for diagnosis. Automatic
progression advances only after the current stage's physical terminal
criterion, and the dashboard reports the active stage rather than merely
displaying `auto`.

All stages retain the same continuous action contract: horizontal, vertical
(world `+Y` up, negative descends), and signed grip-force rate. They change
only reset distributions and verified terminal conditions; they do not issue
robot actions for the policy.

Every lesson starts with the open, detached gripper at `InitialCarriageX` and
the object at its own scene position. The policy receives the object-relative
observation and must issue both the horizontal approach and negative vertical
command; no curriculum reset moves the carriage onto the object or attaches it.
A contact is physically verified over
`ContactStableSteps` frames, and automatic progression requires
`ContactSuccessesRequired` consecutive successful contact **episodes** (ten
by default). A timeout or other failed episode resets that streak. The `grasp`
lesson then requires a secure, non-slipping attachment for
`GraspHoldSeconds` (30 seconds by default) without interruption; a detach or
slip restarts that timer. Configure these verification settings when changing
its difficulty. `align-and-contact` uses a 250-step horizon by default so it
resets/explores quickly; `grasp`, `lift`, `transport-and-release`, and the
full task use 1,000 steps so later milestones have enough time to repeat
prerequisite skills without a scripted reset state.

The learner should use `LEARNER_GAMMA=0.999` for this 0.1-second control loop:
it preserves the future consequence of a break across the 30-second hold
lesson. The force reward also includes a quadratic near-break barrier, so the
policy receives a negative signal before it damages a contacted object.

With `FORCE_CONTROL_CURRICULUM=auto`, every reset after the manually configured
baseline varies object/target X, mass, friction, approach height, and terrain
control-point heights from a deterministic per-worker seed. The object and
target are then placed on that episode's physical terrain. The dashboard shows
the active worker's terrain, not a static frontend copy. Pause the simulation
to edit a scene; dragging the object changes only X because its resting Y is
always derived from terrain. The **Review & Advance** button is a deliberate
human curriculum override while paused; it does not award a reward or inject a
successful replay transition.

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
