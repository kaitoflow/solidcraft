# Independent thermal simplification experiment

This fork explores thermal model simplification independently of the production thermal
preprocessor. The upstream starting point is `6c60f1a9ed1fe61bc5cd8390e49e74b0655e72d5`.
The lab branch is `codex/thermal-simplification-lab`. The production core, schema, GUI and CAD
worker are not dependencies of this experiment.

## First implementation

`thermal.envelope_plan` is a registered, read-only command available through the same engine
as the CLI and MCP. It accepts explicit world-placement body names. It reports each source's
definition, placement transform, session geometry key, raw topology, checked candidate volume
and a world-mm axis-aligned box proposal. Explicitly protected placements, mesh bodies and
bodies failing the named solid checks have no proposal.

The command does not add a feature, remove a source, change visibility, consume an undo step,
infer a device category or copy material, loss or contact properties. It does not adopt a
thermal representation. An axis-aligned box can add material and alter passages; its volume
change and fill fraction describe geometry only.

Proposal dimensions are millimetres. A future caller must use explicit `"N mm"` lengths or a
millimetre design when creating a box; bare `solid.box` lengths follow the design's display
unit. The demonstration fixes its separate preview design to millimetres.

The bounding box comes from the kernel measurement's coarse/fine surface samples. It is
identified as sampled surface bounds, not as an exact analytic or certified enclosing box.
The faster `outline_bounds` only looks at boundary geometry and can miss a face bulging past
its edges, so it is not used for proposals. Optional `padding_mm` is explicit user padding,
not a proven bound on geometric error.

Body names identify placements in the currently loaded design, not durable engineering IDs.
`definition_body` and `definition_to_world` preserve the definition/instance distinction.
`geometry_key` is session identity, not a file digest or a persistent object identifier.
Hidden placements remain eligible when explicitly named; the command does not change or
infer visibility. Protection currently covers whole placements; interface/face-level
protection remains outside the first implementation.

## Run

```text
cargo test --profile ci -p solidcraft-engine thermal::tests
cargo run --profile ci -p solidcraft-engine --example thermal_envelope_lab -- target/thermal-envelope-demo
cargo run --profile ci -p solidcraft-cli -- run examples/thermal-envelope-plan.json
```

The Rust example generates an analytic L-bracket and a protected plate, runs the plan, and
builds the one candidate in a separate design using the existing `solid.box` command. It
writes source/preview `.solidcraft` and STEP files, `plan.json` and `report.json` to a new
output directory, then imports the preview STEP and checks its closure and candidate volume.
An existing output directory is never overwritten. It is an isolated demonstration; there
is no apply operation on the source design and no completed assembly simplification.

The first local run on 2026-10-22 produced an 8-face L-bracket of candidate volume 4,480 mm³
and a separate 6-face envelope of 16,000 mm³. The source stayed unchanged, the protected plate
had no proposal, and the preview STEP reopened closed without import warnings. The large
volume increase is an observed geometric tradeoff, not an accepted thermal simplification
or a comparison win over SpaceClaim.

## Boundary reserved for a future adapter

The current stable experimental entry is the registered command and its generated input and
output schemas (`engine.describe`). A future process adapter can wrap it without depending
on an application's UI. Its host should bind an input artifact digest, a run ID and a fixed
operation, resolve host object IDs to explicit placements, and constrain input/output paths.
Outputs should retain millimetres, frame, source/placement mapping, checks and proposed
geometry separately from any adoption decision.

Transport, asynchronous jobs, cancellation, artifact hashing/publication and persistent
cross-file IDs are not implemented by this command. They must be supplied and verified by
the future adapter; this document does not register an interface in the production project.
Only numeric dimensions and fixed shape types should cross that boundary. Do not forward
arbitrary model-generated code, expressions or unreviewed command scripts.

## Comparison with SpaceClaim

Compare the same explicitly selected input and operation. Begin with analytic fixtures,
then a permitted local component and finally a representative assembly. Record import
losses, placements/units, protected interfaces, topology and geometric differences,
successful/refused operations, elapsed time, peak memory, manual interventions, export and
independent reopen. Thermal acceptance additionally needs fixed conditions and solver
comparison against a selected engineering baseline.

No SpaceClaim comparison has run in this fork. A plan, a smaller face count or a successful
STEP round trip does not establish better geometry processing or thermal equivalence.
Commercial CAD baselines and company models belong in a separate private comparison
workspace and must not enter this public fork, its logs, fixtures, issues or pull requests.

## Upstream maintenance

The first change stays in the command/schema layer and reuses the existing geometry kernel.
It does not fork the editor, alter the design format or change kernel algorithms. Upstream
is evaluating its geometry kernel; broader changes should be pinned and evaluated against
the same fixtures. Root README, product artwork and generated command inventories stay
unchanged. Development is on this fork; no upstream issue claim or pull request is required
for the user's isolated experiment.

## Initial validation record

On Windows with Rust 1.95.0 and the existing MSVC build tools, the engine test binaries built
and the 10 focused thermal tests passed. They cover unchanged documents and undo history,
explicit protection, bad input, repeated and rotated occurrences, non-uniform source scale,
display units, open solids, saved designs, and a separately created preview STEP round trip.
The final example run produced the source/preview artifacts and report described above.

`cargo xtask ci` was attempted. Formatting, assets/icons, inventory, layering and UI-container
checks reached the Clippy stage. It stopped at the unchanged upstream
`crates/geom/src/fill_general.rs:166` (`clippy::nonminimal_bool`). A package-level
`cargo clippy -p solidcraft-engine --lib --no-deps --profile ci -- -D warnings` also stopped
at three unchanged upstream `component_clone.rs` branches (`clippy::collapsible_match`).
These files match the fork starting point and were not edited or lint-suppressed by this
experiment. The full repository CI is therefore not recorded as passing; the workspace-wide
tests and wasm stage were not reached by that command.

Build logs, local checks and generated designs remain under ignored `target/lab/`. Only
source code, the analytic command recipe and this documentation belong in the fork.
