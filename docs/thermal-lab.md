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
infer visibility. The envelope plan protects whole placements. The separate coplanar preview
below adds explicit snapshot face protection.

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

Commercial CAD comparisons run in a separate private workspace. A plan, a smaller face
count or a successful STEP round trip does not establish better geometry processing or
thermal equivalence. Commercial CAD baselines and company models belong in that private
workspace and must not enter this public fork, its logs, fixtures, issues or pull requests.

## Upstream maintenance

The envelope command stays in the command/schema layer and reuses the geometry kernel.
The diagnostics add an independent topology-closure observation; the STEP reader also has
the narrowly gated placement compatibility described below. This experiment does not fork
the editor or alter the design format. Upstream
is evaluating its geometry kernel; broader changes should be pinned and evaluated against
the same fixtures. Root README, product artwork and generated command inventories stay
unchanged. Development is on this fork; no upstream issue claim or pull request is required
for the user's isolated experiment.

## Private input diagnostic

`thermal_import_probe` is an engine-only example for a separate, bounded comparison runner:

```text
cargo build --profile ci -p solidcraft-engine --example thermal_import_probe --locked
thermal_import_probe.exe <absolute-input.step> <new-absolute-private-output> <input-sha256> [inventory-index,...]
```

It calls the registered `doc.open` path with unchanged source axes, inventories world
placements, and records source hashes, import warnings, model errors, shell counts and
observed topological closure. Input and reports must stay outside this public checkout.
The caller must enforce process time and memory budgets; the example does not provide an
operating-system sandbox. It admits STEP files up to 256 MiB and at most 20,000 placements.
No application crash collector is installed. Existing output directories are refused.

At most eight explicit inventory indices can request sampled measurement, face summaries,
a 0.01 mm requested tessellation and a separate STEP export. The kernel may clamp meshing
tolerances to its scale-dependent minimum. A face's area centroid can lie outside its trim;
it is not automatically a point for a subsequent edit or a confirmed engineering interface.
These observations do not repair, simplify or adopt any geometry.

The raw import tree is saved with each node's parent-relative, column-major transform.
This separates source placement interpretation from application occurrence expansion.

`Body::topology_closure` observes edge incidence independently of STEP shell declarations.
Empty and mesh bodies return unknown; a nonempty B-rep whose shells are closed and
consistently oriented returns true. This does not check vertex manifoldness, meshing,
surface accuracy, positive volume or full solid validity. Import warnings remain intact:
the importer's existing `closed` flag can be false for a source-declared surface model,
even when this separate topological check passes. Warning count is not an open-body count.

The fast inventory's outline bounds include spline control points and can extend far
beyond the trimmed geometry, or miss a face that bulges past its boundary. Compare
selected surface samples and a common world frame before interpreting box differences
as displaced or missing parts. Shell count also differs from material-region count when
a body contains disconnected pieces or cavities.

## STEP placement compatibility

For `ITEM_DEFINED_TRANSFORMATION`, the normal relationship order and standard rigid-frame
composition remain unchanged. If the two referenced frames appear in the opposite
representations' direct `Items` lists, and the declared assignment is not also valid, the
reader reassigns the frame IDs to the relationship endpoints before interpreting their
length units. It emits a deduplicated `matched by representation membership` warning.

Direct membership is intentional: linked or nested representations do not establish
exclusive frame ownership. Shared frames that make the declared assignment valid, frames
listed in both representations, and unlisted frames retain the declared order. There is
no blanket transform inversion and no writer change.

Analytic regressions exercise a nonzero child reference in centimetres, a millimetre parent,
a rotated second occurrence, either relationship endpoint as the child, crossed or declared
frame order, and a shared origin. They check parent-relative tree transforms, world boxes,
volumes and compatibility warnings. Ambiguous and unlisted frames preserve existing behavior.
The STEP import test group passed 65 tests, with two pre-existing manual benchmarks ignored.
The two placement regressions and the ten thermal command tests also passed after the final
change. A CLI GPU snapshot of a generic two-box STEP reproduction was inspected; `eval`
placed the second box at the analytically expected +100 mm translation. These tests use
generic analytic geometry; private comparison evidence is not a fixture.

## Protected coplanar candidates

`thermal.merge_coplanar_preview` computes a separate candidate on one explicit world body.
It requires the current 16-digit `geometry_key`, an explicit protection mode, and current
`protected_faces` indices. `unconfirmed` and `whole_body` return a blocked result without
candidate geometry. `explicit_faces` permits only the specified geometric experiment; an
empty list also requires `allow_empty_protection: true`. The caller must retain an object
whose engineering role is unknown instead of using that opt-in to infer permission.

The guarded kernel method `Body::merge_coplanar_preview` joins edge-adjacent polygon faces
on the same oriented planar support. It reuses the strict topological loop builder rather
than general healing. Plane agreement uses its existing 4096-times-machine-epsilon support
checks; it is not a thermal equivalence certificate. Curved faces and boundaries are not
refitted. Every surviving edge and each protected face stay the original topology objects.
Protected faces cannot join a multi-face group, and all their edge identities are excluded
from removal. Authored split edges cannot be bypassed through another adjacency path.

The first slice admits one B-rep shell with up to 1024 faces and 16384 edge uses. It caps
authored boundary work, checks solid validity and crossed trims, and rejects candidate
volume differences over 1e-6 relative to the input. Whole-body authored-boundary protection
and per-face paint are retained by refusing the operation. Source/undo/visibility remain
unchanged. An attempted group that cannot reconstruct its closed outer and hole boundaries
returns an error; no eligible group returns `unchanged`. Only an actual face reduction
returns `merged`.
The host still supplies process time and memory budgets for imported geometry; these
topology limits do not replace an operating-system execution budget.

`output_face_sources` maps each output to the immediate input's face indices. It does not
compose ancestor history or create persistent engineering identities. The session geometry
key binds these indices to the current world placement. Optional `include_native: true`
returns at most 8 MiB of native B-rep bytes as hex, identified as an experimental archive
for the same producer. No transport path, document adoption or STEP acceptance is implied;
`step_interchange_check` remains `not_run` at this command boundary.

```text
cargo test --profile ci -p solidcraft-kernel --lib body::planar_regularize::tests
cargo test --profile ci -p solidcraft-engine --lib thermal::tests
cargo run --profile ci -p solidcraft-engine --example thermal_merge_lab -- target/thermal-merge-demo
```

The fixed analytic example constructs a tiled plate with a square through opening and
defines its eight top tiles as protected. Its expected reductions are 32 to 17 faces with
protection, and 32 to 10 without protection. It writes a separate native candidate, STEP
and design, independently opens the STEP in a new session, and requires eight unique
protected face/trim matches, unchanged input, a retained opening and volume checks.
The STEP writer itself can merge planar subdivisions, so the analytic runner supplies
all surviving boundaries to an export copy before writing and checks the actual reopened
file. The source geometry and its document are not modified. This is a local analytic
demonstration; private same-part commercial CAD comparison remains a separate experiment.

The local run passed the six focused planar topology tests and thirteen thermal command
tests. The fixed example retained all eight protected regions after actual STEP export
and independent reopen, with zero observed boundary-distance and volume differences on
this analytic fixture. Before/after CLI CPU snapshots were generated and inspected.
The default GPU snapshot path exited with Windows access violation `0xC0000005` for both
this fixture and the separate two-box control; the renderer cause remains unresolved.
No GPU code was changed or GPU stability pass recorded. The documented per-process
`SOLIDCRAFT_GPU=cpu` policy produced the reviewed snapshots.

Full CI was attempted again and stopped at the unchanged upstream geometry Clippy check.
A kernel-only Clippy check also reported six existing `nonminimal_bool` errors in unchanged
`coplanar.rs`, `curveblend.rs`, `ops.rs`, `step_in/brep.rs`, `tess/grid.rs` and `tess/seam.rs`.
No unrelated fixes or lint suppressions were added.

## Controlled local repair

```text
cargo build --profile ci -p solidcraft-engine --example thermal_repair_probe --locked
thermal_repair_probe.exe <single-body.step> <new-absolute-private-output> <input-sha256>
```

This fixed experiment admits one valid six-plane body up to 1 MiB with a four-edge target
face. It clones the design, removes that face without healing, exports a common five-face
STEP defect, independently opens that exact file, then runs the registered `surface.patch`
and `surface.stitch` commands. It exports through `kernel.step_export` and independently
reopens through `doc.open`.

Success requires six faces before and after the final STEP round trip, valid closed solids,
volume relative error at most 1e-6, and unique geometric matches for all six original faces.
The five non-target faces and reconstructed face are checked before export and after reopen:
centroid within 0.01 mm, planar normal within 0.1 degrees, area within 0.1%, equal boundary
edge count, and bidirectional mesh-boundary sample distance within 0.01 mm. A failed check
writes a failed report and returns a nonzero exit. These are sampled geometric criteria,
not an exact B-rep certificate or confirmed engineering-interface classification.

The source file/design and common-defect hash must remain unchanged. The source defect is
deliberately created; a successful run does not establish repair of naturally damaged
assembly geometry. The generic analytic envelope from `thermal_envelope_lab` can exercise
the probe without company data. Private comparison inputs and results stay outside this
checkout and its pull request.

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
