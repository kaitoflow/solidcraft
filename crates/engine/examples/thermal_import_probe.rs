//! Fixed import/inventory probe. Private inputs and reports must stay outside this checkout.

use std::error::Error;
use std::fs::{self, File};
use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::time::Instant;

use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use solidcraft_engine::Session;

fn fail(message: &str) -> io::Error {
    io::Error::other(message)
}

fn digest(path: &Path) -> io::Result<String> {
    let mut file = File::open(path)?;
    let mut hash = Sha256::new();
    let mut buffer = [0_u8; 65536];
    loop {
        let read = file.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        hash.update(buffer.get(..read).ok_or_else(|| fail("invalid file read length"))?);
    }
    Ok(format!("{:x}", hash.finalize()))
}

fn within(path: &Path, root: &Path) -> bool {
    if cfg!(windows) {
        let p = path.to_string_lossy().replace('/', "\\").to_lowercase();
        let r = root.to_string_lossy().replace('/', "\\").to_lowercase();
        p == r || p.starts_with(&(r.trim_end_matches('\\').to_string() + "\\"))
    } else {
        path.starts_with(root)
    }
}

struct Run {
    output: PathBuf,
    journal: File,
    start: Instant,
}

impl Run {
    fn phase(&mut self, phase: &str) -> io::Result<()> {
        let message = json!({"phase":phase,"elapsed_ms":self.start.elapsed().as_secs_f64()*1000.0});
        writeln!(self.journal, "{}", serde_json::to_string(&message)?)?;
        self.journal.flush()?;
        self.journal.sync_data()?;
        println!("{}", serde_json::to_string(&message)?);
        io::stdout().flush()
    }

    fn save(&self, name: &str, value: &Value) -> io::Result<()> {
        fs::write(self.output.join(name), serde_json::to_vec_pretty(value)?)
    }
}

fn probe(run: &mut Run, input: &Path, expected: &str, selected: &[usize]) -> Result<Value, Box<dyn Error>> {
    run.phase("hashing_input")?;
    let before = digest(input)?;
    if before != expected {
        return Err(fail("input SHA-256 does not match the registered snapshot").into());
    }
    run.phase("opening_step")?;
    let mut session = Session::default();
    if solidcraft_engine::crash::enabled() {
        return Err(fail("this probe must not install application crash collection").into());
    }
    let opening = Instant::now();
    let reply = session.execute("doc.open", &json!({"path":input.to_string_lossy(),"up":"z","names":"solid"}))?;
    let import_ms = opening.elapsed().as_secs_f64() * 1000.0;
    let mut saved_reply = reply.clone();
    if let Some(object) = saved_reply.as_object_mut() {
        object.remove("path");
    }
    run.save("import-reply.json", &saved_reply)?;
    run.phase("inventory_world_placements")?;
    let inventory_start = Instant::now();
    let world = session.world_state();
    if world.bodies.is_empty() {
        return Err(fail("STEP import produced no world body placements").into());
    }
    if world.bodies.len() > 20_000 {
        return Err(fail("world inventory exceeds the explicit 20000-body budget").into());
    }
    let mut faces = 0_usize;
    let mut edges = 0_usize;
    let mut shells = 0_usize;
    let mut closed = 0_usize;
    let mut open = 0_usize;
    let mut unknown = 0_usize;
    let mut closure_errors = 0_usize;
    let mut inventory = Vec::with_capacity(world.bodies.len());
    for (index, body) in world.bodies.iter().enumerate() {
        let face_count = body.body.face_count();
        let edge_count = body.body.edge_count();
        let shell_count = body.body.shell_count();
        faces = faces.checked_add(face_count).ok_or_else(|| fail("face count overflow"))?;
        edges = edges.checked_add(edge_count).ok_or_else(|| fail("edge count overflow"))?;
        shells = shells.checked_add(shell_count).ok_or_else(|| fail("shell count overflow"))?;
        let (closure, closure_error) = match body.body.topology_closure() {
            Ok(Some(true)) => {
                closed += 1;
                (Some(true), None)
            }
            Ok(Some(false)) => {
                open += 1;
                (Some(false), None)
            }
            Ok(None) => {
                unknown += 1;
                (None, None)
            }
            Err(error) => {
                closure_errors += 1;
                (None, Some(error.to_string()))
            }
        };
        let bounds = body.outline_bounds();
        inventory.push(json!({
            "index":index,"world_body":body.name,"definition_body":body.definition_name(),
            "feature_id":body.feature,"component_id":body.component,
            "definition_to_world_column_major":body.source_transform,
            "geometry_key":body.geometry_key().map(|key|format!("{key:016x}")),
            "raw_faces":face_count,"raw_edges":edge_count,"shell_count":shell_count,"mesh_body":body.body.is_mesh(),
            "outline_bbox_mm":{"min":bounds.min,"max":bounds.max},
            "bounds_quality":"boundary_only_not_certified",
            "observed_topology_closed":closure,"closure_error":closure_error,"volume_status":"not_observed"
        }));
    }
    let inventory_ms = inventory_start.elapsed().as_secs_f64() * 1000.0;
    run.save("inventory.json", &json!({"length_unit":"mm","frame":"world","bodies":inventory}))?;
    // Optional fixed read-only observation, bounded by the controller and at most eight
    // placements. No model-generated command or expression is evaluated.
    let detail_start = Instant::now();
    if !selected.is_empty() {
        run.phase("raw_import_provenance")?;
        let mut raw_bodies = Vec::new();
        for feature in &session.doc.features {
            if let solidcraft_doc::FeatureKind::Import { step, .. } = &feature.kind {
                let imported = solidcraft_kernel::step_import_shared(step)?;
                for (index, body) in imported.bodies.iter().enumerate() {
                    let bounds = body.body.outline_bounds();
                    raw_bodies.push(json!({
                        "feature_id":feature.id,"import_index":index,"source_item":body.source_item,
                        "source_face_entities":body.face_origin,"name":body.name,"path":body.path,"instance":body.instance,
                        "import_closed_flag":body.closed,"observed_topology_closed":body.body.topology_closure()?,
                        "file_faces":body.file_faces,"raw_faces":body.body.face_count(),"shell_count":body.body.shell_count(),
                        "outline_bbox_mm":{"min":bounds.min,"max":bounds.max}
                    }));
                }
            }
        }
        run.save(
            "raw-import-inventory.json",
            &json!({"bodies":raw_bodies,"frame":"source_world","length_unit":"mm","bounds_quality":"boundary_only_not_certified"}),
        )?;
    }
    for index in selected {
        run.phase("selected_geometry")?;
        let body = world.bodies.get(*index).ok_or_else(|| fail("selected body index is outside this inventory"))?;
        let measured = solidcraft_kernel::measure(&body.body)?;
        let tolerance_mm = 0.01;
        let faces = body.body.faces(tolerance_mm)?;
        let mesh = body.body.tessellate(tolerance_mm)?;
        let step = solidcraft_kernel::step_export(&[&body.body], "selected-comparison-body")?;
        let step_name = format!("selected-{index}.step");
        fs::write(run.output.join(&step_name), &step)?;
        run.save(
            &format!("selected-{index}.json"),
            &json!({
                "inventory_index":index,"frame":"world","length_unit":"mm","tessellation_tolerance_mm":tolerance_mm,
                "world_body":body.name,"measure":measured,"faces":faces,"mesh":mesh,
                "export_sha256":digest(&run.output.join(step_name))?,
                "face_centroid_scope":"area_centroid_not_guaranteed_on_trimmed_face",
                "protection_role":"unassigned","repair":"not_run","simplification":"not_run"
            }),
        )?;
    }
    let detail_ms = detail_start.elapsed().as_secs_f64() * 1000.0;
    run.phase("verifying_source_unchanged")?;
    let after = digest(input)?;
    if after != before {
        return Err(fail("input changed during import").into());
    }
    let warning_count = reply.get("warnings").and_then(Value::as_array).ok_or_else(|| fail("doc.open omitted its warning array"))?.len();
    let open_warning_messages = reply
        .get("warnings")
        .and_then(Value::as_array)
        .ok_or_else(|| fail("doc.open omitted its warning array"))?
        .iter()
        .filter(|warning| warning.as_str().is_some_and(|text| text.contains("imported as an open body (the shell is not closed)")))
        .count();
    let error_count = reply.get("errors").and_then(Value::as_u64).ok_or_else(|| fail("doc.open omitted its model-error count"))?;
    Ok(json!({
        "format":"solidcraft.import-inventory-probe/1","engine":"SolidCraft","version":env!("CARGO_PKG_VERSION"),
        "status":if error_count>0 {"model_errors"} else if closure_errors>0 {"completed_with_observation_errors"} else if warning_count>0 {"completed_with_warnings"} else {"completed"},
        "operation_scope":"import_and_basic_inventory","source_sha256":before,"source_unchanged":after==before,
        "input_bytes":fs::metadata(input)?.len(),"normalised_length_unit":"mm","frame":"world",
        "world_body_count":world.bodies.len(),"definition_body_count":session.model.state().bodies.len(),
        "component_definition_count":session.doc.components.len(),"resolved_component_placements":session.doc.placements().len(),
        "raw_faces":faces,"raw_edges":edges,"shell_count":shells,"warnings":warning_count,"model_errors":error_count,
        "open_body_warning_messages":open_warning_messages,"warning_count_unit":"deduplicated_message_may_include_multiplicity",
        "import_ms":import_ms,"basic_inventory_ms":inventory_ms,"total_ms":run.start.elapsed().as_secs_f64()*1000.0,
        "selected_body_indices":selected,"selected_geometry_ms":detail_ms,
        "observed_topology_closure":{"closed_bodies":closed,"open_bodies":open,"unknown_bodies":unknown,"errors":closure_errors},
        "closure_scope":"edge_incidence_only_not_solid_validity",
        "repair":"not_run","simplification":"not_run","interfaces":"not_assessed"
    }))
}

#[cfg(test)]
mod tests {
    use super::within;
    use std::path::Path;

    #[test]
    fn checkout_guard_requires_a_component_boundary() {
        let root = Path::new("/workspace/public");
        assert!(within(root, root));
        assert!(within(Path::new("/workspace/public/inputs/model.step"), root));
        assert!(!within(Path::new("/workspace/public-private/model.step"), root));
        assert!(!within(Path::new("/workspace/private/model.step"), root));
        #[cfg(windows)]
        assert!(within(Path::new("C:\\WORKSPACE\\PUBLIC\\model.step"), Path::new("c:\\workspace\\public")));
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let input = PathBuf::from(args.next().ok_or_else(|| fail("input STEP path required"))?).canonicalize()?;
    let output = PathBuf::from(args.next().ok_or_else(|| fail("new private output directory required"))?);
    let expected = args.next().ok_or_else(|| fail("registered SHA-256 required"))?.to_ascii_lowercase();
    let selected =
        args.next().map(|arg| arg.split(',').map(str::parse::<usize>).collect::<std::result::Result<Vec<_>, _>>()).transpose()?.unwrap_or_default();
    if selected.len() > 8
        || selected.iter().any(|index| *index >= 20_000)
        || selected.iter().enumerate().any(|(i, n)| selected.iter().take(i).any(|old| old == n))
    {
        return Err(fail("select at most eight distinct inventory indices below 20000").into());
    }
    if args.next().is_some() || expected.len() != 64 || !expected.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err(fail("expected input, new private output directory and SHA-256").into());
    }
    let extension = input.extension().and_then(|s| s.to_str()).unwrap_or("").to_ascii_lowercase();
    if !matches!(extension.as_str(), "step" | "stp") || fs::metadata(&input)?.len() > 256 * 1024 * 1024 {
        return Err(fail("only STEP inputs up to 256 MiB are admitted").into());
    }
    if !output.is_absolute() || output.exists() {
        return Err(fail("output must be a new absolute directory").into());
    }
    let output = output
        .parent()
        .ok_or_else(|| fail("output parent missing"))?
        .canonicalize()?
        .join(output.file_name().ok_or_else(|| fail("output name missing"))?);
    let build_checkout = Path::new(env!("CARGO_MANIFEST_DIR")).parent().and_then(Path::parent).ok_or_else(|| fail("build checkout root missing"))?;
    // A copied probe can run on another host without its source checkout being installed.
    let checkout = if build_checkout.exists() { build_checkout.canonicalize()? } else { build_checkout.to_path_buf() };
    if within(&input, &checkout) || within(&output, &checkout) {
        return Err(fail("comparison inputs and reports must remain outside the public checkout").into());
    }
    fs::create_dir(&output)?;
    let journal = File::create(output.join("journal.jsonl"))?;
    let mut run = Run { output, journal, start: Instant::now() };
    match probe(&mut run, &input, &expected, &selected) {
        Ok(report) => {
            run.save("report.json", &report)?;
            run.phase("finished")?;
            println!("{}", serde_json::to_string(&report)?);
            Ok(())
        }
        Err(error) => {
            run.save("report.json", &json!({"status":"failed","error":error.to_string(),"total_ms":run.start.elapsed().as_secs_f64()*1000.0}))?;
            run.phase("failed")?;
            Err(error)
        }
    }
}
