//! Controlled missing-face experiment on one admitted planar six-face working copy.
//! The generated defect is a common comparison input, not an observed source defect.

use std::error::Error;
use std::fs::{self, File};
use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::time::Instant;

use serde_json::json;
use sha2::{Digest, Sha256};
use solidcraft_engine::Session;
use solidcraft_geom::{Mesh, Vec3};
use solidcraft_kernel::FaceInfo;

fn invalid(message: &str) -> io::Error {
    io::Error::other(message)
}

fn hash(path: &Path) -> io::Result<String> {
    let mut source = File::open(path)?;
    let mut digest = Sha256::new();
    let mut buffer = [0_u8; 65536];
    loop {
        let count = source.read(&mut buffer)?;
        if count == 0 {
            break;
        }
        digest.update(buffer.get(..count).ok_or_else(|| invalid("invalid read length"))?);
    }
    Ok(format!("{:x}", digest.finalize()))
}

fn phase(journal: &mut File, start: Instant, name: &str) -> io::Result<()> {
    writeln!(journal, "{}", json!({"phase":name,"elapsed_ms":start.elapsed().as_secs_f64()*1000.0}))?;
    journal.flush()?;
    journal.sync_data()
}

fn boundary(mesh: &Mesh, face: usize) -> Vec<Vec<Vec3>> {
    mesh.edge_faces
        .iter()
        .zip(&mesh.edges)
        .filter_map(|(faces, points)| faces.iter().any(|index| usize::try_from(*index).ok() == Some(face)).then_some(points.clone()))
        .collect()
}

fn directed_boundary_distance(from: &[Vec<Vec3>], to: &[Vec<Vec3>]) -> f64 {
    from.iter()
        .flatten()
        .map(|point| {
            to.iter()
                .flat_map(|line| line.windows(2))
                .filter_map(|segment| Some(point.dist_to_segment(*segment.first()?, *segment.get(1)?)))
                .fold(f64::INFINITY, f64::min)
        })
        .fold(0.0, f64::max)
}

fn face_delta(a: &FaceInfo, b: &FaceInfo) -> Option<(f64, f64, f64)> {
    let (Some(an), Some(bn)) = (a.plane_normal, b.plane_normal) else {
        return None;
    };
    Some((a.centroid.dist(b.centroid), an.dot(bn).clamp(-1.0, 1.0).acos().to_degrees(), (a.area - b.area).abs() / a.area))
}

fn face_checks(faces: &[FaceInfo], mesh: &Mesh, after: &[FaceInfo], after_mesh: &Mesh) -> Vec<serde_json::Value> {
    faces
        .iter()
        .map(|original| {
            let candidates: Vec<_> = after
                .iter()
                .filter_map(|face| face_delta(original, face).map(|delta| (face, delta)))
                .filter(|(_, (distance, angle, area))| *distance <= 0.01 && *angle <= 0.1 && *area <= 0.001)
                .collect();
            let (mut passed, mut boundary_mm) = (false, None);
            if let [(face, _)] = candidates.as_slice() {
                let (a, b) = (boundary(mesh, original.index), boundary(after_mesh, face.index));
                let delta = directed_boundary_distance(&a, &b).max(directed_boundary_distance(&b, &a));
                passed = !a.is_empty() && a.len() == b.len() && delta.is_finite() && delta <= 0.01;
                boundary_mm = Some(delta);
            }
            json!({"source_face":original.index,"candidate_matches":candidates.len(),"boundary_sample_delta_mm":boundary_mm,"passed":passed})
        })
        .collect()
}

fn all_passed(checks: &[serde_json::Value]) -> bool {
    checks.len() == 6 && checks.iter().all(|check| check.get("passed").and_then(serde_json::Value::as_bool) == Some(true))
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let input = PathBuf::from(args.next().ok_or_else(|| invalid("input STEP required"))?).canonicalize()?;
    let output = PathBuf::from(args.next().ok_or_else(|| invalid("new private output directory required"))?);
    let expected = args.next().ok_or_else(|| invalid("input SHA-256 required"))?.to_ascii_lowercase();
    if args.next().is_some() || expected.len() != 64 || !expected.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err(invalid("expected input, output and SHA-256").into());
    }
    if fs::metadata(&input)?.len() > 1024 * 1024 || !output.is_absolute() || output.exists() {
        return Err(invalid("admit one STEP up to 1 MiB and a new absolute output directory").into());
    }
    let output = output
        .parent()
        .ok_or_else(|| invalid("output parent missing"))?
        .canonicalize()?
        .join(output.file_name().ok_or_else(|| invalid("output name missing"))?);
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).parent().and_then(Path::parent).ok_or_else(|| invalid("checkout root missing"))?;
    let root = if root.exists() { root.canonicalize()? } else { root.to_path_buf() };
    for path in [&input, &output] {
        let path = path.to_string_lossy().replace('/', "\\").to_lowercase();
        let checkout = root.to_string_lossy().replace('/', "\\").to_lowercase();
        if path == checkout || path.starts_with(&(checkout.trim_end_matches('\\').to_owned() + "\\")) {
            return Err(invalid("inputs and results must remain outside the public checkout").into());
        }
    }
    if hash(&input)? != expected {
        return Err(invalid("input SHA-256 mismatch").into());
    }
    fs::create_dir(&output)?;
    let mut journal = File::create(output.join("journal.jsonl"))?;
    let started = Instant::now();
    phase(&mut journal, started, "opening_single_body")?;
    if solidcraft_engine::crash::enabled() {
        return Err(invalid("application crash collection must be disabled").into());
    }
    let mut source = Session::default();
    source.execute("doc.open", &json!({"path":input.to_string_lossy(),"up":"z","names":"solid"}))?;
    source.execute("document.units", &json!({"units":"mm"}))?;
    let source_snapshot = source.doc.to_json();
    let world = source.world_state();
    if world.bodies.len() != 1 {
        return Err(invalid("the experiment requires exactly one body").into());
    }
    let body = world.bodies.first().ok_or_else(|| invalid("body missing"))?;
    let faces = body.body.faces(0.01)?;
    if faces.len() != 6
        || faces.iter().any(|face| face.plane_normal.is_none() || !face.area.is_finite() || face.area <= 0.0)
        || !body.body.validity().is_empty()
    {
        return Err(invalid("only a valid, closed six-plane body is admitted").into());
    }
    let mesh = body.body.tessellate(0.01)?;
    let removed = faces.iter().max_by(|a, b| a.area.total_cmp(&b.area)).ok_or_else(|| invalid("face missing"))?;
    if boundary(&mesh, removed.index).len() != 4 {
        return Err(invalid("the target face must have four boundary edges").into());
    }
    let pick = mesh
        .triangles
        .iter()
        .zip(&mesh.tri_face)
        .find_map(|(triangle, face)| {
            if usize::try_from(*face).ok() != Some(removed.index) {
                return None;
            }
            let [a, b, c] = mesh.tri(triangle)?;
            Some((a + b + c) / 3.0)
        })
        .ok_or_else(|| invalid("no interior triangle point for the selected face"))?;
    let baseline = solidcraft_kernel::measure(&body.body)?;
    let import_ms = started.elapsed().as_secs_f64() * 1000.0;
    phase(&mut journal, started, "creating_controlled_defect")?;
    let mut defect = Session::new(solidcraft_doc::Document::from_json(&source_snapshot)?);
    defect.execute("face.delete", &json!({"body":body.name,"faces":[pick],"heal":false}))?;
    let defect_state = defect.world_state();
    let open = defect_state.bodies.first().filter(|_| defect_state.bodies.len() == 1).ok_or_else(|| invalid("defect changed the body count"))?;
    if open.body.topology_closure()? != Some(false) || open.body.face_count() != 5 {
        return Err(invalid("the controlled defect is not an open five-face body").into());
    }
    let common_input = output.join("controlled-defect.step");
    fs::write(&common_input, solidcraft_kernel::step_export(&[&open.body], "controlled-missing-face")?)?;
    let common_hash = hash(&common_input)?;
    let defect_ms = started.elapsed().as_secs_f64() * 1000.0 - import_ms;
    phase(&mut journal, started, "reopening_common_defect")?;
    let repair_start = Instant::now();
    let mut repaired = Session::default();
    repaired.execute("doc.open", &json!({"path":common_input.to_string_lossy(),"up":"z","names":"solid"}))?;
    repaired.execute("document.units", &json!({"units":"mm"}))?;
    let reopened = repaired.world_state();
    let open =
        reopened.bodies.first().filter(|_| reopened.bodies.len() == 1).ok_or_else(|| invalid("common defect reopen changed the body count"))?;
    if open.body.topology_closure()? != Some(false) {
        return Err(invalid("common defect did not reopen open").into());
    }
    let open_mesh = open.body.tessellate(0.01)?;
    let rim: Vec<_> = open_mesh
        .edge_faces
        .iter()
        .enumerate()
        .filter(|(_, faces)| faces.len() == 1)
        .map(|(index, _)| -> io::Result<Vec3> {
            let points = open_mesh.edges.get(index).filter(|points| points.len() >= 2).ok_or_else(|| invalid("free-edge identity missing"))?;
            Ok((*points.first().ok_or_else(|| invalid("free-edge start missing"))?
                + *points.last().ok_or_else(|| invalid("free-edge end missing"))?)
                / 2.0)
        })
        .collect::<Result<_, _>>()?;
    if rim.len() != 4 {
        return Err(invalid("the common defect must have exactly four free edges").into());
    }
    phase(&mut journal, started, "patching_and_stitching")?;
    repaired.execute("surface.patch", &json!({"body":open.name,"edges":rim,"body_name":"RepairPatch"}))?;
    repaired.execute("surface.stitch", &json!({"bodies":[open.name,"RepairPatch"],"body_name":"Repaired"}))?;
    let repair_ms = repair_start.elapsed().as_secs_f64() * 1000.0;
    let result = repaired.world_state();
    let fixed = result.bodies.first().filter(|_| result.bodies.len() == 1).ok_or_else(|| invalid("repair must produce one body"))?;
    let measured = solidcraft_kernel::measure(&fixed.body)?;
    let fixed_faces = fixed.body.faces(0.01)?;
    let fixed_mesh = fixed.body.tessellate(0.01)?;
    let checks = face_checks(&faces, &mesh, &fixed_faces, &fixed_mesh);
    phase(&mut journal, started, "exporting_and_independently_reopening")?;
    let repaired_path = output.join("repaired.step");
    fs::write(&repaired_path, solidcraft_kernel::step_export(&[&fixed.body], "repaired-planar-patch")?)?;
    let mut independent = Session::default();
    independent.execute("doc.open", &json!({"path":repaired_path.to_string_lossy(),"up":"z","names":"solid"}))?;
    independent.execute("document.units", &json!({"units":"mm"}))?;
    let final_state = independent.world_state();
    let reimported =
        final_state.bodies.first().filter(|_| final_state.bodies.len() == 1).ok_or_else(|| invalid("repaired STEP reopen changed the body count"))?;
    let reopened_measure = solidcraft_kernel::measure(&reimported.body)?;
    let reopened_checks = face_checks(&faces, &mesh, &reimported.body.faces(0.01)?, &reimported.body.tessellate(0.01)?);
    let protected_pass = fixed_faces.len() == 6 && reimported.body.face_count() == 6 && all_passed(&checks) && all_passed(&reopened_checks);
    let volume_delta = (measured.volume - baseline.volume).abs() / baseline.volume;
    let valid = fixed.body.topology_closure()? == Some(true)
        && fixed.body.validity().is_empty()
        && reimported.body.topology_closure()? == Some(true)
        && reimported.body.validity().is_empty();
    let unchanged = hash(&input)? == expected && source.doc.to_json() == source_snapshot && hash(&common_input)? == common_hash;
    let passed =
        protected_pass && valid && unchanged && volume_delta <= 1e-6 && (reopened_measure.volume - baseline.volume).abs() / baseline.volume <= 1e-6;
    fs::write(
        output.join("protected-faces.json"),
        serde_json::to_vec_pretty(&json!({"removed_face":removed.index,"before_export":checks,"after_reopen":reopened_checks}))?,
    )?;
    let report = json!({
        "format":"solidcraft.controlled-repair-probe/1","operation_scope":"controlled_missing_planar_face",
        "status":if passed {"passed_controlled_planar_patch"} else {"checks_failed"},
        "source_sha256":expected,"source_unchanged":unchanged,"common_defect_sha256":common_hash,"repaired_sha256":hash(&repaired_path)?,
        "source_faces":faces.len(),"defect_faces":5,"repaired_faces":fixed_faces.len(),"protected_faces_checked":5,"protected_faces_passed":protected_pass,
        "reconstructed_face_and_roundtrip_geometry_passed":protected_pass,
        "validity_and_reopen_passed":valid,"source_volume_mm3":baseline.volume,"repaired_volume_mm3":measured.volume,"relative_volume_delta":volume_delta,
        "import_and_initial_observation_ms":import_ms,"create_defect_ms":defect_ms,"reopen_patch_stitch_ms":repair_ms,"total_ms":started.elapsed().as_secs_f64()*1000.0,
        "interface_scope":"five_non_target_geometric_faces_no_confirmed_engineering_role","thermal_equivalence":"not_evaluated"
        ,"export_path":"kernel.step_export","boundary_check_method":"bidirectional_mesh_boundary_samples_not_brep_certificate"
    });
    fs::write(output.join("report.json"), serde_json::to_vec_pretty(&report)?)?;
    phase(&mut journal, started, if passed { "finished" } else { "checks_failed" })?;
    println!("{}", serde_json::to_string(&report)?);
    if passed { Ok(()) } else { Err(invalid("controlled repair checks failed; see report.json").into()) }
}
