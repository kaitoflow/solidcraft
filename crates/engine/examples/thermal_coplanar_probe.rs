//! Controlled coplanar subdivisions on one admitted six-plane working copy.
//! Only deliberately introduced seams are eligible; original face geometry is checked after reopen.

use std::error::Error;
use std::fs::{self, File};
use std::io::{self, Read, Write};
use std::path::{Path, PathBuf};
use std::time::Instant;

use serde_json::json;
use sha2::{Digest, Sha256};
use solidcraft_engine::Session;
use solidcraft_geom::{Mesh, Plane, Vec3};
use solidcraft_kernel::{Body, FaceInfo, SplitTool};

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
                passed = !a.is_empty() && boundary_loops(&a) == Some(1) && boundary_loops(&b) == Some(1) && delta.is_finite() && delta <= 0.01;
                boundary_mm = Some(delta);
            }
            json!({"source_face":original.index,"candidate_matches":candidates.len(),"boundary_sample_delta_mm":boundary_mm,"collinear_subdivision_allowed":true,"passed":passed})
        })
        .collect()
}

fn all_passed(checks: &[serde_json::Value]) -> bool {
    checks.len() == 6 && checks.iter().all(|check| check.get("passed").and_then(serde_json::Value::as_bool) == Some(true))
}

fn boundary_loops(lines: &[Vec<Vec3>]) -> Option<usize> {
    if lines.is_empty() || lines.len() > 32 {
        return None;
    }
    let mut points = Vec::<Vec3>::new();
    let mut links = Vec::new();
    for line in lines {
        let mut pair = Vec::new();
        for p in [line.first()?, line.last()?] {
            if !p.is_finite() {
                return None;
            }
            let found: Vec<_> = points.iter().enumerate().filter(|(_, q)| q.dist(*p) < 1e-6).map(|(i, _)| i).collect();
            let index = match found.as_slice() {
                [] => {
                    points.push(*p);
                    points.len() - 1
                }
                [i] => *i,
                _ => return None,
            };
            pair.push(index);
        }
        links.push((*pair.first()?, *pair.get(1)?));
    }
    if (0..points.len()).any(|i| links.iter().filter(|(a, b)| *a == i || *b == i).count() != 2) {
        return None;
    }
    let mut seen = std::collections::BTreeSet::new();
    let mut loops = 0;
    for index in 0..points.len() {
        if seen.contains(&index) {
            continue;
        }
        loops += 1;
        let mut stack = vec![index];
        while let Some(i) = stack.pop() {
            if !seen.insert(i) {
                continue;
            }
            for (a, b) in &links {
                if *a == i {
                    stack.push(*b);
                }
                if *b == i {
                    stack.push(*a);
                }
            }
        }
    }
    Some(loops)
}

fn export_with_boundaries(body: &Body) -> Result<String, Box<dyn Error>> {
    let lines = body.edges(0.01)?.into_iter().map(|e| e.points).collect();
    let copy = body.clone().with_split_lines(Some(std::sync::Arc::new(lines)));
    Ok(solidcraft_kernel::step_export(&[&copy], "controlled-coplanar-subdivisions")?)
}

fn one(session: &Session) -> Result<Body, io::Error> {
    let world = session.world_state();
    if world.bodies.len() != 1 {
        return Err(invalid("expected exactly one world body"));
    }
    world.bodies.first().map(|b| b.body.clone()).ok_or_else(|| invalid("missing body"))
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let input = PathBuf::from(args.next().ok_or_else(|| invalid("source STEP required"))?).canonicalize()?;
    let output = PathBuf::from(args.next().ok_or_else(|| invalid("new private output required"))?);
    let expected = args.next().ok_or_else(|| invalid("source SHA-256 required"))?.to_ascii_lowercase();
    let common_expected = args.next().map(|s| s.to_ascii_lowercase());
    let input_tolerance: f64 = args.next().map(|s| s.parse()).transpose()?.unwrap_or(0.);
    if args.next().is_some()
        || !input_tolerance.is_finite()
        || !(0.0..=1e-6).contains(&input_tolerance)
        || (input_tolerance > 0. && common_expected.is_none())
        || [&expected].into_iter().chain(common_expected.iter()).any(|s| s.len() != 64 || !s.bytes().all(|b| b.is_ascii_hexdigit()))
    {
        return Err(
            invalid("expected source, output, SHA-256, optional frozen common SHA-256 and fixed-experiment boundary allowance0..1e-6mm").into()
        );
    }
    if fs::metadata(&input)?.len() > 1024 * 1024
        || !output.is_absolute()
        || output.exists()
        || !input.extension().and_then(|s| s.to_str()).is_some_and(|s| s.eq_ignore_ascii_case("step") || s.eq_ignore_ascii_case("stp"))
    {
        return Err(invalid("admit one STEP up to 1MiB and new absolute output directory").into());
    }
    let output =
        output.parent().ok_or_else(|| invalid("output parent"))?.canonicalize()?.join(output.file_name().ok_or_else(|| invalid("output name"))?);
    let checkout = Path::new(env!("CARGO_MANIFEST_DIR")).parent().and_then(Path::parent).ok_or_else(|| invalid("checkout"))?;
    let checkout = if checkout.exists() { checkout.canonicalize()? } else { checkout.to_path_buf() };
    for path in [&input, &output] {
        let p = path.to_string_lossy().replace('/', "\\").to_lowercase();
        let root = checkout.to_string_lossy().replace('/', "\\").to_lowercase();
        if p == root || p.starts_with(&(root.trim_end_matches('\\').to_owned() + "\\")) {
            return Err(invalid("company inputs and results must stay outside public checkout").into());
        }
    }
    if hash(&input)? != expected || solidcraft_engine::crash::enabled() {
        return Err(invalid("source hash changed or crash collection enabled").into());
    }
    fs::create_dir(&output)?;
    let mut journal = File::create(output.join("journal.jsonl"))?;
    let started = Instant::now();
    phase(&mut journal, started, "open_baseline")?;
    let mut source = Session::default();
    source.execute("doc.open", &json!({"path":input.to_string_lossy(),"up":"z","names":"solid"}))?;
    let source_doc = source.doc.to_json();
    let baseline = one(&source)?;
    let faces = baseline.faces(0.01)?;
    let mesh = baseline.tessellate(0.01)?;
    if faces.len() != 6
        || baseline.shell_count() != 1
        || !baseline.validity().is_empty()
        || faces.iter().any(|f| f.plane_normal.is_none() || boundary(&mesh, f.index).len() != 4)
    {
        return Err(invalid("requires one valid six-plane quadrilateral body").into());
    }
    let measure = solidcraft_kernel::measure(&baseline)?;
    let target = faces.iter().max_by(|a, b| a.area.total_cmp(&b.area)).ok_or_else(|| invalid("target face"))?;
    let edges = boundary(&mesh, target.index);
    let longest = edges
        .iter()
        .filter_map(|e| Some((*e.first()?, *e.last()?)))
        .max_by(|(a, b), (c, d)| a.dist(*b).total_cmp(&c.dist(*d)))
        .ok_or_else(|| invalid("target edge"))?;
    let axis = (longest.1 - longest.0).normalized().ok_or_else(|| invalid("target edge direction"))?;
    let min = edges.iter().flatten().map(|p| p.dot(axis)).fold(f64::INFINITY, f64::min);
    let max = edges.iter().flatten().map(|p| p.dot(axis)).fold(f64::NEG_INFINITY, f64::max);
    let mut derived = baseline.clone();
    let mut baseline_face: Vec<_> = (0..6).collect();
    phase(&mut journal, started, "create_controlled_subdivisions")?;
    let prepare = Instant::now();
    for fraction in [1. / 3., 2. / 3.] {
        let offset = min + (max - min) * fraction;
        let mesh = derived.tessellate(0.01)?;
        let candidates: Vec<_> = baseline_face
            .iter()
            .enumerate()
            .filter_map(|(i, old)| {
                if *old != target.index {
                    return None;
                }
                let points: Vec<_> = boundary(&mesh, i).into_iter().flatten().collect();
                let low = points.iter().map(|p| p.dot(axis)).fold(f64::INFINITY, f64::min);
                let high = points.iter().map(|p| p.dot(axis)).fold(f64::NEG_INFINITY, f64::max);
                (low < offset - 1e-6 && high > offset + 1e-6).then_some(i)
            })
            .collect();
        let [index] = candidates.as_slice() else { return Err(invalid("controlled plane does not cross one target patch").into()) };
        let u = axis.any_perp();
        let plane = Plane::new(axis * offset, u, axis.cross(u)).ok_or_else(|| invalid("controlled cut plane"))?;
        let (next, origin) = solidcraft_kernel::split_faces_with_map(&derived, &[*index], &SplitTool::Plane(plane))?;
        baseline_face = origin.iter().map(|i| baseline_face.get(*i).copied().ok_or_else(|| invalid("split lineage"))).collect::<Result<_, _>>()?;
        derived = next;
    }
    if derived.face_count() != 8 || !derived.validity().is_empty() {
        return Err(invalid("controlled source did not produce8validfaces").into());
    }
    let common = output.join("controlled-subdivision.step");
    fs::write(&common, export_with_boundaries(&derived)?)?;
    let common_hash = hash(&common)?;
    if common_expected.as_ref().is_some_and(|h| h != &common_hash) {
        return Err(invalid("controlled subdivision hash differs from frozen shared input").into());
    }
    let preparation_ms = prepare.elapsed().as_secs_f64() * 1000.;
    phase(&mut journal, started, "open_common")?;
    let open_started = Instant::now();
    let mut current = Session::default();
    current.execute("doc.open", &json!({"path":common.to_string_lossy(),"up":"z","names":"solid"}))?;
    let common_open_ms = open_started.elapsed().as_secs_f64() * 1000.;
    let common_doc = current.doc.to_json();
    let world = current.world_state();
    let body = world.bodies.first().filter(|_| world.bodies.len() == 1).ok_or_else(|| invalid("common body"))?;
    let common_faces = body.body.faces(0.01)?;
    if common_faces.len() != 8 || !body.body.validity().is_empty() {
        return Err(invalid("shared input did not reopen as8validfaces").into());
    }
    let mut protected = Vec::new();
    for f in faces.iter().filter(|f| f.index != target.index) {
        let matches: Vec<_> = common_faces
            .iter()
            .filter_map(|g| face_delta(f, g).filter(|(d, a, v)| *d <= 0.01 && *a <= 0.1 && *v <= 0.001).map(|_| g.index))
            .collect();
        let [index] = matches.as_slice() else { return Err(invalid("baseline non-target face is not unique in common input").into()) };
        protected.push(*index);
    }
    let key = body.geometry_key().map(|k| format!("{k:016x}")).ok_or_else(|| invalid("common geometry key"))?;
    phase(&mut journal, started, "merge_coplanar")?;
    let merge_started = Instant::now();
    let mut preview = current.execute(
        "thermal.merge_coplanar_preview",
        &json!({"body":body.name,"geometry_key":key,"protection":"explicit_faces","protected_faces":protected,"include_native":true,"input_boundary_tolerance_mm":input_tolerance}),
    )?;
    let merge_ms = merge_started.elapsed().as_secs_f64() * 1000.;
    if preview.get("status").and_then(serde_json::Value::as_str) != Some("merged")
        || preview.get("candidate_faces").and_then(serde_json::Value::as_u64) != Some(6)
    {
        return Err(invalid("expected a6face merged candidate").into());
    }
    let hex = preview.get("candidate_native_hex").and_then(serde_json::Value::as_str).ok_or_else(|| invalid("native candidate"))?;
    if !hex.len().is_multiple_of(2) || hex.len() > 16 * 1024 * 1024 {
        return Err(invalid("native candidate budget").into());
    }
    let bytes =
        hex.as_bytes().chunks_exact(2).map(|s| Ok(u8::from_str_radix(std::str::from_utf8(s)?, 16)?)).collect::<Result<Vec<_>, Box<dyn Error>>>()?;
    let candidate = solidcraft_kernel::decode_native_body(&bytes)?;
    let native_faces = candidate.faces(0.01)?;
    let native_mesh = candidate.tessellate(0.01)?;
    let before_checks = face_checks(&faces, &mesh, &native_faces, &native_mesh);
    let step = output.join("candidate.step");
    fs::write(&step, export_with_boundaries(&candidate)?)?;
    fs::write(output.join("candidate.scbrep"), bytes)?;
    phase(&mut journal, started, "independent_STEP_reopen")?;
    let mut reopened = Session::default();
    reopened.execute("doc.open", &json!({"path":step.to_string_lossy(),"up":"z","names":"solid"}))?;
    let again = one(&reopened)?;
    let final_faces = again.faces(0.01)?;
    let final_mesh = again.tessellate(0.01)?;
    let final_checks = face_checks(&faces, &mesh, &final_faces, &final_mesh);
    let volume = solidcraft_kernel::measure(&again)?;
    let delta = (volume.volume - measure.volume).abs() / measure.volume;
    let observed_boundary_error = preview
        .get("max_merged_group_boundary_error_mm")
        .and_then(serde_json::Value::as_f64)
        .filter(|v| v.is_finite())
        .ok_or_else(|| invalid("missing boundary consistency observation"))?;
    let unchanged =
        hash(&input)? == expected && hash(&common)? == common_hash && source.doc.to_json() == source_doc && current.doc.to_json() == common_doc;
    let passed = all_passed(&before_checks)
        && all_passed(&final_checks)
        && final_faces.len() == 6
        && again.validity().is_empty()
        && delta <= 1e-6
        && (input_tolerance == 0. || observed_boundary_error <= input_tolerance)
        && unchanged;
    preview.as_object_mut().ok_or_else(|| invalid("preview object"))?.remove("candidate_native_hex");
    let report = json!({"format":"solidcraft.controlled-coplanar-probe/1","status":if passed{"passed_controlled_coplanar_merge"}else{"checks_failed"},
        "source_sha256":expected,"source_unchanged":unchanged,"common_subdivision_sha256":common_hash,"candidate_step_sha256":hash(&step)?,
        "source_faces":6,"common_faces":8,"candidate_faces":final_faces.len(),"source_edges":baseline.edge_count(),"common_edges":body.body.edge_count(),"candidate_edges":again.edge_count(),
        "operation_protected_faces":5,"original_faces_checked":6,"baseline_target_face":target.index,"controlled_collinear_subdivision_allowed":true,
        "input_boundary_tolerance_mm":input_tolerance,"max_merged_group_boundary_error_mm":observed_boundary_error,
        "native_baseline_checks":before_checks,"STEP_reopen_baseline_checks":final_checks,"relative_volume_delta":delta,"source_volume_mm3":measure.volume,
        "controlled_preparation_ms":preparation_ms,"common_open_ms":common_open_ms,"merge_preview_ms":merge_ms,"total_ms":started.elapsed().as_secs_f64()*1000.,
        "preview":preview,"thermal_equivalence":"not_evaluated","scope":"controlled_subdivision_on_admitted_source_not_natural_assembly_simplification"});
    fs::write(output.join("report.json"), serde_json::to_vec_pretty(&report)?)?;
    phase(&mut journal, started, if passed { "finished" } else { "checks_failed" })?;
    println!("{}", report);
    if passed { Ok(()) } else { Err(invalid("controlled merge checks failed").into()) }
}
