//! Local analytic coplanar merge, explicit face protection and separate STEP reopen.
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use solidcraft_engine::Session;
use solidcraft_geom::{Mesh, Vec3};
use solidcraft_kernel::{Body, FaceInfo};
use std::{error::Error, io, path::PathBuf};
#[path = "support/thermal_tiled_plate.rs"]
mod fixture;

fn invalid(reason: &str) -> io::Error {
    io::Error::other(reason)
}
fn geometry(session: &Session) -> Result<Body, io::Error> {
    let world = session.world_state();
    if world.bodies.len() != 1 {
        return Err(invalid("expected one candidate body"));
    }
    world.bodies.first().map(|b| b.body.clone()).ok_or_else(|| invalid("missing candidate body"))
}
fn boundary(mesh: &Mesh, face: usize) -> Vec<Vec<Vec3>> {
    mesh.edge_faces
        .iter()
        .zip(&mesh.edges)
        .filter_map(|(faces, line)| faces.iter().any(|i| usize::try_from(*i).ok() == Some(face)).then_some(line.clone()))
        .collect()
}
fn distance(a: &[Vec<Vec3>], b: &[Vec<Vec3>]) -> f64 {
    a.iter()
        .flatten()
        .map(|point| {
            b.iter()
                .flat_map(|line| line.windows(2))
                .filter_map(|seg| Some(point.dist_to_segment(*seg.first()?, *seg.get(1)?)))
                .fold(f64::INFINITY, f64::min)
        })
        .fold(0., f64::max)
}
fn checks(original: &[FaceInfo], source_mesh: &Mesh, protected: &[usize], candidate: &Body) -> Result<Vec<Value>, Box<dyn Error>> {
    let faces = candidate.faces(0.001)?;
    let mesh = candidate.display_mesh(0.001)?;
    let mut result = Vec::new();
    let mut matched = std::collections::BTreeSet::new();
    for input in protected {
        let source = original.get(*input).ok_or_else(|| invalid("missing protected source"))?;
        let matches: Vec<_> = faces
            .iter()
            .filter(|f| {
                source.centroid.dist(f.centroid) <= 0.01
                    && (source.area - f.area).abs() / source.area <= 0.001
                    && source.plane_normal.zip(f.plane_normal).is_some_and(|(a, b)| a.dot(b).clamp(-1., 1.).acos().to_degrees() <= 0.1)
            })
            .collect();
        let [face] = matches.as_slice() else { return Err(invalid("protected STEP face match is not unique").into()) };
        if !matched.insert(face.index) {
            return Err(invalid("protected STEP faces reused the same match").into());
        }
        let a = boundary(source_mesh, *input);
        let b = boundary(&mesh, face.index);
        let error = distance(&a, &b).max(distance(&b, &a));
        if a.is_empty() || a.len() != b.len() || !error.is_finite() || error > 0.01 {
            return Err(invalid("protected STEP trim changed").into());
        }
        result.push(json!({"source_face":input,"candidate_face":face.index,"boundary_distance_mm":error,"passed":true}));
    }
    Ok(result)
}
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let directory = PathBuf::from(args.next().unwrap_or_else(|| "target/thermal-merge-demo".into()));
    if args.next().is_some() {
        return Err(invalid("pass one new output directory").into());
    }
    if let Some(parent) = directory.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::create_dir(&directory)?;
    let directory = directory.canonicalize()?;
    let analytic = fixture::tiled_plate()?;
    let input = directory.join("source.step");
    std::fs::write(&input, fixture::step_with_boundaries(&analytic, "thermal-merge-analytic")?)?;
    let input_bytes = std::fs::read(&input)?;
    let mut source = Session::default();
    source.execute("doc.open", &json!({"path":input.to_string_lossy(),"up":"z","names":"solid"}))?;
    let before = source.doc.to_json();
    let world = source.world_state();
    let body = world.bodies.first().ok_or_else(|| invalid("missing source placement"))?;
    let key = body.geometry_key().map(|k| format!("{k:016x}")).ok_or_else(|| invalid("missing geometry key"))?;
    let original = body.body.faces(0.001)?;
    let original_mesh = body.body.display_mesh(0.001)?;
    // The fixture explicitly defines its eight top tiles as protected interfaces.
    let protected: Vec<_> = original.iter().filter(|f| f.plane_normal.is_some_and(|n| n.z > 0.9)).map(|f| f.index).collect();
    if body.body.face_count() != 32 || protected.len() != 8 {
        return Err(invalid("unexpected analytic source").into());
    }
    let request = json!({"body":body.name,"geometry_key":key,"protection":"explicit_faces","protected_faces":protected,"include_native":true});
    let mut preview = source.execute("thermal.merge_coplanar_preview", &request)?;
    let free = source.execute(
        "thermal.merge_coplanar_preview",
        &json!({"body":body.name,"geometry_key":key,"protection":"explicit_faces","protected_faces":[],"allow_empty_protection":true}),
    )?;
    if preview.get("candidate_faces").and_then(Value::as_u64) != Some(17) || free.get("candidate_faces").and_then(Value::as_u64) != Some(10) {
        return Err(invalid("unexpected merge counts").into());
    }
    let hex = preview.get("candidate_native_hex").and_then(Value::as_str).ok_or_else(|| invalid("missing native candidate"))?;
    if !hex.len().is_multiple_of(2) || hex.len() > 16 * 1024 * 1024 {
        return Err(invalid("native encoding budget").into());
    }
    let mut bytes = Vec::with_capacity(hex.len() / 2);
    for pair in hex.as_bytes().chunks_exact(2) {
        bytes.push(u8::from_str_radix(std::str::from_utf8(pair)?, 16)?);
    }
    std::fs::write(directory.join("candidate.scbrep"), &bytes)?;
    let candidate = solidcraft_kernel::decode_native_body(&bytes)?;
    let native_checks = checks(&original, &original_mesh, &protected, &candidate)?;
    let step = directory.join("candidate.step");
    std::fs::write(&step, fixture::step_with_boundaries(&candidate, "thermal-merge-candidate")?)?;
    let mut reopened = Session::default();
    let opened = reopened.execute("doc.open", &json!({"path":step.to_string_lossy(),"up":"z","names":"solid"}))?;
    let candidate = geometry(&reopened)?;
    let step_checks = checks(&original, &original_mesh, &protected, &candidate)?;
    if candidate.face_count() != 17
        || !candidate.validity().is_empty()
        || candidate.contains(Vec3::new(1.5, 1.5, 0.5))
        || (solidcraft_kernel::measure(&candidate)?.volume - 8.).abs() > 8e-6
        || source.doc.to_json() != before
        || std::fs::read(&input)? != input_bytes
    {
        return Err(invalid("candidate reopen, opening, volume or source check failed").into());
    }
    std::fs::write(directory.join("source.solidcraft"), solidcraft_io::write_design(&source.doc))?;
    std::fs::write(directory.join("candidate.solidcraft"), solidcraft_io::write_design(&reopened.doc))?;
    preview.as_object_mut().ok_or_else(|| invalid("preview report"))?.remove("candidate_native_hex");
    let report = json!({"format":"solidcraft.thermal-coplanar-lab/1","source_faces":32,"protected_preview_faces":17,"unprotected_preview_faces":10,
        "protected_faces":8,"native_protection_checks":native_checks,"step_reopen_protection_checks":step_checks,
        "source_unchanged":true,"square_through_opening_retained":true,"step_reopen_faces":candidate.face_count(),"reopen_reply":opened,
        "preview":preview,"unprotected_preview":free,"thermal_equivalence":"not_evaluated","input_scope":"analytic_fixture_only",
        "source_sha256":format!("{:x}",Sha256::digest(&input_bytes)),"candidate_step_sha256":format!("{:x}",Sha256::digest(std::fs::read(&step)?)),
        "candidate_native_sha256":format!("{:x}",Sha256::digest(&bytes))});
    std::fs::write(directory.join("report.json"), serde_json::to_vec_pretty(&report)?)?;
    println!(
        "{}",
        json!({"source_faces":32,"protected_candidate_faces":17,"unprotected_candidate_faces":10,"protected_faces_checked_after_STEP_reopen":8,"source_unchanged":true,"opening_retained":true,"directory":directory})
    );
    Ok(())
}
