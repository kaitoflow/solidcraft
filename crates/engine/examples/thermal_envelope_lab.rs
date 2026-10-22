//! Analytic, public-safe end-to-end demonstration for the isolated thermal lab.
//! Creates source geometry, reads a plan, and builds one proposal in a separate design.

use std::error::Error;
use std::io;
use std::path::PathBuf;

use serde_json::{Value, json};
use solidcraft_engine::Session;

fn missing(what: &str) -> io::Error {
    io::Error::other(format!("missing or invalid {what}"))
}

fn mm(v: &Value) -> Result<String, io::Error> {
    v.as_f64().filter(|n| n.is_finite()).map(|n| format!("{n} mm")).ok_or_else(|| missing("finite millimetre dimension"))
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let output = PathBuf::from(args.next().unwrap_or_else(|| "target/thermal-envelope-demo".into()));
    if args.next().is_some() {
        return Err(missing("arguments: pass one new output directory").into());
    }
    // An existing run is never overwritten.
    if let Some(parent) = output.parent().filter(|p| !p.as_os_str().is_empty()) {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::create_dir(&output)?;

    let mut source = Session::default();
    for (command, params) in [
        ("solid.box", json!({"length":40,"width":20,"height":4,"body_name":"Bracket"})),
        ("solid.box", json!({"corner":[0,0,4],"length":4,"width":20,"height":16,"operation":"join","targets":["Bracket"]})),
        ("solid.box", json!({"corner":[60,0,0],"length":10,"width":20,"height":1,"body_name":"ProtectedPlate"})),
    ] {
        source.execute(command, &params)?;
    }
    let before = source.doc.to_json();
    let plan = source.execute("thermal.envelope_plan", &json!({"bodies":["Bracket","ProtectedPlate"],"protected_bodies":["ProtectedPlate"]}))?;
    if source.doc.to_json() != before || plan.get("candidates").and_then(Value::as_u64) != Some(1) {
        return Err(missing("read-only plan with exactly one candidate").into());
    }
    std::fs::write(output.join("plan.json"), serde_json::to_vec_pretty(&plan)?)?;
    std::fs::write(output.join("source.solidcraft"), solidcraft_io::write_design(&source.doc))?;
    let world = source.world_state();
    let bodies: Vec<_> = world.bodies.iter().map(|b| &b.body).collect();
    std::fs::write(output.join("source.step"), solidcraft_kernel::step_export(&bodies, "thermal-lab-source")?)?;

    let entry = plan
        .get("entries")
        .and_then(Value::as_array)
        .and_then(|entries| entries.iter().find(|entry| entry.pointer("/source/world_body").and_then(Value::as_str) == Some("Bracket")))
        .ok_or_else(|| missing("Bracket plan entry"))?;
    let proposal = entry.get("proposal").filter(|p| !p.is_null()).ok_or_else(|| missing("box proposal"))?;
    let corner = proposal.get("corner_mm").and_then(Value::as_array).filter(|v| v.len() == 3).ok_or_else(|| missing("proposal corner"))?;
    let corner: Vec<String> = corner.iter().map(mm).collect::<Result<_, _>>()?;
    let size = proposal.get("size_mm").and_then(Value::as_array).ok_or_else(|| missing("proposal size"))?;
    let [length, width, height] = size.as_slice() else { return Err(missing("three proposal dimensions").into()) };
    // This deliberately separate design uses the existing registered box command. It is
    // not an apply operation on the original document or a thermal acceptance decision.
    let mut preview = Session::default();
    preview.execute("document.units", &json!({"units":"mm"}))?;
    preview
        .execute("solid.box", &json!({"corner":corner,"length":mm(length)?,"width":mm(width)?,"height":mm(height)?,"body_name":"BracketEnvelope"}))?;
    std::fs::write(output.join("preview.solidcraft"), solidcraft_io::write_design(&preview.doc))?;
    let preview_state = preview.world_state();
    let body = &preview_state.bodies.first().ok_or_else(|| missing("preview body"))?.body;
    let step = solidcraft_kernel::step_export(&[body], "thermal-lab-preview")?;
    std::fs::write(output.join("preview.step"), &step)?;
    let reopened = solidcraft_kernel::step_import(&step)?;
    if reopened.bodies.len() != 1 || !reopened.warnings.is_empty() || reopened.bodies.iter().any(|b| !b.closed) {
        return Err(missing("closed warning-free preview STEP round trip").into());
    }
    let rebuilt = &reopened.bodies.first().ok_or_else(|| missing("reopened preview"))?.body;
    let measure = solidcraft_kernel::measure(rebuilt)?;
    let expected = proposal.get("volume_mm3").and_then(Value::as_f64).ok_or_else(|| missing("proposal volume"))?;
    if (measure.volume - expected).abs() > expected * 1e-6 {
        return Err(missing("preview volume round trip").into());
    }
    let report = json!({
        "fixture":"analytic_L_bracket_and_protected_plate",
        "length_unit":"mm",
        "original_unchanged":source.doc.to_json() == before,
        "source_faces":entry.pointer("/source/raw_faces"),
        "preview_faces":measure.faces,
        "source_candidate_volume_mm3":entry.pointer("/source/candidate_volume_mm3"),
        "preview_volume_mm3":measure.volume,
        "protected_body_has_no_proposal":plan.pointer("/entries/1/proposal").is_some_and(Value::is_null),
        "preview_step_reopened":true,
        "preview_scope":"one_candidate_in_a_separate_design",
        "thermal_equivalence":"not_evaluated",
        "spaceclaim_comparison":"not_run"
    });
    std::fs::write(output.join("report.json"), serde_json::to_vec_pretty(&report)?)?;
    println!("{}", serde_json::to_string_pretty(&report)?);
    Ok(())
}
