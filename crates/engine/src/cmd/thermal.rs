//! Read-only, explicit-selection envelope planning for the independent thermal lab.
//! No source body is removed, no feature is added, and no engineering parameter is inferred.

use std::collections::BTreeSet;

use serde_json::Value;
use solidcraft_geom::Vec3;

use super::CommandSpec;
use crate::params::bad;
use crate::schema::thermal::{Bounds, BoxProposal, EnvelopeEntry, EnvelopeParams, EnvelopePlan, EnvelopeSource};
use crate::{Result, Session};

const ID: &str = "thermal.envelope_plan";
const MAX_BODIES: usize = 128;
const MAX_PADDING_MM: f64 = 1_000_000.0;
const CONTRACT: crate::schema::TypedContract<EnvelopeParams> = crate::schema::TypedContract::new();

pub static COMMANDS: &[CommandSpec] = &[CommandSpec::new(ID, "Plan Thermal Envelopes", envelope_plan)
    .icon("inspect")
    .noundo()
    .contract(&CONTRACT)
    .output(crate::schema::output_schema_for::<EnvelopePlan>)
    .params(
        "bodies: [1..128 exact world body names]; protected_bodies?: [names]; padding_mm?: finite number 0..1000000 (default 0). \
         Read-only world-mm axis-aligned box proposals, source definition/placement, raw topology and checked candidate volume; \
         protected, mesh or invalid bodies are blocked. No inferred thermal role, material, loss, acceptance or geometry change.",
    )];

fn xyz(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn finite(v: Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

fn names(names: &[String], key: &str, required: bool) -> Result<()> {
    if (required && names.is_empty()) || names.len() > MAX_BODIES {
        return Err(bad(ID, format!("`{key}` must contain {} to {MAX_BODIES} names", usize::from(required))));
    }
    let mut seen = BTreeSet::new();
    for name in names {
        if name.trim().is_empty() || name.len() > 1024 || !seen.insert(name) {
            return Err(bad(ID, format!("`{key}` must contain distinct, non-empty names of at most 1024 UTF-8 bytes")));
        }
    }
    Ok(())
}

fn envelope_plan(s: &mut Session, raw: &Value) -> Result<Value> {
    // Always validate, including when the upstream optional parameter-check mode is Off.
    let params: EnvelopeParams = serde_json::from_value(raw.clone()).map_err(|e| bad(ID, e.to_string()))?;
    names(&params.bodies, "bodies", true)?;
    names(&params.protected_bodies, "protected_bodies", false)?;
    if !params.padding_mm.is_finite() || !(0.0..=MAX_PADDING_MM).contains(&params.padding_mm) {
        return Err(bad(ID, "`padding_mm` must be finite and between 0 and 1000000 mm"));
    }

    let world = s.world_state();
    for name in params.bodies.iter().chain(params.protected_bodies.iter()) {
        if world.bodies.iter().filter(|b| &b.name == name).count() != 1 {
            return Err(bad(ID, format!("`{name}` must identify exactly one world body placement")));
        }
    }

    let mut entries = Vec::with_capacity(params.bodies.len());
    for name in &params.bodies {
        let body = world.body(name).ok_or_else(|| bad(ID, "selected world body is no longer available"))?;
        // Outline bounds intentionally ignore a face bulging beyond its boundary. Use the
        // measurement's coarse/fine surface bounds, and identify their sampled nature.
        let measured = solidcraft_kernel::measure(&body.body).map_err(|e| bad(ID, format!("`{name}` could not be measured: {e}")))?;
        let bb = measured.bbox;
        let size = bb.max - bb.min;
        if bb.is_empty() || !finite(bb.min) || !finite(bb.max) || !finite(size) || size.x <= 0.0 || size.y <= 0.0 || size.z <= 0.0 {
            return Err(bad(ID, format!("`{name}` has no finite three-dimensional bounds")));
        }
        if body.source_transform.iter().flatten().any(|v| !v.is_finite()) {
            return Err(bad(ID, format!("`{name}` has a non-finite placement")));
        }

        let padding = Vec3::new(params.padding_mm, params.padding_mm, params.padding_mm);
        let corner = bb.min - padding;
        let upper = bb.max + padding;
        let envelope_size = upper - corner;
        let envelope_volume = envelope_size.x * envelope_size.y * envelope_size.z;
        if !finite(corner) || !finite(upper) || !finite(envelope_size) || !envelope_volume.is_finite() || envelope_volume <= 0.0 {
            return Err(bad(ID, format!("`{name}` has an unusable padded envelope")));
        }

        let mut blocked = Vec::new();
        if params.protected_bodies.contains(name) {
            blocked.push("protected_body".to_string());
        }
        let mut issues = Vec::new();
        let mut volume = None;
        let method;
        if body.body.is_mesh() {
            method = "not_checked_mesh_body";
            blocked.push("mesh_body".to_string());
        } else {
            method = "solidcraft_kernel.Body.validity";
            issues = body.body.validity();
            if issues.is_empty() {
                if measured.volume.is_finite() && measured.volume > 0.0 {
                    volume = Some(measured.volume);
                } else {
                    issues.push("candidate volume is not finite and positive".into());
                }
            }
            if !issues.is_empty() {
                blocked.push("solid_checks_failed".to_string());
            }
        }
        // A measure inconsistent with its sampled bounds cannot become a proposal.
        if volume.is_some_and(|v| v > envelope_volume * (1.0 + 1e-6)) {
            blocked.push("volume_exceeds_envelope".to_string());
        }
        let proposal = if blocked.is_empty() {
            volume.map(|v| BoxProposal {
                shape: "axis_aligned_box".into(),
                corner_mm: xyz(corner),
                size_mm: xyz(envelope_size),
                volume_mm3: envelope_volume,
                faces: 6,
                source_fill_fraction: v / envelope_volume,
                volume_change_mm3: envelope_volume - v,
            })
        } else {
            None
        };
        entries.push(EnvelopeEntry {
            source: EnvelopeSource {
                world_body: name.clone(),
                definition_body: body.definition_name().into(),
                feature_id: body.feature,
                component_id: body.component,
                geometry_key: body.geometry_key().map(|key| format!("{key:016x}")),
                definition_to_world: body.source_transform,
                bbox_mm: Bounds { min: xyz(bb.min), max: xyz(bb.max) },
                raw_faces: body.body.face_count(),
                raw_edges: body.body.edge_count(),
                candidate_volume_mm3: volume,
                solid_check_method: method.into(),
                solid_check_issues: issues,
            },
            blocked_reasons: blocked,
            proposal,
        });
    }
    let candidates = entries.iter().filter(|entry| entry.proposal.is_some()).count();
    serde_json::to_value(EnvelopePlan {
        format: "solidcraft.thermal-envelope-plan/1".into(),
        operation: ID.into(),
        frame: "world".into(),
        length_unit: "mm".into(),
        bounds_method: "solidcraft_kernel.measure.sampled_surface_bounds".into(),
        design_modified: false,
        thermal_equivalence: "not_evaluated".into(),
        requested: params.bodies.len(),
        candidates,
        blocked: entries.len() - candidates,
        entries,
    })
    .map_err(|e| bad(ID, format!("could not serialize the envelope plan: {e}")))
}

#[cfg(test)]
#[path = "thermal_tests.rs"]
mod tests;
