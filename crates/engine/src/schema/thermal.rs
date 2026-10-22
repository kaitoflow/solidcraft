//! The isolated thermal simplification experiment's read-only planning contract.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct EnvelopeParams {
    /// Exact world-placement body names, not component definitions. Between 1 and 128.
    pub bodies: Vec<String>,
    /// Exact placements that the caller has explicitly protected. At most 128.
    #[serde(default)]
    pub protected_bodies: Vec<String>,
    /// Padding on every side in millimetres, independent of the design's display unit.
    #[serde(default)]
    pub padding_mm: f64,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct Bounds {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct EnvelopeSource {
    pub world_body: String,
    pub definition_body: String,
    pub feature_id: u64,
    pub component_id: u64,
    /// Session geometry identity only; not an artifact hash or a cross-file persistent ID.
    pub geometry_key: Option<String>,
    pub definition_to_world: [[f64; 4]; 4],
    pub bbox_mm: Bounds,
    pub raw_faces: usize,
    pub raw_edges: usize,
    /// A kernel candidate measure, present only after the checks named below passed.
    pub candidate_volume_mm3: Option<f64>,
    pub solid_check_method: String,
    pub solid_check_issues: Vec<String>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct BoxProposal {
    pub shape: String,
    pub corner_mm: [f64; 3],
    pub size_mm: [f64; 3],
    pub volume_mm3: f64,
    pub faces: usize,
    pub source_fill_fraction: f64,
    pub volume_change_mm3: f64,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct EnvelopeEntry {
    pub source: EnvelopeSource,
    /// Machine-readable reasons. A blocked entry never contains an executable proposal.
    pub blocked_reasons: Vec<String>,
    pub proposal: Option<BoxProposal>,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct EnvelopePlan {
    pub format: String,
    pub operation: String,
    pub frame: String,
    pub length_unit: String,
    pub bounds_method: String,
    pub design_modified: bool,
    pub thermal_equivalence: String,
    pub requested: usize,
    pub candidates: usize,
    pub blocked: usize,
    pub entries: Vec<EnvelopeEntry>,
}

#[derive(Debug, Deserialize, JsonSchema, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum InterfaceProtection {
    Unconfirmed,
    WholeBody,
    ExplicitFaces,
}

#[derive(Debug, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MergePreviewParams {
    pub body: String,
    /// Current session geometry key for this exact world placement; mandatory freshness check.
    pub geometry_key: String,
    pub protection: InterfaceProtection,
    /// Snapshot face indices, never a cross-file or semantic engineering identity.
    pub protected_faces: Vec<usize>,
    #[serde(default)]
    pub allow_empty_protection: bool,
    /// Include bounded, producer-specific native geometry for a separate candidate consumer.
    #[serde(default)]
    pub include_native: bool,
    /// Explicit allowance for existing Line boundary/support inconsistency in mm,0..0.01.
    /// Default0 retains machine-arithmetic checks; it does not relax plane recognition.
    #[serde(default)]
    pub input_boundary_tolerance_mm: f64,
}

#[derive(Debug, Serialize, JsonSchema)]
pub struct MergePreview {
    pub format: String,
    pub operation: String,
    pub status: String,
    pub blocked_reasons: Vec<String>,
    pub world_body: String,
    pub source_geometry_key: String,
    pub frame: String,
    pub length_unit: String,
    pub design_modified: bool,
    pub thermal_equivalence: String,
    pub protection_scope: String,
    pub source_faces: usize,
    pub candidate_faces: Option<usize>,
    pub source_volume_mm3: Option<f64>,
    pub candidate_volume_mm3: Option<f64>,
    pub relative_volume_delta: Option<f64>,
    /// Output index is the vector position; each list contains its input face indices.
    pub output_face_sources: Vec<Vec<usize>>,
    /// Immediate input snapshot only; not a persistent engineering or composed ancestor identity.
    pub lineage_scope: String,
    /// Input/output indices, whose face and boundary identities were retained in memory.
    pub protected_face_mapping: Vec<[usize; 2]>,
    pub protected_edges_preserved: usize,
    /// Native B-rep bytes in hex, at most 8 MiB before encoding. Not a file path or executable code.
    pub candidate_native_hex: Option<String>,
    /// Experimental, same-producer archive encoding; absent when no archive is requested.
    pub candidate_encoding: Option<String>,
    /// STEP interchange is checked separately by a fixed export/reopen runner.
    pub step_interchange_check: String,
    pub input_boundary_tolerance_mm: f64,
    pub max_merged_group_boundary_error_mm: Option<f64>,
}
