//! Exact topological removal of construction-induced planar subdivisions.
//! Shared edge identity and complete construction lineage select the groups.
use super::{Body, from_p3};
use crate::history::FaceOrigin;
use crate::{Result, guard};
use solidcraft_geom::Plane;
use std::collections::{BTreeMap, HashMap, HashSet};
use std::sync::Arc;
use truck_modeling as mt;

fn root(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}

fn merged_face(faces: &[&mt::Face], members: &[usize], plane: Plane, eligible: &HashSet<mt::EdgeID>) -> Option<mt::Face> {
    let mut counts = HashMap::<mt::EdgeID, (usize, bool)>::new();
    for i in members {
        for edge in faces.get(*i)?.boundary_iters().into_iter().flatten() {
            // Whole linear supports, not a sampled plane fit.
            let mt::Curve::Line(line) = edge.curve() else { return None };
            let (a, b) = (from_p3(line.0), from_p3(line.1));
            if !a.is_finite() || !b.is_finite() || a.len().max(b.len()) > 1e9 {
                return None;
            }
            let front = from_p3(edge.absolute_front().point());
            let back = from_p3(edge.absolute_back().point());
            if !front.is_finite() || !back.is_finite() {
                return None;
            }
            let error = 4096. * f64::EPSILON * (1. + a.len() + b.len() + plane.origin.len());
            if plane.height(a).abs() > error || plane.height(b).abs() > error || a.dist(front) > error || b.dist(back) > error {
                return None;
            }
            let count = counts.entry(edge.id()).or_insert((0, edge.orientation()));
            if count.0 == 1 && (!eligible.contains(&edge.id()) || count.1 == edge.orientation()) {
                return None;
            }
            count.0 += 1;
            if count.0 > 2 {
                return None;
            }
        }
    }
    let mut boundary = Vec::new();
    for i in members {
        for edge in faces.get(*i)?.boundary_iters().into_iter().flatten() {
            if counts.get(&edge.id()).is_some_and(|(n, _)| *n == 1) {
                boundary.push(edge);
            }
        }
    }
    let mut next = HashMap::new();
    let mut incoming = HashMap::new();
    for (i, edge) in boundary.iter().enumerate() {
        if next.insert(edge.front().id(), i).is_some() || incoming.insert(edge.back().id(), i).is_some() {
            return None;
        }
    }
    if next.len() != incoming.len() || next.keys().any(|id| !incoming.contains_key(id)) {
        return None;
    }
    let mut used = vec![false; boundary.len()];
    let mut outer = None;
    let mut holes = Vec::new();
    for first in 0..boundary.len() {
        if used[first] {
            continue;
        }
        let start = boundary.get(first)?.front().id();
        let origin = from_p3(boundary.get(first)?.front().point());
        let mut index = first;
        let mut edges = Vec::new();
        let (mut area, mut magnitude, mut arithmetic_error) = (0., 0., 0.);
        loop {
            if *used.get(index)? {
                return None;
            }
            used[index] = true;
            let edge = boundary.get(index)?.clone();
            let da = from_p3(edge.front().point()) - origin;
            let db = from_p3(edge.back().point()) - origin;
            let a = solidcraft_geom::Vec2::new(da.dot(plane.x), da.dot(plane.y));
            let b = solidcraft_geom::Vec2::new(db.dot(plane.x), db.dot(plane.y));
            let term = a.cross(b);
            area += term;
            magnitude += term.abs();
            let coordinate_error =
                128. * f64::EPSILON * (1. + origin.len() + from_p3(edge.front().point()).len() + from_p3(edge.back().point()).len());
            arithmetic_error += coordinate_error * (da.len() + db.len() + coordinate_error) * 16.;
            let back = edge.back().id();
            edges.push(edge);
            if back == start {
                break;
            }
            index = *next.get(&back)?;
        }
        if edges.len() < 3 || !area.is_finite() || area.abs() <= arithmetic_error + 64. * f64::EPSILON * (edges.len() as f64 + 1.) * magnitude.max(1.)
        {
            return None;
        }
        let wire = mt::Wire::from(edges);
        if area > 0. {
            if outer.replace(wire).is_some() {
                return None;
            }
        } else {
            holes.push(wire);
        }
    }
    let mut wires = vec![outer?];
    wires.extend(holes);
    mt::Face::try_new(wires, faces.get(*members.first()?)?.oriented_surface()).ok()
}

/// `body` with its history: `input`'s faces with each output face made of the input faces
/// `members` lists for it (every edge of `body` one of `input`'s, by identity), composed with
/// the history of the operation that made `input`; none where `input` has none.
fn with_joined_history(body: Body, input: &Body, members: Vec<Vec<usize>>) -> Body {
    let Some(inner) = input.history() else { return body };
    let faces = members.into_iter().map(|m| (m.iter().map(|&face| FaceOrigin::Face { operand: 0, face }).collect(), m.len() == 1)).collect();
    let joined = crate::record::traced(
        body,
        &crate::record::counts(&[input]),
        Some(crate::record::regions(faces)),
        Some(&crate::record::edge_origins(&input.solid, 0)),
    );
    // Both steps are checked (`traced` against `input`, `through` composing over its checked
    // history); the composed operands' counts are not known here, so it is set in place.
    let history = crate::boolean_record::through(&joined, 0, Some(inner));
    Body { history: history.map(Arc::new), ..joined }
}

impl Body {
    pub(crate) fn regularize_certified_planar_faces(self) -> Result<Self> {
        self.regularize_planar_faces(false)
    }

    /// Only newly produced Cut walls call this path. The temporary planes carry
    /// construction support, never persistent ancestor/source-role authority.
    pub(crate) fn regularize_generated_planar_faces(self) -> Result<Self> {
        let body = self.regularize_planar_faces(true)?;
        let sources = body.face_sources().map(|sources| {
            sources
                .iter()
                .map(|source| {
                    let mut source = source.clone();
                    if let Some(super::FaceSource::ExtrudeSide { plane, coalesced, .. }) = &mut source {
                        if plane.is_some() && !*coalesced {
                            source = None;
                        } else {
                            *plane = None;
                        }
                    }
                    source
                })
                .collect()
        });
        Ok(body.with_face_sources(sources.map(Arc::new)))
    }

    fn regularize_planar_faces(self, generated: bool) -> Result<Self> {
        if self.regularization_protected {
            return Ok(self);
        }
        let Some(sources) = self.face_sources() else { return Ok(self) };
        if sources.len() > 16_384 || self.paint().is_some_and(|p| !p.faces.is_empty()) {
            return Ok(self);
        }
        let split_points = self.split_keep().iter().try_fold(0usize, |sum, line| sum.checked_add(line.len()));
        let Some(split_points) = split_points.filter(|n| *n <= 65_536) else { return Ok(self) };
        let result = guard("certified planar topology regularization", || {
            let mut faces = Vec::new();
            let mut shell_owner = Vec::new();
            let mut use_budget = 65_536usize;
            for (shell_index, shell) in self.solid.boundaries().iter().enumerate() {
                for face in shell.face_iter() {
                    if faces.len() >= sources.len() {
                        return Ok(None);
                    }
                    for _ in face.edge_iter() {
                        let Some(left) = use_budget.checked_sub(1) else { return Ok(None) };
                        use_budget = left;
                    }
                    faces.push(face);
                    shell_owner.push(shell_index);
                }
            }
            if faces.len() != sources.len() {
                return Ok(None);
            }
            let valid: Vec<_> = faces
                .iter()
                .zip(sources)
                .map(|(f, s)| {
                    s.as_ref()
                        .and_then(|source| {
                            if generated {
                                match source {
                                    super::FaceSource::ExtrudeSide { segment: solidcraft_geom::Seg2::Line { .. }, plane: Some(p), .. } => Some(*p),
                                    _ => None,
                                }
                            } else {
                                super::planar_sources::certified_plane(source)
                            }
                        })
                        .filter(|p| super::planar_sources::actual_plane(f).is_some_and(|actual| super::planar_sources::same_support(p, &actual)))
                })
                .collect();
            let mut adjacency = HashMap::<mt::EdgeID, Vec<(usize, mt::Edge)>>::new();
            for (i, face) in faces.iter().enumerate() {
                for edge in face.boundary_iters().into_iter().flatten() {
                    adjacency.entry(edge.id()).or_default().push((i, edge));
                }
            }
            let mut eligible = HashSet::new();
            let mut parent: Vec<_> = (0..faces.len()).collect();
            let mut identity = 16_000_000usize;
            let mut work = 2_000_000usize;
            for uses in adjacency.values() {
                let [(a, ea), (b, eb)] = uses.as_slice() else { continue };
                if a == b || shell_owner[*a] != shell_owner[*b] || ea.orientation() == eb.orientation() {
                    continue;
                }
                let (Some(pa), Some(pb)) = (valid[*a], valid[*b]) else { continue };
                if pa != pb {
                    continue;
                }
                let (Some(sa), Some(sb)) = (sources[*a].as_ref(), sources[*b].as_ref()) else { continue };
                let Some(cost) = super::planar_sources::descriptor_cost(sa)
                    .and_then(|a| super::planar_sources::descriptor_cost(sb).and_then(|b| a.checked_add(b)))
                else {
                    continue;
                };
                let Some(left) = identity.checked_sub(cost) else { return Ok(None) };
                identity = left;
                if !generated && sa != sb {
                    continue;
                }
                let Some(cost) = split_points.checked_mul(6).and_then(|n| n.checked_add(1)) else { return Ok(None) };
                let Some(left) = work.checked_sub(cost) else { return Ok(None) };
                work = left;
                if crate::heal::on_split_line(&crate::heal::edge_points(ea), self.split_keep()) {
                    continue;
                }
                eligible.insert(ea.id());
                let (ra, rb) = (root(&mut parent, *a), root(&mut parent, *b));
                parent[ra.max(rb)] = ra.min(rb);
            }
            let mut groups = BTreeMap::<usize, Vec<usize>>::new();
            for i in 0..faces.len() {
                groups.entry(root(&mut parent, i)).or_default().push(i);
            }
            let mut replacements = HashMap::new();
            let mut changed = false;
            let mut metadata = 2_000_000usize;
            for (leader, members) in &groups {
                if members.len() < 2 {
                    continue;
                }
                let Some(face) = valid[*leader].and_then(|p| merged_face(&faces, members, p, &eligible)) else { continue };
                replacements.insert(*leader, face);
                changed = true;
            }
            if !changed {
                return Ok(None);
            }
            let mut shells = Vec::new();
            let mut output_sources = Vec::new();
            let mut output_ids = Vec::new();
            // The input faces each output face is made of, for its history.
            let mut output_members: Vec<Vec<usize>> = Vec::new();
            let mut offset = 0;
            for shell in self.solid.boundaries() {
                let mut output = Vec::new();
                for face in shell.face_iter() {
                    let leader = root(&mut parent, offset);
                    if let Some(merged) = replacements.get(&leader) {
                        if offset == leader {
                            output.push(merged.clone());
                            let Some(members) = groups.remove(&leader) else { return Ok(None) };
                            output_members.push(members);
                        } else {
                            offset += 1;
                            continue;
                        }
                    } else {
                        output.push(face.clone());
                        output_members.push(vec![offset]);
                    }
                    output_ids.push(output.last().ok_or_else(|| crate::KernelError::Failed("missing regularized face".into()))?.id());
                    let source = sources[offset].as_ref();
                    if let Some(source) = source {
                        let Some(cost) = super::planar_sources::descriptor_cost(source) else { return Ok(None) };
                        let Some(left) = metadata.checked_sub(cost) else { return Ok(None) };
                        metadata = left;
                    }
                    let mut source = source.cloned();
                    if generated
                        && replacements.contains_key(&leader)
                        && let Some(super::FaceSource::ExtrudeSide { coalesced, .. }) = &mut source
                    {
                        *coalesced = true;
                    }
                    output_sources.push(source);
                    offset += 1;
                }
                shells.push(mt::Shell::from(output));
            }
            let solid = mt::Solid::try_new(shells).map_err(|e| crate::KernelError::Failed(format!("planar regularization does not close: {e}")))?;
            let body = Body::new(solid)?.with_color(self.color()).with_paint(self.paint().cloned()).with_split_lines(self.split_lines().cloned());
            if !body.solid.face_iter().map(|f| f.id()).eq(output_ids) {
                return Ok(None);
            }
            Ok(Some(with_joined_history(body.with_face_sources(Some(Arc::new(output_sources))), &self, output_members)))
        })?;
        Ok(result.unwrap_or(self))
    }
}

/// An independent candidate, with each output face mapped to the immediate input's indices.
/// Lineage is snapshot-local, not a composition over the input's earlier history or engineering roles.
#[derive(Clone, Debug)]
pub struct CoplanarMergePreview {
    pub body: Body,
    pub changed: bool,
    pub output_face_sources: Vec<Vec<usize>>,
    /// (input face, output face), retained as the original face and boundary objects.
    pub protected_faces: Vec<(usize, usize)>,
    pub protected_edges: usize,
    pub source_volume_mm3: f64,
    pub candidate_volume_mm3: f64,
    pub relative_volume_delta: f64,
}

impl Body {
    /// Merge only adjacent planar polygon faces on the same oriented support, in a new body.
    /// Protected faces and every surviving edge remain the very input topology objects.
    /// Existing authored split lines also veto internal-edge removal, including bypass paths.
    /// An attempted group that cannot be rebuilt is an error, distinct from no eligible group.
    pub fn merge_coplanar_preview(&self, protected_faces: &[usize]) -> Result<CoplanarMergePreview> {
        let invalid = |reason: &str| crate::KernelError::Invalid(format!("coplanar preview: {reason}"));
        if self.is_mesh() || self.shell_count() != 1 || self.face_count() > 1024 || self.face_count() == 0 || protected_faces.len() > 1024 {
            return Err(invalid("requires one B-rep shell with 1..1024 faces"));
        }
        if self.regularization_protected || self.paint().is_some_and(|p| !p.faces.is_empty()) {
            return Err(invalid("authored whole-body boundaries or per-face paint require preservation"));
        }
        let protected: HashSet<_> = protected_faces.iter().copied().collect();
        if protected.len() != protected_faces.len() || protected.iter().any(|i| *i >= self.face_count()) {
            return Err(invalid("protected faces must be distinct current face indices"));
        }
        guard("protected coplanar preview", || {
            let uses = self
                .solid
                .face_iter()
                .try_fold(0usize, |sum, f| f.absolute_boundaries().iter().try_fold(sum, |n, w| n.checked_add(w.len()).filter(|n| *n <= 16_384)))
                .ok_or_else(|| invalid("topology budget exceeded"))?;
            let split_points = self
                .split_keep()
                .iter()
                .try_fold(0usize, |n, line| n.checked_add(line.len()).filter(|n| *n <= 4096))
                .ok_or_else(|| invalid("authored-boundary budget exceeded"))?;
            if uses.saturating_mul(split_points.saturating_add(1)) > 2_000_000 {
                return Err(invalid("authored-boundary work budget exceeded"));
            }
            if self.split_keep().iter().any(|line| line.len() < 2 || line.iter().any(|p| !p.is_finite())) {
                return Err(invalid("authored boundaries must contain finite polylines"));
            }
            if !self.validity().is_empty() || self.crossed_face_loops()? != 0 {
                return Err(invalid("input solid checks failed"));
            }
            let measured = crate::measure(self)?;
            if !measured.volume.is_finite() || measured.volume <= 0.0 {
                return Err(invalid("input volume must be finite and positive"));
            }
            let faces: Vec<_> = self.solid.face_iter().collect();
            let protected_edge_ids: HashSet<_> = protected.iter().filter_map(|i| faces.get(*i)).flat_map(|f| f.edge_iter().map(|e| e.id())).collect();
            let planes: Vec<_> = faces.iter().map(|f| super::planar_sources::actual_plane(f)).collect();
            let mut adjacency = HashMap::<mt::EdgeID, Vec<(usize, mt::Edge)>>::new();
            for (i, face) in faces.iter().enumerate() {
                for edge in face.boundary_iters().into_iter().flatten() {
                    adjacency.entry(edge.id()).or_default().push((i, edge));
                }
            }
            let polygon: Vec<_> = faces.iter().map(|f| f.edge_iter().all(|e| matches!(e.curve(), mt::Curve::Line(_)))).collect();
            let mut parent: Vec<_> = (0..faces.len()).collect();
            let mut eligible = HashSet::new();
            for es in adjacency.values() {
                let [(a, ea), (b, eb)] = es.as_slice() else { continue };
                if a == b
                    || protected.contains(a)
                    || protected.contains(b)
                    || protected_edge_ids.contains(&ea.id())
                    || ea.orientation() == eb.orientation()
                {
                    continue;
                }
                let (Some(Some(pa)), Some(Some(pb))) = (planes.get(*a), planes.get(*b)) else { continue };
                if !polygon.get(*a).copied().unwrap_or(false)
                    || !polygon.get(*b).copied().unwrap_or(false)
                    || !super::planar_sources::same_support(pa, pb)
                    || crate::heal::on_split_line(&crate::heal::edge_points(ea), self.split_keep())
                {
                    continue;
                }
                eligible.insert(ea.id());
                let (ra, rb) = (crate::heal::find(&mut parent, *a), crate::heal::find(&mut parent, *b));
                if let Some(p) = parent.get_mut(ra.max(rb)) {
                    *p = ra.min(rb);
                }
            }
            let mut groups = BTreeMap::<usize, Vec<usize>>::new();
            for i in 0..faces.len() {
                groups.entry(crate::heal::find(&mut parent, i)).or_default().push(i);
            }
            let mut output = Vec::new();
            let mut lineage = Vec::new();
            let mut source_metadata = Vec::new();
            for members in groups.values() {
                let first = *members.first().ok_or_else(|| invalid("empty face group"))?;
                let face = if members.len() == 1 {
                    (**faces.get(first).ok_or_else(|| invalid("missing input face"))?).clone()
                } else {
                    let plane = planes.get(first).copied().flatten().ok_or_else(|| invalid("group has no plane"))?;
                    if members.iter().any(|i| planes.get(*i).and_then(Option::as_ref).is_none_or(|p| !super::planar_sources::same_support(&plane, p)))
                    {
                        return Err(invalid("group exceeds common support accuracy"));
                    }
                    merged_face(&faces, members, plane, &eligible)
                        .ok_or_else(|| invalid("attempted merge cannot retain closed outer, hole and authored boundaries"))?
                };
                output.push(face);
                lineage.push(members.clone());
                // A merged face has explicit operation lineage, never an invented source certificate.
                source_metadata.push(if members.len() == 1 { self.face_sources().and_then(|s| s.get(first)).cloned().flatten() } else { None });
            }
            let changed = output.len() < faces.len();
            let mut candidate = if changed {
                let solid = mt::Solid::try_new(vec![mt::Shell::from(output)])
                    .map_err(|e| crate::KernelError::Failed(format!("coplanar preview does not close: {e}")))?;
                let body = Body::new_placed(solid, false, None, true)?
                    .with_color(self.color())
                    .with_paint(self.paint().cloned())
                    .with_split_lines(self.split_lines().cloned())
                    .with_boundary_tol(self.boundary_tol)
                    .with_face_sources(Some(Arc::new(source_metadata)));
                let prov = lineage.iter().map(|m| (m.iter().map(|&face| FaceOrigin::Face { operand: 0, face }).collect(), m.len() == 1)).collect();
                crate::record::traced(
                    body,
                    &crate::record::counts(&[self]),
                    Some(crate::record::regions(prov)),
                    Some(&crate::record::edge_origins(&self.solid, 0)),
                )
            } else {
                self.clone()
            };
            if changed && candidate.history().is_none() {
                return Err(invalid("complete operation lineage could not be recorded"));
            }
            let candidate_faces: Vec<_> = candidate.solid.face_iter().collect();
            let mut kept = Vec::new();
            let mut protected_edges = HashSet::new();
            for input in protected_faces {
                let matches: Vec<_> = lineage.iter().enumerate().filter(|(_, m)| m.as_slice() == [*input]).map(|(i, _)| i).collect();
                let [out] = matches.as_slice() else { return Err(invalid("protected face lineage changed")) };
                let before = faces.get(*input).ok_or_else(|| invalid("missing protected input"))?;
                let after = candidate_faces.get(*out).ok_or_else(|| invalid("missing protected output"))?;
                if before.id() != after.id()
                    || before.orientation() != after.orientation()
                    || !before.boundary_iters().into_iter().flatten().map(|e| (e.id(), e.orientation())).eq(after
                        .boundary_iters()
                        .into_iter()
                        .flatten()
                        .map(|e| (e.id(), e.orientation())))
                {
                    return Err(invalid("protected face or boundary objects changed"));
                }
                protected_edges.extend(before.edge_iter().map(|e| e.id()));
                kept.push((*input, *out));
            }
            if !candidate.validity().is_empty() || candidate.crossed_face_loops()? != 0 {
                return Err(invalid("candidate solid checks failed"));
            }
            let after = crate::measure(&candidate)?;
            let relative_volume_delta = (after.volume - measured.volume).abs() / measured.volume;
            if !relative_volume_delta.is_finite() || relative_volume_delta > 1e-6 {
                return Err(invalid("candidate volume difference exceeds 1e-6"));
            }
            if changed && !protected_faces.is_empty() {
                candidate.regularization_protected = true;
            }
            Ok(CoplanarMergePreview {
                body: candidate,
                changed,
                output_face_sources: lineage,
                protected_faces: kept,
                protected_edges: protected_edges.len(),
                source_volume_mm3: measured.volume,
                candidate_volume_mm3: after.volume,
                relative_volume_delta,
            })
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Analytic unit-height 3x3 grid, optionally with a square through opening.
    fn tiled_plate(hole: bool) -> Body {
        type Point = (i32, i32, i32);
        let cells: HashSet<_> = (0..3).flat_map(|x| (0..3).map(move |y| (x, y))).filter(|p| !hole || *p != (1, 1)).collect();
        let mut vertices = BTreeMap::<Point, mt::Vertex>::new();
        let mut edges = BTreeMap::<(Point, Point), mt::Edge>::new();
        let mut faces = Vec::new();
        for x in 0..3 {
            for y in 0..3 {
                if !cells.contains(&(x, y)) {
                    continue;
                }
                let quads = [
                    (true, [(x, y, 0), (x, y + 1, 0), (x + 1, y + 1, 0), (x + 1, y, 0)]),
                    (true, [(x, y, 1), (x + 1, y, 1), (x + 1, y + 1, 1), (x, y + 1, 1)]),
                    (!cells.contains(&(x - 1, y)), [(x, y, 0), (x, y, 1), (x, y + 1, 1), (x, y + 1, 0)]),
                    (!cells.contains(&(x + 1, y)), [(x + 1, y, 0), (x + 1, y + 1, 0), (x + 1, y + 1, 1), (x + 1, y, 1)]),
                    (!cells.contains(&(x, y - 1)), [(x, y, 0), (x + 1, y, 0), (x + 1, y, 1), (x, y, 1)]),
                    (!cells.contains(&(x, y + 1)), [(x, y + 1, 0), (x, y + 1, 1), (x + 1, y + 1, 1), (x + 1, y + 1, 0)]),
                ];
                for (_, quad) in quads.into_iter().filter(|(exposed, _)| *exposed) {
                    let wire: mt::Wire = (0..4)
                        .map(|i| {
                            let (a, b) = (quad[i], quad[(i + 1) % 4]);
                            let pair = (a.min(b), a.max(b));
                            let va = vertices
                                .entry(pair.0)
                                .or_insert_with(|| mt::builder::vertex(mt::Point3::new(pair.0.0 as f64, pair.0.1 as f64, pair.0.2 as f64)))
                                .clone();
                            let vb = vertices
                                .entry(pair.1)
                                .or_insert_with(|| mt::builder::vertex(mt::Point3::new(pair.1.0 as f64, pair.1.1 as f64, pair.1.2 as f64)))
                                .clone();
                            let edge = edges.entry(pair).or_insert_with(|| mt::builder::line(&va, &vb));
                            if a == pair.0 { edge.clone() } else { edge.inverse() }
                        })
                        .collect();
                    faces.push(mt::builder::try_attach_plane(&[wire]).unwrap());
                }
            }
        }
        Body::new(mt::Solid::try_new(vec![mt::Shell::from(faces)]).unwrap()).unwrap()
    }

    #[test]
    fn coplanar_preview_reduces_tiled_faces_and_retains_square_opening() {
        let input = tiled_plate(true);
        assert_eq!(input.face_count(), 32);
        let before = crate::encode_native_body(&input).unwrap();
        let preview = input.merge_coplanar_preview(&[]).unwrap();
        assert!(preview.changed);
        assert_eq!(preview.body.face_count(), 10);
        assert!((preview.source_volume_mm3 - 8.).abs() < 1e-10);
        assert!(preview.relative_volume_delta < 1e-12);
        assert!(!preview.body.contains(solidcraft_geom::Vec3::new(1.5, 1.5, 0.5)), "through opening must remain empty");
        assert!(preview.body.contains(solidcraft_geom::Vec3::new(0.5, 0.5, 0.5)));
        let mut source: Vec<_> = preview.output_face_sources.iter().flatten().copied().collect();
        source.sort_unstable();
        assert_eq!(source, (0..32).collect::<Vec<_>>());
        let holes = preview.body.solid.face_iter().filter(|f| f.absolute_boundaries().len() == 2).count();
        assert_eq!(holes, 2, "top and bottom retain the through opening's inner loop");
        let native = crate::encode_native_body(&preview.body).unwrap();
        let reopened = crate::decode_native_body(&native).unwrap();
        assert_eq!(reopened.face_count(), 10);
        assert_eq!(crate::encode_native_body(&reopened).unwrap(), native);
        assert_eq!(crate::encode_native_body(&input).unwrap(), before, "source is unchanged");
    }

    #[test]
    fn coplanar_preview_keeps_protected_tiles_and_all_their_edges() {
        let input = tiled_plate(true);
        let protected: Vec<_> = input
            .solid
            .face_iter()
            .enumerate()
            .filter_map(|(i, f)| super::super::planar_sources::actual_plane(f).filter(|p| p.normal().z > 0.9).map(|_| i))
            .collect();
        assert_eq!(protected.len(), 8);
        let preview = input.merge_coplanar_preview(&protected).unwrap();
        assert_eq!(preview.body.face_count(), 17);
        assert_eq!(preview.protected_faces.len(), 8);
        assert!(preview.protected_edges >= 16);
        for (a, b) in preview.protected_faces {
            let original = input.solid.face_iter().nth(a).unwrap();
            let after = preview.body.solid.face_iter().nth(b).unwrap();
            assert_eq!(original.id(), after.id());
            assert_eq!(
                original.boundary_iters().into_iter().flatten().map(|e| (e.id(), e.orientation())).collect::<Vec<_>>(),
                after.boundary_iters().into_iter().flatten().map(|e| (e.id(), e.orientation())).collect::<Vec<_>>()
            );
        }
    }

    #[test]
    fn coplanar_preview_noop_and_invalid_protection_are_distinct() {
        let input = tiled_plate(false);
        let all: Vec<_> = (0..input.face_count()).collect();
        let preview = input.merge_coplanar_preview(&all).unwrap();
        assert!(!preview.changed);
        assert!(preview.body.same(&input));
        assert_eq!(preview.output_face_sources, all.iter().map(|i| vec![*i]).collect::<Vec<_>>());
        assert!(input.merge_coplanar_preview(&[0, 0]).is_err());
        assert!(input.merge_coplanar_preview(&[input.face_count()]).is_err());
        let plain = crate::box_solid(solidcraft_geom::Vec3::ZERO, solidcraft_geom::Vec3::new(1., 2., 3.)).unwrap();
        assert!(!plain.merge_coplanar_preview(&[]).unwrap().changed);
        let curved = crate::cylinder(solidcraft_geom::Vec3::ZERO, solidcraft_geom::Vec3::Z, 1., 2.).unwrap();
        assert!(!curved.merge_coplanar_preview(&[]).unwrap().changed, "curved faces are not refitted");
    }

    #[test]
    fn coplanar_preview_refuses_mesh_open_and_multiple_shell_inputs() {
        let mesh = crate::mesh_body(&[solidcraft_geom::Vec3::ZERO, solidcraft_geom::Vec3::X, solidcraft_geom::Vec3::Y], &[[0, 1, 2]]).unwrap();
        assert!(mesh.merge_coplanar_preview(&[]).is_err());
        let input = tiled_plate(false);
        let shell = input.solid.boundaries().first().unwrap().clone();
        let other = mt::builder::translated(&shell, mt::Vector3::new(10., 0., 0.));
        let compound = Body::new(mt::Solid::try_new(vec![shell, other]).unwrap()).unwrap();
        assert!(compound.merge_coplanar_preview(&[]).is_err());
        let sheet = mt::Shell::from(vec![input.solid.face_iter().next().unwrap().clone()]);
        let open = Body::new(mt::Solid::new_unchecked(vec![sheet])).unwrap();
        assert!(open.merge_coplanar_preview(&[]).is_err());
    }

    #[test]
    fn coplanar_preview_refuses_bypass_of_an_authored_internal_edge() {
        let mut input = tiled_plate(false)
            .with_split_lines(Some(Arc::new(vec![vec![solidcraft_geom::Vec3::new(1., 0., 1.), solidcraft_geom::Vec3::new(1., 1., 1.)]])));
        assert!(input.merge_coplanar_preview(&[]).is_err(), "the whole-body authored veto is respected");
        // Exercise per-line structural protection without the stronger whole-body veto.
        input.regularization_protected = false;
        let before = crate::encode_native_body(&input).unwrap();
        let error = input.merge_coplanar_preview(&[]).unwrap_err().to_string();
        assert!(error.contains("attempted merge"), "{error}");
        assert_eq!(crate::encode_native_body(&input).unwrap(), before);
    }

    #[test]
    fn alternate_adjacency_path_cannot_remove_protected_internal_edge() {
        let vertices: Vec<_> =
            (0..9).map(|i| mt::builder::vertex(super::super::p3(solidcraft_geom::Vec3::new((i % 3) as f64, (i / 3) as f64, 0.)))).collect();
        let mut edges = HashMap::<(usize, usize), mt::Edge>::new();
        let mut faces = Vec::new();
        for (x, y) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
            let ids = [y * 3 + x, y * 3 + x + 1, (y + 1) * 3 + x + 1, (y + 1) * 3 + x];
            let wire: mt::Wire = (0..4)
                .map(|i| {
                    let (a, b) = (ids[i], ids[(i + 1) % 4]);
                    let key = (a.min(b), a.max(b));
                    let edge = edges.entry(key).or_insert_with(|| mt::builder::line(&vertices[key.0], &vertices[key.1]));
                    if a == key.0 { edge.clone() } else { edge.inverse() }
                })
                .collect();
            faces.push(mt::builder::try_attach_plane(&[wire]).unwrap());
        }
        let refs: Vec<_> = faces.iter().collect();
        let mut eligible: HashSet<_> = edges.values().map(|e| e.id()).collect();
        assert!(merged_face(&refs, &[0, 1, 2, 3], Plane::XY, &eligible).is_some());
        eligible.remove(&edges[&(1, 4)].id());
        assert!(
            merged_face(&refs, &[0, 1, 2, 3], Plane::XY, &eligible).is_none(),
            "remaining three internal joins cannot bypass authored split protection"
        );
    }
}
