//! Analytic tiled plate with a square through opening, built without commercial CAD data.
use solidcraft_geom::Vec3;
use solidcraft_kernel::{Body, EdgeSpec, FaceSpec, Result, SurfSpec, sew};

pub fn tiled_plate() -> Result<Body> {
    let occupied = |x: i32, y: i32| (0..3).contains(&x) && (0..3).contains(&y) && (x, y) != (1, 1);
    let mut faces = Vec::new();
    for x in 0..3 {
        for y in 0..3 {
            if !occupied(x, y) {
                continue;
            }
            let quads = [
                (true, [(x, y, 0), (x, y + 1, 0), (x + 1, y + 1, 0), (x + 1, y, 0)]),
                (true, [(x, y, 1), (x + 1, y, 1), (x + 1, y + 1, 1), (x, y + 1, 1)]),
                (!occupied(x - 1, y), [(x, y, 0), (x, y, 1), (x, y + 1, 1), (x, y + 1, 0)]),
                (!occupied(x + 1, y), [(x + 1, y, 0), (x + 1, y + 1, 0), (x + 1, y + 1, 1), (x + 1, y, 1)]),
                (!occupied(x, y - 1), [(x, y, 0), (x + 1, y, 0), (x + 1, y, 1), (x, y, 1)]),
                (!occupied(x, y + 1), [(x, y + 1, 0), (x, y + 1, 1), (x + 1, y + 1, 1), (x + 1, y + 1, 0)]),
            ];
            for (_, quad) in quads.into_iter().filter(|(exposed, _)| *exposed) {
                let points: Vec<_> = quad.into_iter().map(|(x, y, z)| Vec3::new(x as f64, y as f64, z as f64)).collect();
                let mut edges = Vec::new();
                for i in 0..4 {
                    let a = points.get(i).copied().ok_or_else(|| solidcraft_kernel::KernelError::Invalid("analytic quad".into()))?;
                    let b = points.get((i + 1) % 4).copied().ok_or_else(|| solidcraft_kernel::KernelError::Invalid("analytic quad".into()))?;
                    edges.push(EdgeSpec::Line { a, b });
                }
                faces.push(FaceSpec { loops: vec![edges], surface: SurfSpec::Plane, probe: None });
            }
        }
    }
    sew(&faces)
}

/// Preserve the analytic fixture's remaining subdivisions while the STEP writer runs.
/// The receiving runner must still independently reopen and check protected geometry.
pub fn step_with_boundaries(body: &Body, system: &str) -> Result<String> {
    let mut lines = body.split_lines().map(|s| s.as_ref().clone()).unwrap_or_default();
    lines.extend(body.edges(0.001)?.into_iter().map(|edge| edge.points));
    let export = body.clone().with_split_lines(Some(std::sync::Arc::new(lines)));
    solidcraft_kernel::step_export(&[&export], system)
}
