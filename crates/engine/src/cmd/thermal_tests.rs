use serde_json::{Value, json};

use crate::Session;

#[path = "../../examples/support/thermal_tiled_plate.rs"]
mod tiled_fixture;

fn merge_source(s: &mut Session) -> (String, String) {
    let body = tiled_fixture::tiled_plate().unwrap();
    assert_eq!(body.face_count(), 32);
    let text = tiled_fixture::step_with_boundaries(&body, "analytic-coplanar-source").unwrap();
    let owner = s.active_component;
    let id = s
        .doc_mut()
        .add_feature(
            solidcraft_doc::FeatureKind::Import { file: "analytic.step".into(), step: text.into(), components: vec![], skip: vec![] },
            Some("Plate"),
        )
        .unwrap();
    let f = s.doc_mut().feature_mut(id).unwrap();
    f.component = owner;
    f.body_names = vec!["Plate".into()];
    s.refresh();
    let world = s.world_state();
    let b = &world.bodies[0];
    (b.name.clone(), format!("{:016x}", b.geometry_key().unwrap()))
}

#[test]
fn merge_preview_retains_interfaces_in_an_independent_native_candidate() {
    let mut s = Session::default();
    let (name, key) = merge_source(&mut s);
    let protected: Vec<_> =
        s.world_state().bodies[0].body.faces(0.01).unwrap().iter().filter(|f| f.plane_normal.is_some_and(|n| n.z > 0.9)).map(|f| f.index).collect();
    assert_eq!(protected.len(), 8);
    let before = s.doc.to_json();
    let result = run(
        &mut s,
        "thermal.merge_coplanar_preview",
        json!({"body":name,"geometry_key":key,"protection":"explicit_faces","protected_faces":protected,"include_native":true}),
    );
    assert_eq!(result["status"], "merged");
    assert_eq!(result["source_faces"], 32);
    assert_eq!(result["candidate_faces"], 17);
    assert_eq!(result["protected_face_mapping"].as_array().unwrap().len(), 8);
    assert_eq!(s.doc.to_json(), before);
    assert_eq!(result["design_modified"], false);
    assert_eq!(result["lineage_scope"], "immediate_input_snapshot_face_indices");
    assert_eq!(result["candidate_encoding"], "solidcraft-native-brep-experimental-same-producer");
    assert_eq!(result["step_interchange_check"], "not_run");
    let hex = result["candidate_native_hex"].as_str().unwrap();
    let bytes: Vec<_> = (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap()).collect();
    let candidate = solidcraft_kernel::decode_native_body(&bytes).unwrap();
    assert_eq!(candidate.face_count(), 17);
    assert!(candidate.validity().is_empty());
    assert!(!candidate.contains(solidcraft_geom::Vec3::new(1.5, 1.5, 0.5)));
    let original = s.world_state().bodies[0].body.faces(0.01).unwrap();
    let after = candidate.faces(0.01).unwrap();
    for mapping in result["protected_face_mapping"].as_array().unwrap() {
        let (a, b) = (mapping[0].as_u64().unwrap() as usize, mapping[1].as_u64().unwrap() as usize);
        assert!(original[a].centroid.dist(after[b].centroid) < 1e-9);
        assert!((original[a].area - after[b].area).abs() < 1e-9);
    }
    run(&mut s, "solid.box", json!({"corner":[10,0,0],"length":1,"width":1,"height":1,"body_name":"UndoWitness"}));
    let world = s.world_state();
    let current = world.bodies.iter().find(|b| b.name == name).unwrap();
    let key = format!("{:016x}", current.geometry_key().unwrap());
    run(&mut s, "thermal.merge_coplanar_preview", json!({"body":name,"geometry_key":key,"protection":"unconfirmed","protected_faces":[]}));
    run(&mut s, "edit.undo", json!({}));
    assert_eq!(s.world_state().bodies.len(), 1);
}

#[test]
fn merge_preview_blocks_unknown_roles_and_rejects_stale_or_bad_selections() {
    let mut s = Session::default();
    let (name, key) = merge_source(&mut s);
    let before = s.doc.to_json();
    for protection in ["unconfirmed", "whole_body"] {
        let result = run(
            &mut s,
            "thermal.merge_coplanar_preview",
            json!({"body":name,"geometry_key":key,"protection":protection,"protected_faces":[],"include_native":true}),
        );
        assert_eq!(result["status"], "blocked");
        assert!(result["candidate_faces"].is_null());
        assert!(result["candidate_native_hex"].is_null());
    }
    for value in [
        json!({"body":name,"geometry_key":"0000000000000000","protection":"explicit_faces","protected_faces":[0]}),
        json!({"body":name,"geometry_key":key,"protection":"explicit_faces","protected_faces":[]}),
        json!({"body":name,"geometry_key":key,"protection":"explicit_faces","protected_faces":[0,0]}),
        json!({"body":name,"geometry_key":key,"protection":"explicit_faces","protected_faces":[9999]}),
        json!({"body":"missing","geometry_key":key,"protection":"explicit_faces","protected_faces":[0]}),
        json!({"body":name,"geometry_key":key,"protection":"explicit_faces","protected_faces":[0],"apply":true}),
    ] {
        assert!(s.execute("thermal.merge_coplanar_preview", &value).is_err(), "{value}");
        assert_eq!(s.doc.to_json(), before);
    }
    let all: Vec<_> = (0..32).collect();
    let kept =
        run(&mut s, "thermal.merge_coplanar_preview", json!({"body":name,"geometry_key":key,"protection":"explicit_faces","protected_faces":all}));
    assert_eq!(kept["status"], "unchanged");
    assert_eq!(kept["candidate_faces"], 32);
    assert!(kept["candidate_native_hex"].is_null());
    let reduced = run(
        &mut s,
        "thermal.merge_coplanar_preview",
        json!({"body":name,"geometry_key":key,"protection":"explicit_faces","protected_faces":[],"allow_empty_protection":true}),
    );
    assert_eq!(reduced["candidate_faces"], 10);
    assert_eq!(s.doc.to_json(), before);
}

#[test]
fn merge_preview_uses_current_world_placement_and_mm_after_occurrence_move() {
    let mut s = Session::default();
    let component = run(&mut s, "component.create", json!({"name":"PlacedPlate"}))["component"].as_u64().unwrap();
    let (name, old_key) = merge_source(&mut s);
    let occurrence = s.doc.occurrences.iter().find(|o| o.component == component).unwrap().id;
    run(&mut s, "occurrence.move", json!({"occurrence":occurrence,"translate":[20,30,0],"axis":[0,0,1],"angle":"90 deg"}));
    assert!(
        s.execute("thermal.merge_coplanar_preview", &json!({"body":name,"geometry_key":old_key,"protection":"explicit_faces","protected_faces":[0]}))
            .is_err()
    );
    run(&mut s, "document.units", json!({"units":"cm"}));
    let world = s.world_state();
    let b = world.bodies.iter().find(|b| b.name == name).unwrap();
    let key = format!("{:016x}", b.geometry_key().unwrap());
    let params = json!({"body":name,"geometry_key":key,"protection":"explicit_faces","protected_faces":[],"allow_empty_protection":true,"include_native":true});
    let before = s.doc.to_json();
    let result = run(&mut s, "thermal.merge_coplanar_preview", params);
    let hex = result["candidate_native_hex"].as_str().unwrap();
    let bytes: Vec<_> = (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap()).collect();
    let candidate = solidcraft_kernel::decode_native_body(&bytes).unwrap();
    let measured = solidcraft_kernel::measure(&candidate).unwrap();
    assert!(measured.bbox.min.dist(solidcraft_geom::Vec3::new(17., 30., 0.)) < 1e-9);
    assert!(measured.bbox.max.dist(solidcraft_geom::Vec3::new(20., 33., 1.)) < 1e-9);
    assert!((measured.volume - 8.).abs() < 1e-9);
    assert_eq!(s.doc.to_json(), before);
}

fn run(s: &mut Session, command: &str, params: Value) -> Value {
    s.execute(command, &params).unwrap_or_else(|e| panic!("{command}: {e}"))
}

fn source_box(s: &mut Session) {
    run(s, "solid.box", json!({"length":10,"width":20,"height":30,"body_name":"Module"}));
}

fn near(actual: &Value, expected: f64) {
    let actual = actual.as_f64().unwrap();
    assert!((actual - expected).abs() < 1e-6, "{actual} != {expected}");
}

#[test]
fn envelope_plan_is_read_only_and_does_not_add_an_undo_step() {
    let mut s = Session::default();
    source_box(&mut s);
    let before = s.doc.to_json();
    let plan = run(&mut s, "thermal.envelope_plan", json!({"bodies":["Module"]}));
    assert_eq!(s.doc.to_json(), before);
    assert_eq!(plan["design_modified"], false);
    assert_eq!(plan["thermal_equivalence"], "not_evaluated");
    assert_eq!(plan["frame"], "world");
    assert_eq!(plan["length_unit"], "mm");
    assert_eq!(plan["candidates"], 1);
    let entry = &plan["entries"][0];
    assert_eq!(entry["source"]["bbox_mm"]["min"], json!([0.0, 0.0, 0.0]));
    assert_eq!(entry["proposal"]["size_mm"], json!([10.0, 20.0, 30.0]));
    near(&entry["source"]["candidate_volume_mm3"], 6000.0);
    near(&entry["proposal"]["source_fill_fraction"], 1.0);
    assert_eq!(entry["source"]["raw_faces"], 6);
    assert_eq!(run(&mut s, "thermal.envelope_plan", json!({"bodies":["Module"]})), plan);
    run(&mut s, "edit.undo", json!({}));
    assert!(s.world_state().bodies.is_empty(), "planning must not consume an undo slot");
}

#[test]
fn explicit_protection_blocks_only_the_selected_placement() {
    let mut s = Session::default();
    source_box(&mut s);
    run(&mut s, "solid.box", json!({"corner":[40,0,0],"length":10,"width":20,"height":30,"body_name":"Cooler"}));
    let before = s.doc.to_json();
    let plan = run(&mut s, "thermal.envelope_plan", json!({"bodies":["Module","Cooler"],"protected_bodies":["Cooler"]}));
    assert_eq!(plan["requested"], 2);
    assert_eq!(plan["candidates"], 1);
    assert_eq!(plan["blocked"], 1);
    assert_eq!(plan["entries"][1]["blocked_reasons"], json!(["protected_body"]));
    assert!(plan["entries"][1]["proposal"].is_null());
    assert_eq!(s.doc.to_json(), before);
    assert!(s.execute("thermal.envelope_plan", &json!({"bodies":["Module"],"protected_bodies":["Typo"]})).is_err());
}

#[test]
fn bad_parameters_fail_without_changing_the_design() {
    let mut s = Session::default();
    source_box(&mut s);
    let before = s.doc.to_json();
    for params in [
        json!({}),
        json!({"bodies":[]}),
        json!({"bodies":["Missing"]}),
        json!({"bodies":["Module","Module"]}),
        json!({"bodies":["Module"],"padding_mm":-1}),
        json!({"bodies":["Module"],"padding_mm":1e100}),
        json!({"bodies":["Module"],"padding_mm":null}),
        json!({"bodies":["Module"],"apply":true}),
        json!({"bodies":["Module"],"protected_bodies":["Module","Module"]}),
        json!({"bodies":(0..129).map(|i|format!("Body{i}")).collect::<Vec<_>>()}),
    ] {
        assert!(s.execute("thermal.envelope_plan", &params).is_err(), "accepted {params}");
        assert_eq!(s.doc.to_json(), before);
    }
}

#[test]
fn occurrences_keep_their_world_bounds_and_shared_definition() {
    let mut s = Session::default();
    let component = run(&mut s, "component.create", json!({"name":"PowerUnit"}))["component"].as_u64().unwrap();
    source_box(&mut s);
    run(&mut s, "occurrence.copy", json!({"component":component,"translate":[40,0,0]}));
    let names: Vec<_> = s.world_state().bodies.iter().map(|b| b.name.clone()).collect();
    assert_eq!(names.len(), 2);
    let before = s.doc.to_json();
    let plan = run(&mut s, "thermal.envelope_plan", json!({"bodies":names}));
    assert_eq!(plan["candidates"], 2);
    let a = &plan["entries"][0];
    let b = &plan["entries"][1];
    assert_eq!(a["source"]["definition_body"], "Module");
    assert_eq!(b["source"]["definition_body"], "Module");
    assert_ne!(a["source"]["world_body"], b["source"]["world_body"]);
    assert_ne!(a["source"]["definition_to_world"], b["source"]["definition_to_world"]);
    assert_ne!(a["source"]["geometry_key"], b["source"]["geometry_key"]);
    let mut xs: Vec<_> = plan["entries"].as_array().unwrap().iter().map(|e| e["proposal"]["corner_mm"][0].as_f64().unwrap()).collect();
    xs.sort_by(f64::total_cmp);
    assert_eq!(xs, vec![0.0, 40.0]);
    assert_eq!(s.doc.to_json(), before);
}

#[test]
fn rotated_occurrence_is_measured_where_it_is_placed() {
    let mut s = Session::default();
    let component = run(&mut s, "component.create", json!({"name":"Placed"}))["component"].as_u64().unwrap();
    source_box(&mut s);
    let occurrence = s.doc.occurrences.iter().find(|o| o.component == component).unwrap().id;
    run(&mut s, "occurrence.move", json!({"occurrence":occurrence,"translate":[100,20,0],"axis":[0,0,1],"angle":"90 deg"}));
    let names: Vec<_> = s.world_state().bodies.iter().map(|b| b.name.clone()).collect();
    let plan = run(&mut s, "thermal.envelope_plan", json!({"bodies":names}));
    let proposal = &plan["entries"][0]["proposal"];
    near(&proposal["corner_mm"][0], 80.0);
    near(&proposal["corner_mm"][1], 20.0);
    near(&proposal["size_mm"][0], 20.0);
    near(&proposal["size_mm"][1], 10.0);
    near(&proposal["volume_mm3"], 6000.0);
}

#[test]
fn display_units_do_not_rescale_mm_bounds_or_padding() {
    let mut s = Session::default();
    source_box(&mut s);
    run(&mut s, "document.units", json!({"units":"cm"}));
    let plan = run(&mut s, "thermal.envelope_plan", json!({"bodies":["Module"],"padding_mm":1}));
    let proposal = &plan["entries"][0]["proposal"];
    assert_eq!(proposal["corner_mm"], json!([-1.0, -1.0, -1.0]));
    assert_eq!(proposal["size_mm"], json!([12.0, 22.0, 32.0]));
    near(&proposal["volume_mm3"], 8448.0);
}

#[test]
fn scaled_source_uses_actual_world_geometry_and_volume() {
    let mut s = Session::default();
    source_box(&mut s);
    run(&mut s, "solid.scale", json!({"bodies":["Module"],"factors":[2,3,4]}));
    let plan = run(&mut s, "thermal.envelope_plan", json!({"bodies":["Module"]}));
    let entry = &plan["entries"][0];
    assert_eq!(entry["proposal"]["size_mm"], json!([20.0, 60.0, 120.0]));
    near(&entry["source"]["candidate_volume_mm3"], 144000.0);
    near(&entry["proposal"]["volume_mm3"], 144000.0);
}

#[test]
fn open_source_has_no_valid_volume_or_box_proposal() {
    let mut s = Session::default();
    source_box(&mut s);
    run(&mut s, "face.delete", json!({"body":"Module","faces":[[5,10,30]],"heal":false}));
    let plan = run(&mut s, "thermal.envelope_plan", json!({"bodies":["Module"]}));
    assert_eq!(plan["candidates"], 0);
    let entry = &plan["entries"][0];
    assert!(entry["source"]["candidate_volume_mm3"].is_null());
    assert!(!entry["source"]["solid_check_issues"].as_array().unwrap().is_empty());
    assert_eq!(entry["blocked_reasons"], json!(["solid_checks_failed"]));
    assert!(entry["proposal"].is_null());
}

#[test]
fn source_design_round_trip_preserves_the_plan_geometry() {
    let mut s = Session::default();
    source_box(&mut s);
    let first = run(&mut s, "thermal.envelope_plan", json!({"bodies":["Module"]}));
    let doc = solidcraft_doc::Document::from_json(&s.doc.to_json()).unwrap();
    let mut reopened = Session::new(doc);
    let second = run(&mut reopened, "thermal.envelope_plan", json!({"bodies":["Module"]}));
    assert_eq!(first["entries"][0]["source"]["bbox_mm"], second["entries"][0]["source"]["bbox_mm"]);
    assert_eq!(first["entries"][0]["proposal"], second["entries"][0]["proposal"]);
}

#[test]
fn planned_box_can_be_built_exported_and_reopened_separately() {
    let mut closed = Session::default();
    source_box(&mut closed);
    let plan = run(&mut closed, "thermal.envelope_plan", json!({"bodies":["Module"],"padding_mm":1}));
    let p = &plan["entries"][0]["proposal"];
    let size = &p["size_mm"];
    let mut preview = Session::default();
    run(&mut preview, "document.units", json!({"units":"cm"}));
    run(
        &mut preview,
        "solid.box",
        json!({
            "corner":[format!("{} mm",p["corner_mm"][0].as_f64().unwrap()),
                format!("{} mm",p["corner_mm"][1].as_f64().unwrap()),
                format!("{} mm",p["corner_mm"][2].as_f64().unwrap())],
            "length":format!("{} mm",size[0].as_f64().unwrap()),
            "width":format!("{} mm",size[1].as_f64().unwrap()),
            "height":format!("{} mm",size[2].as_f64().unwrap()),"body_name":"Envelope"
        }),
    );
    let state = preview.world_state();
    let body = &state.bodies[0].body;
    let step = solidcraft_kernel::step_export(&[body], "thermal-envelope-lab").unwrap();
    let imported = solidcraft_kernel::step_import(&step).unwrap();
    assert_eq!(imported.bodies.len(), 1);
    assert!(imported.warnings.is_empty());
    assert!(imported.bodies[0].closed);
    let measure = solidcraft_kernel::measure(&imported.bodies[0].body).unwrap();
    assert!((measure.volume - p["volume_mm3"].as_f64().unwrap()).abs() < 1e-6);
    assert!((measure.bbox.min.x - p["corner_mm"][0].as_f64().unwrap()).abs() < 1e-6);
    assert!((measure.bbox.min.y - p["corner_mm"][1].as_f64().unwrap()).abs() < 1e-6);
    assert!((measure.bbox.min.z - p["corner_mm"][2].as_f64().unwrap()).abs() < 1e-6);
    assert_eq!(closed.world_state().bodies.len(), 1);
}
