//! Microbenchmark for Computational Geometry and Spatial Indexing.

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use rand::rngs::StdRng;
use rand::Rng;
use rand::SeedableRng;

use rustv2xsimulator::geometry::kd_tree::{NodeEntry, SpatialIndex};
use rustv2xsimulator::geometry::polygon::BuildingPolygon;
use rustv2xsimulator::geometry::segment::{Point2D, Segment2D};
use rustv2xsimulator::phy::knife_edge::FresnelKnifeEdge;

fn bench_kd_tree_vs_all_pairs(c: &mut Criterion) {
    let mut rng = StdRng::seed_from_u64(42);
    let n = 2000;
    let mut entries = Vec::with_capacity(n);
    for i in 0..n {
        let x = rng.gen_range(0.0..3000.0);
        let y = rng.gen_range(0.0..3000.0);
        entries.push(NodeEntry::new((i + 1) as u32, Point2D::new(x, y)));
    }

    let mut kd_tree = SpatialIndex::new();
    kd_tree.rebuild(entries.clone());

    let mut all_pairs = SpatialIndex::with_all_pairs(true);
    all_pairs.rebuild(entries);

    let query_pt = Point2D::new(1500.0, 1500.0);

    c.bench_function("spatial_query_kdtree_r800m", |b| {
        b.iter(|| {
            let res = kd_tree.query_radius(black_box(query_pt), 800.0, 1);
            black_box(res);
        });
    });

    c.bench_function("spatial_query_allpairs_r800m", |b| {
        b.iter(|| {
            let res = all_pairs.query_radius(black_box(query_pt), 800.0, 1);
            black_box(res);
        });
    });
}

fn bench_building_raycast_and_diffraction(c: &mut Criterion) {
    let vertices = vec![
        Point2D::new(100.0, 100.0),
        Point2D::new(200.0, 100.0),
        Point2D::new(200.0, 200.0),
        Point2D::new(100.0, 200.0),
    ];
    let building = BuildingPolygon::new("b1".to_string(), vertices);
    let ray = Segment2D::new(Point2D::new(50.0, 150.0), Point2D::new(250.0, 150.0));
    let diff = FresnelKnifeEdge::default();

    c.bench_function("building_multi_segment_intersection", |b| {
        b.iter(|| {
            let hits = building.intersection_points(black_box(&ray));
            let pen = building.penetration_distance(black_box(&ray));
            black_box((hits, pen));
        });
    });

    c.bench_function("knife_edge_diffraction_loss", |b| {
        b.iter(|| {
            let loss = diff.diffraction_loss_db(black_box(1.5));
            black_box(loss);
        });
    });
}

criterion_group!(
    benches,
    bench_kd_tree_vs_all_pairs,
    bench_building_raycast_and_diffraction
);
criterion_main!(benches);
