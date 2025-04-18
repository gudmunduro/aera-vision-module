use opencv::prelude::*;
use opencv::imgcodecs;
use opencv::imgcodecs::IMREAD_COLOR;
use opencv::imgproc::{cvt_color_def, COLOR_BGR2RGB};
use sift_processing::cluster_points;

fn main() {
    let img = imgcodecs::imread("./sift_test_2.png", IMREAD_COLOR).unwrap();
    let mut img_rgb = Mat::default();
    cvt_color_def(&img, &mut img_rgb, COLOR_BGR2RGB).unwrap();

    let mut vs = vision::VisionSystem::new();
    let kp = vs.process_with_sift(&img_rgb).unwrap();
    let clusters = cluster_points(&kp);
    println!("Number of clusters {}", clusters.len());
}