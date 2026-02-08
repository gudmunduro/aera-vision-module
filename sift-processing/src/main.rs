use std::fs;
use opencv::prelude::*;
use opencv::imgcodecs;
use opencv::imgcodecs::{imread, IMREAD_COLOR};
use opencv::imgproc::{cvt_color_def, COLOR_BGR2RGB};
use sift_processing::{cluster_points, SiftProcessing};
use vision::{SiftKeyPoint, VisionSystem};

fn main() {
    let mut vis = VisionSystem::new();
    let mut sift = SiftProcessing::new();

    for i in 1..3 {
        println!("Processing frame {i}");
        let frame_bgr = imread(&format!("outputs/{i}_frame.jpg"), IMREAD_COLOR).unwrap();
        let mut frame = Mat::default();
        cvt_color_def(&frame_bgr, &mut frame, COLOR_BGR2RGB).unwrap();
        let kp = vis.process_with_sift(&frame).unwrap();
        //let Some(kp) = load_sift_frame(i) else {
        //    break;
        //};
        let clusters = sift.get_feature_cluster(&kp);
        println!("Got {} clusters", clusters.len());
    }
}

fn load_sift_frame(frame: i32) -> Option<Vec<SiftKeyPoint>> {
    println!("Start of frame {frame}");
    let content = fs::read_to_string(&format!("outputs/{frame}_sift.json")).ok()?;
    serde_json::from_str(&content).unwrap()
}