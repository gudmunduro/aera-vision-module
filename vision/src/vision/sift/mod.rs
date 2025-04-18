use itertools::Itertools;
use nalgebra::Vector2;
use opencv::core::{Point, Vector};
use opencv::features2d::SIFT;
use opencv::imgproc::{cvt_color_def, COLOR_BGR2GRAY};
use opencv::prelude::*;

pub struct SiftKeyPoint {
    pub point: Vector2<f64>,
    pub feature_vec: Vec<f64>,
}

pub fn get_sift_points(img: &Mat) -> anyhow::Result<Vec<SiftKeyPoint>> {
    let mut img_gray = Mat::default();
    cvt_color_def(img, &mut img_gray, COLOR_BGR2GRAY)?;

    let mut sift = SIFT::create_def()?;
    sift.set_contrast_threshold(0.10)?;
    let mut keypoints = Vector::new();
    let mut descriptors = Mat::default();
    sift.detect_and_compute_def(&img_gray, &Vector::<u8>::new(), &mut keypoints, &mut descriptors)?;

    let keypoints = keypoints
        .iter()
        .enumerate()
        .map(|(i, p)| SiftKeyPoint {
            point: Vector2::new(p.pt().x as f64, p.pt().y as f64),
            feature_vec: descriptors
                .row(i as i32)
                .unwrap()
                .iter()
                .unwrap()
                .map(|(_, v): (_, f32)| v as f64)
                .collect()
        })
        .collect_vec();

    Ok(keypoints)
}