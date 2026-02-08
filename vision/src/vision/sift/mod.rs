use itertools::Itertools;
use nalgebra::{ArrayStorage, SVector, VecStorage, Vector2};
use opencv::core::{Point, Vector};
use opencv::features2d::SIFT;
use opencv::imgproc::{cvt_color_def, COLOR_BGR2GRAY};
use opencv::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SiftKeyPoint {
    pub point: Vector2<f64>,
    pub feature_vec: SVector<f64, 128>,
}

pub fn get_sift_points(img: &Mat) -> anyhow::Result<Vec<SiftKeyPoint>> {
    let mut img_gray = Mat::default();
    cvt_color_def(img, &mut img_gray, COLOR_BGR2GRAY)?;

    let mut sift = SIFT::create_def()?;
    sift.set_contrast_threshold(0.07)?;
    let mut keypoints = Vector::new();
    let mut descriptors = Mat::default();
    sift.detect_and_compute_def(&img_gray, &Vector::<u8>::new(), &mut keypoints, &mut descriptors)?;

    println!("Descriptors shape ({}, {})", descriptors.size().unwrap().width, descriptors.size().unwrap().height);

    let keypoints = keypoints
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let features = descriptors
                .row(i as i32)
                .unwrap()
                .iter()
                .unwrap()
                .map(|(_, v): (_, f32)| v as f64)
                .collect_vec();
            SiftKeyPoint {
                point: Vector2::new(p.pt().x as f64, p.pt().y as f64),
                feature_vec: SVector::from_row_iterator(features.into_iter()),
            }
        })
        .collect_vec();

    Ok(keypoints)
}

pub fn create_feature_vec(keypoints: &Vec<&SiftKeyPoint>, object_feature_vecs: &Vec<SVector<f64, 128>>) -> (Vec<bool>, Vec<SVector<f64, 128>>) {
    let mut matching_keypoints = (0..keypoints.len()).map(|_| false).collect_vec();
    let mut matching_features = Vec::with_capacity(80);
    for stored_feature in object_feature_vecs {
        let matching_index = keypoints.iter().enumerate().filter(|(_, kp)| (kp.feature_vec - stored_feature).norm() < 300.0).map(|(i, _)| i).next();
        if let Some(i) = matching_index {
            matching_keypoints[i] = true;
            matching_features.push(true);
        }
        else {
            matching_features.push(false);
        }
    }
    let remaining_space = 80-matching_features.len();
    let new_feature_vecs = matching_keypoints
        .iter()
        .enumerate()
        .filter(|(_, m)| !**m)
        .map(|(i, _)| keypoints[i].feature_vec.clone_owned())
        .take(remaining_space)
        .collect_vec();

    matching_features.extend((0..(new_feature_vecs.len())).map(|_| true));
    matching_features.extend((0..(80-matching_features.len())).map(|_| false));
    (matching_features, new_feature_vecs)
}