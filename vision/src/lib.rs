mod preprocess;
mod vision;
mod utils;

use std::fs::File;
use std::io::Write;
use std::path::Path;
use std::sync::Mutex;
use anyhow::Ok;
use itertools::Itertools;
use nalgebra::{DMatrix, Vector2};
use opencv::{core::{Mat, Point, Rect, Scalar}, highgui::{self, imshow, wait_key}, imgcodecs::{self, IMREAD_COLOR}, imgproc::{arrowed_line, rectangle_def}};
use opencv::core::{AlgorithmHint, MatTraitConst};
use opencv::imgcodecs::{imread, imwrite, imwrite_def};
use opencv::imgproc::{circle_def, cvt_color, cvt_color_def, rectangle, COLOR_BGR2RGB, COLOR_RGB2BGR, COLOR_RGB2GRAY, COLOR_RGB2HSV, LINE_8};
use preprocess::preprocess_image;
use vision::{classifier::{utils::get_image_data_for_classification, Classifier}, proposals::{fastsam, motion::{self, motion_matrix::calc_motion_matrix}, proposal_area::ProposalArea}};

pub use vision::proposals::proposal_area::RecognizedArea;
use crate::vision::classifier::color_category;
use crate::vision::sift::{create_feature_vec, get_sift_points};
pub use crate::vision::sift::SiftKeyPoint;

pub struct VisionSystem {
    classifier: Classifier,
}

impl VisionSystem {
    pub fn new() -> Self {
        let mut classifier = Classifier::new();
        // TODO: Re-add if we are going to continue the scenario experiment
        //classifier.read_sift_features().unwrap();
        Self {
            classifier
        }
    }

    pub fn process_frame(&mut self, img_rgb: &Mat) -> anyhow::Result<Vec<RecognizedArea>> {
        let mut img = Mat::default();
        cvt_color(&img_rgb, &mut img, COLOR_RGB2BGR, 0, AlgorithmHint::ALGO_HINT_DEFAULT)?;

        let h = img.mat_size()[0] as f32;
        let w = img.mat_size()[1] as f32;

        let proposals = fastsam::make_proposals(&img)?;
        let sift_keypoints = self.process_with_sift(img_rgb)?;
        let (img_gray, _img_small) = preprocess_image(&img)?;

        let mut results = Vec::new();
        for prop in proposals {
            let preprocessed_image: Vec<f32> = get_image_data_for_classification(&img_gray, &prop.scaled(128.0 / w, 80.0 / h))
                .iter()
                .map(|v| *v as f32)
                .collect();
            let class = self.classifier.classify_and_add(&preprocessed_image);
            let color = color_category::get_color_of_proposal(&img, &prop)?;

            // Skip areas with no color to limit results for testing
            if color == 0 {
                continue
            }

            let region_keypoints = sift_keypoints
                .iter()
                .filter(|kp| prop.contains_point(kp.point.x as i32, kp.point.y as i32))
                .collect_vec();
            log::debug!("Found {} region keypoints", region_keypoints.len());
            let features = self.classifier.get_sift_feature_vector(&region_keypoints);
            
            results.push(RecognizedArea::new(class as i64, color as i64, prop, features, region_keypoints.iter().map(|kp| Vector2::new(kp.point.x as i64, kp.point.y as i64)).collect()));
        }

        visualize_proposals(&img, &results)?;

        /*static FRAME: Mutex<i32> = Mutex::new(0);
        *FRAME.lock().unwrap() += 1;
        if *FRAME.lock().unwrap() == 10 {
            self.classifier.write_sift_features().unwrap();
        }*/


        Ok(results)
    }

    pub fn process_with_sift(&mut self, img_rgb: &Mat) -> anyhow::Result<Vec<SiftKeyPoint>> {
        let mut img = Mat::default();
        cvt_color(&img_rgb, &mut img, COLOR_RGB2BGR, 0, AlgorithmHint::ALGO_HINT_DEFAULT)?;

        static FRAME: Mutex<i32> = Mutex::new(0);
        *FRAME.lock().unwrap() += 1;

        let keypoints = get_sift_points(&img)?;
        let content = serde_json::to_string(&keypoints)?;
        let mut file = File::create(format!("outputs/{}_sift.json", *FRAME.lock().unwrap()))?;
        write!(file, "{content}")?;

        //visualize_sift_points(&img, &keypoints)?;

        Ok(keypoints)
    }

    pub fn read_frame(&self, path: &str) -> anyhow::Result<Mat> {
        let img_rgb = imread(path, IMREAD_COLOR)?;
        let mut img = Mat::default();
        cvt_color_def(&img_rgb, &mut img, COLOR_RGB2BGR)?;
        Ok(img)
    }
}

fn visualize_sift_points(img_rgb: &Mat, sift_points: &Vec<SiftKeyPoint>) -> anyhow::Result<()> {
    highgui::named_window("Display window", highgui::WINDOW_NORMAL)?;
    highgui::resize_window("Display window", 1280, 720)?;
    let mut dbg_canvas = img_rgb.clone();

    for point in sift_points {
        circle_def(&mut dbg_canvas, opencv::core::Point::new(point.point.x as i32, point.point.y as i32), 2, Scalar::new(0.0, 0.0, 255.0, 255.0))?;
    }

    highgui::imshow("Display window", &dbg_canvas)?;
    wait_key(0)?;

    static FRAME: Mutex<i32> = Mutex::new(0);
    *FRAME.lock().unwrap() += 1;
    //imwrite_def(&format!("outputs/{}_sift.jpg", *FRAME.lock().unwrap()), &dbg_canvas)?;

    Ok(())
}

fn visualize_proposals(img: &Mat, recognized_areas: &Vec<RecognizedArea>) -> anyhow::Result<()> {
    highgui::named_window("Display window", highgui::WINDOW_NORMAL)?;
    highgui::resize_window("Display window", 1280, 720)?;
    let mut dbg_canvas = img.clone();

    for area in recognized_areas {
        let drawing_col = if area.color == 1 {
            Scalar::new(6.0, 128.0, 212.0, 255.0)
        }
        else if area.color == 2 {
            Scalar::new(0.0, 0.0, 255.0, 255.0)
        }
        else if area.color == 3 {
            Scalar::new(234.0, 179.0, 8.0, 255.0)
        }
        else if area.color == 4 {
            Scalar::new(34.0, 197.0, 9.0, 255.0)
        }
        else {
            Scalar::new(255.0, 0.0, 0.0, 255.0)
        };

        let prop = &area.area;
        let rect = Rect::new(prop.min.x, prop.min.y, prop.max.x-prop.min.x, prop.max.y-prop.min.y);
        rectangle(&mut dbg_canvas, rect, drawing_col, 1, LINE_8, 0)?;

        for kp in &area.keypoints {
            circle_def(&mut dbg_canvas, opencv::core::Point::new(kp.x as i32, kp.y as i32), 1, Scalar::new(0.0, 0.0, 255.0, 255.0))?;
        }
    }

    static FRAME: Mutex<i32> = Mutex::new(0);
    *FRAME.lock().unwrap() += 1;
    imwrite_def(&format!("outputs/{}_frame_visual.jpg", *FRAME.lock().unwrap()), &dbg_canvas)?;
    imwrite_def(&format!("outputs/{}_frame.jpg", *FRAME.lock().unwrap()), &img)?;
    // highgui::imshow("Display window", &dbg_canvas)?;
    // wait_key(0)?;


    Ok(())
}