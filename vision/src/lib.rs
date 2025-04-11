mod preprocess;
mod vision;
mod utils;

use anyhow::Ok;
use nalgebra::{DMatrix, Vector2};
use opencv::{core::{Mat, Point, Rect, Scalar}, highgui::{self, imshow, wait_key}, imgcodecs::{self, IMREAD_COLOR}, imgproc::{arrowed_line, rectangle_def}};
use opencv::core::{AlgorithmHint, MatTraitConst};
use opencv::imgproc::{cvt_color, rectangle, COLOR_RGB2BGR, COLOR_RGB2HSV, LINE_8};
use preprocess::preprocess_image;
use vision::{classifier::{utils::get_image_data_for_classification, Classifier}, proposals::{fastsam, motion::{self, motion_matrix::calc_motion_matrix}, proposal_area::ProposalArea}};

pub use vision::proposals::proposal_area::RecognizedArea;
use crate::vision::classifier::color_category;
use crate::vision::sift::{get_sift_points};
pub use crate::vision::sift::SiftKeyPoint;

pub struct VisionSystem {
    classifier: Classifier,
}

impl VisionSystem {
    pub fn new() -> Self {
        Self {
            classifier: Classifier::new()
        }
    }

    pub fn process_frame(&mut self, img_rgb: &Mat) -> anyhow::Result<Vec<RecognizedArea>> {
        let mut img = Mat::default();
        cvt_color(&img_rgb, &mut img, COLOR_RGB2BGR, 0, AlgorithmHint::ALGO_HINT_DEFAULT)?;

        let h = img.mat_size()[0] as f32;
        let w = img.mat_size()[1] as f32;

        let proposals = fastsam::make_proposals(&img)?;
        let (img_gray, _img_small) = preprocess_image(&img)?;

        let mut results = Vec::new();
        for prop in proposals {
            let preprocessed_image: Vec<f32> = get_image_data_for_classification(&img_gray, &prop.scaled(128.0 / w, 80.0 / h))
                .iter()
                .map(|v| *v as f32)
                .collect();
            let class = self.classifier.classify_and_add(&preprocessed_image);
            let color = color_category::get_color_of_proposal(&img, &prop)?;
            
            results.push(RecognizedArea::new(class as i64, color as i64, prop));
        }

        //visualize_proposals(&img, &results)?;

        Ok(results)
    }

    pub fn process_with_sift(&mut self, img_rgb: &Mat) -> anyhow::Result<Vec<SiftKeyPoint>> {
        let mut img = Mat::default();
        cvt_color(&img_rgb, &mut img, COLOR_RGB2BGR, 0, AlgorithmHint::ALGO_HINT_DEFAULT)?;

        let keypoints = get_sift_points(&img)?;

        Ok(keypoints)
    }
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
    }

    highgui::imshow("Display window", &dbg_canvas)?;
    wait_key(0)?;


    Ok(())
}