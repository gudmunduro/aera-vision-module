mod preprocess;
mod vision;
mod utils;

use anyhow::Ok;
use nalgebra::{DMatrix, Vector2};
use opencv::{core::{Mat, Point, Rect, Scalar}, highgui::{self, imshow, wait_key}, imgcodecs::{self, IMREAD_COLOR}, imgproc::{arrowed_line, rectangle_def}};
use preprocess::preprocess_image;
use vision::{classifier::{utils::get_image_data_for_classification, Classifier}, proposals::{color_gradient, motion::{self, motion_matrix::calc_motion_matrix}, proposal_area::ProposalArea}};

pub use vision::proposals::proposal_area::RecognizedArea;

pub struct VisionSystem {
    classifier: Classifier,
}

impl VisionSystem {
    pub fn new() -> Self {
        Self {
            classifier: Classifier::new()
        }
    }

    pub fn process_frame(&mut self, img: &Mat) -> anyhow::Result<Vec<RecognizedArea>> {
        let proposals = color_gradient::make_proposals(&img)?;
        let (img_gray, _img_small) = preprocess_image(&img)?;

        let mut results = Vec::new();
        for prop in proposals {
            let preprocessed_image: Vec<f32> = get_image_data_for_classification(&img_gray, &prop.scaled(128.0 / 1280.0, 80.0 / 720.0))
                .iter()
                .map(|v| *v as f32)
                .collect();
            let class = self.classifier.classify_and_add(&preprocessed_image);
            
            results.push(RecognizedArea::new(class as i64, prop));
        }

        Ok(results)
    }
}
