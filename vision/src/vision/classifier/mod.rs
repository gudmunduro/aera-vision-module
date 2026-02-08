use std::collections::HashMap;
use std::fs::File;
use std::io::Write;
use candle_core::{Device, Tensor};
use candle_nn::Sequential;
use itertools::Itertools;
use nalgebra::{DVector, SVector};
use nn::create_nn;
use crate::SiftKeyPoint;
use crate::vision::sift::create_feature_vec;

pub mod nn;
pub mod clip;
pub mod utils;
pub mod color_category;

const CLASSIFIER_SIM_THRESHOLD: f32 = 0.65;

pub struct Classifier {
    classes: HashMap<i32, DVector<f32>>,
    observed_sift_features: Vec<SVector<f64, 128>>,
    model: Sequential,
}

impl Classifier {
    pub fn new() -> Self {
        let model = create_nn();

        Self {
            classes: HashMap::new(),
            observed_sift_features: Vec::new(),
            model,
        }
    }

    // Image must be of length 256 (16*16)
    pub fn classify_and_add(&mut self, image: &Vec<f32>) -> i32 {
        let image = Tensor::from_slice(image.as_slice(), image.len(), &Device::Cpu).unwrap().unsqueeze(0).unwrap();
        let new_sample: Vec<f32> = self.model.forward_all(&image)
            .unwrap()
            .into_iter()
            .last()
            .unwrap()
            .reshape(12)
            .unwrap()
            .try_into()
            .unwrap();
        let new_sample = DVector::from_vec(new_sample);
        let class = match self.classify(&new_sample) {
            Some((class, _sim)) => class,
            None => {
                let class = self.classes.keys().max().map(|c| c+1).unwrap_or(0);
                self.classes.insert(class, new_sample);
                class
            }
        };

        class
    }

    // Comparisons between classes needs a lot of work
    fn classify(&self, sample: &DVector<f32>) -> Option<(i32, f32)> {
        let (class, sim) = self.classes.iter()
            .map(|(class, orig_sample)| {
                let sim = (1.0 - ((orig_sample - sample).norm() / 10.0)).clamp(0.0, 1.0);
                (*class, sim)
            })
            .max_by(|(_, sim_x), (_, sim_y)| sim_x.partial_cmp(sim_y).unwrap())?;

        if sim > CLASSIFIER_SIM_THRESHOLD {
            Some((class, sim))
        } else {
            None
        }
    }

    pub fn get_sift_feature_vector(&mut self, keypoints: &Vec<&SiftKeyPoint>) -> Vec<bool> {
        let (feature_vec, new_features) = create_feature_vec(keypoints, &self.observed_sift_features);
        self.observed_sift_features.extend(new_features);
        feature_vec
    }

    pub fn write_sift_features(&self) -> anyhow::Result<()> {
        let sift_json = serde_json::to_string(&self.observed_sift_features)?;
        let mut output_file = File::create("outputs/sift_map.json")?;
        output_file.write_all(sift_json.as_bytes())?;
        drop(output_file);

        Ok(())
    }

    pub fn read_sift_features(&mut self) -> anyhow::Result<()> {
        let sift_json = std::fs::read_to_string("outputs/sift_map.json")?;
        self.observed_sift_features = serde_json::from_str(&sift_json)?;

        Ok(())
    }
}