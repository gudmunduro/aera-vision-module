use std::collections::HashMap;

use candle_core::{Device, Tensor};
use candle_nn::Sequential;
use nalgebra::DVector;
use nn::create_nn;

pub mod nn;
pub mod clip;
pub mod utils;

const CLASSIFIER_SIM_THRESHOLD: f32 = 0.65;

pub struct Classifier {
    classes: HashMap<i32, DVector<f32>>,
    model: Sequential,
}

impl Classifier {
    pub fn new() -> Self {
        let model = create_nn();

        Self {
            classes: HashMap::new(),
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

    // Comparisions between classes needs a lot of work
    fn classify(&self, sample: &DVector<f32>) -> Option<(i32, f32)> {
        let (class, sim) = self.classes.iter()
            .map(|(class, orig_sample)| {
                let sim = (1.0 - ((orig_sample - sample).norm() / 10.0)).clamp(0.0, 1.0);
                (*class, sim)
            })
            .max_by(|(_, sim_x), (_, sim_y)| sim_x.partial_cmp(sim_y).unwrap())?;

        log::debug!("Similairity score during classification: {sim}");
        if sim > CLASSIFIER_SIM_THRESHOLD {
            Some((class, sim))
        } else {
            None
        }
    }
}