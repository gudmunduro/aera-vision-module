#![allow(warnings, unused)]

use anyhow::Ok;
use candle_onnx::onnx::ModelProto;
use nalgebra::DMatrix;

fn load_clip_model() -> anyhow::Result<ModelProto> {
    let model = candle_onnx::read_file("vision.onnx")?;

    Ok(model)
}

pub fn encode_image(model: &ModelProto, image: &DMatrix<f64>) -> Vec<f64> {

    Vec::new()
}