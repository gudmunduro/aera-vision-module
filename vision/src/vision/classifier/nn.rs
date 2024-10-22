use candle_core::{DType, Device};
use candle_nn::{VarBuilder, VarMap};

const INPUT_DIM: usize = 16 * 16;
const HIDDEN_DIM: usize = 12;

pub fn create_nn() -> candle_nn::Sequential {
    let mut varmap = VarMap::new();
    varmap.load("vision.safetensors").expect("Failed to load nn model");
    let vs = VarBuilder::from_varmap(&varmap, DType::F32, &Device::Cpu);
    let net: candle_nn::Sequential = candle_nn::seq()
        .add(candle_nn::linear(INPUT_DIM, HIDDEN_DIM, vs.pp("0")).unwrap())
        .add(candle_nn::activation::Activation::Relu)
        .add(candle_nn::linear(HIDDEN_DIM, HIDDEN_DIM, vs.pp("2")).unwrap());

    net
}