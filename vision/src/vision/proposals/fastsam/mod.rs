use crate::vision::proposals::proposal_area::ProposalArea;
use candle_transformers::object_detection::{iou, non_maximum_suppression, soft_non_maximum_suppression, Bbox};
use itertools::Itertools;
use nalgebra::Vector2;
use ndarray::{s, Array, Axis};
use opencv::core::{Mat, Range, Size};
use opencv::imgproc::{cvt_color_def, resize, COLOR_BGR2RGB, INTER_LINEAR};
use opencv::prelude::{MatExprTraitConst, MatTrait, MatTraitConst, MatTraitConstManual};
use ort::session::builder::GraphOptimizationLevel;
use ort::session::Session;
use ort::value::Tensor;
use std::hash::Hash;

const CONF_THRESHOLD: f32 = 0.25;
const NMS_THRESHOLD: f32 = 0.20;
const IOU: f32 = 0.2;

const TARGET_SIZE: f32 = 1024.0;

fn preprocess_img(img: &Mat) -> (Tensor<f32>, i32, i32, i32, i32) {
    // Convert from bgr to rgb
    let mut img_rgb = Mat::default();
    cvt_color_def(img, &mut img_rgb, COLOR_BGR2RGB).unwrap();

    // Resize the image
    let mat_size = img_rgb.mat_size().to_vec();
    let h = mat_size[0] as f32;
    let w = mat_size[1] as f32;

    let (img_resized, nh, nw, ah, aw) = if h > w {
        let mut img_resized = Mat::zeros(TARGET_SIZE as i32, TARGET_SIZE as i32, img_rgb.typ())
            .unwrap()
            .to_mat()
            .unwrap();
        let scale = (TARGET_SIZE / h).min(TARGET_SIZE / w);
        let nw = (w * scale) as i32;
        let nh = (h * scale) as i32;
        let a = (nh - nw) / 2;

        let mut img_resized_sub = img_resized
            .rowscols_mut(Range::new(0, nh).unwrap(), Range::new(a, a + nw).unwrap())
            .unwrap();
        resize(
            &img_rgb,
            &mut img_resized_sub,
            Size::new(nw, nh),
            0.0,
            0.0,
            INTER_LINEAR,
        )
            .unwrap();
        (img_resized, nh, nw, 0, a)
    } else {
        let mut img_resized = Mat::zeros(TARGET_SIZE as i32, TARGET_SIZE as i32, img_rgb.typ())
            .unwrap()
            .to_mat()
            .unwrap();
        let scale = (TARGET_SIZE / h).min(TARGET_SIZE / w);
        let nw = (w * scale) as i32;
        let nh = (h * scale) as i32;
        let a = (nw - nh) / 2;

        let mut img_resized_sub = img_resized
            .rowscols_mut(Range::new(a, a + nh).unwrap(), Range::new(0, nw).unwrap())
            .unwrap();
        resize(
            &img_rgb,
            &mut img_resized_sub,
            Size::new(nw, nh),
            0.0,
            0.0,
            INTER_LINEAR,
        )
            .unwrap();
        (img_resized, nh, nw, a, 0)
    };

    // Create the matrix of correct size
    let img = crate::utils::colored_mat_to_dmatrix(&img_resized)
        .unwrap()
        .transpose();
    let img_flat = img
        .as_slice()
        .iter()
        .flat_map(|v| v.as_slice().iter())
        .cloned()
        .collect_vec();
    let mut img_array = Array::from_shape_vec((img.shape().0, img.shape().1, 3), img_flat).unwrap();
    img_array = img_array.permuted_axes([2, 0, 1]);
    let img_array = img_array.insert_axis(Axis(0));
    let img_array = img_array.mapv(|x| x as f32);
    let img_tensor = Tensor::from_array(img_array).unwrap();

    (img_tensor, nh, nw, ah, aw)
}

pub fn make_proposals(img: &Mat) -> anyhow::Result<Vec<ProposalArea>> {
    let input_h = img.mat_size()[0] as f32;
    let input_w = img.mat_size()[1] as f32;
    let (img_array, output_h, output_w, output_h_blank_space, output_w_blank_space) =
        preprocess_img(img);

    let model = Session::builder()?
        .with_optimization_level(GraphOptimizationLevel::Level3)?
        .with_intra_threads(11)?
        .commit_from_file("FastSAM-x.onnx")?;

    let output = model.run(ort::inputs![&model.inputs[0].name => img_array]?)?;
    let preds = output[0].try_extract_tensor::<f32>()?;

    let mut max_conf: f32 = 0.0;

    let nclasses = preds.shape()[1] - 5;
    let mut bboxes: Vec<Vec<Bbox<()>>> = (0..nclasses).map(|_| vec![]).collect();
    for index in 0..preds.shape()[2] {
        let pred = preds.slice(s![.., .., index]);
        let confidence = pred[[0, 4]];

        max_conf = max_conf.max(confidence);
        if confidence > CONF_THRESHOLD {
            let mut class_index = 0;
            for i in 0..nclasses {
                if pred[[0, 5 + i]] > pred[[0, 5 + class_index]] {
                    class_index = i
                }
            }
            if pred[[0, class_index + 5]] > 0. {
                let bbox = Bbox {
                    xmin: pred[[0, 0]] - pred[[0, 2]] / 2.,
                    ymin: pred[[0, 1]] - pred[[0, 3]] / 2.,
                    xmax: pred[[0, 0]] + pred[[0, 2]] / 2.,
                    ymax: pred[[0, 1]] + pred[[0, 3]] / 2.,
                    confidence,
                    data: (),
                };

                bboxes[class_index].push(bbox);
            }
        }
    }
    non_maximum_suppression(&mut bboxes, NMS_THRESHOLD);
    soft_non_maximum_suppression(&mut bboxes, Some(IOU), Some(CONF_THRESHOLD), None);

    Ok(bboxes
        .into_iter()
        .flatten()
        .map(|b| ProposalArea {
            // Scale so bounding boxes fit input image
            min: Vector2::new(
                (((b.xmin - output_w_blank_space as f32) / output_w as f32) * input_w) as i32,
                (((b.ymin - output_h_blank_space as f32) / output_h as f32) * input_h) as i32,
            ),
            max: Vector2::new(
                (((b.xmax - output_w_blank_space as f32) / output_w as f32) * input_w) as i32,
                (((b.ymax - output_h_blank_space as f32) / output_h as f32) * input_h) as i32,
            ),
        })
        .collect())
}
