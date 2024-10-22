use anyhow::{bail, Ok};
use nalgebra::{DMatrix, Vector3};
use opencv::{core::{Mat, MatTraitConst, Size, CV_8UC1, CV_8UC3}, highgui::{imshow, wait_key}, imgproc::{cvt_color_def, resize, COLOR_BGR2GRAY, INTER_LINEAR}};

use crate::utils::{colored_mat_to_dmatrix, mat_to_dmatrix};


pub fn preprocess_image(img: &Mat) -> anyhow::Result<(DMatrix<f64>, DMatrix<Vector3<f64>>)> {
    let mut original_img_gray = Mat::default();
    match img.typ() {
        CV_8UC1 => {
            original_img_gray = img.clone();
        },
        CV_8UC3 => {
            cvt_color_def(img, &mut original_img_gray, COLOR_BGR2GRAY)?;
        },
        _ => {
            bail!("Image of unknown color type");
        }
    }
    
    let mut img_gray = Mat::default();
    resize(&original_img_gray, &mut img_gray, Size::new(128, 80), 0.0, 0.0, INTER_LINEAR)?;
    let img_gray = mat_to_dmatrix(&img_gray)?;

    let mut img_small = Mat::default();
    resize(&img, &mut img_small, Size::new(128, 80), 0.0, 0.0, INTER_LINEAR)?;
    let img_small = colored_mat_to_dmatrix(&img_small)?;

    Ok((img_gray, img_small))
}