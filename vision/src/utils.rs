use nalgebra::{DMatrix, Vector3};
use opencv::{core::Vec3b, prelude::*};


pub fn mat_to_dmatrix(mat: &Mat) -> anyhow::Result<DMatrix<f64>> {
    let mat_size = mat.mat_size().to_vec();
    let mut res = DMatrix::zeros(mat_size[0] as usize, mat_size[1] as usize);

    for r in 0..mat_size[0] as usize {
        for c in 0..mat_size[1] as usize {
            res[(r, c)] = *mat.at_2d::<u8>(r as i32, c as i32)? as f64 / 255.0;
        }
    }

    Ok(res)
}

pub fn colored_mat_to_dmatrix(mat: &Mat) -> anyhow::Result<DMatrix<Vector3<f64>>> {
    let mat_size = mat.mat_size().to_vec();
    let mut res = DMatrix::from_element(mat_size[0] as usize, mat_size[1] as usize, Vector3::new(0.0, 0.0, 0.0));

    for r in 0..mat_size[0] as usize {
        for c in 0..mat_size[1] as usize {
            let pixel = mat.at_2d::<Vec3b>(r as i32, c as i32)?; 
            res[(r, c)].x = pixel[0] as f64 / 255.0;
            res[(r, c)].y = pixel[1] as f64 / 255.0;
            res[(r, c)].z = pixel[2] as f64 / 255.0;
        }
    }

    Ok(res)
}