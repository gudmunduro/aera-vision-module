use std::cmp::max;

use anyhow::Ok;
use itertools::Itertools;
use nalgebra::{DMatrix, Vector3};
use opencv::{core::{Mat, Range, Rect, Size}, imgproc::{resize, INTER_LINEAR}};

use crate::{utils::{colored_mat_to_dmatrix, mat_to_dmatrix}, vision::proposals::proposal_area::ProposalArea};

pub fn get_image_data_for_classification(m: &DMatrix<f64>, area: &ProposalArea) -> Vec<f64> {
    let cropped = crop(m, area);
    let resized = to_size(&cropped, 16);
    resized.column_iter().flatten().map(|v| *v).collect()
}

pub fn get_image_data_for_clip_classification(orig_image: &Mat, area: &ProposalArea, scale_x: i32, scale_y: i32) -> anyhow::Result<Vec<f64>> {
    let mean = Vector3::new(0.48145466, 0.4578275, 0.40821073);
    let std = Vector3::new(0.26862954, 0.26130258, 0.27577711);

    let mut area = area.clone();
    area.min.x *= scale_x;
    area.max.x *= scale_x;
    area.min.y *= scale_y;
    area.max.y *= scale_y;

    let cropped = Mat::roi(orig_image, Rect::new(area.min.x, area.min.y, area.max.x - area.min.x, area.max.y - area.min.y))?;
    let mut processed_image = Mat::default();
    resize(&cropped, &mut processed_image, Size::new(224, 224), 0.0, 0.0, INTER_LINEAR)?;
    let processed = colored_mat_to_dmatrix(&processed_image)?;
    let pixel_values = processed.iter()
        .map(|p| (p - mean).component_div(&std))
        .flat_map(|e| e.iter().copied().collect_vec())
        .collect();


    Ok(pixel_values)
}

fn crop(m: &DMatrix<f64>, area: &ProposalArea) -> DMatrix<f64> {
    let cols = (area.max.x - area.min.x) as usize;
    let rows = (area.max.y - area.min.y) as usize;

    DMatrix::from_fn(rows, cols, |y, x| {
        *m.get((area.max.y as usize + y, area.min.x as usize + x)).unwrap_or(&0.0)
    })
}

fn to_size(m: &DMatrix<f64>, size: usize) -> DMatrix<f64> {
    let center_x = m.nrows() as f64 / 2.0;
    let center_y = m.nrows() as f64 / 2.0;

    let max_size = max(m.nrows(), m.ncols()) as f64;
    DMatrix::from_fn(size, size, |y: usize, x| {
        // Location (colun) between -1 and 1
        let rel_x = (x as f64 / size as f64) * 2.0 - 1.0;
        // Location (row) between -1 and 1
        let rel_y = (y as f64 / size as f64) * 2.0 - 1.0;

        let index_x = (center_x + rel_x * max_size * 0.5) as usize;
        let index_y = (center_y + rel_y * max_size * 0.5) as usize;

        *m.get((index_y, index_x)).unwrap_or(&0.0)
    })
}