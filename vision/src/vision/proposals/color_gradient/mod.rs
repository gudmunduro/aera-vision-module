use anyhow::Ok;
use nalgebra::Vector2;
use opencv::{
    core::{bitwise_or_def, in_range, Mat, Point, Vector},
    imgproc::{
        bounding_rect, canny_def, cvt_color, find_contours_def, gaussian_blur_def,CHAIN_APPROX_SIMPLE, COLOR_BGR2HSV, RETR_EXTERNAL
    },
};

use super::proposal_area::ProposalArea;

pub fn make_proposals(orig_image_a: &Mat) -> anyhow::Result<Vec<ProposalArea>> {
    let mut img_blur = Mat::default();
    gaussian_blur_def(&orig_image_a, &mut img_blur, (5, 5).into(), 0.0)?;

    let mut img_hsv = Mat::default();
    cvt_color(&img_blur, &mut img_hsv, COLOR_BGR2HSV, 0)?;

    let mut masked_blue = Mat::default();
    // Blue
    in_range(
        &img_hsv,
        &[100, 150, 50],
        &[140, 255, 255],
        &mut masked_blue,
    )?;
    // Red
    let mut masked_red_1 = Mat::default();
    in_range(&img_hsv, &[0, 120, 70], &[10, 255, 255], &mut masked_red_1)?;
    let mut masked_red_2 = Mat::default();
    in_range(
        &img_hsv,
        &[170, 120, 70],
        &[180, 255, 255],
        &mut masked_red_2,
    )?;
    // Yellow
    let mut masked_yellow = Mat::default();
    in_range(
        &img_hsv,
        &[20, 100, 100],
        &[30, 255, 255],
        &mut masked_yellow,
    )?;

    let mut combined_1 = Mat::default();
    bitwise_or_def(&masked_blue, &masked_red_1, &mut combined_1)?;
    let mut combined_2 = Mat::default();
    bitwise_or_def(&combined_1, &masked_red_2, &mut combined_2)?;
    let mut combined_3 = Mat::default();
    bitwise_or_def(&combined_2, &masked_yellow, &mut combined_3)?;

    // Calculate edges
    let mut edges = Mat::default();
    canny_def(&combined_3, &mut edges, 100.0, 250.0)?;

    // Countours (boxes)
    let mut contours: Vector<Vector<Point>> = Vector::new();
    find_contours_def(&edges, &mut contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)?;

    let changed_areas: anyhow::Result<Vec<ProposalArea>> = contours
        .iter()
        .map(|contour| {
            let rect = bounding_rect(&contour)?;

            Ok(ProposalArea {
                min: Vector2::new(rect.x, rect.y),
                max: Vector2::new(rect.x + rect.width, rect.y + rect.height),
            })
        })
        .collect();

    Ok(changed_areas?
        .into_iter()
        .filter(|a| (a.max - a.min).cast::<f32>().norm() > 10.0)
        .collect())
}
