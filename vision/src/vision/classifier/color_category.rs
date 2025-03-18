use opencv::core::{bitwise_or_def, count_non_zero, in_range, AlgorithmHint, Mat, MatTraitConst, Rect};
use opencv::highgui;
use opencv::highgui::wait_key;
use opencv::imgproc::{cvt_color, gaussian_blur_def, COLOR_BGR2HSV, COLOR_RGB2HSV};
use crate::vision::proposals::proposal_area::ProposalArea;

pub fn get_color_of_proposal(img: &Mat, proposal: &ProposalArea) -> anyhow::Result<i32> {
    let proposal_img_rgb = img.roi(Rect {
        x: proposal.min.x,
        y: proposal.min.y,
        width: proposal.max.x - proposal.min.x,
        height: proposal.max.y - proposal.min.y,
    })?;
    let mut proposal_img = Mat::default();
    cvt_color(&proposal_img_rgb, &mut proposal_img, COLOR_BGR2HSV, 0, AlgorithmHint::ALGO_HINT_DEFAULT)?;

    // Blue
    let mut masked = Mat::default();
    in_range(
        &proposal_img,
        &[100, 100, 50],
        &[140, 255, 255],
        &mut masked,
    )?;
    let blue_ratio = count_non_zero(&masked)? as f32 / proposal_img.total() as f32;

    // Red
    let mut masked_red_1 = Mat::default();
    in_range(
        &proposal_img,
        &[0, 100, 50],
        &[10, 255, 255],
        &mut masked_red_1)?;
    let mut masked_red_2 = Mat::default();
    in_range(
        &proposal_img,
        &[162, 100, 50],
        &[180, 255, 255],
        &mut masked_red_2,
    )?;
    bitwise_or_def(&masked_red_1, &masked_red_2, &mut masked)?;

    let red_ratio = count_non_zero(&masked)? as f32 / proposal_img.total() as f32;

    // Yellow
    in_range(
        &proposal_img,
        &[20, 100, 100],
        &[45, 255, 255],
        &mut masked,
    )?;
    let yellow_ratio = count_non_zero(&masked)? as f32 / proposal_img.total() as f32;

    // Green
    in_range(
        &proposal_img,
        &[40, 100, 100],
        &[67, 255, 255],
        &mut masked,
    )?;
    let green_ratio = count_non_zero(&masked)? as f32 / proposal_img.total() as f32;

    let category = if blue_ratio > 0.6 {
        log::debug!("Saw blue object {blue_ratio} at ({}, {}) - ({}, {})", proposal.min.x, proposal.min.y, proposal.max.x, proposal.max.y);
        1
    }
    else if red_ratio > 0.6 {
        log::debug!("Saw red object {red_ratio} at ({}, {}) - ({}, {})", proposal.min.x, proposal.min.y, proposal.max.x, proposal.max.y);
        2
    }
    else if yellow_ratio > 0.6 {
        log::debug!("Saw yellow object {red_ratio} at ({}, {}) - ({}, {})", proposal.min.x, proposal.min.y, proposal.max.x, proposal.max.y);
        3
    }
    else if green_ratio > 0.6 {
        log::debug!("Saw green object {red_ratio} at ({}, {}) - ({}, {})", proposal.min.x, proposal.min.y, proposal.max.x, proposal.max.y);
        4
    }
    else {
        0
    };

    Ok(category)
}