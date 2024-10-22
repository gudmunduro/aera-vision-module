use nalgebra::{DMatrix, Vector2};

const MOTION_VEC_SEARCH_DIST_PENALITY: f64 = 0.001;
const SEARCH_WIDTH: i32 = 10;
const RECT_WIDTH: i32 = 6;
const PIXELS_MOVED_THRESHOLD: f64 = 0.05;

pub fn calc_motion_matrix(am: &DMatrix<f64>, bm: &DMatrix<f64>) -> DMatrix::<Vector2<i32>> {
    let mut motions = DMatrix::from_element(am.nrows(), am.ncols(), Vector2::new(0, 0));

    for iy in 0..am.nrows() {
        for ix in 0..am.ncols() {
            let motion = calc_best_motion_vector(am, bm, &Vector2::new(ix as i32, iy as i32));
            motions[(iy, ix)] = motion;
        }
    }

    motions
}

fn calc_best_motion_vector(am: &DMatrix<f64>, bm: &DMatrix<f64>, center: &Vector2<i32>) -> Vector2<i32> {
    let mut best = (Vector2::new(0, 0), 1.0e12);

    // Optimization for when pixels have most likeley not moved
    if calc_diff(am, bm, center, &Vector2::new(0, 0), RECT_WIDTH) < PIXELS_MOVED_THRESHOLD {
        return Vector2::new(0, 0);
    }

    for dy in -(SEARCH_WIDTH / 2)..(SEARCH_WIDTH / 2) {
        for dx in -(SEARCH_WIDTH / 2)..(SEARCH_WIDTH / 2) {
            let delta_vec = Vector2::new(dx, dy);
            let dist = delta_vec.cast::<f64>().norm();
            let dist_cost = MOTION_VEC_SEARCH_DIST_PENALITY * dist;

            let diff = calc_diff(am, bm, center, &delta_vec, RECT_WIDTH);
            let val = diff + dist_cost;
            if val < best.1 {
                best = (delta_vec, val);
            }
        }
    }

    best.0
}


fn calc_diff(
    am: &DMatrix<f64>,
    bm: &DMatrix<f64>,
    center: &Vector2<i32>,
    delta: &Vector2<i32>,
    rect_width: i32,
) -> f64 {
    let mut diff_sum: f64 = 0.0;
    let half_width = rect_width / 2;

    for iy in -half_width..half_width {
        for ix in -half_width..half_width {
            let a = am.get(((iy + center.y) as usize, (ix + center.x) as usize)).unwrap_or(&0.0);
            let b = bm.get(((iy + delta.y + center.y) as usize, (ix + delta.x + center.x) as usize)).unwrap_or(&0.0);
            let diff = a - b;
            diff_sum += diff * diff; // compute MSE because it has nice properties
        }
    }

    diff_sum
}
