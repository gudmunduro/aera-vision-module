pub mod motion_matrix;

use std::collections::HashMap;

use motion_matrix::calc_motion_matrix;
use nalgebra::{DMatrix, Vector2};

use super::{proposal_area::ProposalArea, filter::filter_too_small_regions};

const USE_SPINGLASS: bool = true;

pub fn make_proposals(am: &DMatrix<f64>, bm: &DMatrix<f64>) -> Vec<ProposalArea> {
    let mut motion_matrix = calc_motion_matrix(am, bm);
    let changed_areas = calc_changed_areas(&mut motion_matrix);
    let changed_areas = filter_too_small_regions(changed_areas);

    changed_areas
}

// Converts a motion map of motion vectors to a list of changed areas
pub fn calc_changed_areas(motion_map: &mut DMatrix<Vector2<i32>>) -> Vec<ProposalArea> {
    // TODO SCIFI LATER< use a NN to compute set of "ChangedAreaObj" we return. This has the advantage that it can take care of parallax motion and other "weird" motion types which are either hard or impossible to handle with handcrafted code >
    
    if USE_SPINGLASS {
        execute_spinglass(motion_map);
    }

    // * classify motion based on vector
    const N_DIM_BUCKETS: i32 = 3; // how many buckets for each motion component?
    const HYSTERESIS_MIN_MOTION_MAG: i32 = 1; // minimal magnitude of motion

    // * classify motion based on vector
    let bucket_count = (N_DIM_BUCKETS * 2 + 1) * (N_DIM_BUCKETS * 2 + 1);
    let mut motion_buckets: Vec<DMatrix<i32>> = (0..bucket_count)
        .map(|_| DMatrix::zeros(motion_map.nrows(), motion_map.ncols()))
        .collect();

    // algorithm to put each vector of the velocity field into the right bucket
    for iy in 0..motion_map.nrows() {
        for ix in 0..motion_map.ncols() {
            let vel = motion_map[(iy, ix)];
            if vel.x.abs() < HYSTERESIS_MIN_MOTION_MAG && vel.y.abs() < HYSTERESIS_MIN_MOTION_MAG {
                continue; // not fast enough, ignore
            }

            // compute index of velocity by dimension
            let mut bucket_idx_x: i32 = (vel.x.signum() * (vel.x.abs() / 3)) + N_DIM_BUCKETS + 1;
            bucket_idx_x = bucket_idx_x.clamp(0, N_DIM_BUCKETS * 2);

            let mut bucket_idx_y: i32 = (vel.y.signum() * (vel.y.abs() / 3)) + N_DIM_BUCKETS + 1;
            bucket_idx_y = bucket_idx_y.clamp(0, N_DIM_BUCKETS * 2);
            
            //motion_map[(iy, ix)] = Vector2::new(bucket_idx_x, bucket_idx_y);

            let i_bucket_idx = bucket_idx_x + bucket_idx_y * (N_DIM_BUCKETS * 2 + 1);
            motion_buckets[i_bucket_idx as usize][(iy, ix)] = 1;
        }
    }

    // algorithm to fill pixels which are adjacent to each other
    // with a color that tells you which area it belongs to
    for i_motion_bucket_idx in 0..motion_buckets.len() {
        for iy in 0..motion_map.nrows() {
            for ix in 0..motion_map.ncols() {
                let val = motion_buckets[i_motion_bucket_idx][(iy, ix)];
                if val == 1 {
                    let itcol = 2 + ix as i32 + iy as i32 * motion_map.ncols() as i32 + 1;
                    boundary_fill(ix as i32, iy as i32, 0, itcol, &mut motion_buckets[i_motion_bucket_idx]);
                }
            }
        }
    }

    // * compose groups
    let mut region_by_color: HashMap<i32, ProposalArea> = HashMap::new();

    for sel_quadrant_map in &motion_buckets {
        for iy in 0..motion_map.nrows() {
            for ix in 0..motion_map.ncols() {
                let v: i32 = sel_quadrant_map[(iy, ix)];
                // Incolored pixles (with no motion vector)
                if v == 0 {
                    continue;
                }

                
                if let Some(area) = region_by_color.get_mut(&v) {
                    // Increase the size of the area for this color
                    area.area_add(&Vector2::new(ix as i32, iy as i32));
                } else {
                    // Add a new 1px area of this color
                    region_by_color.insert(
                        v,
                        ProposalArea {
                            min: Vector2::new(ix as i32, iy as i32),
                            max: Vector2::new(ix as i32, iy as i32),
                        },
                    );
                }
            }
        }
    }

    // * translate groups to flat list of groups
    region_by_color.into_values().collect()
}

// * group by coloring algorithm
fn boundary_fill(
    pos_x: i32,
    pos_y: i32,
    boundary_color: i32,
    fill_color: i32,
    img: &mut DMatrix<i32>,
) {
    // Stack of positions to fill
    let mut stack: Vec<Vector2<i32>> = Vec::new();
    stack.push(Vector2::new(pos_x, pos_y));

    while !stack.is_empty() {
        let current = stack.pop().unwrap();
        let col = img[(current.y as usize, current.x as usize)];

        let top = Vector2::new(current.x, current.y + 1);
        let bottom = Vector2::new(current.x, current.y - 1);
        let right = Vector2::new(current.x + 1, current.y);
        let left = Vector2::new(current.x - 1, current.y);
        let surrounding_positions = [
            top.clone(),
            bottom.clone(),
            right.clone(),
            left.clone()
        ];

        if col != fill_color && col != boundary_color {
            img[(current.y as usize, current.x as usize)] = fill_color;

            for pos in surrounding_positions {
                stack.push(pos);
            }
        }
    }
}

fn execute_spinglass(motion_map: &mut DMatrix<Vector2<i32>>) {
    // Function to calculate coupling factor
    fn calc_coupling_factor(a: Vector2<f64>, b: Vector2<f64>, spinglass_coupling_factor: f64) -> f64 {
        // calc dot product
        let dot_res: f64 = a.dot(&b);
        let dir_coupling_val: f64 = (dot_res - spinglass_coupling_factor) * (1.0 / (1.0 - spinglass_coupling_factor));

        if dir_coupling_val < 0.0 {
            return 0.0; // not the same direction thus they are not coupled
        }

        // calculate same velocity-ness
        let diff = a - b;
        let len2 = diff.norm();

        let vel_coupling: f64 = (1.0 - len2).max(0.0);

        dir_coupling_val * vel_coupling
    }

    // new spinglass/firefly inspired algorithm to even out motion
    // this setting worked good enough in streetscene, but 'real world' seems to be problematic
    const SPINGLASS_STEPS: i32 = 5;
    const SPINGLASS_COUPLING_FACTOR: f64 = 0.8;

    // matrix with field of motion vectors
    let mut m = motion_map.clone().cast();

    for _ in 0..SPINGLASS_STEPS {
        let mut m2 = DMatrix::from_element(motion_map.nrows(), motion_map.ncols(), Vector2::new(0.0, 0.0));

        for iy in 1..m.nrows() - 1 {
            for ix in 1..m.ncols() - 1 {
                let this_dir = m[(iy, ix)];

                let l = m[(iy, ix - 1)]; // left
                let r = m[(iy, ix + 1)]; // right
                let t = m[(iy - 1, ix)]; // top
                let b = m[(iy + 1, ix)]; // bottom

                // compute couplings, inspired by spinglass / firefly algorithm
                let coupling_factor_l = calc_coupling_factor(this_dir, l, SPINGLASS_COUPLING_FACTOR);
                let coupling_factor_r = calc_coupling_factor(this_dir, r, SPINGLASS_COUPLING_FACTOR);
                let coupling_factor_t = calc_coupling_factor(this_dir, t, SPINGLASS_COUPLING_FACTOR);
                let coupling_factor_b = calc_coupling_factor(this_dir, b, SPINGLASS_COUPLING_FACTOR);

                // transfer by coupling
                let mut this_accu = this_dir;
                this_accu += l * coupling_factor_l;
                this_accu += r * coupling_factor_r;
                this_accu += t * coupling_factor_t;
                this_accu += b * coupling_factor_b;
                this_accu /= 1.0 + coupling_factor_l + coupling_factor_r + coupling_factor_t + coupling_factor_b; // normalize while preserving scale
                m2[(iy, ix)] = this_accu;
            }
        }

        m = m2; // swap
    }

    // convert back to integer motion vectors
    *motion_map = m.map(|e| e.map(|v| v as i32));
}
