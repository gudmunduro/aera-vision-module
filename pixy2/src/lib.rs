use std::ptr::null_mut;

use anyhow::{bail, Ok};
use opencv::core::{Mat, Scalar};

const PIXY2_RAW_FRAME_WIDTH: usize = 316;
const PIXY2_RAW_FRAME_HEIGHT: usize = 208;
const PIXY2_BAYER_FRAME_BUFFER_SIZE: usize = PIXY2_RAW_FRAME_WIDTH * PIXY2_RAW_FRAME_HEIGHT;

#[allow(unused_imports)]
#[cxx::bridge]
mod ffi {
    unsafe extern "C++" {
        include!("bridge.h");

        fn init() -> i32;
        fn set_lamp(upper: i32, lower: i32);
        fn stop() -> i32;
        unsafe fn get_raw_frame(bayer_frame: *mut *mut u8) -> i32;
    }
}

pub struct PixyCamera {}

impl PixyCamera {
    pub fn init() -> anyhow::Result<Self> {
        let res = ffi::init();
        if ffi::init() != 0 {
            bail!("Failed to initialize pixy camera with result {res}");
        }
        if ffi::stop() != 0 {
            bail!("Failed to stop pixy camera program with result {res}");
        }
        Ok(PixyCamera {})
    }

    pub fn get_frame(&self) -> anyhow::Result<Mat> {
        let mut bayer_frame: *mut u8 = null_mut();
        unsafe {
            ffi::get_raw_frame(&mut bayer_frame);
        }
        let bayer_frame = unsafe {
            Vec::from_raw_parts(bayer_frame, PIXY2_BAYER_FRAME_BUFFER_SIZE, PIXY2_BAYER_FRAME_BUFFER_SIZE)
        };
        let rgb_frame = demosic(PIXY2_RAW_FRAME_WIDTH, PIXY2_RAW_FRAME_HEIGHT, bayer_frame.as_slice());
        let image_data = rgb_frame
            .as_slice()
            .iter()
            .map(|v| Scalar::new(((*v >> 16) & 0xff) as f64 / 255.0, ((*v >> 8) & 0xff) as f64 / 255.0, (*v & 0xFF) as f64 / 255.0, 1.0))
            .collect::<Vec<_>>();
        let mat = Mat::new_rows_cols_with_data(PIXY2_RAW_FRAME_HEIGHT as i32, PIXY2_RAW_FRAME_WIDTH as i32, image_data.as_slice())?.clone_pointee();
        Ok(mat)
    }
}

fn demosic(width: usize, height: usize, bayer_image: &[u8]) -> Vec<u32> {
    let mut image = vec![0; bayer_image.len()];

    // Gpt converted code, should probably be rewritten
    for y in 0..height {
        for y in 0..height {
            let mut yy = y;
            if yy == 0 {
                yy += 1;
            } else if yy == height - 1 {
                yy -= 1;
            }
            let pixel0_offset = yy * width;
            for x in 0..width {
                let mut xx = x;
                if xx == 0 {
                    xx += 1;
                } else if xx == width - 1 {
                    xx -= 1;
                }
                let pixel_index = pixel0_offset + xx;
    
                let (r, g, b) = if yy % 2 == 1 {
                    if xx % 2 == 1 {
                        // Red pixel
                        let r = bayer_image[pixel_index] as u32;
                        let g = (
                            bayer_image[pixel_index - 1] as u32
                                + bayer_image[pixel_index + 1] as u32
                                + bayer_image[pixel_index - width] as u32
                                + bayer_image[pixel_index + width] as u32
                        ) >> 2;
                        let b = (
                            bayer_image[pixel_index - width - 1] as u32
                                + bayer_image[pixel_index - width + 1] as u32
                                + bayer_image[pixel_index + width - 1] as u32
                                + bayer_image[pixel_index + width + 1] as u32
                        ) >> 2;
                        (r, g, b)
                    } else {
                        // Green pixel on red row
                        let r = (
                            bayer_image[pixel_index - 1] as u32 + bayer_image[pixel_index + 1] as u32
                        ) >> 1;
                        let g = bayer_image[pixel_index] as u32;
                        let b = (
                            bayer_image[pixel_index - width] as u32
                                + bayer_image[pixel_index + width] as u32
                        ) >> 1;
                        (r, g, b)
                    }
                } else {
                    if xx % 2 == 1 {
                        // Green pixel on blue row
                        let r = (
                            bayer_image[pixel_index - width] as u32
                                + bayer_image[pixel_index + width] as u32
                        ) >> 1;
                        let g = bayer_image[pixel_index] as u32;
                        let b = (
                            bayer_image[pixel_index - 1] as u32 + bayer_image[pixel_index + 1] as u32
                        ) >> 1;
                        (r, g, b)
                    } else {
                        // Blue pixel
                        let r = (
                            bayer_image[pixel_index - width - 1] as u32
                                + bayer_image[pixel_index - width + 1] as u32
                                + bayer_image[pixel_index + width - 1] as u32
                                + bayer_image[pixel_index + width + 1] as u32
                        ) >> 2;
                        let g = (
                            bayer_image[pixel_index - 1] as u32
                                + bayer_image[pixel_index + 1] as u32
                                + bayer_image[pixel_index - width] as u32
                                + bayer_image[pixel_index + width] as u32
                        ) >> 2;
                        let b = bayer_image[pixel_index] as u32;
                        (r, g, b)
                    }
                };
    
                image[x + width * y] = (r << 16) | (g << 8) | b;
            }
        }
    }

    image
}