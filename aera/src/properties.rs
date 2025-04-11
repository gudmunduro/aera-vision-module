use std::collections::HashMap;
use nalgebra::{Vector2, Vector4};

#[derive(Debug, Clone)]
pub struct Properties {
    pub cam_objs: HashMap<String, CameraObject>,
    pub sift_keypoints: Vec<AeraSiftKeyPoint>,
    pub h: HandObject,
}

impl Properties {
    pub fn new(cam_obj_count: usize, sift_keypoint_count: usize) -> Properties {
        Properties {
            cam_objs: (1..cam_obj_count+1).map(|i| (format!("co{i}"), CameraObject::new())).collect(),
            sift_keypoints: (1..sift_keypoint_count+1).map(|i| AeraSiftKeyPoint::new(format!("sift{i}"))).collect(),
            h: HandObject::new(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct CameraObject {
    pub position: Vector2<f64>,
    pub approximate_pos: Vector4<f64>,
    pub class: i64,
    pub color: i64,
    pub size: i64
}

impl CameraObject {
    pub fn new() -> CameraObject {
        CameraObject {
            position: Vector2::new(-1.0, -1.0),
            approximate_pos: Vector4::new(-1.0, -1.0, -1.0, -1.0),
            class: -1,
            color: -1,
            size: -1
        }
    }

    pub fn set_default(&mut self) {
        self.position = Vector2::new(-1.0, -1.0);
        self.class = -1;
        self.color = -1;
        self.size = -1;
    }
}

#[derive(Debug, Clone)]
pub struct HandObject {
    pub position: Vector4<f64>,
    pub holding: Option<String>
}

impl HandObject {
    pub fn new() -> HandObject {
        HandObject {
            position: Vector4::new(0.0, 0.0, 0.0, 0.0),
            holding: None
        }
    }
}

#[derive(Debug, Clone)]
pub struct AeraSiftKeyPoint {
    pub name: String,
    pub detected: bool,
    pub point: Vector2<f64>,
    pub feature_vec: Vec<f64>,
}

impl AeraSiftKeyPoint {
    pub fn new(name: String) -> AeraSiftKeyPoint {
        AeraSiftKeyPoint {
            name,
            detected: false,
            point: Vector2::new(-1.0, -1.0),
            feature_vec: Vec::new(),
        }
    }
}