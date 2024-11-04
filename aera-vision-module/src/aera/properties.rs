use nalgebra::{Vector2, Vector4};

#[derive(Debug, Clone)]
pub struct Properties {
    pub co1: CameraObject,
    pub co2: CameraObject,
    pub co3: CameraObject,
    pub h: HandObject,
}

impl Properties {
    pub fn new() -> Properties {
        Properties {
            co1: CameraObject::new(),
            co2: CameraObject::new(),
            co3: CameraObject::new(),
            h: HandObject::new(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct CameraObject {
    pub position: Vector2<i64>,
    pub class: i64,
}

impl CameraObject {
    pub fn new() -> CameraObject {
        CameraObject {
            position: Vector2::new(-1, -1),
            class: -1,
        }
    }

    pub fn set_default(&mut self) {
        self.position = Vector2::new(-1, -1);
        self.class = -1;
    }
}

#[derive(Debug, Clone)]
pub struct HandObject {
    pub position: Vector4<i64>,
}

impl HandObject {
    pub fn new() -> HandObject {
        HandObject {
            position: Vector4::new(0, 0, 0, 0),
        }
    }
}