use nalgebra::Vector2;

#[derive(Debug, Clone)]
pub struct Properties {
    pub co1: CameraObject,
    pub co2: CameraObject,
    pub co3: CameraObject,
}

impl Properties {
    pub fn new() -> Properties {
        Properties {
            co1: CameraObject::new(),
            co2: CameraObject::new(),
            co3: CameraObject::new(),
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
}
