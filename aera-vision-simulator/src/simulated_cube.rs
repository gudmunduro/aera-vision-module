use nalgebra::{Vector2, Vector4};

const CAM_GRAB_POS: Vector2<i64> = Vector2::new(148, 171);

pub struct SimCube {
    pub pos: Vector2<i64>,
    pub approximte_pos: Vector4<f64>,
    pub visible: bool
}

impl SimCube {
    pub fn initial() -> Self {
        Self {
            pos: Vector2::new(101, 121),
            approximte_pos: Vector4::new(0.0, 0.0, 0.0, 0.0),
            visible: true
        }
    }

    pub fn move_hand(&mut self, hand_change: &Vector4<f64>, hand_pos: &Vector4<f64>) {
        // Coords on hand are reversed compared to cube coords
        self.pos.x += (hand_change.y as f64 * 1.175) as i64;
        self.pos.y += hand_change.x as i64;

        self.approximte_pos.x = hand_pos.x + (CAM_GRAB_POS.y - self.pos.y) as f64;
        self.approximte_pos.y = hand_pos.y + (CAM_GRAB_POS.x - self.pos.x) as f64 / 1.175;
        self.approximte_pos.z = -140.0;
        self.approximte_pos.w = 45.0;
    }
}