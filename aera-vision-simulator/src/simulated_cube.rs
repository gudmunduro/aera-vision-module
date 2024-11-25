use nalgebra::{Vector2, Vector4};

const CAM_GRAB_POS: Vector2<i64> = Vector2::new(148, 171);

pub struct SimCube {
    pub pos: Vector2<i64>,
    pub predicted_grab_pos: Vector4<i64>
}

impl SimCube {
    pub fn initial() -> Self {
        Self {
            pos: Vector2::new(101, 121),
            predicted_grab_pos: Vector4::new(0, 0, 0, 0)
        }
    }

    pub fn move_hand(&mut self, hand_change: &Vector4<i64>, hand_pos: &Vector4<i64>) {
        // Coords on hand are reversed compared to cube coords
        self.pos.x += (hand_change.y as f64 * 1.175) as i64;
        self.pos.y += hand_change.x;

        self.predicted_grab_pos.x = hand_pos.x + (CAM_GRAB_POS.y - self.pos.y);
        self.predicted_grab_pos.y = hand_pos.y + ((CAM_GRAB_POS.x - self.pos.x) as f64 / 1.175) as i64;
        self.predicted_grab_pos.z = 0;
        self.predicted_grab_pos.w = 45;
    }
}