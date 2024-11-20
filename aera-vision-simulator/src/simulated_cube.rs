use nalgebra::Vector2;


pub struct SimCube {
    pub pos: Vector2<i64>
}

impl SimCube {
    pub fn initial() -> Self {
        Self {
            pos: Vector2::new(101, 121)
        }
    }

    // Coords on hand are reversed
    pub fn move_hand_by(&mut self, x: i64, y: i64) {
        self.pos.x += (y as f64 * 1.175) as i64;
        self.pos.y += x;
    }
}