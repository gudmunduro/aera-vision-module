use nalgebra::Vector2;

#[derive(Debug, Clone)]
pub struct ProposalArea {
    pub min: Vector2<i32>,
    pub max: Vector2<i32>
}

impl ProposalArea {
    pub fn area_add(&mut self, p: &Vector2<i32>) {
        self.min.x = p.x.min(self.min.x);
        self.min.y = p.y.min(self.min.y);
        self.max.x = p.x.max(self.max.x);
        self.max.y = p.y.max(self.max.y);
    }

    pub fn scaled(&self, x: f32, y: f32) -> ProposalArea {
        let mut scaled = self.clone();
        scaled.min.x = (scaled.min.x as f32 * x) as i32;
        scaled.min.y = (scaled.min.y as f32 * y) as i32;
        scaled.max.x = (scaled.max.x as f32 * x) as i32;
        scaled.max.y = (scaled.max.y as f32 * y) as i32;
        scaled
    }

    pub fn contains_point(&self, x: i32, y: i32) -> bool {
        x >= self.min.x && x <= self.max.x && y >= self.min.y && y <= self.max.y
    }

    pub fn size(&self) -> i32 {
        let rect = (self.max - self.min);
        rect.x * rect.y
    }
}

pub struct RecognizedArea {
    pub class: i64,
    pub color: i64,
    pub area: ProposalArea,
    pub features: Vec<bool>,
    pub keypoints: Vec<Vector2<i64>>,
}

impl RecognizedArea {
    pub fn new(class: i64, color: i64, area: ProposalArea, features: Vec<bool>, keypoints: Vec<Vector2<i64>>) -> Self {
        Self {
            class,
            color,
            area,
            features,
            keypoints,
        }
    }
}