#[derive(Debug)]
pub enum Command {
    MovJ(i64, i64, i64, i64),
    EnableRobot,
    Grab,
    Release
}