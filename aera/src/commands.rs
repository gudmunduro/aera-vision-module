#[derive(Debug)]
pub enum Command {
    // Absolute move
    MovJ(i64, i64, i64, i64),
    // Relative move
    Move(f64, f64, f64, f64),
    // Grab with the robot
    Grab,
    // Release what the robot is holding
    Release,
    // Enable the robot
    EnableRobot
}