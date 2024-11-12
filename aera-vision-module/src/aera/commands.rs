#[derive(Debug)]
pub enum Command {
    // Absolute move
    MovJ(i64, i64, i64, i64),
    // Relative move
    Move(i64, i64, i64, i64),
    // Grab with the robot
    Grab,
    // Release what the robot is holding
    Release,
    // Enable the robot
    EnableRobot
}