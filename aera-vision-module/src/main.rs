use std::{fmt, thread::sleep, time::Duration, u64};

use aera::{commands::Command, properties::Properties, protobuf::{tcp_message, variable_description, DataMessage, ProtoVariable, VariableDescription}, AeraConn};
use opencv::imgcodecs::{self, IMREAD_COLOR};
use pixy2::PixyCamera;
use robot::RobotConn;
use vision::{RecognizedArea, VisionSystem};

pub mod aera;
pub mod robot;

fn main() -> anyhow::Result<()> {
    let mut aera = AeraConn::connect("127.0.0.1")?;
    aera.wait_for_start_message()?;
    let mut vision = VisionSystem::new();
    let pixy = PixyCamera::init()?;
    let mut robot = RobotConn::connect().expect("Failed to connect to robot");

    let mut properties = Properties::new();
    loop {
        sleep(Duration::from_secs(1));
        
        let frame = pixy.get_frame()?;
        let objects = vision.process_frame(&frame)?;
        let cam_objs = [&mut properties.co1, &mut properties.co2, &mut properties.co3];
        for i in 0..objects.len().min(3) {
            let area = &objects[i].area;

            cam_objs[i].class = objects[i].class;
            cam_objs[i].position = (area.min + (area.max - area.min)).cast();
        }

        aera.send_properties(&properties)?;
        
        let cmd = match aera.listen_for_command() {
            Ok(cmd) => cmd,
            Err(e) => {
                eprintln!("Error receiving command from AERA: {e}");
                continue;
            }
        };
        match cmd {
            Command::MovJ(x, y, z, r) => {
                println!("Got movj command from AERA to {x}, {y}, {z}, {r}");
                log_err(|| robot.mov_j(x, y, z, r));
            }
        }
    }
}

fn log_err<T, E: fmt::Display>(f: impl FnOnce() -> Result<T, E>) {
    match f() {
        Ok(_) => {},
        Err(e) => eprintln!("Error: Failed to send command to robot\n{e}"),
    }
}