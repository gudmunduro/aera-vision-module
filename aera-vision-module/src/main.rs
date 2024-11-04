use std::{fmt, process::exit, sync::{Arc, Mutex}, thread::{self, sleep}, time::Duration, u64};

use aera::{commands::Command, properties::Properties, protobuf::{tcp_message, variable_description, DataMessage, ProtoVariable, VariableDescription}, AeraConn};
use nalgebra::Vector4;
use opencv::imgcodecs::{self, IMREAD_COLOR};
use pixy2::PixyCamera;
use robot::{feedback_data::{self, FeedbackData}, RobotConn, RobotFeedbackConn};
use vision::{RecognizedArea, VisionSystem};

pub mod aera;
pub mod robot;

fn main() -> anyhow::Result<()> {
    setup_logging();
    let mut robot = RobotConn::connect().expect("Failed to connect to robot");
    let mut robot_feedback = RobotFeedbackConn::connect().expect("Failed to connect to robot feedback");
    let mut aera = AeraConn::connect("127.0.0.1")?;
    aera.wait_for_start_message()?;
    let mut vision = VisionSystem::new();
    let pixy = PixyCamera::init()?;
    let feedback_data = Arc::new(Mutex::new(robot_feedback.receive_feedback()?));
    let mut properties = Properties::new();

    {
        log::info!("Getting initial feedback...");
        let feedback_data = feedback_data.clone();
        thread::spawn(move || {
            run_feedback_loop(robot_feedback, feedback_data);
        });
    }

    log::info!("Starting main loop");
    loop {
        sleep(Duration::from_secs(3));
        
        // Get data from camera
        let frame = pixy.get_frame()?;
        let objects = vision.process_frame(&frame)?;
        let mut cam_objs = [&mut properties.co1, &mut properties.co2, &mut properties.co3];
        cam_objs.iter_mut().for_each(|c| c.set_default());
        for i in 0..objects.len().min(3) {
            let area = &objects[i].area;

            cam_objs[i].class = objects[i].class;
            cam_objs[i].position = (area.min + (area.max - area.min)).cast();
        }

        // Get data from robot
        let feedback_data = feedback_data.lock().unwrap();
        let [.., x, y, z] = feedback_data.tool_vector_actual;
        let [r, ..] = feedback_data.tcp_speed_actual;
        properties.h.position = Vector4::new(x.round() as i64, y.round() as i64, z.round() as i64, r.round() as i64);
        drop(feedback_data);

        // Send to AERA
        log::debug!("Sending hand position ({}, {}, {}, {})", properties.h.position.x, properties.h.position.y, properties.h.position.z, properties.h.position.w);
        aera.send_properties(&properties)?;
        
        // Handle command from AERA
        let cmd = match aera.listen_for_command() {
            Ok(cmd) => cmd,
            Err(e) => {
                log::error!("Error receiving command from AERA: {e}");
                continue;
            }
        };
        match cmd {
            Command::MovJ(x, y, z, r) => {
                log::debug!("Got movj command from AERA to {x}, {y}, {z}, {r}");
                log_err(|| robot.mov_j(x as f64, y as f64, z as f64, r as f64));
            }
            Command::EnableRobot => {
                log::debug!("Got enable_robot command from AERA");
                log_err(|| robot.enable_robot());
            }
        }

        aera.increase_timestamp();
    }
}

fn log_err<T, E: fmt::Display>(f: impl FnOnce() -> Result<T, E>) {
    match f() {
        Ok(_) => {},
        Err(e) => log::error!("Error: Failed to send command to robot\n{e}"),
    }
}

fn run_feedback_loop(mut robot_feedback_conn: RobotFeedbackConn, feedback: Arc<Mutex<FeedbackData>>) {
    loop {
        let res = match robot_feedback_conn.receive_feedback() {
            Ok(feedback) => feedback,
            Err(e) => {
                log::error!("Error receiving feedback {e:?}");
                sleep(Duration::from_secs(1));
                continue;
            }
        };
        *feedback.lock().unwrap() = res;

        sleep(Duration::from_millis(10));
    }
}

fn setup_logging() {
    simple_log::quick!();
}