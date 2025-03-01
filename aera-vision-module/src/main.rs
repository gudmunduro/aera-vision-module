use std::{fmt, process::exit, sync::{Arc, Mutex}, thread::{self, sleep}, time::Duration, u64};

use aera::{commands::Command, properties::Properties, protobuf::{tcp_message, variable_description, DataMessage, ProtoVariable, VariableDescription}, AeraConn};
use nalgebra::{Vector2, Vector4};
use opencv::imgcodecs::{self, IMREAD_COLOR};
use pixy2::PixyCamera;
use robot::{feedback_data::{self, FeedbackData}, RobotConn, RobotFeedbackConn};
use vision::{RecognizedArea, VisionSystem};


fn main() -> anyhow::Result<()> {
    setup_logging();

    log::info!("Connecting to robot");
    let mut robot = RobotConn::connect().expect("Failed to connect to robot");

    loop {
        match run_main_loop(&mut robot) {
            Ok(_) => break,
            Err(e) => {
                log::error!("Error occurred in main loop {e:?}");
                log::debug!("Trying to reconnect");
                continue
            },
        }
    }

    Ok(())
}

fn run_main_loop(robot: &mut RobotConn) -> anyhow::Result<()> {
    let mut robot_feedback = RobotFeedbackConn::connect().expect("Failed to connect to robot feedback");
    let feedback_data = Arc::new(Mutex::new(robot_feedback.receive_feedback()?));

    log::info!("Connecting to AERA");
    let mut aera = AeraConn::connect("192.168.1.44")?;
    let mut properties = Properties::new();
    log::debug!("Wating for start message");
    aera.wait_for_start_message()?;

    log::info!("Connecting to pixy");
    let pixy = PixyCamera::init()?;
    let mut vision = VisionSystem::new();

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
        println!("Recognized {}", objects.len());
        let mut cam_objs = vec![&mut properties.co1, &mut properties.co2, &mut properties.co3];
        if properties.h.holding.is_some() {
            // Don't overwrite camera object that is being held (currently only co1)
            cam_objs = vec![&mut properties.co2, &mut properties.co3];
            // Approximate position of held camera object is always equal to the hand position
            properties.co1.approximate_pos = properties.h.position;
        }
        cam_objs.iter_mut().for_each(|c| c.set_default());
        for i in 0..objects.len().min(3) {
            let area = &objects[i].area;

            cam_objs[i].class = objects[i].class;
            cam_objs[i].position = (area.min + (area.max - area.min)).cast();
            log::debug!("Sending CO pos ({}, {})", cam_objs[i].position.x, cam_objs[i].position.y);
        }

        // Get data from robot
        let feedback_data = feedback_data.lock().unwrap();
        let [x, y, z, r, ..] = feedback_data.tool_vector_actual;
        properties.h.position = Vector4::new(x, y, z, r);
        if (((feedback_data.digital_outputs >> 2) & 1)) != 0 && objects.len() == 0 {
            properties.h.holding = Some("co1".to_string());
        }
        for co in cam_objs.iter_mut().filter(|co| co.class != -1) {
            co.approximate_pos = calculate_predicted_grab_pos(&properties.h.position, &co.position);
            log::debug!("Sending approximate cube pos ({}, {}, {}, {})", co.approximate_pos.x, co.approximate_pos.y, co.approximate_pos.z, co.approximate_pos.w);
        }
        drop(feedback_data);

        // Send to AERA
        log::debug!("Sending hand position ({}, {}, {}, {})", properties.h.position.x, properties.h.position.y, properties.h.position.z, properties.h.position.w);
        log::debug!("Hand holding: {:?}", properties.h.holding);
        aera.send_properties(&properties, None)?;

        // Handle command from AERA
        log::debug!("Listening for command");
        let cmd = match aera.listen_for_command() {
            Ok(Some(cmd)) => cmd,
            Ok(None) => {
                log::error!("Timed out waiting for command from AERA");
                continue;
            }
            Err(e) => {
                log::error!("Error receiving command from AERA: {e}");
                continue;
            }
        };
        match cmd {
            Command::EnableRobot => {
                log::debug!("Got enable_robot command from AERA");
                log_err(|| robot.enable_robot());
            }
            Command::MovJ(x, y, z, r) => {
                log::debug!("Got movj command from AERA to {x}, {y}, {z}, {r}");
                log_err(|| robot.mov_j(x as f64, y as f64, z as f64, r as f64));
            }
            Command::Move(x, y, z, r) => {
                log::debug!("Got move (relative) command from AERA by {x}, {y}, {z}, {r}");
                let pos = &properties.h.position;
                log_err(|| robot.mov_j(pos.x + x, pos.y + y, pos.z + z, pos.w + r));
            }
            Command::Grab => {
                log::debug!("Got grab command from AERA");
                log_err(|| -> anyhow::Result<()> {
                    let pos = properties.h.position + Vector4::new(0.0, 0.0, -137.0, 0.0);
                    robot.mov_j(pos.x, pos.y, pos.z, pos.w)?;
                    sleep(Duration::from_secs(1));
                    robot.set_do(3, true)?;
                    sleep(Duration::from_secs(1));
                    let orig_pos = &properties.h.position;
                    robot.mov_j(orig_pos.x, orig_pos.y, orig_pos.z, orig_pos.w)?;

                    Ok(())
                });
            },
            Command::Release => {
                log::debug!("Got release command from AERA");
                log_err(|| -> anyhow::Result<()> {
                    robot.set_do(3, false)?;
                    properties.h.holding = None;

                    Ok(())
                });
            }
        }

        aera.increase_timestamp();
    }

    Ok(())
}

fn calculate_predicted_grab_pos(hand_pos: &Vector4<f64>, co_pos: &Vector2<i64>) -> Vector4<f64> {
    const CAM_GRAB_POS: Vector2<i64> = Vector2::new(162, 191);
    let pred_x = hand_pos.x + (CAM_GRAB_POS.y - co_pos.y) as f64;
    let pred_y = hand_pos.y + ((CAM_GRAB_POS.x - co_pos.x) as f64 / 1.175);
    let pred_z = -140_f64;
    let pred_w = 45_f64;

    Vector4::new(pred_x, pred_y, pred_z, pred_w)
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