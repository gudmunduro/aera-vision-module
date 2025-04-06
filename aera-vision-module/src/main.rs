use std::{fmt, process::exit, sync::{Arc, Mutex}, thread::{self, sleep}, time::Duration, u64};
use itertools::Itertools;
use aera::{commands::Command, properties::Properties, protobuf::{tcp_message, variable_description, DataMessage, ProtoVariable, VariableDescription}, AeraConn, CAM_OBJ_COUNT};
use nalgebra::{distance, Vector2, Vector4};
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
                thread::sleep(Duration::from_secs(5));
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
    let mut properties = Properties::new(CAM_OBJ_COUNT);
    let mut aera = AeraConn::connect("127.0.0.1", &properties.cam_objs.keys().map(|id| id.as_str()).collect::<Vec<_>>())?;
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
        let objects = vision.process_frame(&frame)?.into_iter().filter(|o| o.color > 0).collect_vec();
        println!("Recognized {}", objects.len());

        // Get data from robot
        let feedback_data = feedback_data.lock().unwrap();
        let [x, y, z, r, ..] = feedback_data.tool_vector_actual;
        properties.h.position = Vector4::new(x, y, z, r);
        //if (((feedback_data.digital_outputs >> 2) & 1)) != 0 && objects.len() < properties.cam_objs.values().filter(|co| co.class != -1).count() {
            // Say we are holding the object that was closest to center in last frame
        //    properties.h.holding = Some(get_object_closest_to_center(&properties));
        //}
        drop(feedback_data);

        // Update based on data from camera
        let mut cam_obj_keys = properties.cam_objs.keys().cloned().sorted().collect::<Vec<_>>();
        if let Some(co) = &properties.h.holding {
            // Don't overwrite camera object that is being held
            cam_obj_keys.retain(|k| k != co);
        }
        cam_obj_keys.iter().for_each(|c| properties.cam_objs.get_mut(c).unwrap().set_default());
        for (object, co_key) in objects.iter().zip(cam_obj_keys.iter()).take(cam_obj_keys.len()) {
            let area = &object.area;
            let cam_obj = properties.cam_objs.get_mut(co_key).unwrap();

            cam_obj.class = 0;
            cam_obj.color = object.color;
            cam_obj.position = (area.min + (area.max - area.min) / 2).cast();
            log::debug!("Sending {co_key} pos ({}, {})", cam_obj.position.x, cam_obj.position.y);
        }

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
                log::info!("Got enable_robot command from AERA");
                log_err(|| robot.enable_robot());
            }
            Command::MovJ(x, y, z, r) => {
                log::info!("Got movj command from AERA to {x}, {y}, {z}, {r}");
                log_err(|| robot.mov_j(x as f64, y as f64, z as f64, r as f64));
            }
            Command::Move(x, y, z, r) => {
                log::info!("Got move (relative) command from AERA by {x}, {y}, {z}, {r}");
                //ask_to_continue();
                let pos = &properties.h.position;
                log_err(|| robot.mov_j(pos.x + x, pos.y + y, pos.z + z, pos.w + r));
            }
            Command::Grab => {
                log::info!("Got grab command from AERA");
                //ask_to_continue();
                log_err(|| -> anyhow::Result<()> {
                    let pos = properties.h.position + Vector4::new(0.0, 0.0, -106.0, 0.0);
                    robot.set_do(3, true)?;
                    sleep(Duration::from_secs(2));
                    robot.mov_j(pos.x, pos.y, pos.z, pos.w)?;
                    sleep(Duration::from_secs(1));
                    robot.set_do(3, false)?;
                    sleep(Duration::from_secs(3));
                    robot.set_do(1, true)?;
                    sleep(Duration::from_secs(3));
                    let orig_pos = &properties.h.position;
                    robot.mov_j(orig_pos.x, orig_pos.y, orig_pos.z, orig_pos.w)?;
                    properties.h.holding = Some(get_object_closest_to_center(&properties));

                    Ok(())
                });
            },
            Command::Release => {
                log::info!("Got release command from AERA");
                log_err(|| -> anyhow::Result<()> {
                    robot.set_do(1, false)?;
                    sleep(Duration::from_secs(1));
                    robot.set_do(3, true)?;
                    properties.h.holding = None;

                    Ok(())
                });
            },
            Command::NoAction => {
                log::info!("Got no action command from AERA");
            }
        }

        aera.increase_timestamp();
    }

    Ok(())
}

fn calculate_predicted_grab_pos(hand_pos: &Vector4<f64>, co_pos: &Vector2<f64>) -> Vector4<f64> {
    const CAM_GRAB_POS: Vector2<f64> = Vector2::new(162.0, 191.0);
    let pred_x = hand_pos.x + (CAM_GRAB_POS.y - co_pos.y);
    let pred_y = hand_pos.y + ((CAM_GRAB_POS.x - co_pos.x) / 1.175);
    let pred_z = -140_f64;
    let pred_w = 45_f64;

    Vector4::new(pred_x, pred_y, pred_z, pred_w)
}

fn get_object_closest_to_center(properties: &Properties) -> String {
    const CAM_GRAB_POS: Vector2<f64> = Vector2::new(145.0, 173.0);

    properties.cam_objs.iter()
        .filter(|(_, co)| co.class != -1)
        .sorted_by_key(|(_, co)| ((co.position - CAM_GRAB_POS).norm() * 100.0) as i32)
        .map(|(k, _)| k.clone())
        .next()
        .unwrap()
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

fn ask_to_continue() {
    loop {
        print!("Continue? ");
        let mut line = String::new();
        std::io::stdin().read_line(&mut line).unwrap();

        if line == "y" || line == "y\n" {
            break;
        }
    }
}
