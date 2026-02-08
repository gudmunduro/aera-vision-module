use std::{fmt, fs, process::exit, sync::{Arc, Mutex}, thread::{self, sleep}, time::Duration, u64};
use std::fs::File;
use std::io::Write;
use itertools::Itertools;
use aera::{commands::Command, properties::Properties, protobuf::{tcp_message, variable_description, DataMessage, ProtoVariable, VariableDescription}, AeraConn, CAM_OBJ_COUNT, MAX_SIFT_POINT_COUNT};
use nalgebra::{distance, Vector2, Vector4};
use opencv::core::{AlgorithmHint, Mat, Scalar};
use opencv::highgui;
use opencv::highgui::wait_key;
use opencv::imgcodecs::{self, imwrite_def, IMREAD_COLOR};
use opencv::imgproc::{circle, circle_def, cvt_color, cvt_color_def, COLOR_RGB2BGR};
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
    let mut properties = Properties::new(CAM_OBJ_COUNT, MAX_SIFT_POINT_COUNT);
    let mut aera = AeraConn::connect("127.0.0.1", &properties.cam_objs.iter().map(|(name, _)| name.as_str()).collect::<Vec<_>>())?;
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

        // Get data from robot
        let feedback_data = feedback_data.lock().unwrap();
        let [x, y, z, r, ..] = feedback_data.tool_vector_actual;
        properties.h.position = Vector4::new(x, y, z, r);
        drop(feedback_data);

        // Update based on data from camera
        let mut cam_obj_keys = properties.cam_objs.keys().cloned().sorted().collect::<Vec<_>>();
        if let Some(co_key) = &properties.h.holding {
            // Don't overwrite camera object that is being held
            cam_obj_keys.retain(|k| k != co_key);

            let cam_obj = properties.cam_objs.get_mut(co_key).unwrap();
            cam_obj.approximate_pos = properties.h.position.clone();
        }
        cam_obj_keys.iter().for_each(|c| properties.cam_objs.get_mut(c).unwrap().set_default());
        for co_key in &cam_obj_keys {
            // Have each co as the object of that color
            let co_color: i64 = (&co_key[2..3]).parse().unwrap();
            let Some(object) = objects.iter().find(|o| o.color == co_color) else {
                continue;
            };
            let area = &object.area;
            let cam_obj = properties.cam_objs.get_mut(co_key).unwrap();

            cam_obj.class = 0;
            cam_obj.color = object.color;
            cam_obj.position = (area.min + (area.max - area.min) / 2).cast();
            cam_obj.approximate_pos = calculate_predicted_grab_pos(&properties.h.position, &cam_obj.position);
            cam_obj.features = object.features.clone();
            log::debug!("Sending {co_key} pos ({}, {})", cam_obj.position.x, cam_obj.position.y);
        }

        // Send to AERA
        log::debug!("Sending hand position ({}, {}, {}, {})", properties.h.position.x, properties.h.position.y, properties.h.position.z, properties.h.position.w);
        log::debug!("Hand holding: {:?}", properties.h.holding);
        save_properties(&properties)?;
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
                    let pos = properties.h.position + Vector4::new(0.0, 0.0, -110.0, 0.0);
                    robot.set_do(3, true)?;
                    sleep(Duration::from_secs(1));
                    robot.set_do(1, false)?;
                    sleep(Duration::from_secs(2));
                    robot.mov_j(pos.x, pos.y, pos.z, pos.w)?;
                    sleep(Duration::from_secs(1));
                    robot.set_do(3, false)?;
                    sleep(Duration::from_secs(3));
                    robot.set_do(1, true)?;
                    sleep(Duration::from_secs(3));
                    let orig_pos = &properties.h.position;
                    robot.mov_j(orig_pos.x, orig_pos.y, orig_pos.z, orig_pos.w)?;
                    // TODO: Determine if the center keypoint is still there
                    properties.h.holding = Some(get_object_closest_to_center(&properties));

                    Ok(())
                });
            },
            Command::Release => {
                log::info!("Got release command from AERA");
                log_err(|| -> anyhow::Result<()> {
                    let close_objects = properties.cam_objs
                        .iter()
                        .filter(|(_, o)| (o.approximate_pos - properties.h.position).norm() < 40.0)
                        .count();
                    log::debug!("{close_objects} are close to the hand");
                    let down_distance = -100.0;
                    // TODO: Temp change for scenario 2 (stacking)
                    let down_distance = -80.0;

                    let pos = properties.h.position + Vector4::new(0.0, 0.0, down_distance, 0.0);
                    let orig_pos = &properties.h.position;
                    robot.mov_j(pos.x, pos.y, pos.z, pos.w)?;
                    sleep(Duration::from_secs(1));
                    robot.set_do(1, false)?;
                    sleep(Duration::from_secs(1));
                    robot.set_do(3, true)?;
                    sleep(Duration::from_secs(1));
                    robot.mov_j(orig_pos.x, orig_pos.y, orig_pos.z, orig_pos.w)?;
                    properties.h.holding = None;

                    Ok(())
                });
            },
            Command::NoAction => {
                log::info!("Got no action command from AERA");
            },
            Command::Push => {
                log_err(|| -> anyhow::Result<()> {
                    let pos = properties.h.position + Vector4::new(-40.0, 0.0, -100.0, 0.0);
                    let orig_pos = &properties.h.position;

                    robot.set_do(3, false)?;
                    sleep(Duration::from_secs(1));
                    robot.set_do(1, true)?;
                    sleep(Duration::from_secs(1));
                    robot.mov_j(pos.x, pos.y, pos.z+100.0, pos.w)?;
                    sleep(Duration::from_secs(1));
                    robot.mov_j(pos.x, pos.y, pos.z, pos.w)?;
                    sleep(Duration::from_secs(1));
                    robot.mov_j(pos.x + 50.0, pos.y, pos.z, pos.w)?;
                    sleep(Duration::from_secs(1));
                    robot.mov_j(orig_pos.x, orig_pos.y, orig_pos.z, orig_pos.w)?;

                    Ok(())
                });
            }
        }

        aera.increase_timestamp();
    }

    Ok(())
}

fn calculate_predicted_grab_pos(hand_pos: &Vector4<f64>, co_pos: &Vector2<f64>) -> Vector4<f64> {
    const CAM_GRAB_POS: Vector2<f64> = Vector2::new(145.0, 160.0);
    let pred_x = hand_pos.x + (CAM_GRAB_POS.y - co_pos.y);
    let pred_y = hand_pos.y + ((CAM_GRAB_POS.x - co_pos.x) / 1.175);
    let pred_z = -100_f64;
    let pred_w = 180_f64;

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

fn get_sift_keypoint_closest_to_center(properties: &Properties) -> Option<String> {
    const CAM_GRAB_POS: Vector2<f64> = Vector2::new(139.0, 172.0);

    properties.sift_clusters.iter()
        .filter(|sift| sift.active)
        .sorted_by_key(|sift| ((sift.center - CAM_GRAB_POS).norm() * 100.0) as i32)
        .map(|sift| sift.name.clone())
        .next()
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

/*fn visualize_sift_clusters(img_rgb: &Mat, sift_clusters: &Vec<SiftClusterResult>) -> anyhow::Result<()> {
    highgui::named_window("Display window", highgui::WINDOW_NORMAL)?;
    highgui::resize_window("Display window", 1280, 720)?;
    let mut dbg_canvas = Mat::default();
    cvt_color(&img_rgb, &mut dbg_canvas, COLOR_RGB2BGR, 0, AlgorithmHint::ALGO_HINT_DEFAULT)?;

    for cluster in sift_clusters {
        circle_def(&mut dbg_canvas, opencv::core::Point::new(cluster.center.x as i32, cluster.center.y as i32), 8, Scalar::new(0.0, 0.0, 255.0, 255.0))?;
    }

    highgui::imshow("Display window", &dbg_canvas)?;
    wait_key(0)?;

    static FRAME: Mutex<i32> = Mutex::new(0);
    *FRAME.lock().unwrap() += 1;
    //imwrite_def(&format!("outputs/{}_cluster.jpg", *FRAME.lock().unwrap()), &dbg_canvas)?;

    Ok(())
}*/

fn capture_image() -> anyhow::Result<()> {
    let pixy = PixyCamera::init()?;
    let frame_rgb = pixy.get_frame()?;
    let mut frame = Mat::default();
    cvt_color_def(&frame_rgb, &mut frame, COLOR_RGB2BGR)?;
    imwrite_def("outputs/extra_11_side_view.jpg", &frame)?;

    Ok(())
}

fn save_properties(properties: &Properties) -> anyhow::Result<()> {
    static FRAME: Mutex<i32> = Mutex::new(0);
    *FRAME.lock().unwrap() += 1;
    let properties_json = serde_json::to_string(&properties)?;
    let mut output_file = File::create(format!("outputs/{}_properties.json", *FRAME.lock().unwrap()))?;
    output_file.write_all(properties_json.as_bytes())?;

    Ok(())
}