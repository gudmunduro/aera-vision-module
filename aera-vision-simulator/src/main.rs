use std::{collections::VecDeque, fs, thread::sleep, time::Duration};
use itertools::Itertools;
use aera::{commands::Command, properties::Properties, AeraConn, CAM_OBJ_COUNT, MAX_SIFT_POINT_COUNT};
use nalgebra::{Vector2, Vector4};
use rand::{rngs::ThreadRng, thread_rng, Rng};
use simulated_cube::SimCube;
use vision::{SiftKeyPoint, VisionSystem};

pub mod simulated_cube;

fn main() -> anyhow::Result<()> {
    setup_logging();

    log::info!("Connecting to AERA");
    let mut properties = Properties::new(CAM_OBJ_COUNT, MAX_SIFT_POINT_COUNT);
    //let mut aera = AeraConn::connect("192.168.1.44", &properties.sift_clusters.iter().map(|kp| kp.name.as_str()).collect::<Vec<_>>())?;
    let mut aera = AeraConn::connect("192.168.1.44", &properties.cam_objs.keys().map(|k| k.as_str()).collect::<Vec<_>>())?;
    log::debug!("Wating for start message");
    aera.wait_for_start_message()?;

    let mut vision_system = VisionSystem::new();
    let mut sim_cube = SimCube::initial();
    set_initial_state(&mut properties, &mut sim_cube);

    //let mut forced_commands = VecDeque::from([]);
    let mut frame = 0;

    log::info!("Starting main loop");
    loop {
        sleep(Duration::from_millis(500));

        let cmd_to_send: Option<Command> = None;
        /*let cmd_to_send = forced_commands.pop_front();
        if sim_cube.visible {
            let mut co1 = properties.cam_objs.get_mut("co1").unwrap();
            co1.position = sim_cube.pos;
            co1.approximate_pos = sim_cube.approximte_pos;
            co1.class = 0;
            co1.color = 2;
        }
        else {
            let mut co1 = properties.cam_objs.get_mut("co1").unwrap();
            co1.position = Vector2::new(-1.0, -1.0);
            if properties.h.holding.is_some() {
                co1.approximate_pos = properties.h.position;
                co1.class = 0;
                co1.color = 2;
            }
            else {
                co1.approximate_pos = Vector4::new(-1.0, -1.0, -1.0, -1.0);
                co1.class = -1;
                co1.color = -1;
            }
        }*/

        frame += 1;

        // Update based on data from camera
        /*let objects = vision_system.process_frame(&vision_system.read_frame(&format!("./outputs/{frame}_frame.jpg")).unwrap())?;
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
            cam_obj.approximate_pos = calculate_predicted_grab_pos(&properties.h.position, &cam_obj.position);
            cam_obj.features = object.features.clone();
            log::debug!("Sending {co_key} pos ({}, {})", cam_obj.position.x, cam_obj.position.y);
        }*/

        /*properties.sift_clusters.iter_mut().for_each(|kp| kp.active = false);
        for (i, c) in clusters.into_iter().take(30).enumerate() {
            properties.sift_clusters[i].active = true;
            properties.sift_clusters[i].center = c.center.cast();
            properties.sift_clusters[i].approximate_pos = calculate_predicted_grab_pos(&properties.h.position, &c.center.cast());
            properties.sift_clusters[i].features = c.features;
            properties.sift_clusters[i].obj_type = c.class_id;
        }*/
        properties = serde_json::from_str(&fs::read_to_string(&format!("outputs/scenario_2_with_sift_v2_success/{frame}_properties.json"))?)?;

        log::debug!("Sending properties");
        aera.send_properties(&properties, cmd_to_send.as_ref())?;

        let cmd = if let Some(cmd) = cmd_to_send {
            log::debug!("Command injected by controller");
            cmd
        } else {
            log::debug!("Listening for command");
            match aera.listen_for_command() {
                Ok(Some(cmd)) => cmd,
                Ok(None) => {
                    log::error!("Timed out waiting for command from AERA");
                    continue;
                }
                Err(e) => {
                    log::error!("Error receiving command from AERA: {e}");
                    continue;
                }
            }
        };
        match cmd {
            Command::EnableRobot => {
                log::info!("Got enable_robot command from AERA");
            }
            Command::MovJ(x, y, z, r) => {
                log::info!("Got movj command from AERA to {x}, {y}, {z}, {r}");
                let old_pos = properties.h.position;
                properties.h.position = Vector4::new(x as f64, y as f64, z as f64, r as f64);
                sim_cube.move_hand(&(properties.h.position - old_pos), &properties.h.position);
            }
            Command::Move(x, y, z, r) => {
                log::info!("Got move (relative) command from AERA by {x}, {y}, {z}, {r}");
                let (x, y, z, r) = (x + random_noise(), y + random_noise(), z + random_noise(), r + random_noise());
                log::debug!("Moving by {x}, {y}, {z}, {r}");
                let current_pos = properties.h.position;
                properties.h.position = Vector4::new(current_pos.x + x, current_pos.y + y, current_pos.z + z, current_pos.w + r);
                sim_cube.move_hand(&Vector4::new(x, y, z, r), &properties.h.position);
            }
            Command::Grab => {
                log::info!("Got grab command from AERA");
                properties.h.holding = Some("co1".to_string());
                sim_cube.visible = false;
            }
            Command::Push => {

            }
            Command::Release => {
                log::info!("Got release command from AERA");

                properties.h.holding = None;
                sim_cube.visible = true;
            }
            Command::NoAction => {
                log::info!("Got no action command from AERA");
                sleep(Duration::from_secs(30));
            }
        }

        aera.increase_timestamp();
    }
}

fn set_initial_state(properties: &mut Properties, sim_cube: &mut SimCube) {
    properties.h.position = Vector4::new(200.0, 200.0, 0.0, 45.0);

    let mut co1 = properties.cam_objs.get_mut("co1").unwrap();
    co1.position = sim_cube.pos;
    co1.class = 0;
    co1.size = 1;

    sim_cube.move_hand(&Vector4::new(0.0, 0.0, 0.0, 0.0), &properties.h.position);
}

fn load_sift_frame(frame: i32) -> Option<Vec<SiftKeyPoint>> {
    log::debug!("Start of frame {frame}");
    let content = fs::read_to_string(&format!("outputs/{frame}_sift.json")).ok()?;
    serde_json::from_str(&content).unwrap()
}

fn setup_logging() {
    simple_log::quick!();
}

fn gen_random_command(rng: &mut ThreadRng) -> Command {
    let c = rng.gen_range(0..10);
    match c {
        1 => Command::Grab,
        2 => Command::Release,
        _ => Command::Move(rng.gen_range(0.0..20.0), rng.gen_range(0.0..20.0), rng.gen_range(0.0..20.0), rng.gen_range(0.0..20.0)),
    }
}

fn random_noise() -> f64 {
    let mut rng = thread_rng();
    rng.gen_range(0.0..0.2)
}

fn calculate_predicted_grab_pos(hand_pos: &Vector4<f64>, co_pos: &Vector2<f64>) -> Vector4<f64> {
    const CAM_GRAB_POS: Vector2<f64> = Vector2::new(230.0, 194.0);
    let pred_x = hand_pos.x + (CAM_GRAB_POS.y - co_pos.y);
    let pred_y = hand_pos.y + ((CAM_GRAB_POS.x - co_pos.x) / 1.175);
    let pred_z = -140_f64;
    let pred_w = 180_f64;

    Vector4::new(pred_x, pred_y, pred_z, pred_w)
}
