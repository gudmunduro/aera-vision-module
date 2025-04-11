use std::{collections::VecDeque, thread::sleep, time::Duration};

use aera::{commands::Command, properties::Properties, AeraConn, CAM_OBJ_COUNT, MAX_SIFT_POINT_COUNT};
use nalgebra::{Vector2, Vector4};
use rand::{rngs::ThreadRng, thread_rng, Rng};
use simulated_cube::SimCube;

pub mod simulated_cube;

fn main() -> anyhow::Result<()> {
    setup_logging();

    log::info!("Connecting to AERA");
    let mut properties = Properties::new(CAM_OBJ_COUNT, MAX_SIFT_POINT_COUNT);
    let mut aera = AeraConn::connect("192.168.1.44", &properties.sift_keypoints.iter().map(|kp| kp.name.as_str()).collect::<Vec<_>>())?;
    //let mut aera = AeraConn::connect("127.0.0.1", &properties.cam_objs.keys().map(|k| k.as_str()).collect::<Vec<_>>())?;
    log::debug!("Wating for start message");
    aera.wait_for_start_message()?;

    let mut sim_cube = SimCube::initial();
    set_initial_state(&mut properties, &mut sim_cube);
    set_demo_sift_points(&mut properties);

    let mut forced_commands = VecDeque::from([]);

    log::info!("Starting main loop");
    loop {
        sleep(Duration::from_millis(500));

        let cmd_to_send = forced_commands.pop_front();
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
        }

        {
            let mut co1 = properties.cam_objs.get_mut("co1").unwrap();

            log::debug!("Holding {}", properties.h.holding.clone().unwrap_or("Nothing".to_owned()));
            let hp = properties.h.position;
            log::debug!("Hand position ({}, {}, {}, {})", hp.x, hp.y, hp.z, hp.w);
            let ap = co1.approximate_pos;
            log::debug!("Cam obj (co1) pos: ({}, {}, {}, {})", ap.x, ap.y, ap.z, ap.w);
            let cp = co1.position;
            log::debug!("Cam obj (co1) cam pos: ({}, {})", cp.x, cp.y);
        }

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
                properties.h.holding = Some("sift1".to_string());
                sim_cube.visible = false;
            }
            Command::Release => {
                log::info!("Got release command from AERA");

                properties.h.holding = None;
                sim_cube.visible = true;
            }
            Command::NoAction => {
                log::info!("Got no action command from AERA");
                sleep(Duration::from_secs(10));
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

fn set_demo_sift_points(properties: &mut Properties) {
    properties.sift_keypoints[0].detected = true;
    properties.sift_keypoints[0].point = Vector2::new(200.88758850097656, 268.90301513671875);
    properties.sift_keypoints[0].feature_vec = vec![38.00, 26.00, 3.00, 1.00, 0.00, 0.00, 2.00, 6.00, 10.00, 2.00, 1.00, 1.00, 1.00, 4.00, 42.00, 32.00, 0.00, 0.00, 1.00, 7.00, 20.00, 74.00, 46.00, 6.00, 14.00, 4.00, 4.00, 6.00, 10.00, 50.00, 11.00, 3.00, 67.00, 4.00, 0.00, 0.00, 0.00, 2.00, 19.00, 30.00, 129.00, 19.00, 2.00, 2.00, 3.00, 18.00, 93.00, 129.00, 14.00, 6.00, 3.00, 17.00, 113.00, 129.00, 129.00, 45.00, 1.00, 0.00, 0.00, 5.00, 73.00, 93.00, 4.00, 0.00, 41.00, 14.00, 0.00, 1.00, 2.00, 4.00, 9.00, 5.00, 129.00, 129.00, 80.00, 15.00, 4.00, 5.00, 5.00, 29.00, 17.00, 50.00, 129.00, 129.00, 129.00, 39.00, 8.00, 9.00, 0.00, 0.00, 2.00, 92.00, 91.00, 9.00, 0.00, 0.00, 5.00, 11.00, 13.00, 5.00, 1.00, 0.00, 0.00, 0.00, 6.00, 51.00, 51.00, 11.00, 1.00, 0.00, 0.00, 0.00, 0.00, 8.00, 72.00, 83.00, 5.00, 2.00, 1.00, 0.00, 0.00, 0.00, 4.00, 49.00, 27.00, 9.00, 1.00, 0.00];

    properties.sift_keypoints[1].detected = true;
    properties.sift_keypoints[1].point = Vector2::new(327.4416809082031, 148.53501892089844);
    properties.sift_keypoints[1].feature_vec = vec![109.00, 75.00, 0.00, 0.00, 0.00, 0.00, 0.00, 2.00, 51.00, 30.00, 1.00, 0.00, 6.00, 20.00, 15.00, 22.00, 0.00, 0.00, 1.00, 4.00, 26.00, 81.00, 29.00, 3.00, 1.00, 4.00, 11.00, 7.00, 5.00, 10.00, 41.00, 11.00, 120.00, 33.00, 0.00, 0.00, 0.00, 0.00, 0.00, 41.00, 120.00, 41.00, 10.00, 5.00, 7.00, 26.00, 39.00, 106.00, 7.00, 7.00, 11.00, 49.00, 120.00, 117.00, 35.00, 16.00, 3.00, 19.00, 23.00, 31.00, 49.00, 4.00, 0.00, 0.00, 65.00, 5.00, 0.00, 0.00, 0.00, 18.00, 105.00, 120.00, 79.00, 104.00, 81.00, 25.00, 7.00, 36.00, 18.00, 20.00, 3.00, 19.00, 80.00, 120.00, 99.00, 18.00, 6.00, 1.00, 9.00, 5.00, 2.00, 29.00, 32.00, 3.00, 10.00, 14.00, 0.00, 0.00, 2.00, 33.00, 56.00, 120.00, 120.00, 21.00, 0.00, 2.00, 30.00, 53.00, 43.00, 106.00, 15.00, 0.00, 0.00, 5.00, 22.00, 21.00, 12.00, 44.00, 29.00, 0.00, 0.00, 1.00, 1.00, 1.00, 2.00, 15.00, 38.00, 8.00];

    for i in 2..properties.sift_keypoints.len() {
        properties.sift_keypoints[i].detected = false;
    }
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