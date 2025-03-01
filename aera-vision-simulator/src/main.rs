use std::{collections::VecDeque, thread::sleep, time::Duration};

use aera::{commands::Command, properties::Properties, AeraConn};
use nalgebra::{Vector2, Vector4};
use rand::{rngs::ThreadRng, thread_rng, Rng};
use simulated_cube::SimCube;

pub mod simulated_cube;

fn main() -> anyhow::Result<()> {
    setup_logging();

    log::info!("Connecting to AERA");
    let mut aera = AeraConn::connect("127.0.0.1")?;
    let mut properties = Properties::new();
    log::debug!("Wating for start message");
    aera.wait_for_start_message()?;

    let mut sim_cube = SimCube::initial();
    set_initial_state(&mut properties, &mut sim_cube);

    let mut forced_commands = VecDeque::from([]);

    log::info!("Starting main loop");
    loop {
        sleep(Duration::from_millis(500));

        let cmd_to_send = forced_commands.pop_front();
        if sim_cube.visible {
            properties.co1.position = sim_cube.pos;
            properties.co1.approximate_pos = sim_cube.approximte_pos;
            properties.co1.class = 0;
        }
        else {
            properties.co1.position = Vector2::new(-1, -1);
            if properties.h.holding.is_some() {
                properties.co1.approximate_pos = properties.h.position;
                properties.co1.class = 0;
            }
            else {
                properties.co1.approximate_pos = Vector4::new(-1.0, -1.0, -1.0, -1.0);
                properties.co1.class = -1;
            }
        }

        log::debug!("Holding {}", properties.h.holding.clone().unwrap_or("Nothing".to_owned()));
        let hp = properties.h.position;
        log::debug!("Hand position ({}, {}, {}, {})", hp.x, hp.y, hp.z, hp.w);
        let ap = properties.co1.approximate_pos;
        log::debug!("Cam obj (co1) pos: ({}, {}, {}, {})", ap.x, ap.y, ap.z, ap.w);

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
                log::debug!("Got enable_robot command from AERA");
            }
            Command::MovJ(x, y, z, r) => {
                log::debug!("Got movj command from AERA to {x}, {y}, {z}, {r}");
                let old_pos = properties.h.position;
                properties.h.position = Vector4::new(x as f64, y as f64, z as f64, r as f64);
                sim_cube.move_hand(&(properties.h.position - old_pos), &properties.h.position);
            }
            Command::Move(x, y, z, r) => {
                log::debug!("Got move (relative) command from AERA by {x}, {y}, {z}, {r}");
                let (x, y, z, r) = (x + random_noise(), y + random_noise(), z + random_noise(), r + random_noise());
                log::debug!("Moving by {x}, {y}, {z}, {r}");
                let current_pos = properties.h.position;
                properties.h.position = Vector4::new(current_pos.x + x, current_pos.y + y, current_pos.z + z, current_pos.w + r);
                sim_cube.move_hand(&Vector4::new(x, y, z, r), &properties.h.position);
            }
            Command::Grab => {
                log::debug!("Got grab command from AERA");
                properties.h.holding = Some("co1".to_string());
                sim_cube.visible = false;
            }
            Command::Release => {
                log::debug!("Got release command from AERA");
                properties.h.holding = None;
                properties.co1.approximate_pos.z = -140.0;
                sim_cube.visible = true;
            }
        }

        aera.increase_timestamp();
    }
}

fn set_initial_state(properties: &mut Properties, sim_cube: &mut SimCube) {
    properties.h.position = Vector4::new(240.0, 0.0, 0.0, 45.0);

    properties.co1.position = sim_cube.pos;
    properties.co1.class = 0;
    properties.co1.size = 1;

    sim_cube.move_hand(&Vector4::new(0.0, 0.0, 0.0, 0.0), &properties.h.position);
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