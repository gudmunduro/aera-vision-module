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
    set_initial_state(&mut properties, &sim_cube);

    let mut rng = thread_rng();

    let random_commands = (0..200).into_iter()
        .map(|_| gen_random_command(&mut rng))
        .collect::<Vec<_>>();

    let mut forced_commands = VecDeque::new();

    log::info!("Starting main loop");
    loop {
        sleep(Duration::from_millis(500));

        let cmd_to_send = forced_commands.pop_front();
        properties.co1.position = sim_cube.pos;

        log::debug!("Sending properties");
        aera.send_properties(&properties, cmd_to_send.as_ref())?;

        let cmd = if let Some(cmd) = cmd_to_send {
            log::debug!("Command injected by controller");
            cmd
        } else {
            log::debug!("Listening for command");
            match aera.listen_for_command() {
                Ok(cmd) => cmd,
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
                properties.h.position = Vector4::new(x, y, z, r);
                sim_cube.move_hand_by(properties.h.position.x - old_pos.x, properties.h.position.y - old_pos.y);
            }
            Command::Move(x, y, z, r) => {
                log::debug!("Got move (relative) command from AERA by {x}, {y}, {z}, {r}");
                let current_pos = properties.h.position;
                properties.h.position = Vector4::new(current_pos.x + x, current_pos.y + y, current_pos.z + z, current_pos.w + r);
                sim_cube.move_hand_by(x, y);
            }
            Command::Grab => {
                log::debug!("Got grab command from AERA");
                properties.h.holding = Some("co1".to_string());
            }
            Command::Release => {
                log::debug!("Got release command from AERA");
                properties.h.holding = None;
            }
        }

        aera.increase_timestamp();
    }
}

fn set_initial_state(properties: &mut Properties, sim_cube: &SimCube) {
    properties.h.position = Vector4::new(240, -40, -6, 55);

    properties.co1.position = sim_cube.pos;
    properties.co1.class = 0;
    properties.co1.size = 1;
}

fn setup_logging() {
    simple_log::quick!();
}

fn gen_random_command(rng: &mut ThreadRng) -> Command {
    let c = rng.gen_range(0..10);
    match c {
        1 => Command::Grab,
        2 => Command::Release,
        _ => Command::Move(rng.gen_range(0..20), rng.gen_range(0..20), rng.gen_range(0..20), rng.gen_range(0..20)),
    }
}