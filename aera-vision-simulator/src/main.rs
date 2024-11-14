use std::{collections::VecDeque, thread::sleep, time::Duration};

use aera::{commands::Command, properties::Properties, AeraConn};
use nalgebra::{Vector2, Vector4};
use rand::{thread_rng, Rng};


fn main() -> anyhow::Result<()> {
    setup_logging();

    log::info!("Connecting to AERA");
    let mut aera = AeraConn::connect("192.168.72.143")?;
    let mut properties = Properties::new();
    log::debug!("Wating for start message");
    aera.wait_for_start_message()?;

    set_initial_state(&mut properties);

    let mut rng = thread_rng();

    let move_cmds = (0..100).into_iter()
        .map(|_| Command::Move(rng.gen_range(0..20), rng.gen_range(0..20), rng.gen_range(0..20), rng.gen_range(0..20)))
        .collect::<Vec<_>>();

    let mut forced_commands = VecDeque::from(move_cmds);

    log::info!("Starting main loop");
    loop {
        sleep(Duration::from_millis(500));

        let cmd_to_send = forced_commands.pop_front();

        log::debug!("Sending properties");
        aera.send_properties(&properties, cmd_to_send.as_ref())?;

        let cmd = if let Some(cmd) = cmd_to_send {
            log::debug!("Command executed by controller");
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
                properties.h.position = Vector4::new(x, y, z, r);
            }
            Command::Move(x, y, z, r) => {
                log::debug!("Got move (relative) command from AERA by {x}, {y}, {z}, {r}");
                let current_pos = properties.h.position;
                properties.h.position = Vector4::new(current_pos.x + x, current_pos.y + y, current_pos.z + z, current_pos.w + r);
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

fn set_initial_state(properties: &mut Properties) {
    properties.h.position = Vector4::new(240, -5, -6, 45);

    properties.co1.position = Vector2::new(200, 200);
    properties.co1.class = 0;
    properties.co1.size = 1;
}

fn setup_logging() {
    simple_log::quick!();
}