use std::{io::Write, net::TcpStream};

pub mod feedback_data;

pub struct RobotConn {
    dashboard_cmd_stream: TcpStream,
    motion_cmd_stream: TcpStream,
}

impl RobotConn {

    pub fn connect() -> anyhow::Result<RobotConn> {
        let dasboard_conn = TcpStream::connect("192.168.1.6:29999")?;
        let motion_conn = TcpStream::connect("192.168.1.6:30003")?;

        Ok(RobotConn {
            dashboard_cmd_stream: dasboard_conn,
            motion_cmd_stream: motion_conn 
        })
    }

    pub fn enable_robot(&mut self) -> anyhow::Result<()> {
        write!(&mut self.dashboard_cmd_stream, "EnableRobot()\n")?;

        Ok(())
    }

    pub fn disable_robot(&mut self) -> anyhow::Result<()> { 
        write!(&mut self.dashboard_cmd_stream, "DisableRobot()\n")?;

        Ok(())
    }
 
    pub fn mov_j(&mut self, x: f64, y: f64, z: f64, r: f64) -> anyhow::Result<()> {
        write!(&mut self.motion_cmd_stream, "MovJ({x}, {y}, {z}, {r})\n")?;

        Ok(())
    }
}