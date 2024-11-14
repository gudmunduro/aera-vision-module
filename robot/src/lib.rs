use std::{io::{Read, Write}, net::TcpStream};

use anyhow::Ok;
use feedback_data::FeedbackData;

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
            motion_cmd_stream: motion_conn,
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

    pub fn set_do(&mut self, index: i32, status: bool) -> anyhow::Result<()> {
        let status = status as i32;
        write!(&mut self.dashboard_cmd_stream, "DO({index}, {status})\n")?;

        Ok(())
    }
 
    pub fn mov_j(&mut self, x: f64, y: f64, z: f64, r: f64) -> anyhow::Result<()> {
        write!(&mut self.motion_cmd_stream, "MovJ({x}, {y}, {z}, {r})\n")?;

        Ok(())
    }
}

pub struct RobotFeedbackConn {
    feedback_conn: TcpStream,
}

impl RobotFeedbackConn {
    pub fn connect() -> anyhow::Result<RobotFeedbackConn> {
        let feedback_conn = TcpStream::connect("192.168.1.6:30004")?;
        Ok(RobotFeedbackConn { feedback_conn })
    }

    pub fn receive_feedback(&mut self) -> anyhow::Result<FeedbackData> {
        let mut buffer = [0u8; 1440];
        self.feedback_conn.read_exact(&mut buffer)?;
        let feedback_data = bincode::deserialize(&buffer)?;
        
        Ok(feedback_data)
    }
}