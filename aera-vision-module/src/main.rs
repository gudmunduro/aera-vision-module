use std::u64;

use aera::{properties::Properties, protobuf::{tcp_message, variable_description, DataMessage, ProtoVariable, VariableDescription}, AeraConn};
use opencv::imgcodecs::{self, IMREAD_COLOR};
use vision::{RecognizedArea, VisionSystem};

pub mod aera;

fn main() -> anyhow::Result<()> {
    let mut aera = AeraConn::connect("127.0.0.1")?;
    let mut vision = VisionSystem::new();

    let mut properties = Properties::new();
    loop {
        // TODO: Get frame from pixy

        // TODO: Pass frame to vision
        let objects = vision.process_frame(todo!())?;
        let cam_objs = [&mut properties.co1, &mut properties.co2, &mut properties.co3];
        for i in 0..objects.len().min(3) {
            let area = &objects[i].area;

            cam_objs[i].class = objects[i].class;
            cam_objs[i].position = (area.min + (area.max - area.min)).cast();
        }

        aera.send_properties(&properties)?;
    }
}
