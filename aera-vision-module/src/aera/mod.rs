use std::{collections::HashMap, io::Write, net::TcpStream};

use properties::{CameraObject, Properties};
use prost::Message as _;
use protobuf::{
    tcp_message, variable_description, CommandDescription, ProtoVariable, TcpMessage,
    VariableDescription,
};

pub mod protobuf {
    include!(concat!(env!("OUT_DIR"), "/tcp_io_device.rs"));
}
pub mod properties;

pub struct AeraConn {
    stream: TcpStream,
    comm_ids: CommIds,
    timestamp: u64
}

impl AeraConn {
    pub fn connect(aera_ip: &str) -> anyhow::Result<AeraConn> {
        let stream = TcpStream::connect(format!("{aera_ip}:8080"))?;
        let comm_ids = CommIds::from_list(&["c", "co1", "co2", "co3", "position", "size", "class"]);

        let mut aera_conn = AeraConn { stream, comm_ids, timestamp: 0 };
        aera_conn.send_setup_command()?;

        Ok(aera_conn)
    }

    fn send_tcp_message(&mut self, message: &TcpMessage) -> anyhow::Result<()> {
        let encoded = message.encode_to_vec();
        let size_bytes = (encoded.len() as u64).to_le_bytes();
        self.stream.write(&size_bytes)?;
        self.stream.write(&encoded)?;

        Ok(())
    }

    fn send_setup_command(&mut self) -> anyhow::Result<()> {
        let message = TcpMessage {
            message_type: tcp_message::Type::Setup as i32,
            message: Some(tcp_message::Message::SetupMessage(protobuf::SetupMessage {
                entities: HashMap::from([
                    ("c".to_string(), self.comm_ids.get("c")),
                    ("co1".to_string(), self.comm_ids.get("co1")),
                    ("co2".to_string(), self.comm_ids.get("co2")),
                    ("co3".to_string(), self.comm_ids.get("co3")),
                ]),
                objects: HashMap::from([
                    ("position".to_string(), self.comm_ids.get("position")),
                    ("size".to_string(), self.comm_ids.get("size")),
                    ("class".to_string(), self.comm_ids.get("class")),
                ]),
                commands: HashMap::new(),
                command_descriptions: Vec::new(),
            })),
            timestamp: 0,
        };
        self.send_tcp_message(&message)?;

        Ok(())
    }

    pub fn send_properties(&mut self, properties: &Properties) -> anyhow::Result<()> {
        let message = TcpMessage {
            message_type: tcp_message::Type::Data as i32,
            message: Some(tcp_message::Message::DataMessage(protobuf::DataMessage {
                variables: vec![
                    self.camera_object_properties("co1", &properties.co1),
                    self.camera_object_properties("co2", &properties.co2),
                    self.camera_object_properties("co3", &properties.co3)
                ].into_iter().flatten().collect(),
                time_span: 0,
            })),
            timestamp: self.timestamp,
        };
        self.send_tcp_message(&message)?;

        Ok(())
    }
    
    fn camera_object_properties(&self, name: &str, object: &CameraObject) -> Vec<ProtoVariable> {
        vec![
            ProtoVariable {
                meta_data: Some(VariableDescription {
                    entity_id: self.comm_ids.get(name),
                    id: self.comm_ids.get("position"),
                    data_type: variable_description::DataType::Int64 as i32,
                    dimensions: vec![2],
                    opcode_string_handle: "vec2".to_string(),
                }),
                data: object.position.as_slice().iter().flat_map(|v| v.to_le_bytes()).collect(),
            },
            ProtoVariable {
                meta_data: Some(VariableDescription {
                    entity_id: self.comm_ids.get(name),
                    id: self.comm_ids.get("class"),
                    data_type: variable_description::DataType::Int64 as i32,
                    dimensions: vec![1],
                    opcode_string_handle: "set".to_string(),
                }),
                data: object.class.to_le_bytes().to_vec(),
            },
        ]
    }
}

struct CommIds {
    id_map: HashMap<String, i32>,
}

impl CommIds {
    pub fn new() -> CommIds {
        CommIds {
            id_map: HashMap::new(),
        }
    }

    pub fn from_list(list: &[&str]) -> CommIds {
        CommIds {
            id_map: list
                .iter()
                .enumerate()
                .map(|(id, key)| (key.to_string(), id as i32))
                .collect(),
        }
    }

    pub fn get(&self, key: &str) -> i32 {
        *self.id_map.get(key).unwrap()
    }
}

fn le_bytes_to_f64(slice: &[u8]) -> f64 {
    let bytes: [u8; 8] = slice.try_into().expect("Incorrect slice length");
    f64::from_le_bytes(bytes)
}

fn le_bytes_to_u64(slice: &[u8]) -> u64 {
    let bytes: [u8; 8] = slice.try_into().expect("Incorrect slice length");
    u64::from_le_bytes(bytes)
}