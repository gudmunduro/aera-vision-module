use std::{collections::HashMap, io::{Read as _, Write}, net::TcpStream};

use anyhow::bail;
use commands::Command;
use properties::{CameraObject, HandObject, Properties};
use prost::Message as _;
use protobuf::{
    tcp_message, variable_description, CommandDescription, ProtoVariable, TcpMessage,
    VariableDescription,
};

pub mod protobuf {
    include!(concat!(env!("OUT_DIR"), "/tcp_io_device.rs"));
}
pub mod properties;
pub mod commands;

pub struct AeraConn {
    stream: TcpStream,
    comm_ids: CommIds,
    timestamp: u64
}

impl AeraConn {
    pub fn connect(aera_ip: &str) -> anyhow::Result<AeraConn> {
        let stream = TcpStream::connect(format!("{aera_ip}:8080"))?;
        let comm_ids = CommIds::from_list(&["h", "c", "co1", "co2", "co3", "position", "size", "obj_type", "mov_j", "enable_robot"]);

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
                    ("h".to_string(), self.comm_ids.get("h")),
                    ("c".to_string(), self.comm_ids.get("c")),
                    ("co1".to_string(), self.comm_ids.get("co1")),
                    ("co2".to_string(), self.comm_ids.get("co2")),
                    ("co3".to_string(), self.comm_ids.get("co3")),
                ]),
                objects: HashMap::from([
                    ("position".to_string(), self.comm_ids.get("position")),
                    ("size".to_string(), self.comm_ids.get("size")),
                    ("obj_type".to_string(), self.comm_ids.get("obj_type")),
                ]),
                commands: HashMap::from([
                    ("mov_j".to_string(), self.comm_ids.get("mov_j")),
                    ("enable_robot".to_string(), self.comm_ids.get("enable_robot"))
                ]),
                command_descriptions: vec![
                    CommandDescription {
                        // Params: [x, y, z, r (as deg)]
                        description: Some(VariableDescription {
                            entity_id: self.comm_ids.get("h"),
                            id: self.comm_ids.get("mov_j"),
                            data_type: variable_description::DataType::Int64 as i32,
                            dimensions: vec![4],
                            opcode_string_handle: "vec4".to_string(),
                        }),
                        name: "mov_j".to_string(),
                    },
                    CommandDescription {
                        description: Some(VariableDescription {
                            entity_id: self.comm_ids.get("h"),
                            id: self.comm_ids.get("enable_robot"),
                            data_type: variable_description::DataType::CommunicationId as i32,
                            dimensions: vec![0],
                            opcode_string_handle: String::new(),
                        }),
                        name: "enable_robot".to_string(),
                    },
                ],
            })),
            timestamp: 0,
        };
        self.send_tcp_message(&message)?;

        Ok(())
    }

    pub fn wait_for_start_message(&mut self) -> anyhow::Result<()> {
        let message = self.listen_for_message()?;
        if message.message_type == tcp_message::Type::Start as i32 {
            Ok(())
        } else {
            Err(anyhow::anyhow!("Received wrong message while waiting for start message"))
        }
    }

    pub fn send_properties(&mut self, properties: &Properties) -> anyhow::Result<()> {
        let message = TcpMessage {
            message_type: tcp_message::Type::Data as i32,
            message: Some(tcp_message::Message::DataMessage(protobuf::DataMessage {
                variables: [
                    self.camera_object_properties("co1", &properties.co1),
                    self.camera_object_properties("co2", &properties.co2),
                    self.camera_object_properties("co3", &properties.co3),
                    self.hand_object_properties("h", &properties.h),
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
                data: object.position.iter().flat_map(|v| v.to_le_bytes()).collect(),
            },
            ProtoVariable {
                meta_data: Some(VariableDescription {
                    entity_id: self.comm_ids.get(name),
                    id: self.comm_ids.get("obj_type"),
                    data_type: variable_description::DataType::Int64 as i32,
                    dimensions: vec![1],
                    opcode_string_handle: "set".to_string(),
                }),
                data: object.class.to_le_bytes().to_vec(),
            },
        ]
    }

    fn hand_object_properties(&self, name: &str, object: &HandObject) -> Vec<ProtoVariable> {
        vec![
            ProtoVariable {
                meta_data: Some(VariableDescription {
                    entity_id: self.comm_ids.get(name),
                    id: self.comm_ids.get("position"),
                    data_type: variable_description::DataType::Int64 as i32,
                    dimensions: vec![4],
                    opcode_string_handle: "vec4".to_string(),
                }),
                data: object.position.iter().flat_map(|v| v.to_le_bytes()).collect(),
            },
        ]
    }

    fn listen_for_message(&mut self) -> anyhow::Result<TcpMessage> {
        let mut size_buf = vec![0; 8];
        self.stream.read_exact(&mut size_buf[0..8])?;
        let size = le_bytes_to_u64(&size_buf[..]);

        let mut data_buf = vec![0; size as usize];
        self.stream.read_exact(&mut data_buf[..])?;

        Ok(protobuf::TcpMessage::decode(data_buf.as_slice())?)
    }

    pub fn listen_for_command(&mut self) -> anyhow::Result<Command> {
        let message = self.listen_for_message()?;

        let dm = match message.message {
            Some(tcp_message::Message::DataMessage(dm)) => dm,
            _ => {
                bail!("Received message of type {}. Not a data message", message.message_type.to_string())
            }
        };

        let command_var = dm
            .variables
            .iter()
            .next()
            .ok_or(anyhow::anyhow!("Empty data message"))?;
        let meta = command_var
            .meta_data
            .as_ref()
            .ok_or(anyhow::anyhow!("Missing metadata in cmd"))?;

        let res_cmd = if meta.id == self.comm_ids.get("mov_j") {
            Command::MovJ(
                le_bytes_to_i64(&command_var.data[0..8]),
                le_bytes_to_i64(&command_var.data[8..16]),
                le_bytes_to_i64(&command_var.data[16..24]),
                le_bytes_to_i64(&command_var.data[24..32]),
            )
        } else if meta.id == self.comm_ids.get("enable_robot") {
            Command::EnableRobot
        } else {
            bail!("Invalid cmd with id {}", meta.id)
        };

        Ok(res_cmd)
    }

    pub fn increase_timestamp(&mut self) {
        self.timestamp += 100;
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
                .map(|(id, key)| (key.to_string(), id as i32+1))
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

fn le_bytes_to_i64(slice: &[u8]) -> i64 {
    let bytes: [u8; 8] = slice.try_into().expect("Incorrect slice length");
    i64::from_le_bytes(bytes)
}