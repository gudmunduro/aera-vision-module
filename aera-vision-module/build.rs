use std::io::Result;

fn main() -> Result<()> {
    prost_build::compile_protos(&["src/tcp_data_message.proto"], &["src/"])?;
    Ok(())
}