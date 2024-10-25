fn main() {
    cxx_build::bridge("src/lib.rs")
        .compiler("g++")
        .include("/usr/include/libusb-1.0")
        .include("/home/gudmundur/Projects/Experiments/pixy2_rust/pixy2/src/host/libpixyusb2/include")
        .include("/home/gudmundur/Projects/Experiments/pixy2_rust/pixy2/src/host/arduino/libraries/Pixy2")
        .include("/home/gudmundur/Projects/Experiments/pixy2_rust/src")
        .object("/home/gudmundur/Projects/Experiments/pixy2_rust/src/bridge.o")
        .compile("pixy2_rust");
    
    println!("cargo:rustc-link-lib=usb-1.0");
    println!("cargo:rerun-if-changed=src/main.rs");
    println!("cargo:rerun-if-changed=bridge.cpp");
}