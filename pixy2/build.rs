fn main() {
    cxx_build::bridge("src/lib.rs")
        .compiler("g++")
        .include("/usr/include/libusb-1.0")
        .include("/home/gudmundur/Projects/HR/Thesis/aera-vision-module/pixy2/pixy2/src/host/libpixyusb2/include")
        .include("/home/gudmundur/Projects/HR/Thesis/aera-vision-module/pixy2/pixy2/src/host/arduino/libraries/Pixy2")
        .include("/home/gudmundur/Projects/HR/Thesis/aera-vision-module/pixy2/src")
        .object("/home/gudmundur/Projects/HR/Thesis/aera-vision-module/pixy2/src/bridge.o")
        .compile("pixy2");
    
    println!("cargo:rustc-link-lib=usb-1.0");
    println!("cargo:rerun-if-changed=src/main.rs");
    println!("cargo:rerun-if-changed=bridge.cpp");
}