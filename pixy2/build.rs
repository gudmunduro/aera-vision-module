fn main() {
    cxx_build::bridge("src/lib.rs")
        .compiler("g++")
        .std("c++17")
        .include("/opt/homebrew/include/")
        .include("/opt/homebrew/include/libusb-1.0/")
        .include("/Users/gudmundur/Projects/HR/Thesis/aera-vision-module/pixy2/pixy2/src/host/libpixyusb2/include")
        .include("/Users/gudmundur/Projects/HR/Thesis/aera-vision-module/pixy2/pixy2/src/host/arduino/libraries/Pixy2")
        .include("/Users/gudmundur/Projects/HR/Thesis/aera-vision-module/pixy2/src")
        .object("/Users/gudmundur/Projects/HR/Thesis/aera-vision-module/pixy2/src/bridge.o")
        .compile("pixy2");
    
    println!("cargo:rustc-link-lib=usb-1.0");
    println!("cargo:rerun-if-changed=src/main.rs");
    println!("cargo:rerun-if-changed=bridge.cpp");
}