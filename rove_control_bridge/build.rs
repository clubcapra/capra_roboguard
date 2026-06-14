// Compile the udp_multiplexer front-door protos (same set the Steam Deck speaks):
// RoveControl (teleop), Mission, CameraSwitch — all package `telemetry`.
fn main() {
    // self-contained protoc (no system protobuf-compiler needed)
    if let Ok(p) = protoc_bin_vendored::protoc_bin_path() {
        std::env::set_var("PROTOC", p);
    }
    prost_build::Config::new()
        .compile_protos(
            &[
                "proto/RoveControl.proto",
                "proto/Mission.proto",
                "proto/CameraSwitch.proto",
                "proto/RoveTelemetry.proto",
                "proto/Estop.proto",
                "proto/IKEngineMessages.proto",
            ],
            &["."], // imports are `proto/core/...`, so the include root is the crate root
        )
        .expect("failed to compile front-door protobufs");
    println!("cargo:rerun-if-changed=proto");
}
