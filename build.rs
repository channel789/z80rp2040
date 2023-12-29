use std::process::Command;

fn main() {
    println!("cargo:rerun-if-changed=z80");
    println!("cargo:rerun-if-changed=memory.x");
    Command::new("make")
        .current_dir("z80")
        .status()
        .expect("failed to build z80");
}
