//! Build script that selects the correct linker script based on target.

use std::{env, fs::File, io::Write, path::PathBuf};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let out_dir = env::var_os("OUT_DIR");
    let out = &PathBuf::from(out_dir.ok_or("OUT_DIR environment variable is not set")?);

    let target = env::var("TARGET")?;

    let memory_x = match target.as_str() {
        "thumbv6m-none-eabi" => include_bytes!("memory_rp2040.x").as_slice(),
        "thumbv8m.main-none-eabihf" => include_bytes!("memory_rp2350.x").as_slice(),
        _ => panic!("Unsupported target: {target}"),
    };

    File::create(out.join("memory.x"))?.write_all(memory_x)?;
    println!("cargo:rustc-link-search={}", out.display());

    println!("cargo:rerun-if-changed=memory_rp2040.x");
    println!("cargo:rerun-if-changed=memory_rp2350.x");
    println!("cargo:rerun-if-changed=build.rs");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    //println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    Ok(())
}
