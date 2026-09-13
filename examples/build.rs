//! Build script that selects the correct linker script based on target.

use std::{env, fs::File, io::Write, path::PathBuf};

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let target = env::var("TARGET").unwrap();

    let memory_x = match target.as_str() {
        "thumbv6m-none-eabi" => include_bytes!("memory_rp2040.x").as_slice(),
        "thumbv8m.main-none-eabihf" => include_bytes!("memory_rp2350.x").as_slice(),
        _ => panic!("Unsupported target: {target}"),
    };

    File::create(out.join("memory.x")).unwrap().write_all(memory_x).unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    println!("cargo:rerun-if-changed=memory_rp2040.x");
    println!("cargo:rerun-if-changed=memory_rp2350.x");
    println!("cargo:rerun-if-changed=build.rs");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    //println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
