use rustc_version::{Channel, version_meta};

fn main() {
    // Tell Cargo that 'rp' is an expected custom cfg flag name so it doesn't warn us
    println!("cargo::rustc-check-cfg=cfg(rp)");
    // println!("cargo::rustc-check-cfg=cfg(nightly)");

    // ----------------------------------------------------
    // Compiler Channel & SIMD Verification
    // ----------------------------------------------------

    // Check if the "simd" feature is enabled
    let simd_enabled = std::env::var_os("CARGO_FEATURE_SIMD").is_some();

    // Check the current compiler channel
    let channel = version_meta().unwrap().channel;

    if simd_enabled && channel != Channel::Nightly {
        panic!(
            "\n\nError: The 'simd' feature requires a nightly compiler.\n\
            Please use\n\
            \t'rustup run nightly cargo build --features simd'\n\
            or set\n\
            \t'rustup default nightly'.\n"
        );
    }

    // Optional: Emit a custom cfg flag if you want to use it in your code
    if channel == Channel::Nightly {
        println!("cargo:rustc-cfg=nightly");
    }

    // ----------------------------------------------------
    // RP Target Chip Selection & CFG Generation
    // ----------------------------------------------------

    // Gather all active RP chip features from Cargo's environment variables
    let has_rp2040 = std::env::var_os("CARGO_FEATURE_RP2040").is_some();
    let has_rp235xa = std::env::var_os("CARGO_FEATURE_RP235XA").is_some();
    let has_rp235xb = std::env::var_os("CARGO_FEATURE_RP235XB").is_some();

    // Count how many chips are selected
    let count = [has_rp2040, has_rp235xa, has_rp235xb].iter().filter(|&&active| active).count();

    // Safety Check: Prevent multiple mutually exclusive chips from building
    if count > 1 {
        panic!(
            "\n\nError: Multiple RP chip features were enabled simultaneously!\n\
             Please select exactly one target chip feature.\n"
        );
    }

    // Emit the unified 'rp' config flag if exactly one chip is chosen
    if count == 1 {
        println!("cargo:rustc-cfg=rp");
    }
}
