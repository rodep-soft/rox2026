use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let cpp_dir = manifest_dir.join("cpp");

    let include_dir = cpp_dir.join("include");
    let src_bno055 = cpp_dir.join("src").join("bno055.cpp");
    let src_bno055_c = cpp_dir.join("src").join("bno055_c.cpp");

    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .include(&include_dir)
        .file(&src_bno055)
        .file(&src_bno055_c)
        .compile("bno055-linux");

    println!("cargo:rerun-if-changed={}", include_dir.join("libbno055-linux").join("bno055_c.h").display());
    println!("cargo:rerun-if-changed={}", include_dir.join("libbno055-linux").join("bno055.hpp").display());
    println!("cargo:rerun-if-changed={}", src_bno055.display());
    println!("cargo:rerun-if-changed={}", src_bno055_c.display());
}
