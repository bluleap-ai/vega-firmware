fn main() {
    println!("cargo:rustc-link-search=libs");
    println!("cargo:rustc-link-lib=static=oaq_2nd_gen");
    println!("cargo:rustc-link-arg-bins=-Tlinkall.x");
    println!("cargo:rustc-link-arg=-Trom_functions.x");
}
