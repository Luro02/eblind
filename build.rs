fn main() {
    println!("cargo:rerun-if-changed=.env");
    embuild::espidf::sysenv::output();
}
