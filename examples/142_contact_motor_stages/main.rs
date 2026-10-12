fn main() {
    match contact_motor_stages::run_cli(std::env::args().skip(1)) {
        Ok(code) => std::process::exit(i32::from(code)),
        Err(error) => {
            eprintln!("contact motor stages: {error}");
            std::process::exit(1);
        }
    }
}
