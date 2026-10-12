fn main() {
    match position_motor_calibration::run_cli(std::env::args().skip(1)) {
        Ok(code) => std::process::exit(i32::from(code)),
        Err(error) => {
            eprintln!("position-motor calibration: {error}");
            std::process::exit(1);
        }
    }
}
