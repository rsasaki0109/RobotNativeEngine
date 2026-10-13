fn main() -> std::process::ExitCode {
    match motor_physical_collector::run_cli(std::env::args().skip(1)) {
        Ok(code) => std::process::ExitCode::from(code),
        Err(error) => {
            eprintln!("isolated physical motor collection failed: {error}");
            std::process::ExitCode::from(1)
        }
    }
}
