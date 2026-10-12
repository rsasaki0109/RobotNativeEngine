fn main() -> std::process::ExitCode {
    match isolated_motor_construction_collector::run_cli(std::env::args().skip(1)) {
        Ok(code) => std::process::ExitCode::from(code),
        Err(error) => {
            eprintln!("isolated motor construction collection failed: {error}");
            std::process::ExitCode::from(1)
        }
    }
}
