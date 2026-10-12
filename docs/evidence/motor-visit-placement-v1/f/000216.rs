fn main() -> std::process::ExitCode {
    match motor_visit_placement_collector::run_cli(std::env::args().skip(1)) {
        Ok(code) => std::process::ExitCode::from(code),
        Err(error) => {
            eprintln!("motor visitation-placement collection failed: {error}");
            std::process::ExitCode::from(1)
        }
    }
}
