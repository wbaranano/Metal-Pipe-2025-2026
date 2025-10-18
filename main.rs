use std::io::{self, Write};

fn main() {
    // Helper function to read a float from user
    fn read_input(prompt: &str) -> f64 {
        loop {
            print!("{}", prompt);
            io::stdout().flush().unwrap();
            let mut input = String::new();
            io::stdin().read_line(&mut input).unwrap();
            match input.trim().parse::<f64>() {
                Ok(val) => return val,
                Err(_) => println!("Invalid input, please enter a number."),
            }
        }
    }

    let g = 9.8;  // gravity (m/s²)
    let v = read_input("Enter launch velocity v (m/s): ");
    let dx = read_input("Enter horizontal distance Δx to the goal (m): ");
    let dy = read_input("Enter vertical displacement Δy (m): ");

    let tol = 1e-6;
    let max_iter = 1000;

    // Bisection method to solve for theta
    let mut low = 0.01_f64; // avoid zero to prevent tan(0)
    let mut high = std::f64::consts::FRAC_PI_2 - 0.01; // just below 90°
    let mut theta = 0.0;

    for _ in 0..max_iter {
        theta = (low + high) / 2.0;

        // Original transcendental function
        let f = 2.0 * v*v * dy * theta.cos().powi(2) - v*v * dx * theta.tan() - g * dx;

        if f.abs() < tol {
            break;
        }

        let f_low = 2.0 * v*v * dy * low.cos().powi(2) - v*v * dx * low.tan() - g * dx;

        if f_low * f < 0.0 {
            high = theta;
        } else {
            low = theta;
        }
    }

    println!("Solution: theta ≈ {:.4} radians ≈ {:.2} degrees", theta, theta.to_degrees());
}
