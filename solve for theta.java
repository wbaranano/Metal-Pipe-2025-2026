import java.util.Scanner;

public class ProjectileSolver {

    public static void main(String[] args) {
        Scanner scanner = new Scanner(System.in);

        double v = readInput(scanner, "Enter launch velocity v (m/s): ");
        double dx = readInput(scanner, "Enter horizontal distance Δx to the goal (m): ");
        double dy = readInput(scanner, "Enter vertical displacement Δy (m): ");
        double g = 9.8; // gravity (m/s²)

        double tol = 1e-6;
        int maxIter = 1000;

        double low = 0.01; // Avoid 0 to prevent tan(0)
        double high = Math.PI / 2 - 0.01; // Just below 90°
        double theta = 0.0;

        for (int i = 0; i < maxIter; i++) {
            theta = (low + high) / 2.0;

            double f = 2 * v * v * dy * Math.pow(Math.cos(theta), 2)
         - v * v * dx * Math.tan(theta)
         - g * dx * dx;


            if (Math.abs(f) < tol) {
                break;
            }

            double fLow = 2 * v * v * dy * Math.pow(Math.cos(low), 2)
                        - v * v * dx * Math.tan(low)
                        - g * dx * dx;

            if (fLow * f < 0) {
                high = theta;
            } else {
                low = theta;
            }
        }

        double thetaDegrees = Math.toDegrees(theta);
        System.out.printf("Solution: theta = %.6f radians (%.6f degrees)%n", theta, thetaDegrees);

        scanner.close();
    }

    private static double readInput(Scanner scanner, String prompt) {
        while (true) {
            System.out.print(prompt);
            String input = scanner.nextLine();
            try {
                return Double.parseDouble(input.trim());
            } catch (NumberFormatException e) {
                System.out.println("Invalid input, please enter a number.");
            }
        }
    }
}
