package org.firstinspires.ftc.teamcode.PID;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class PIDWrapper {
    private final double max;
    private final PIDController pid;
    private final PIDRead read;
    private final PIDWrite write;

    private double target, power;

    public PIDWrapper(double p, double i, double d, double max, double tolerance, PIDRead read, PIDWrite write) {
        this.max = max;
        pid = new PIDController(p, i, d);
        pid.setTolerance(tolerance);

        this.write = write;
        this.read = read;
    }

    public void setTarget(double target) {
        // TODO: can replace with just pid.setSetPoint?
        pid.reset();
        this.target = target;
    }

    public void update() {
        power = pid.calculate(target, read.read());
        power = Math.signum(power) * Math.min(Math.abs(power), max);

        write.write(Range.clip(power, -1, 1));
    }

    public void log(String name) {
        Robot.telemetry.addLine("=== " + name.toUpperCase() + " PID ===");
        Robot.telemetry.addData(name + "tick", read.read());
        Robot.telemetry.addData(name + "target", target);
        Robot.telemetry.addData(name + "power", power);
        Robot.telemetry.addData(name + "is done", done());
        Robot.telemetry.addLine();
    }

    public boolean done() {
        return pid.atSetPoint();
    }

}
