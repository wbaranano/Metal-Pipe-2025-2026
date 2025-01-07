package org.firstinspires.ftc.teamcode.PID;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class PIDWrapper {
    private final double max;
    private final PIDController pid;
    private final PIDRead read;
    private final PIDWrite write;
    private final int offset;

    private double target, power;
    private int timeoutThres = 1000, lastTick;
    private long lastTime, timeout;

    public PIDWrapper(double p, double i, double d, double max, double tolerance, PIDRead read, PIDWrite write, boolean negative) {
        this.max = max;
        pid = new PIDController(p, i, d);
        pid.setTolerance(tolerance);

        this.write = write;
        this.read = read;

        this.offset = negative ? -1 : 1;
    }

    public void setPID(double p, double i, double d) {
        pid.setPID(p, i, d);
    }

    public void setTarget(double target) {
        pid.reset();
        this.timeout = 0;
        this.target = target;
    }

    public void update() {
        double tick = offset * read.read();
        power = pid.calculate(target, tick);
        power = Math.signum(power) * Math.min(Math.abs(power), max);

        write.write(Range.clip(power, -1, 1));

        // sometimes we cant get to a position, in that case give up after enough time has elapsed
        if (!pid.atSetPoint() && timeoutThres != -1) {
            long t = System.currentTimeMillis();
            if (Math.floor(tick) == lastTick) {
                timeout += t - lastTime;
                if (timeout > timeoutThres) setTarget(tick);
            } else timeout = 0;
        }

        lastTick = (int) tick;
        lastTime = System.currentTimeMillis();
    }

    public void increment(int amt) {
        this.setTarget(target + amt);
    }

    public void log(String name) {
        Robot.telemetry.addLine("=== " + name.toUpperCase() + " PID ===");
        Robot.telemetry.addData(name + " tick", offset * read.read());
        Robot.telemetry.addData(name + " target", target);
        Robot.telemetry.addData(name + " power", power);
        Robot.telemetry.addData(name + " is done", done());
        Robot.telemetry.addData(name + " timeout", timeout);
        Robot.telemetry.addLine();
    }

    public void setTimeoutThres(int t)  {
        timeoutThres = t;
    }

    public boolean done() {
        if (pid == null) return true;
        return pid.atSetPoint();
    }

}
