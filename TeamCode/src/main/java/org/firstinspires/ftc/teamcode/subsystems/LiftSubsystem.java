package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.PID.PIDWrapper;

@Config
public class LiftSubsystem extends SubsystemBase {
    public static double p = 0.03, i = 0.01, d = 0;
    public PIDWrapper pid;
    public boolean dampenRetraction = true;

    @Config
    public static class constants {
        public static int aboveBucket = 200;
        public static int specimenCollection = 0;
        public static int specimenDeposit = 1200; // TODO: this was tuned to a pole that was too higher, adjust this yourself
        public static double rollForwards = 0.4956;
        public static double rollBackwards = 0.378;
        public static double retractExtension = 0.26;
        public static double extendExtension = 0.62;
        public static double clawClose = 0.725;
        public static double clawOpen = 0.5;
        public static int specimenDepositLift = 400;
        public static liftPreset specimenCollectionPreset = new liftPreset(0.52, rollBackwards, 0.62, retractExtension);
        public static liftPreset specimenDepositPrepPreset = new liftPreset(0.5, rollForwards, 0.54, extendExtension);
    }

    public LiftSubsystem() {
        this.pid = new PIDWrapper(
            p, i, d, 1, 10,
            Robot.liftEncoder::getCurrentPosition, this::setPower,
            false
        );
    }

    @Override
    public void periodic(){
        pid.setPID(p, i, d);
        pid.update();
        pid.log("Lift");
    }

    public void setTick(int tick) {
        pid.setTarget(tick);
    }

    public void setPower(double power){
        // positive power means we are going downwards
        if (dampenRetraction && power > 0) power /= 4.0;

        Robot.liftFront.setPower(power);
        Robot.liftBack.setPower(-power);
    }

    public void setPitch(double p) {
        Robot.clawPitch.setPosition(p);
    }

    public void setRoll(double p) {
        Robot.clawRoll.setPosition(p);
    }

    public void setExtension(double p) {
        Robot.liftHorzExtFront.setPosition(p);
        Robot.liftHorzExtBack.setPosition(p);
    }

    public void setPivot(double p) {
        Robot.armPivotBack.setPosition(p);
        Robot.armPivotFront.setPosition(p);
    }

    public void setClaw(boolean closed) {
        Robot.claw.setPosition(closed ? constants.clawClose : constants.clawOpen);
    }

    public void apply(liftPreset preset) {
        setPitch(preset.pitch);
        setRoll(preset.roll);
        setExtension(preset.extension);
        setPivot(preset.pivot);
    }

    public static class liftPreset {
        public double pitch, roll, pivot, extension;
        public liftPreset(double pitch, double roll, double pivot, double extension) {
            this.pitch = pitch;
            this.roll = roll;
            this.pivot = pivot;
            this.extension = extension;
        }
    }
}
