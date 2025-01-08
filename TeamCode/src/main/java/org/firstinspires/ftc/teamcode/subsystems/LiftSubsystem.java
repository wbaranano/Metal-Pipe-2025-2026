package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.PID.PIDWrapper;

@Config
public class LiftSubsystem extends SubsystemBase {
    public static double p = 0.035, i = 0.01, d = 0;
    public PIDWrapper pid;
    public boolean dampenRetraction = true;
    public liftState state = liftState.transfer;

    @Config
    public static class constants {
        public static class tick {
            public static int aboveBucket = 200;
            public static int specimenCollection = 0;
            public static int specimenDeposit = 800; // TODO: this was tuned to a pole that was too higher, adjust this yourself
            public static int specimenDepositSlam = 400;
            public static int highBasket = 2350;
            public static int intermediary = 600;
            public static int transfer = -10;
        }
        public static double rollBackwards = 0.4956;
        public static double rollForwards = 0.378;
        public static double retractExtension = 0.42;
        public static double extendExtension = 0.8;
        public static double clawClose = 0.725;
        public static double clawOpen = 0.525;
        public static double clawBigOpen = 0.475;
        public static liftPreset specimenCollectionPreset = new liftPreset(0.5172, rollForwards, 0.6594, retractExtension);
        public static liftPreset specimenDepositPrepPreset = new liftPreset(0.445, rollBackwards, 0.6, extendExtension);
        public static liftPreset transferPickupPreset = new liftPreset(0.485, rollBackwards, 0.516, retractExtension);
        public static liftPreset basketPreset = new liftPreset(0.5483, rollBackwards, 0.6194, retractExtension);
    }

    private boolean clawIsClosed = false;

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
        if (dampenRetraction && power > 0) power /= 2.0;

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

    public void setClawClosed(boolean closed) {
        clawIsClosed = closed;
        Robot.claw.setPosition(closed ? constants.clawClose : constants.clawOpen);
    }

    public void setClawClosed(boolean closed, boolean isBig) {
        clawIsClosed = closed;
        if (isBig) Robot.claw.setPosition(closed ? constants.clawClose : constants.clawBigOpen);
        else setClawClosed(closed);
    }

    public void toggleClaw() {
        setClawClosed(!clawIsClosed);
    }

    public boolean isClawClosed() {
        return clawIsClosed;
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

    public enum liftState {
        // transfer: in the bucket, ready to pick up a specimen
        // basket: extended up to the basket
        // clip: ready to pickup up specimen with clip from the wall
        // specimen: ready to clip specimen onto pole
        // intermediary: claw ready in correct position for transfer, but lift is slightly raised
        //      it should be raised high enough such that claw can move freely
        transfer, basket, wall, specimen, intermediary
    }
}
