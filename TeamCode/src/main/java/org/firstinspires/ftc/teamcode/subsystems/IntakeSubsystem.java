package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.PID.PIDWrapper;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public WRIST wristPos;
    public COLOR colorDetected = COLOR.blank;
    public float[] colorHSV = {0f, 0f, 0f};
    public COLOR teamColor;
    public COLOR enemyColor;
    public PICKUP_MODE mode = PICKUP_MODE.auto;

    @Config
    public static class constants {
        public static int fullIntakeOutTick = 600;
        public static int intakeInTick = -15;
        public static int intakeTransferTick = 0;
        public static double wristDown = 0.18;
        public static double wristUp = 0.0;
    }

    public static double p = 0.06, i = 0, d = 1;

    public PIDWrapper pid;

    private COLOR lastColor = COLOR.blank;
    private int colorBuffer;

    public IntakeSubsystem(COLOR teamColor) {
        this.pid = new PIDWrapper(
            p, i, d,
            0.75, 10,
            Robot.extendo::getCurrentPosition, Robot.extendo::setPower,
            true);

        this.teamColor = teamColor;
        enemyColor = teamColor == COLOR.red ? COLOR.blue : COLOR.red;
    }

    @Override
    public void periodic(){
        pid.update();
        COLOR c = getColor();

        if (c == lastColor) colorBuffer++;
        else {
            colorBuffer = 0;
            lastColor = c;
        }

        if (colorBuffer >= 3) colorDetected = c;


        Robot.telemetry.addData("Intake Color", colorDetected);
        pid.log("Intake");

        pid.setPID(p, i, d);
    }

    private COLOR getColor(){
        Color.RGBToHSV(
            Robot.intakeColorSensor.red() * 255,
            Robot.intakeColorSensor.green() * 255,
            Robot.intakeColorSensor.blue() * 255,
            colorHSV);

        double h = colorHSV[0];
        if (h > 205 && h < 240) return COLOR.blue;
        else if (h > 44 && h < 80) return COLOR.yellow;
        else if (h > 0 && h < 30) return COLOR.red;

        return COLOR.blank;
    }

    public void setIntakeDist(int ticks) {
        pid.setTarget(ticks);
    }

    public void overrideIntakePower(double power) {
        Robot.extendo.setPower(power);
    }

    public void setRollerSpeed(double power){
        Robot.flipper.setPower(power);
    }

    public void setIntakeSpeed(double power){
        Robot.intake.setPower(power);
    }

    public void cycleMode() {
        if (mode == PICKUP_MODE.transfer) mode = PICKUP_MODE.specimen;
        else if (mode == PICKUP_MODE.specimen) mode = PICKUP_MODE.auto;
        else mode = PICKUP_MODE.transfer;
    }

    public void setWristPos(WRIST pos) {
        wristPos = pos;
        setWrist(pos == WRIST.intake ? constants.wristDown : constants.wristUp);
    }

    public void toggleWristPos() {
        setWristPos(wristPos == WRIST.intake ? WRIST.transfer : WRIST.intake);
    }

    public void setWrist(double pos){
        Robot.intakeWristLeft.setPosition(0.5 + pos);
        Robot.intakeWristRight.setPosition(0.5 - pos);
    }

    public enum WRIST {
        transfer, intake
    }

    public enum COLOR {
        red, blue, yellow, blank
    }

    public enum PICKUP_MODE {
        transfer, specimen, auto
    }
}
