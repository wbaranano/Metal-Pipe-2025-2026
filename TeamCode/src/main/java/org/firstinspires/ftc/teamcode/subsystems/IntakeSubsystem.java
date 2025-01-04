package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.teamcode.PID.PID;
import org.firstinspires.ftc.teamcode.PID.PIDWrapper;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public WRIST wristPos;
    public COLOR colorDetected;
    public float[] colorHSV = {0f, 0f, 0f};
    public COLOR teamColor;
    public COLOR enemyColor;

    public static double wristDown=0.18;
    public static double wristUp=0.0;

    public PIDWrapper pid;

    public IntakeSubsystem(COLOR teamColor) {
        this.pid = new PIDWrapper(
            0.01, 0, 0,
            0.2, 200,
            Robot.extendo::getCurrentPosition, Robot.extendo::setPower);

        this.teamColor = teamColor;
        enemyColor = teamColor == COLOR.red ? COLOR.blue : COLOR.red;
    }

    @Override
    public void periodic(){
        pid.update();
        colorDetected = getColor();

        Robot.telemetry.addData("Intake Color", colorDetected);
        pid.log("Intake");
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
    public void setWristPos(WRIST pos){
        wristPos = pos;
        setWrist(pos == WRIST.intake ? wristDown : wristUp);
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
}
