package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {
    public Gamepad pad;
    public DriveSubsystem(Gamepad pad) {
        this.pad = pad;
    }

    public void setWeightedDrivePowers(Pose2d pose) {
        double y = pose.position.y;
        double x = pose.position.x * 1.1 * -1.0;
        double rx = pose.heading.toDouble() * -1.0;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        setMotorPowers((y + x + rx) / denominator, (y - x - rx) / denominator, (y - x + rx) / denominator, (y + x - rx) / denominator);
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        Robot.leftFront.setPower(fl);
        Robot.rightFront.setPower(fr);
        Robot.leftRear.setPower(bl);
        Robot.rightRear.setPower(br);
    }
}
