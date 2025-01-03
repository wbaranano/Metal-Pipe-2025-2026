package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class Drive extends CommandBase {
    private Gamepad gamepad;
    private MecanumDrive drive;
    public Drive(DriveSubsystem drive, Gamepad gamepad){
        this.drive=drive.drive;
        this.gamepad=gamepad;
        addRequirements(drive);
    }
    @Override
    public void execute(){
        setWeightedDrivePowers(new Pose2d(gamepad.left_stick_x, gamepad.left_stick_y, Math.signum(gamepad.right_stick_x) * Math.pow(gamepad.right_stick_x, 4) / 3.0 * 2.0));
        /*drive.setDrivePowers(
            new PoseVelocity2d(
            new Vector2d(gamepad.left_stick_x, gamepad.left_stick_y),
            gamepad.right_stick_x
            ));
        drive.updatePoseEstimate();*/
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
