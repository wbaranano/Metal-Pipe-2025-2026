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
    private final DriveSubsystem drive;
    public Drive(DriveSubsystem drive){
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute(){
        drive.setWeightedDrivePowers(new Pose2d(
            drive.pad.left_stick_x,
            drive.pad.left_stick_y,
            Math.signum(drive.pad.right_stick_x) * Math.pow(drive.pad.right_stick_x, 4) / 3.0 * 2.0));
    }
}
