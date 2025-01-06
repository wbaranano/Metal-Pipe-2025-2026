package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class Drive extends CommandBase {
    private final DriveSubsystem drive;
    public static boolean disable = false;
    public Drive(DriveSubsystem drive){
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void execute(){
        if (disable) return;

        double offsetX = 0, offsetY = 0, offsetHeading = 0;
        if (drive.pad.dpad_up) offsetY -= 0.25;
        if (drive.pad.dpad_down) offsetY += 0.25;
        if (drive.pad.dpad_right) offsetX += 0.25;
        if (drive.pad.dpad_left) offsetX -= 0.25;

        offsetHeading += drive.pad.right_trigger / 3.0;
        offsetHeading -= drive.pad.left_trigger / 3.0;

        drive.setWeightedDrivePowers(new Pose2d(
            drive.pad.left_stick_x + offsetX,
            drive.pad.left_stick_y + offsetY,
            Math.signum(drive.pad.right_stick_x) * Math.pow(drive.pad.right_stick_x, 4) / 3.0 * 2.0 + offsetHeading));
    }
}
