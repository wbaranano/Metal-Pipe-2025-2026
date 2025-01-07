package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;

import org.firstinspires.ftc.teamcode.commands.Drive;

public class SubsystemHandler {
    public LiftSubsystem lift;
    public IntakeSubsystem intake;
    public DriveSubsystem drive;

    public SubsystemHandler(LiftSubsystem lift, IntakeSubsystem intake, DriveSubsystem drive) {
        this.lift = lift;
        this.intake = intake;
        this.drive = drive;

        CommandScheduler.getInstance().registerSubsystem(lift, intake, drive);

        drive.setDefaultCommand(new Drive(drive));
    }
}
