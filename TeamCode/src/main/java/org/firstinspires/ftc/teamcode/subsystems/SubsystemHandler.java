package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandScheduler;

public class SubsystemHandler {
    public LiftSubsystem lift;
    public IntakeSubsystem intake;
    public DriveSubsystem drive;

    public SubsystemHandler(LiftSubsystem lift, IntakeSubsystem intake, DriveSubsystem drive) {
        this.lift = lift;
        this.intake = intake;
        this.drive = drive;

        CommandScheduler.getInstance().registerSubsystem(lift, intake, drive);
    }
}
