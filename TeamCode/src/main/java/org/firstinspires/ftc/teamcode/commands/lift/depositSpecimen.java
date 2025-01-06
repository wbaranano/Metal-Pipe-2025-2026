package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class depositSpecimen extends SequentialCommandGroup {
    public depositSpecimen() {
        addCommands(
            new ParallelDeadlineGroup(
                new WaitCommand(1500),
                new InstantCommand(() -> {
                    Robot.sys.lift.setTick(LiftSubsystem.constants.specimenDepositLift);
                    Robot.sys.lift.dampenRetraction = false;
                })
            ),
            new InstantCommand(() -> {
                Robot.sys.lift.setClawClosed(false);
                Robot.sys.lift.dampenRetraction = true;
            })
        );
    }
}
