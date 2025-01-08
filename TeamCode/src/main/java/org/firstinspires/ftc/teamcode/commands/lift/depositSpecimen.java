package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class depositSpecimen extends SequentialCommandGroup {
    public depositSpecimen() {
        addCommands(
            new InstantCommand(() -> Robot.sys.lift.dampenRetraction = false),
            new ParallelCommandGroup(
                new liftTo(LiftSubsystem.constants.tick.specimenDepositSlam),
                new SequentialCommandGroup(
                    new WaitCommand(1000),
                    new InstantCommand(() -> {
                        Robot.sys.lift.setClawClosed(false);
                        Robot.sys.lift.dampenRetraction = true;
                    })
                )
            )
        );
    }
}
