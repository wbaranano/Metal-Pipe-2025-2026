package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * From the starting position (lift is fully down and claw is inside bucket), move into the specimen
 * collecting position
 */
public class exitBucketForSpecimen extends SequentialCommandGroup {
    public exitBucketForSpecimen() {
        LiftSubsystem lift = Robot.sys.lift;

        addCommands(
            new liftTo(LiftSubsystem.constants.tick.aboveBucket),
            new InstantCommand(() -> lift.apply(LiftSubsystem.constants.specimenCollectionPreset)),
            new WaitCommand(1000),
            new liftTo(LiftSubsystem.constants.tick.specimenCollection)
        );
    }
}
