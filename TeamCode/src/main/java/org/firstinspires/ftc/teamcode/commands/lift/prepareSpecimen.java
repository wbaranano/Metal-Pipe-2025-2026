package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class prepareSpecimen extends SequentialCommandGroup {
    public prepareSpecimen() {
        LiftSubsystem lift = Robot.sys.lift;

        addCommands(
            new liftTo(LiftSubsystem.constants.specimenDeposit),
            new InstantCommand(() -> {
                LiftSubsystem.liftPreset p = LiftSubsystem.constants.specimenDepositPrepPreset;
                lift.setPivot(p.pivot);
                lift.setPitch(p.pitch);
                lift.setExtension(p.extension);
            }),
            // delay roll so specimen doesnt clip lift bars
            new WaitCommand(300),
            new InstantCommand(() -> lift.apply(LiftSubsystem.constants.specimenDepositPrepPreset))
        );
    }
}
