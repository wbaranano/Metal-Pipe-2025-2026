package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Extend intake and pickup pixel then retract. Transfers specimen depending on isForTransfer
 */
public class intakeRun extends SequentialCommandGroup {
    public intakeRun() {
        IntakeSubsystem in = Robot.sys.intake;

        addCommands(
            new InstantCommand(() -> in.setWristPos(IntakeSubsystem.WRIST.transfer)),
            new intakeTo(IntakeSubsystem.constants.fullIntakeOutTick),
            new InstantCommand(() -> {
                in.setWristPos(IntakeSubsystem.WRIST.intake);
                in.setIntakeSpeed(1);
                in.setRollerSpeed(0.25);
            }),
            new intakeCollect(),
            new InstantCommand(() -> {
                in.setWristPos(IntakeSubsystem.WRIST.transfer);
                in.setIntakeSpeed(0);
                in.setRollerSpeed(0);
            }),
            new WaitCommand(250),
            // since intakeInTick is < 0, occasionally we will not be able to complete the retraction
            // as we we can't retract far enough (this is actually normally intentional to ensure
            // intake is fully retracted)
            new ConditionalCommand(
                new intakeTo(IntakeSubsystem.constants.intakeTransferTick),
                new intakeTo(IntakeSubsystem.constants.intakeInTick),
                () -> in.mode == IntakeSubsystem.PICKUP_MODE.transfer
            ),
            // if we are transferring, put specimen in bucket
            new ConditionalCommand(
                // transfer
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        in.setRollerSpeed(1);
                        in.setIntakeSpeed(1);
                    }),
                    new WaitUntilCommand(() -> in.colorDetected == IntakeSubsystem.COLOR.blank),
                    new WaitCommand(1000),
                    new InstantCommand(() -> {
                        in.setIntakeSpeed(0);
                        in.setRollerSpeed(0);
                    }),
                    new intakeTo(IntakeSubsystem.constants.intakeInTick)
                ),
                new InstantCommand(() -> {}),
                () -> in.mode == IntakeSubsystem.PICKUP_MODE.transfer
            )
        );
    }
}
