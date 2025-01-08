package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Extend intake and pickup pixel then retract. Transfers specimen depending on isForTransfer
 */
public class intakeRun extends SequentialCommandGroup {
    public static boolean isRunning = false;
    private Gamepad pad = null;

    public intakeRun(Gamepad pad) {
        this.pad = pad;
        init();
    }

    public intakeRun() {
        init();
    }

    private void init() {
        IntakeSubsystem in = Robot.sys.intake;
        isRunning = true;

        addCommands(
            // extend intake
            new InstantCommand(() -> in.setWristPos(IntakeSubsystem.WRIST.transfer)),
            new intakeTo(IntakeSubsystem.constants.fullIntakeOutTick),
            new InstantCommand(() -> {
                in.setWristPos(IntakeSubsystem.WRIST.intake);
                in.setIntakeSpeed(1);
                in.setRollerSpeed(0.25);
            }),
            // wait until we pickup a valid specimen (depending on mode)
            new intakeCollect(),
            // if mode is auto and we picked up a specimen vibrate controller so driver knows
            new ConditionalCommand(
                new InstantCommand(() -> pad.rumble(1000)),
                new InstantCommand(),
                () -> pad != null && in.mode == IntakeSubsystem.PICKUP_MODE.auto && !shouldTransfer()
            ),
            // retract intake
            new InstantCommand(() -> {
                in.setWristPos(IntakeSubsystem.WRIST.transfer);
                in.setIntakeSpeed(0);
                in.setRollerSpeed(0);
            }),
            new WaitCommand(250),
            new ConditionalCommand(
                // since our transfer tick is usually further back than what we can actually retract,
                // pid will never reach the desired tick (which is actually desired to pull back as
                // far as possible). it will only stop after our timeout (default 1s) stops it, which
                // would add latency if dont care about transferring - we dont need intake all the way back
                new intakeTo(IntakeSubsystem.constants.intakeTransferTick),
                new intakeTo(IntakeSubsystem.constants.intakeInTick),
                this::shouldTransfer
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
                new InstantCommand(),
                this::shouldTransfer
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        intakeRun.isRunning = false;
    }

    private boolean shouldTransfer() {
        IntakeSubsystem in = Robot.sys.intake;
        // color is blank if we cancel this command early
        if (in.colorDetected == IntakeSubsystem.COLOR.blank) return false;

        if (in.mode == IntakeSubsystem.PICKUP_MODE.transfer) return true;
        else if (in.mode == IntakeSubsystem.PICKUP_MODE.specimen) return false;

        // for auto, check the color
        return in.colorDetected == IntakeSubsystem.COLOR.yellow;
    }
}
