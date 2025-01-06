package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

/**
 * Given an already collected pixel, extend intake and eject it then retract intake
 */
public class intakePlace extends SequentialCommandGroup {
    public intakePlace(int dist) {
        cmd(dist);
    }

    public intakePlace() {
        cmd(IntakeSubsystem.constants.fullIntakeOutTick);
    }

    private void cmd(int tick) {
        IntakeSubsystem in = Robot.sys.intake;

        addCommands(
            new InstantCommand(() -> in.setWristPos(IntakeSubsystem.WRIST.transfer)),
            new intakeTo(tick),
            new InstantCommand(() -> {
                in.setIntakeSpeed(-1);
                in.setRollerSpeed(-1);
            }),
            new WaitUntilCommand(() -> in.colorDetected == IntakeSubsystem.COLOR.blank),
            new InstantCommand(() -> {
                in.setIntakeSpeed(0);
                in.setRollerSpeed(0);
            }),
            new intakeTo(IntakeSubsystem.constants.intakeInTick)
        );
    }
}
