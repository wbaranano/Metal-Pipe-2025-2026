package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class intakeTo extends SequentialCommandGroup {
    public intakeTo(int tick) {
        addCommands(
            new InstantCommand(() -> Robot.sys.intake.setIntakeDist(tick)),
            new WaitUntilCommand(Robot.sys.intake.pid::done)
        );
    }
}
