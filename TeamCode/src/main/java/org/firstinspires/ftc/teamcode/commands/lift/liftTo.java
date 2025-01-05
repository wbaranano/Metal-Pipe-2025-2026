package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class liftTo extends SequentialCommandGroup {
    public liftTo(int tick) {
        addCommands(
            new InstantCommand(() -> Robot.sys.lift.setTick(tick)),
            new WaitUntilCommand(Robot.sys.lift.pid::done)
        );
    }
}
