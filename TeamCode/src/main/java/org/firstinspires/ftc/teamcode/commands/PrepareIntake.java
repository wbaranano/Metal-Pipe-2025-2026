package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
@Config
public class PrepareIntake extends SequentialCommandGroup {
    public static int ticksDistance=-1000;
    public PrepareIntake(IntakeSubsystem intake){
        addCommands(
            new InstantCommand(()->intake.setWrist(-.1)),
            new InstantCommand(()->{intake.setIntakeDistance(ticksDistance);}),
            new WaitUntilCommand(()->IntakeSubsystem.finished),
            new InstantCommand(()->{intake.setWristPos(IntakeSubsystem.WRIST.intake);}),
            new InstantCommand(()->intake.setIntakeSpeed(1)),
            new InstantCommand(()->intake.setRollerSpeed(.25))
        );
    }
}
