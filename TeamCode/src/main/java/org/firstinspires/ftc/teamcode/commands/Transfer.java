package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class Transfer extends SequentialCommandGroup {
    public Transfer(IntakeSubsystem intake){
        /*
        addCommands(

            new InstantCommand(()->intake.setWrist(-.1)),
            new InstantCommand(()->intake.setIntakeSpeed(0.0)),
            new InstantCommand(()->intake.setRollerSpeed(0)),
            new WaitCommand(500),
            new InstantCommand(()->intake.setIntakeDistance(0)),
            new WaitUntilCommand(()->IntakeSubsystem.finished),
            new InstantCommand(()->intake.setWristPos(IntakeSubsystem.WRIST.transfer)),
            new WaitCommand(500),
            new InstantCommand(()->intake.setRollerSpeed(1)),
            new InstantCommand(()->intake.setIntakeSpeed(1)),
            new WaitUntilCommand(()->IntakeSubsystem.colorDetected== IntakeSubsystem.COLOR.blank),
            new WaitCommand(500),
            new InstantCommand(()->intake.setRollerSpeed(0.0)),
            new InstantCommand(()->intake.setIntakeSpeed(0))
            );
         */
    }
}
