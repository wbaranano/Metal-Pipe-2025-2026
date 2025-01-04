package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class Intake extends CommandBase {
    IntakeSubsystem intake;
    boolean isFinished=false;
    public Intake(IntakeSubsystem intake){
        this.intake=intake;

    }
    @Override
    public void execute(){
        /*
        if(IntakeSubsystem.colorDetected== Robot.enemyColor){
            intake.setRollerSpeed(1);
        }
        else if(IntakeSubsystem.colorDetected!= IntakeSubsystem.COLOR.blank){
            intake.setRollerSpeed(0);
            intake.setIntakeSpeed(0);

        }
        else{
            intake.setRollerSpeed(.5);
            intake.setIntakeSpeed(1.0);
        }
        */
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
