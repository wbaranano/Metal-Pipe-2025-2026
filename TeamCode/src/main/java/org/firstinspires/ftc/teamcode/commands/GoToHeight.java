package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class GoToHeight extends CommandBase {
    /*
    IntakeSubsystem intake;
    int[] levels={800,900,1000,1100,1200};//specimen rung1 bucket1 rung2 bucket2
    int bucket1=300;
    int specimen=400;
    int bucket2=800;
    int thresh=700;
    int rung1=500;
    int rung2=800;
    double wristplace=.5;
    double railout=.5;
    double specimenTurn=.5;
    LiftSubsystem lift;
    private boolean isFinished=false;
    private int levelDesired;
    public GoToHeight(IntakeSubsystem intake, LiftSubsystem lift,int level){
        this.lift=lift;
        this.intake=intake;
        this.levelDesired=level;
        lift.setSP(levels[level]);

    }
    @Override
    public void execute(){
        if(lift.getPositionTicks()>thresh){
            lift.armSet(railout);
            lift.wristSet(wristplace);
            if(levelDesired==0){
                lift.turnSet(specimenTurn);
            }
        }
    }
     */
}
