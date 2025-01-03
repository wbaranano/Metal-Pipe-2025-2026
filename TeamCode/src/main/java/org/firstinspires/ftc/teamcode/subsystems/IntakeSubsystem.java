package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.PID.PID;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public enum WRIST{
        transfer,
        intake
    }
    public enum COLOR{
        red,
        blue,
        yellow,
        blank
    }
    public static WRIST wristPos;
    public static COLOR colorDetected;
    public static double wristDown=0.18;
    public static double wristUp=0.0;
    public static double outPosition=0.0;
    public static double p=0.01;
    public static double intakethresh=4.0;
    public static double i=0.0;
    public static double d=0.0;
    public static boolean finished=false;
    //public PID pid=new PID(p,i,d);
    public double set=0;
    PIDController pid=new PIDController(p,i,d);
    @Override
    public void periodic(){

        setExtendoPower(pid.calculate(getExtendoPosition()));
        if(Math.abs(getExtendoPosition()-set)<200){
            finished=true;
        }
        else{
            finished=false;
        }

        colorDetected=getColor();

    }
    private COLOR getColor(){
        if(Robot.hsvIntake[0]>205&&Robot.hsvIntake[0]<240){
            return(COLOR.blue);
        }
        else if(Robot.hsvIntake[0]>44&&Robot.hsvIntake[0]<80){
            return(COLOR.yellow);
        }
        else if(Robot.hsvIntake[0]>0&&Robot.hsvIntake[0]<30){
            return (COLOR.red);
        }
        else{
            return (COLOR.blank);
        }

    }
    public void setIntakeDistance(int ticks){
        pid.setSetPoint(ticks);
        set=ticks;
    }
    public void setExtendoPower(double power){
        Robot.extendo.setPower(power);
    }
    public double getExtendoPosition(){
        return Robot.extendo.getCurrentPosition();
    }
    public void setRollerSpeed(double power){
        Robot.flipper.setPower(power);
    }
    public void setIntakeSpeed(double power){
        Robot.intake.setPower(power);
    }
    public void setWristPos(WRIST pos){
        if(pos==WRIST.intake){
            wristPos=pos;
            setWrist(wristDown);

        }
        else{
            wristPos=pos;
            setWrist(wristUp);
        }
    }
    public void setWrist(double pos){
        Robot.intakeWrist2.setPosition(.5+pos);
        Robot.intakeWrist1.setPosition(.5-pos);
    }

}
