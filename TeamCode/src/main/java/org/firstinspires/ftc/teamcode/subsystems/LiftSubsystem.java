package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.PID.PID;
@Config
public class LiftSubsystem extends SubsystemBase {
    public static int positionTicksSP=0;
    public static int positionTicks;
    public static double p=0;
    public static double i=0;
    public static double d=0;
    public PID pid;
    public LiftSubsystem(){
        pid=new PID(p,i,d);
    }
    @Override
    public void periodic(){
        pid.setSP(positionTicksSP);
        setPower(pid.update(getPositionTicks()));
    }
    public int getPositionTicks(){
        positionTicks=Robot.liftEncoder.getCurrentPosition();
        return positionTicks;
    }
    public void setPower(double power){
        Robot.liftLeft.setPower(-power);
        Robot.liftRight.setPower(power);
        //Robot.liftMotor.setPower(power);



    }
    public void setSP(double sp){
        positionTicksSP=(int)sp;
    }
    public int getPositionTicksSP(){
        return positionTicksSP;
    }
    public void wristSet(double pos){Robot.wrist1.setPosition(pos);Robot.wrist2.setPosition(pos);}
    public void armSet(double pos){Robot.arm1.setPosition(pos);Robot.arm2.setPosition(pos);}
    public void turnSet(double pos){Robot.turn.setPosition(pos);}

}
