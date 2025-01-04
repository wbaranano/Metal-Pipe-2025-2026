package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.PID.PID;
@Config
public class LiftSubsystem extends SubsystemBase {
    @Override
    public void periodic(){

    }

    public void setPower(double power){
        Robot.liftFront.setPower(-power);
        Robot.liftBack.setPower(power);
    }
}
