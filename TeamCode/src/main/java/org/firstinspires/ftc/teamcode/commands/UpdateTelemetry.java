package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class UpdateTelemetry extends CommandBase {
    @Override
    public void execute(){
        Robot.telemetry.update();
    }
}
