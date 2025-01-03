package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class HardwareUpdate extends CommandBase {
    @Override
    public void execute(){
        Robot.sensorUpdate();
        Robot.telemetry.addData("hue",Robot.hsvIntake[0]);
    }
}
