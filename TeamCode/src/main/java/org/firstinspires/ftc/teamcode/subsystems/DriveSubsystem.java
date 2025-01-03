package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {
    public MecanumDrive drive;
    public DriveSubsystem(MecanumDrive drive){
        this.drive=drive;
    }
}
