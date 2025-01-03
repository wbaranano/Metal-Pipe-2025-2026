package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.Drive;
import org.firstinspires.ftc.teamcode.commands.HardwareUpdate;
import org.firstinspires.ftc.teamcode.commands.PrepareIntake;
import org.firstinspires.ftc.teamcode.commands.Transfer;
import org.firstinspires.ftc.teamcode.commands.UpdateTelemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@TeleOp
public class TeleopPrelim extends CommandOpMode {

    LiftSubsystem lift;
    IntakeSubsystem intake;
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        intake=new IntakeSubsystem(); register(intake);
        //lift=new LiftSubsystem(); register(lift);

        Robot.robotInit(hardwareMap);

        MecanumDrive drive=new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
        DriveSubsystem driveSubsystem=new DriveSubsystem(drive);
        register(driveSubsystem);
        //driveSubsystem.setDefaultCommand(new Drive(driveSubsystem,gamepad1));
        GamepadEx pad1=new GamepadEx(gamepad1);
        pad1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new PrepareIntake(intake));
        pad1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new Transfer(intake));

        //CommandScheduler.getInstance().schedule(new UpdateTelemetry());
        //CommandScheduler.getInstance().schedule(new HardwareUpdate());






    }
    @Override
    public void run(){
        CommandScheduler.getInstance().run();
    }

}
