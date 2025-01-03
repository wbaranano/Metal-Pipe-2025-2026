package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class ServoTest extends CommandOpMode {
    public double servoValue=0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        IntakeSubsystem intakeSubsystem=new IntakeSubsystem(); register(intakeSubsystem);
        Robot.robotInit(hardwareMap);
        GamepadEx pad1=new GamepadEx(gamepad1);
        pad1.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(()->{
           servoValue+=.02;
           intakeSubsystem.setWrist(servoValue);
           telemetry.addData("pos",servoValue);
           telemetry.update();
        }));
//        pad1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(()->{
//            servoValue-=.02;
//            intakeSubsystem.setWrist(servoValue);
//            telemetry.addData("pos",servoValue);
//            telemetry.update();
//        }));
        pad1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new Intake(intakeSubsystem));
        pad1.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(()->{
            servoValue=0.0;
            intakeSubsystem.setWrist(0.0);
            telemetry.addData("pos",servoValue);
            telemetry.update();
        }));
        pad1.getGamepadButton(GamepadKeys.Button.X).whenPressed(new InstantCommand(()->schedule(new Intake(intakeSubsystem))));

    }
    @Override
    public void run(){
        CommandScheduler.getInstance().run();
    }
}
