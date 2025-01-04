package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.commands.Drive;
import org.firstinspires.ftc.teamcode.commands.intake.intakeCollect;
import org.firstinspires.ftc.teamcode.commands.intake.intakePlace;
import org.firstinspires.ftc.teamcode.commands.intake.intakeRun;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemHandler;

@TeleOp
public class TeleopMain extends CommandOpMode {
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Robot.robotInit(hardwareMap);
        Robot.registerSubsystems(new SubsystemHandler(
            new LiftSubsystem(),
            new IntakeSubsystem(IntakeSubsystem.COLOR.red), // TODO
            new DriveSubsystem(gamepad1)
        ));

        // initGamepadOne();
        // initGamepadTwo();

        Robot.telemetry.addLine("Ready");
        Robot.telemetry.update();

        waitForStart();
    }

    private void initGamepadOne() {
        GamepadEx pad1 = new GamepadEx(gamepad1);
    }

    private void initGamepadTwo() {
        GamepadEx pad2 = new GamepadEx(gamepad2);

        pad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(() -> Robot.sys.intake.overrideIntakePower(0.25))
            .whenReleased(() -> Robot.sys.intake.overrideIntakePower(0));

        pad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(() -> Robot.sys.intake.overrideIntakePower(-0.25))
            .whenReleased(() -> Robot.sys.intake.overrideIntakePower(0));

        // test intake
        pad2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        Robot.sys.intake.setWristPos(IntakeSubsystem.WRIST.intake);
                        Robot.sys.intake.setIntakeSpeed(1);
                        Robot.sys.intake.setRollerSpeed(0.25);
                    }),
                    new intakeCollect(false),
                    new InstantCommand(() -> {
                        Robot.sys.intake.setWristPos(IntakeSubsystem.WRIST.transfer);
                        Robot.sys.intake.setIntakeSpeed(0);
                        Robot.sys.intake.setRollerSpeed(0);
                    })
                ));

        // test intake extension
        pad2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
            new InstantCommand(() -> Robot.sys.intake.setIntakeDist(-1000))
        );

        pad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
            new InstantCommand(() -> Robot.sys.intake.setIntakeDist(0))
        );

        // pickup specimen
        pad2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new intakeRun(false));

        // drop specimen
        pad2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new intakePlace());
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        Robot.telemetry.update();
    }
}
