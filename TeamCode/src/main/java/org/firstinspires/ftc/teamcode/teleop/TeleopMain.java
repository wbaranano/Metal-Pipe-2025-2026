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
import org.firstinspires.ftc.teamcode.commands.lift.depositSpecimen;
import org.firstinspires.ftc.teamcode.commands.lift.exitBucketForSpecimen;
import org.firstinspires.ftc.teamcode.commands.lift.prepareSpecimen;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemHandler;

@TeleOp
public class TeleopMain extends CommandOpMode {
    private int liftTick = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Robot.robotInit(hardwareMap, telemetry);
        Robot.registerSubsystems(new SubsystemHandler(
            new LiftSubsystem(),
            new IntakeSubsystem(IntakeSubsystem.COLOR.red), // TODO
            new DriveSubsystem(gamepad1)
        ));

        initGamepadOne();
        initGamepadTwo();

        Robot.telemetry.addLine("Ready");
        Robot.telemetry.update();

        waitForStart();
    }

    private void initGamepadOne() {
        GamepadEx pad1 = new GamepadEx(gamepad1);

        pad1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenHeld(new InstantCommand(() -> Robot.flipper.setPower(0.25)));
        pad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenHeld(new InstantCommand(() -> Robot.flipper.setPower(1)));
    }

    private void initGamepadTwo() {
        GamepadEx pad2 = new GamepadEx(gamepad2);

        // stop intake
        /*
        pad2.getGamepadButton(GamepadKeys.Button.X).whenPressed(
            new InstantCommand(() -> {
                Robot.sys.intake.setWristPos(IntakeSubsystem.WRIST.transfer);
                Robot.sys.intake.setIntakeSpeed(0);
                Robot.sys.intake.setRollerSpeed(0);
            }));
         */

        // pickup specimen
        // pad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new intakeRun(false));

        // drop specimen
        // pad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new intakePlace());

        pad2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> Robot.sys.lift.pid.increment(20));
        pad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> Robot.sys.lift.pid.increment(-20));

        pad2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new exitBucketForSpecimen());
        pad2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new prepareSpecimen());
        pad2.getGamepadButton(GamepadKeys.Button.B).whenPressed(new depositSpecimen());


        pad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> Robot.sys.lift.setClaw(true));
        pad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> Robot.sys.lift.setClaw(false));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        Robot.telemetry.update();
    }
}
