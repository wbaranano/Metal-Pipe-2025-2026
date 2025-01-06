package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.intake.intakePlace;
import org.firstinspires.ftc.teamcode.commands.intake.intakeRun;
import org.firstinspires.ftc.teamcode.commands.intake.intakeTo;
import org.firstinspires.ftc.teamcode.commands.lift.depositSpecimen;
import org.firstinspires.ftc.teamcode.commands.lift.exitBucketForSpecimen;
import org.firstinspires.ftc.teamcode.commands.lift.liftSetState;
import org.firstinspires.ftc.teamcode.commands.lift.prepareSpecimen;
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

        // TODO: move into own command?
        // set initial robot conditions
        schedule(new SequentialCommandGroup(
            new InstantCommand(() -> {
              Robot.sys.lift.setClawClosed(false);
              Robot.sys.intake.setWristPos(IntakeSubsystem.WRIST.transfer);
            })
        ));
    }

    private void initGamepadOne() {
        GamepadEx pad = new GamepadEx(gamepad1);

        pad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new intakeRun());
        pad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new intakePlace());

        pad.getGamepadButton(GamepadKeys.Button.B).whenPressed(Robot.sys.intake::toggleMode);
        pad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new intakePlace(150));
        pad.getGamepadButton(GamepadKeys.Button.X).whenPressed(Robot.sys.intake::toggleWristPos);
    }

    private void initGamepadTwo() {
        GamepadEx pad2 = new GamepadEx(gamepad2);

        pad2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenHeld(new InstantCommand(() -> Robot.sys.lift.pid.increment(5)));
        pad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenHeld(new InstantCommand(() -> Robot.sys.lift.pid.increment(-5)));

        pad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(Robot.sys.lift::toggleClaw);

        pad2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new liftSetState(LiftSubsystem.liftState.basket));
        pad2.getGamepadButton(GamepadKeys.Button.A).whenPressed(new liftSetState(LiftSubsystem.liftState.transfer));
        pad2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new liftSetState(LiftSubsystem.liftState.wall));
        pad2.getGamepadButton(GamepadKeys.Button.B).whenPressed(new liftSetState(LiftSubsystem.liftState.specimen));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        Robot.telemetry.addLine("=== STATE ===");
        Robot.telemetry.addData("Pickup mode",
            Robot.sys.intake.mode == IntakeSubsystem.PICKUP_MODE.transfer ? "TRANSFER" : "SPECIMEN");
        Robot.telemetry.addData("Lift state", getLiftState());

        Robot.telemetry.update();
    }

    private String getLiftState() {
        LiftSubsystem.liftState state = Robot.sys.lift.state;
        if (state == LiftSubsystem.liftState.transfer) return "TRANSFER";
        else if (state == LiftSubsystem.liftState.basket) return "BASKET";
        else if (state == LiftSubsystem.liftState.specimen) return "SPECIMEN";
        else if (state == LiftSubsystem.liftState.wall) return "WALL";
        return "INTERMEDIARY";
    }
}
