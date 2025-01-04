package org.firstinspires.ftc.teamcode.teleop;

import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.commands.Intake;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemHandler;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp
public class ServoTest extends CommandOpMode {
    private final List<List<Pair<String, Servo>>> servos = new ArrayList<>();
    private int index = 0;
    private double speed = 0.01;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Robot.robotInit(hardwareMap);
        Robot.registerSubsystems(new SubsystemHandler(
            new LiftSubsystem(),
            new IntakeSubsystem(IntakeSubsystem.COLOR.red),
            new DriveSubsystem(gamepad1)
        ));

        addServo("claw", Robot.claw);
        addServo("claw pitch", Robot.clawPitch);
        addServo("claw roll", Robot.clawRoll);
        servos.add(Arrays.asList(
            new Pair<>("pivot front", Robot.armPivotFront),
            new Pair<>("pivot back", Robot.armPivotBack)));
        servos.add(Arrays.asList(
            new Pair<>("extension front", Robot.liftHorzExtFront),
            new Pair<>("extension back", Robot.liftHorzExtBack)));
        servos.add(Arrays.asList(
            new Pair<>("wrist right", Robot.intakeWristRight),
            new Pair<>("wrist left", Robot.intakeWristLeft)));

        GamepadEx pad = new GamepadEx(gamepad1);
        pad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> {
            index++;
            if (index >= servos.size()) index = 0;
        });

        pad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            index--;
            if (index < 0) index = servos.size() - 1;
        });

        pad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            speed = Math.min(0.02, speed + 0.0005);
        });

        pad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            speed = Math.max(0.001, speed - 0.0005);
        });

        pad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> {
            List<Pair<String, Servo>> _s = servos.get(index);
            for (Pair<String, Servo> s : _s) {
                s.second.setPosition(Math.min(1, s.second.getPosition() + speed));
            }
        });

        pad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> {
            List<Pair<String, Servo>> _s = servos.get(index);
            for (Pair<String, Servo> s : _s) {
                s.second.setPosition(Math.max(0, s.second.getPosition() - speed));
            }
        });

        waitForStart();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        Robot.telemetry.addData("speed", speed);
        Robot.telemetry.addData("index", index);

        List<Pair<String, Servo>> _s = servos.get(index);
        for (Pair<String, Servo> s : _s) {
            Robot.telemetry.addData(s.first + " position", s.second.getPosition());
            Robot.telemetry.addData(s.first + " direction", s.second.getDirection());
        }

        Robot.telemetry.update();
    }

    private void addServo(String name, Servo s) {
        ArrayList<Pair<String, Servo>> arr = new ArrayList<>();
        arr.add(new Pair<>(name, s));

        servos.add(arr);
    }
}
