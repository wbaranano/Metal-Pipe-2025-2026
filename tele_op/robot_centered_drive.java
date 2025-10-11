package org.firstinspires.ftc.teamcode.tele_op;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Odometry Mecanum")
public class robot_centered_drive extends OpMode {
    GoBildaPinpointDriver odo; //declare odometry

    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        BLeft = hardwareMap.get(DcMotor.class, "BLeft");
        BRight = hardwareMap.get(DcMotor.class, "BRight");
        FLeft = hardwareMap.get(DcMotor.class, "FLeft");
        FRight = hardwareMap.get(DcMotor.class, "FRight");

        // reverse the motor direction
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // odometry computer configure
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // odometry starting position
        odo.resetPosAndIMU();
        Pose2D startingpostion = new Pose2D(DistanceUnit.MM, 0.0, 0.0, AngleUnit.RADIANS, 0);
        odo.setPosition(startingpostion);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.update();
    }

    public void moveRobot() {
        double forward = -gamepad1.left_stick_x;  // forward/back
        double strafe = -gamepad1.left_stick_y;    // left/right
        double rotate = gamepad1.right_stick_x;   // turn

        // mecanum wheel math (robot-centric)
        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = forward + strafe + rotate;  // Front Left
        wheelSpeeds[1] = forward - strafe - rotate;  // Front Right
        wheelSpeeds[2] = forward - strafe + rotate;  // Back Left
        wheelSpeeds[3] = forward + strafe - rotate;  // Back Right

        // normalize if any power > 1
        double max = Math.max(1.0, Math.abs(wheelSpeeds[0]));
        for (int i = 1; i < 4; i++) {
            max = Math.max(max, Math.abs(wheelSpeeds[i]));
        }
        for (int i = 0; i < 4; i++) {
            wheelSpeeds[i] /= max;
        }

        FLeft.setPower(wheelSpeeds[0]);
        FRight.setPower(wheelSpeeds[1]);
        BLeft.setPower(wheelSpeeds[2]);
        BRight.setPower(wheelSpeeds[3]);

        Pose2D pos = odo.getPosition();
        telemetry.addData("Robot X", pos.getX(DistanceUnit.MM));
        telemetry.addData("Robot Y", pos.getY(DistanceUnit.MM));
        telemetry.addData("Heading", pos.getHeading(AngleUnit.DEGREES));
    }

    @Override
    public void loop() {
        moveRobot();
        odo.update();
        telemetry.update();
    }
}
